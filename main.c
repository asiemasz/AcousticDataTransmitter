#include "BPSK.h"
#include "MCP4822.h"
#include "SRRC_filter.h"
#include "adc.h"
#include "arm_math.h"
#include "gpio.h"
#include "random_generator.h"
#include "spi.h"
#include "stm32f401xe.h"
#include "timer.h"
#include "uart.h"

#define FS 48000
#define FSystem 84000000
#define FC 8000
#define ROLLOVER_FACTOR 0.5
#define FSPAN 2
#define SPB 24
#define DATA_FRAME_LENGTH 8
#define PREAMBLE_LENGTH 5

extern uint32_t SystemCoreClock;

const static SPI_pinout spi1_pins = {
    .af = GPIO_AF5,
    .CLK_Pin = PIN3,
    .CLK_Port = GPIOB,
    .MISO_Pin = PIN6,
    .MISO_Port = GPIOA,
    .MOSI_Pin = PIN7,
    .MOSI_Port = GPIOA,
};

const static SPI_initStruct spi1 = {
    .clkPrescaller = SPI_MASTER_CLOCK_PRESCALLER_8,
    .clockMode = SPI_CLOCK_MODE_POLARITY_0_PHASE_0,
    .mode = SPI_MODE_MASTER,
    .slaveSelectMode = SPI_SOFTWARE_NSS_MANAGEMENT,
    .pinout = spi1_pins,
    .dataLength = SPI_DATA_FRAME_LENGTH_8_BITS,
    .spi = SPI1,
};

const static TIMER_initStruct tim2 = {
    // sampling frequency
    .tim = TIM2,
    .direction = TIMER_COUNTER_DIRECTION_DOWN,
    .prescaler = 1,
    .autoReload = FSystem / FS,
};

void MCP4822_write(const uint8_t *data, const uint16_t len) {
  spi_write(&spi1, data, len);
}

void MCP4822_CSen(const uint32_t *GPIO_port, const uint8_t GPIO_pin) {
  gpio_resetPin(GPIO_port, GPIO_pin);
}

void MCP4822_CSdis(const uint32_t *GPIO_port, const uint8_t GPIO_pin) {
  gpio_setPin(GPIO_port, GPIO_pin);
}
const static UART_initStruct uart2 = {.baudRate = 115200,
                                      .mode = UART_TRANSMITTER_ONLY,
                                      .oversampling = UART_OVERSAMPLING_BY_16,
                                      .parityControl =
                                          UART_PARITY_CONTROL_DISABLED,
                                      .stopBits = UART_STOP_BITS_1,
                                      .wordLength = UART_WORD_LENGTH_8,
                                      .uart = USART2};

MCP4822_device MCP4822 = {.SS_pin = PIN3,
                          .SS_Port = GPIOC,
                          .writeFunction = MCP4822_write,
                          .CSenableFunction = MCP4822_CSen,
                          .CSdisableFunction = MCP4822_CSdis};

MCP4822_OUTPUT_CONFIG cfg = {
    .gain = MCP4822_OUTPUT_GAIN_x2,
    .output = MCP4822_DAC_B,
    .powerDown = MCP4822_OUTPUT_POWERDOWN_CONTROL_BIT,
};

static const int8_t preamble[PREAMBLE_LENGTH] = {1, 1, 1, -1, 1};

static BPSK_parameters params = {.Fb = FS / SPB,
                                 .Fc = FC,
                                 .frameLength = DATA_FRAME_LENGTH,
                                 .FSpan = FSPAN,
                                 .preambleCode = preamble,
                                 .preambleCodeLength = PREAMBLE_LENGTH,
                                 .samplesPerBit = SPB};

const float32_t FN = (float32_t)FC / FS;

volatile uint8_t dataReady = 0;
volatile uint16_t val;

volatile uint32_t i = 0;
volatile uint32_t j = 0;

int main() {
  float32_t coeffs[FSPAN * SPB + 1];
  float32_t modulated_signal[SPB * (DATA_FRAME_LENGTH + PREAMBLE_LENGTH)] = {0};

  initRandomGenerator(2137);

  SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, coeffs, FSPAN * SPB + 1, 3.7f);

  params.matchedFilterCoeffs = coeffs;
  params.matchedFilterCoeffsLength = FSPAN * SPB + 1;

  gpio_init(GPIOA);
  gpio_init(GPIOC);

  GPIO_pinConfigStruct pin3 = {
      .mode = GPIO_MODE_OUTPUT,
      .outSpeed = GPIO_OUT_SPEED_VERY_HIGH,
  };
  gpio_setPinConfiguration(GPIOC, PIN3, &pin3);

  uart_init(&uart2);
  spi_init(&spi1);
  timer_init(&tim2);
  timer_enableIT(&tim2, TIMER_IT_UPDATE_EVENT);

  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);

  char buf[20];

  uint8_t random_byte = generateRandomFromRange(0, 255);

  timer_start(&tim2);
  while (1) {
    if (dataReady) {
      float32_t y =
          (modulated_signal[j] * arm_cos_f32(FN * i * 2.0f * PI) + 1.0f) / 2.0f;
      val = y * 4000.0f;
      MCP4822_setValue(&MCP4822, val, &cfg);
      dataReady = 0;
      if (j == SPB * (DATA_FRAME_LENGTH + PREAMBLE_LENGTH)) {
        j = 0;
        random_byte = generateRandomFromRange(0, 255);
        arm_fill_f32(0, modulated_signal,
                     SPB * (DATA_FRAME_LENGTH + PREAMBLE_LENGTH));
        BPSK_getOutputSignalWithPreamble(
            &params, &random_byte, 1, modulated_signal,
            1 * SPB * (DATA_FRAME_LENGTH + PREAMBLE_LENGTH));
      }
    }
  }
}

void TIM2_IRQHandler() {
  timer_clearITflag(&tim2);
  if (dataReady) {
    char buf[20];
    sprintf(buf, "\r\n\r\nOverrun");
    uart_sendString(&uart2, buf);
    Default_Handler();
  }

  i = (i + 1) % (FS / FC);
  j++;

  dataReady = 1;
}

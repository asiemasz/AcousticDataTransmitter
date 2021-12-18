#include "BPSK.h"
#include "SRRC_filter.h"
#include "adc.h"
#include "arm_math.h"
#include "gpio.h"
#include "i2c.h"
#include "stm32f401xe.h"
#include "timer.h"
#include "uart.h"

#define SAMPLES 2048

#define FS 48000
#define FSystem 84000000
#define FC 17000
#define ROLLOVER_FACTOR 0.25
#define FSPAN 4
#define SPB 96
#define N_BYTES 1

#define NUM_TAPS_ARRAY_SIZE 30

/*static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    -0.001238f, -0.002175f, -0.000845f, 0.003789f,
    0.007679f,  0.002303f,  -0.013385f, -0.022869f,
    -0.004360f, 0.038050f,  0.059609f,  0.006119f,
    -0.128249f, -0.275917f, 0.660179f,  -0.275917f,
    -0.128249f, 0.006119f,  0.059609f,  0.038050f,
    -0.004360f, -0.022869f, -0.013385f, 0.002303f,
    0.007679f,  0.003789f,  -0.000845f, -0.002175f,
    -0.001238f, 0}; // highpass filter coeffs (15kHz +)*/

static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    0.00279261903117354f, -0.00704872099778532f, 0.00522782761055690f,
    0.00806094259064239f, -0.0241860356361024f,  0.0172911463428934f,
    0.0235154921508081f,  -0.0610485215400974f,  0.0379882142012305f,
    0.0455136710707779f,  -0.105344191064127f,   0.0590337115211551f,
    0.0642021511029681f,  -0.135694004120373f,   0.0697312776608236f,
    0.0697312776608236f,  -0.135694004120373f,   0.0642021511029681f,
    0.0590337115211551f,  -0.105344191064127f,   0.0455136710707779f,
    0.0379882142012305f,  -0.0610485215400974f,  0.0235154921508081f,
    0.0172911463428934f,  -0.0241860356361024f,  0.00806094259064239f,
    0.00522782761055690f, -0.00704872099778532f, 0.00279261903117354f};
// bandpass (15-17)

static q15_t firCoeffs_q15[NUM_TAPS_ARRAY_SIZE];
static q15_t temp[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
// Matched filter and pattern symbol
static float32_t coeffs[FSPAN * SPB + 1];

static q15_t corrRes[2 * SAMPLES * 2 - 1];

// Peripherals
static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

// data storage
static volatile uint16_t dmaBuffer[SAMPLES * 2];
static q15_t buffer_input[2 * SAMPLES], buffer_filtered[2 * SAMPLES];
static float32_t buffer_filtered_f32[2 * SAMPLES];
static volatile uint8_t dataReady;

uint8_t data[1];
uint16_t idx[2 * SAMPLES / 968];
uint16_t foundFrames;

int main() {

  SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, coeffs, FSPAN * SPB + 1);

  BPSK_parameters params = {.Fb = FS / SPB,
                            .Fs = FS,
                            .Fc = FC,
                            .FSpan = FSPAN,
                            .firCoeffs = coeffs,
                            .firCoeffsLength = FSPAN * SPB + 1,
                            .prefixLength = 200,
                            .frameLength = 768,
                            .samplesPerBit = SPB};

  arm_float_to_q15(firCoeffs32, firCoeffs_q15, NUM_TAPS_ARRAY_SIZE);

  // Peripherals initialization
  uart2.baudRate = 115200;
  uart2.mode = UART_TRANSMITTER_ONLY;
  uart2.oversampling = UART_OVERSAMPLING_BY_16;
  uart2.parityControl = UART_PARITY_CONTROL_DISABLED;
  uart2.stopBits = UART_STOP_BITS_1;
  uart2.wordLength = UART_WORD_LENGTH_8;
  uart2.uart = USART2;

  uart_init(&uart2);

  adc.clockPrescaller = ADC_CLOCK_PRESCALLER_4;
  adc.continuous = ADC_CONTINUOUS_CONVERSION_MODE_DISABLED;
  adc.ext_mode = ADC_EXTERNAL_TRIG_MODE_RISING_EDGE;
  adc.ext_trig = ADC_EXTERNAL_TRIG_TIMER2_TRGO;
  adc.resolution = ADC_RESOLUTION_12;
  adc.dma = ADC_DMA_CONTINUOUS_REQUEST_ENABLED;
  adc.conversionNumber = 1;

  chan0.number = 1;
  chan0.GPIO_pin = PIN1;
  chan0.GPIO_port = GPIOA;

  tim2.tim = TIM2;
  tim2.direction = TIMER_COUNTER_DIRECTION_DOWN;
  tim2.prescaler = 1;
  tim2.autoReload = 1750;

  timer_init(&tim2);
  timer_selectTRGOUTEvent(&tim2, TIMER_TRGOUT_EVENT_UPDATE);
  timer_enableIT(&tim2, TIMER_IT_TRGOUT);

  adc_init(&adc);
  adc_configureChannel(&adc, &chan0, 1, ADC_SAMPLING_TIME_56CYCL);
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);

  adc_startDMA(&adc, (uint32_t *)dmaBuffer, (uint16_t)SAMPLES * 2,
               DMA_CIRCULAR_MODE);
  // dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
  dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

  timer_start(&tim2);
  char buf[40];
  while (1) {
    if (dataReady) {
      arm_conv_fast_q15(buffer_input, 2 * SAMPLES, firCoeffs_q15, 30, temp);
      arm_copy_q15(temp + NUM_TAPS_ARRAY_SIZE - 1, buffer_filtered,
                   2 * SAMPLES);

      arm_q15_to_float(buffer_filtered, buffer_filtered_f32, 2 * SAMPLES);
      float32_t maxVal;
      arm_max_f32(buffer_filtered_f32, 2 * SAMPLES, &maxVal, &idx);
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        buffer_filtered_f32[i] = buffer_filtered_f32[i] / maxVal;
      }

      BPSK_syncInputSignal(&params, buffer_filtered_f32, 2 * SAMPLES, &idx,
                           &foundFrames);

      for (uint8_t i = 0; i < 1; ++i) {
        BPSK_demodulateSignal(&params, buffer_filtered_f32 + idx[i], 768, data,
                              1);
        if (data[0] == 25) {
          sprintf(buf, "ok\r\n");
        } else {
          sprintf(buf, "nie ok, %d \r\n", data[0]);
        }
        uart_sendString(&uart2, buf);
      }

      dataReady = 0;
    }
  }
}
void DMA2_Stream4_IRQHandler() {
  char buf[20];
  if (dataReady) {
    sprintf(buf, "Overrun \r\n\n");
    uart_sendString(&uart2, buf);
    Default_Handler();
  }

  /*if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER)) {
          dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
          if(!dataReady) {
                  for(uint16_t i = 0; i < SAMPLES; i++)
                          buffer_input[i] = dmaBuffer[i];
                  dataReady = 0x1;
          }
  }*/

  if (dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
    dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
    if (!dataReady) {
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        buffer_input[i] = dmaBuffer[i];
      }
      dataReady = 0x1;
    }
  }
}

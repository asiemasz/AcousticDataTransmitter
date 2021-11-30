#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "MCP4822.h"
#include "BPSK.h"
#include "SRRC_filter.h"

#define FS 48000
#define FSystem 84000000
#define FC 17000
#define ROLLOVER_FACTOR 0.25
#define FSPAN 4
#define FB 1000
#define SPB (FS / FB)
#define N_BYTES 3
#define PREFIX_LENGTH 60


#define DATA_LENGTH (SPB*(N_BYTES)*8 + PREFIX_LENGTH)

extern uint32_t SystemCoreClock;

const SPI_pinout spi1_pins = {
	.af = GPIO_AF5,
	.CLK_Pin = PIN3,
	.CLK_Port = GPIOB,
	.MISO_Pin = PIN6,
	.MISO_Port = GPIOA,
	.MOSI_Pin = PIN7,
	.MOSI_Port = GPIOA,
};

const SPI_initStruct spi1 = {
	.clkPrescaller = SPI_MASTER_CLOCK_PRESCALLER_8,
	.clockMode = SPI_CLOCK_MODE_POLARITY_0_PHASE_0,
	.mode = SPI_MODE_MASTER,
	.slaveSelectMode = SPI_SOFTWARE_NSS_MANAGEMENT,
	.pinout = spi1_pins,
	.dataLength = SPI_DATA_FRAME_LENGTH_8_BITS,
	.spi = SPI1,
};

const TIMER_initStruct tim2 = { //sampling frequency
	.tim = TIM2,
	.direction = TIMER_COUNTER_DIRECTION_DOWN,
	.prescaler = 1,
	.autoReload = FSystem / FS,
};

void MCP4822_write(const uint8_t *data, const uint16_t len)
{
	spi_write(&spi1, data, len);
}

void MCP4822_CSen(const uint32_t *GPIO_port, const uint8_t GPIO_pin)
{
	gpio_resetPin(GPIO_port, GPIO_pin);
}

void MCP4822_CSdis(const uint32_t *GPIO_port, const uint8_t GPIO_pin)
{
	gpio_setPin(GPIO_port, GPIO_pin);
}
static UART_initStruct uart2;

MCP4822_device MCP4822 = {
	.SS_pin = PIN3,
	.SS_Port = GPIOC,
	.writeFunction = MCP4822_write,
	.CSenableFunction = MCP4822_CSen,
	.CSdisableFunction = MCP4822_CSdis};

MCP4822_OUTPUT_CONFIG cfg = {
	.gain = MCP4822_OUTPUT_GAIN_x2,
	.output = MCP4822_DAC_B,
	.powerDown = MCP4822_OUTPUT_POWERDOWN_CONTROL_BIT,
};

volatile uint8_t dataReady = 0;
volatile float32_t y;
volatile uint16_t val;

volatile uint32_t i = 0;

uint8_t data[N_BYTES] = {25, 161, 149};

float32_t txSignal[DATA_LENGTH]; 
float32_t coeffs[FSPAN*SPB + 1];

int main()
{
	SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, coeffs, FSPAN*SPB + 1);

	BPSK_parameters params = {
	.Fb = FB,
	.Fs = FS,
	.Fc = FC,
	.FSpan = FSPAN,
	.prefixLength = PREFIX_LENGTH,
	.firCoeffs = coeffs,
	.firCoeffsLength = FSPAN*SPB + 1
	};

	gpio_init(GPIOA);
	gpio_init(GPIOC);

	GPIO_pinConfigStruct pin3 = {
		.mode = GPIO_MODE_OUTPUT,
		.outSpeed = GPIO_OUT_SPEED_VERY_HIGH,
	};
	gpio_setPinConfiguration(GPIOC, PIN3, &pin3);

	uart2.baudRate = 115200;
	uart2.mode = UART_TRANSMITTER_ONLY;
	uart2.oversampling = UART_OVERSAMPLING_BY_16;
	uart2.parityControl = UART_PARITY_CONTROL_DISABLED;
	uart2.stopBits = UART_STOP_BITS_1;
	uart2.wordLength = UART_WORD_LENGTH_8;
	uart2.uart = USART2;
		
	uart_init(&uart2);
	spi_init(&spi1);
	timer_init(&tim2);
	timer_enableIT(&tim2, TIMER_IT_UPDATE_EVENT);

	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);

	BPSK_getOutputSignal(&params, data, N_BYTES, txSignal, DATA_LENGTH);

	timer_start(&tim2);

	while (1)
	{
		if (dataReady)
		{
			y = (txSignal[i-1] + 1.0f)/2.0f;
			val = y*4000.0f;
			MCP4822_setValue(&MCP4822, val, &cfg);
			if(i == DATA_LENGTH) {
				i = 0;
			}
			dataReady = 0;
		}
	}
}

void TIM2_IRQHandler()
{
	if(dataReady) {
		char buf[20];
		sprintf(buf, "Overrun");
		uart_sendString(&uart2, buf);
		Default_Handler();

	}
	timer_clearITflag(&tim2);
	++i;

	dataReady = 1;
}

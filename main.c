#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "MCP4822.h"

#define SAMPLES 20
#define SYMBOLS 2
#define FS 48000
#define FSystem 84000000

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

TIMER_initStruct tim3 = { // symbol change frequency
	.direction = TIMER_COUNTER_DIRECTION_DOWN,
	.prescaler = 1000,
	.autoReload = FSystem/1000,
	.tim = TIM3
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

uint16_t freq1 = 20000;
uint16_t freq2 = 19000;
volatile uint8_t dataReady = 0;
uint16_t val;
volatile float32_t y;

volatile uint64_t i = 0;
volatile uint16_t x;
float32_t fn1;
float32_t fn2;

int main()
{
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
	timer_init(&tim3);
	timer_enableIT(&tim2, TIMER_IT_UPDATE_EVENT);
	timer_enableIT(&tim3, TIMER_IT_UPDATE_EVENT);


	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM3_IRQn);

	timer_start(&tim3);
	fn1 = (float32_t)freq1/FS;
	fn2 = (float32_t)freq2/FS;
	timer_start(&tim2);

	while (1)
	{
		if (dataReady)
		{
			y = (((arm_sin_f32(fn1*i*2*PI) + 1)/2) + ((arm_sin_f32(fn2*i*2*PI) + 1)/2)) / 2;
			val = y*4000.0f;
			MCP4822_setValue(&MCP4822, val, &cfg);
			dataReady = 0;
		}
	}
}

void TIM2_IRQHandler()
{
	timer_clearITflag(&tim2);
	++i;

	dataReady = 1;
}

void TIM3_IRQHandler()
{
	timer_clearITflag(&tim3);
	//timer_setReloadVal(&tim2, reload[j]);
	i = 0;
}
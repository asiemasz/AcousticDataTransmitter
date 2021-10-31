#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "MCP4822.h"

SPI_initStruct spi1;
TIMER_initStruct tim2;

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

MCP4822_device MCP4822 = {
	.SS_pin = PIN3,
	.SS_Port = GPIOC,
	.writeFunction = MCP4822_write,
	.CSenableFunction = MCP4822_CSen,
	.CSdisableFunction = MCP4822_CSdis};

MCP4822_OUTPUT_CONFIG cfg = {
	.gain = MCP4822_OUTPUT_GAIN_x1,
	.output = MCP4822_DAC_B,
	.powerDown = 1,
};

const uint16_t sine[10] = {0x7fd,0xa75,0xcaf,0xe73,0xf96,0xffa,0xf96,0xe73,0xcaf,0xa75,0x7fd,0x585,0x34b,0x187,0x64,0x0,0x64,0x187,0x34b,0x585};
volatile uint8_t i = 0;

int main()
{
	gpio_init(GPIOA);
	gpio_init(GPIOC);

	GPIO_pinConfigStruct pin3 = {
		.mode = GPIO_MODE_OUTPUT,
		.outSpeed = GPIO_OUT_SPEED_VERY_HIGH,
	};
	gpio_setPinConfiguration(GPIOC, PIN3, &pin3);

	SPI_pinout spi1_pins = {
		.af = GPIO_AF5,
		.CLK_Pin = PIN3,
		.CLK_Port = GPIOB,
		.MISO_Pin = PIN6,
		.MISO_Port = GPIOA,
		.MOSI_Pin = PIN7,
		.MOSI_Port = GPIOA,
	};

	spi1.clkPrescaller = SPI_MASTER_CLOCK_PRESCALLER_8;
	spi1.clockMode = SPI_CLOCK_MODE_POLARITY_0_PHASE_0;
	spi1.mode = SPI_MODE_MASTER;
	spi1.slaveSelectMode = SPI_SOFTWARE_NSS_MANAGEMENT;
	spi1.pinout = spi1_pins;
	spi1.dataLength = SPI_DATA_FRAME_LENGTH_8_BITS;
	spi1.spi = SPI1;

	tim2.tim = TIM2;
	tim2.direction = TIMER_COUNTER_DIRECTION_DOWN;
	tim2.prescaler = 1;
	tim2.autoReload = 210;

	spi_init(&spi1);
	timer_init(&tim2);
	timer_enableIT(&tim2, TIMER_IT_UPDATE_EVENT);

	NVIC_EnableIRQ(TIM2_IRQn);

	timer_start(&tim2);
	//MCP4822_setVoltage(&MCP4822, 1.3f, &cfg);

	while (1)
	{
	}
}

void TIM2_IRQHandler()
{
	timer_clearITflag(&tim2);
	MCP4822_setValue(&MCP4822, sine[i++], &cfg);
	if(i == 20) {
		i = 0;
	}
}
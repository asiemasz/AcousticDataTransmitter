#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"
#include "MCP4822.h"

SPI_initStruct spi1;

void MCP4822_write(const uint8_t* data, const uint16_t len) {
	spi_write(&spi1, data, len);
}

void MCP4822_CSen(const uint32_t* GPIO_port,const uint8_t GPIO_pin) {
	gpio_resetPin(GPIO_port, GPIO_pin);
}

void MCP4822_CSdis(const uint32_t* GPIO_port,const uint8_t GPIO_pin) {
	gpio_setPin(GPIO_port, GPIO_pin);
}

int main() {
	gpio_init(GPIOA);
	gpio_init(GPIOC);

	GPIO_pinConfigStruct pin3 = {
		.mode = GPIO_MODE_OUTPUT,
		.outSpeed = GPIO_OUT_SPEED_VERY_HIGH,
	};
	gpio_setPinConfiguration(GPIOA, PIN3, &pin3);
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

	spi1.clkPrescaller = SPI_MASTER_CLOCK_PRESCALLER_4;
	spi1.clockMode = SPI_CLOCK_MODE_POLARITY_0_PHASE_0;
	spi1.mode = SPI_MODE_MASTER;
	spi1.slaveSelectMode = SPI_SOFTWARE_NSS_MANAGEMENT;
	spi1.pinout = spi1_pins;
	spi1.dataLength = SPI_DATA_FRAME_LENGTH_8_BITS;
	spi1.spi = SPI1;

	spi_init(&spi1);

	MCP4822_device MCP4822 = {
		.SS_pin = PIN3,
		.SS_Port = GPIOA,
		.writeFunction = MCP4822_write,
		.CSenableFunction = MCP4822_CSen,
		.CSdisableFunction = MCP4822_CSdis
	};

	MCP4822_OUTPUT_CONFIG cfg = {
		.gain = MCP4822_OUTPUT_GAIN_x2,
		.output = MCP4822_DAC_A,
		.powerDown = MCP4822_OUTPUT_POWERDOWN_CONTROL_BIT,
	};

	uint16_t val[4] = {0x0, 0xf, 0xff, 0xfff};
	uint8_t i = 0;

	while(1) {
		if(i > 4)
			i = 0;
		MCP4822_setValue(&MCP4822, val[i++], &cfg);
		gpio_togglePin(GPIOC, PIN3);
	}		
}
 
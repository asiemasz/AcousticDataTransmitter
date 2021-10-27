#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "spi.h"

int main() {
	SPI_initStruct spi1;
	spi1.clkPrescaller = SPI_MASTER_CLOCK_PRESCALLER_64;
	spi1.clockMode = SPI_CLOCK_MODE_POLARITY_0_PHASE_0;
	spi1.mode = SPI_MODE_MASTER;
	spi1.slaveSelectMode = SPI_SOFTWARE_NSS_MANAGEMENT;
	
	spi_init(&spi1);

	int x = 0;
	while(1) {
		x = 1;
	}		
}

#include "../inc/gpio.h"
#include "../inc/i2c.h"
#include "../inc/timer.h"
#include "../inc/adc.h"
#include "../inc/uart.h"
#include "../inc/mcp4725.h"

TIMER_initStruct tim2;
MCP4725 dac;
UART_initStruct uart2;

uint8_t set;

int main() {
	i2c_init(I2C2);

	gpio_init(GPIOA);
	gpio_set_pin_mode(GPIOA, PIN10, GPIO_MODE_OUTPUT);
	gpio_set_pin_output_speed(GPIOA, PIN10, GPIO_OUT_SPEED_VERY_HIGH);


	dac = MCP4725_init(I2C2, MCP4725_ADDR_0, 3.3f, MCP4725_POWER_DOWN_OFF);

	tim2.tim = TIM2;
	tim2.direction = TIMER_COUNTER_DIRECTION_DOWN;
	tim2.prescaler = 1;
	tim2.autoReload = 50;
	timer_init(&tim2);	

	NVIC_EnableIRQ(TIM2_IRQn);

	timer_start(&tim2);
	tim2.tim->DIER |= TIM_DIER_UIE;


	while(1) {
	}

}

void TIM2_IRQHandler(void) {
	timer_clearITflag(&tim2); // wazne 
}
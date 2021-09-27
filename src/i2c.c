#include "../inc/i2c.h"



void i2c_init(struct i2c_init_struct i2c_init) {
	//enable selected i2c peripheral clock
	RCC->APB1ENR |= (1 << ((uint32_t)i2c_init.i2c - APB1PERIPH_BASE)/0x400UL);

	//configure appropriate GPIO pins to be used as SCL, SDA outputs
	struct i2c_pins i2c_pinout;
	if(i2c_init.i2c == I2C1) {
		i2c_pinout = i2c1_pins;
	}
	else if(i2c_init.i2c == I2C2) {
		i2c_pinout = i2c2_pins;
	}
	else {
		i2c_pinout = i2c3_pins;
	}
	//enable peripheral clock for ports of SDA and SCL pins
	gpio_init(i2c_pinout.i2c_scl_port);
	if(i2c_pinout.i2c_scl_port != i2c_pinout.i2c_sda_port)
		gpio_init(i2c_pinout.i2c_sda_port);
	//set SDA and SCL pins as alternate function pins 
	gpio_set_pin_mode(i2c_pinout.i2c_scl_port, i2c_pinout.i2c_scl_pin, GPIO_MODE_ALTERNATE_FUNCTION);
	gpio_set_pin_mode(i2c_pinout.i2c_sda_port, i2c_pinout.i2c_sda_pin, GPIO_MODE_ALTERNATE_FUNCTION);
	//set gpio pins output type to open drain
	gpio_set_pin_output_type(i2c_pinout.i2c_scl_port, i2c_pinout.i2c_scl_pin, GPIO_OUT_TYPE_OPEN_DRAIN);
	gpio_set_pin_output_type(i2c_pinout.i2c_sda_port, i2c_pinout.i2c_sda_pin, GPIO_OUT_TYPE_OPEN_DRAIN);
	//set gpio pins output speed to highest possible
	gpio_set_pin_output_speed(i2c_pinout.i2c_scl_port, i2c_pinout.i2c_scl_pin, GPIO_OUT_SPEED_VERY_HIGH);
	gpio_set_pin_output_speed(i2c_pinout.i2c_sda_port, i2c_pinout.i2c_sda_pin, GPIO_OUT_SPEED_VERY_HIGH);
	//set pull up for both pins
	gpio_set_pin_pull(i2c_pinout.i2c_scl_port, i2c_pinout.i2c_scl_pin, GPIO_PULL_UP);
	gpio_set_pin_pull(i2c_pinout.i2c_sda_port, i2c_pinout.i2c_sda_pin, GPIO_PULL_UP);
	//select appropriate alternate function for both pins
	gpio_set_alternate_function(i2c_pinout.i2c_scl_port, i2c_pinout.i2c_scl_pin, i2c_pinout.i2c_scl_af);
	gpio_set_alternate_function(i2c_pinout.i2c_sda_port, i2c_pinout.i2c_sda_pin, i2c_pinout.i2c_sda_af);
	
	//reset i2c 
	i2c_init.i2c->CR1 |= I2C_CR1_SWRST_Msk;
	i2c_init.i2c->CR1 &= ~I2C_CR1_SWRST_Msk;

	//set i2c clock
	
}

#ifndef I2C_H
#define I2C_H
#include <stm32f401xe.h>
#include "gpio.h"
struct i2c_pins {
	GPIO_TypeDef* i2c_sda_port;
	GPIO_TypeDef* i2c_scl_port;
	uint8_t i2c_sda_pin;
	uint8_t i2c_scl_pin;
	enum GPIO_ALTERNATE_FUNCTION i2c_sda_af;
	enum GPIO_ALTERNATE_FUNCTION i2c_scl_af;
};

static struct i2c_pins i2c1_pins = {GPIOB, GPIOB, PIN7, PIN6, GPIO_AF4, GPIO_AF4};
static struct i2c_pins i2c2_pins = {GPIOB, GPIOB, PIN3, PIN10, GPIO_AF9, GPIO_AF4};
static struct i2c_pins i2c3_pins = {GPIOB, GPIOA, PIN4, PIN8, GPIO_AF9, GPIO_AF4};

struct i2c_init_struct {
	I2C_TypeDef* i2c;
	uint32_t clock_freq;
};

void i2c_init(struct i2c_init_struct);

#endif

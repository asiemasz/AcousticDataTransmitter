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

static struct i2c_pins i2c1_pins = {GPIOB, GPIOB, PIN9, PIN8, GPIO_AF4, GPIO_AF4};
static struct i2c_pins i2c2_pins = {GPIOB, GPIOB, PIN3, PIN10, GPIO_AF9, GPIO_AF4};
static struct i2c_pins i2c3_pins = {GPIOB, GPIOA, PIN4, PIN8, GPIO_AF9, GPIO_AF4};

typedef struct i2c_device {
	I2C_TypeDef* i2c;
	struct i2c_pins pins;
	uint8_t device_addr;
} i2c_device; 

void i2c_init(I2C_TypeDef * i2c);

void i2c_write(i2c_device* dev, uint8_t reg, uint8_t data);

void i2c_read(i2c_device* dev, uint8_t reg, void* data, uint8_t size);

void i2c_start(I2C_TypeDef * i2c);

void i2c_stop(I2C_TypeDef * i2c);

void i2c_byte_write(I2C_TypeDef * i2c, uint8_t data);

void i2c_address_select(I2C_TypeDef *i2c, uint8_t address);
#endif

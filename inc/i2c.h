#ifndef I2C_H
#define I2C_H
#include <stm32f401xe.h>

struct i2c_init_struct {
	I2C_TypeDef* i2c;
};

void i2c_init(struct i2c_init_struct);

#endif

#include "../inc/i2c.h"

void i2c_init(struct i2c_init_struct i2c_init) {
	RCC->APB1ENR |= (1 << ((uint32_t)i2c_init.i2c - APB1PERIPH_BASE)/0x400UL);
}

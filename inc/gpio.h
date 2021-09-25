#ifndef GPIO_H
#define GPIO_H
#include <stm32f401xe.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdio.h>

enum GPIO  {
    GPIO_A,
    GPIO_B,
    GPIO_C,
    GPIO_D,
    GPIO_E,
		res1,
	  res2,
    GPIO_H
};

enum GPIO_PIN {
    PIN0,
    PIN1,
    PIN2,
    PIN3,
    PIN4,
    PIN5,
    PIN6,
    PIN7,
    PIN8,
    PIN9,
    PIN10,
    PIN11,
    PIN12,
    PIN13,
    PIN15,
};

enum GPIO_MODE {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTERNATE_FUNCTION,
    GPIO_MODE_ANALOG
};

enum GPIO_OUTPUT_TYPE {
    GPIO_OUT_TYPE_PUSH_PULL,
    GPIO_OUT_TYPE_OPEN_DRAIN
};

enum GPIO_OUTPUT_SPEED {
    GPIO_OUT_SPEED_LOW,
    GPIO_OUT_SPEED_MEDIUM,
    GPIO_OUT_SPEED_HIGH,
    GPIO_OUT_SPEED_VERY_HIGH
};

enum GPIO_PULL {
    GPIO_NO_PULL,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
};

void gpio_init(enum GPIO gpio);

void gpio_set_pin_mode(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_MODE mode);

void gpio_set_pin_output_type(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_TYPE type);

void gpio_set_pin_output_speed(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_SPEED speed);

void gpio_set_pin_pull(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_PULL pull);

void gpio_get_input_pin_data(enum GPIO gpio, enum GPIO_PIN pin, uint8_t *data);

void gpio_set_pin(enum GPIO gpio, enum GPIO_PIN pin);

void gpio_reset_pin(enum GPIO gpio, enum GPIO_PIN pin);

#endif

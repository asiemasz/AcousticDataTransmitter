#ifndef GPIO_H
#define GPIO_H
#include <stm32f401xe.h>
#include <stm32f4xx.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

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

void gpio_init(GPIO_TypeDef* gpio);

void gpio_set_pin_mode(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_MODE mode);

void gpio_set_pin_output_type(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_TYPE type);

void gpio_set_pin_output_speed(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_SPEED speed);

void gpio_set_pin_pull(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_PULL pull);

void gpio_get_input_pin_data(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t *data);

void gpio_set_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin);

void gpio_reset_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin);

void gpio_toggle_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin);

void gpio_read_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t* data);

#endif

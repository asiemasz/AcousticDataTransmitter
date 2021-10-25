#ifndef GPIO_H
#define GPIO_H
#include "stm32f401xe.h"
#include "stm32f4xx.h"
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

enum GPIO_ALTERNATE_FUNCTION {
    GPIO_AF0,
    GPIO_AF1,
    GPIO_AF2,
    GPIO_AF3,
    GPIO_AF4,
    GPIO_AF5,
    GPIO_AF6,
    GPIO_AF7,
    GPIO_AF8,
    GPIO_AF9,
    GPIO_AF10,
    GPIO_AF11,
    GPIO_AF12,
    GPIO_AF13,
    GPIO_AF14,
    GPIO_AF15
};

typedef struct GPIO_pinConfigStruct {
    enum GPIO_MODE mode;
    enum GPIO_PULL pullConfiguration;
    enum GPIO_OUTPUT_TYPE outType;
    enum GPIO_OUTPUT_SPEED outSpeed;
    enum GPIO_ALTERNATE_FUNCTION alternateFunction;
} GPIO_pinConfigStruct;

void gpio_setPinConfiguration(GPIO_TypeDef* gpio, enum GPIO_PIN pin ,GPIO_pinConfigStruct* cfg);

void gpio_init(GPIO_TypeDef* gpio);

void gpio_getPinInput(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t *data);

void gpio_setPin(GPIO_TypeDef* gpio, enum GPIO_PIN pin);

void gpio_resetPin(GPIO_TypeDef* gpio, enum GPIO_PIN pin);

void gpio_togglePin(GPIO_TypeDef* gpio, enum GPIO_PIN pin);

void gpio_readPin(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t* data);

#endif

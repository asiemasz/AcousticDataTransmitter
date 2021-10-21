#include "../inc/gpio.h"


void gpio_init(GPIO_TypeDef* gpio) {
    SET_BIT(RCC->AHB1ENR, 1 << (((uint32_t)gpio - AHB1PERIPH_BASE)/0x400UL) );
}

static inline void gpio_set_pin_mode(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_MODE mode) {
    gpio->MODER |= (uint32_t)(mode << pin * 2);
}

static inline void gpio_set_pin_output_type(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_TYPE type) {
		gpio->OTYPER |= (uint32_t)(type << pin);
}

static inline void gpio_set_pin_output_speed(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_SPEED speed) {
		gpio->OSPEEDR |= (uint32_t)(speed << pin * 2);
}

static inline void gpio_set_pin_pull(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_PULL pull) {
		gpio->PUPDR |= (uint32_t)(pull << pin * 2);
}

static inline void gpio_set_alternate_function(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_ALTERNATE_FUNCTION function) {
		gpio->AFR[pin/8] |= ((uint32_t) function << ((uint8_t) pin % 8 * 4));
}

void gpio_setPinConfiguration(GPIO_TypeDef* gpio, enum GPIO_PIN pin ,GPIO_pinConfigStruct* cfg) {
	gpio_set_pin_mode(gpio, pin, cfg->mode);
	gpio_set_pin_pull(gpio, pin, cfg->pullConfiguration);

	if(cfg->mode == GPIO_MODE_OUTPUT || cfg->mode == GPIO_MODE_ALTERNATE_FUNCTION) {
		gpio_set_pin_output_type(gpio, pin, cfg->outType);
		gpio_set_pin_output_speed(gpio, pin, cfg->outSpeed);
	}
	if(cfg->mode == GPIO_MODE_ALTERNATE_FUNCTION) {
		gpio_set_alternate_function(gpio, pin, cfg->alternateFunction);
	}
}

void gpio_getPinInput(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t *data) {
		*data = (uint8_t) READ_BIT(gpio->IDR,(uint32_t) 1 << pin);
}

void gpio_setPin(GPIO_TypeDef* gpio, enum GPIO_PIN pin){
		gpio->BSRR |= (1 << pin);
}

void gpio_resetPin(GPIO_TypeDef* gpio, enum GPIO_PIN pin){
		gpio->BSRR |= (1 << (pin + 0x10));
}

void gpio_togglePin(GPIO_TypeDef* gpio, enum GPIO_PIN pin){
		gpio->ODR ^= (1 << pin);
}

void gpio_readPin(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t* data) {
		*data =  (gpio->IDR >> pin & 1);
}



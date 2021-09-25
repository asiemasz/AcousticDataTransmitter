#include "../inc/gpio.h"


void gpio_init(GPIO_TypeDef* gpio) {
    SET_BIT(RCC->AHB1ENR, 1 << (((uint32_t)gpio - AHB1PERIPH_BASE)/0x400UL) );
}

void gpio_set_pin_mode(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_MODE mode) {
    gpio->MODER |= (uint32_t)(mode << pin * 2);
}

void gpio_set_pin_output_type(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_TYPE type) {
		gpio->OTYPER |= (uint32_t)(type << pin);
}

void gpio_set_pin_output_speed(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_SPEED speed) {
		gpio->OSPEEDR |= (uint32_t)(speed << pin * 2);
}

void gpio_set_pin_pull(GPIO_TypeDef* gpio, enum GPIO_PIN pin, enum GPIO_PULL pull) {
		gpio->PUPDR |= (uint32_t)(pull << pin * 2);
}

void gpio_get_input_pin_data(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t *data) {
		*data = (uint8_t) READ_BIT(gpio->IDR,(uint32_t) 1 << pin);
}

void gpio_set_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin){
		gpio->ODR |= (1 << pin);
}

void gpio_reset_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin){
		gpio->ODR &= ~(1 << pin);
}

void gpio_toggle_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin){
		gpio->ODR ^= (1 << pin);
}

void gpio_read_pin(GPIO_TypeDef* gpio, enum GPIO_PIN pin, uint8_t* data) {
		*data =  (gpio->IDR >> pin & 1);
}

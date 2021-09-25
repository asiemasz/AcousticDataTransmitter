#include "../inc/gpio.h"

static GPIO_TypeDef* GPIO_Regs[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, 0, 0, GPIOH
};

void gpio_init(enum GPIO gpio) {
    SET_BIT(RCC->AHB1ENR, 1 << gpio);
}

void gpio_set_pin_mode(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_MODE mode) {
    GPIO_Regs[gpio]->MODER |= (uint32_t)(mode << pin * 2);
}

void gpio_set_pin_output_type(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_TYPE type) {
		GPIO_Regs[gpio]->OTYPER |= (uint32_t)(type << pin);
}

void gpio_set_pin_output_speed(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_OUTPUT_SPEED speed) {
		GPIO_Regs[gpio]->OSPEEDR |= (uint32_t)(speed << pin * 2);
}

void gpio_set_pin_pull(enum GPIO gpio, enum GPIO_PIN pin, enum GPIO_PULL pull) {
		GPIO_Regs[gpio]->PUPDR |= (uint32_t)(pull << pin * 2);
}

void gpio_get_input_pin_data(enum GPIO gpio, enum GPIO_PIN pin, uint8_t *data) {
		*data = (uint8_t) READ_BIT(GPIO_Regs[gpio]->IDR,(uint32_t) 1 << pin);
}

void gpio_set_pin(enum GPIO gpio, enum GPIO_PIN pin){
		GPIO_Regs[gpio]->ODR |= (1 << pin);
}

void gpio_reset_pin(enum GPIO gpio, enum GPIO_PIN pin){
		GPIO_Regs[gpio]->ODR &= ~(1 << pin);
}

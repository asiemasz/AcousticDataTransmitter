#include "../inc/uart.h"

void uart_init(UART_initStruct *init)
{
    struct uart_pins pins;
    //enable appropriate uart clock and select pins to configure
    if (init->uart == USART1)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        pins = uart1_pins;
    }
    else if (init->uart == USART2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        pins = uart2_pins;

    }
    else if (init->uart == USART6)
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        pins = uart6_pins;
    }
    else
    {
        assert(0);
    }

    //configure pins
    gpio_init(pins.uart_tx_port);

    GPIO_pinConfigStruct tx_pin, rx_pin;

    tx_pin.mode = GPIO_MODE_ALTERNATE_FUNCTION;
    rx_pin.mode = GPIO_MODE_ALTERNATE_FUNCTION;

    tx_pin.outSpeed = GPIO_OUT_SPEED_HIGH;
    rx_pin.outSpeed = GPIO_OUT_SPEED_HIGH;
    
    tx_pin.alternateFunction = pins.uart_tx_af;
    rx_pin.alternateFunction = pins.uart_rx_af;

    gpio_setPinConfiguration(pins.uart_tx_port, pins.uart_tx_pin, &tx_pin);
    gpio_setPinConfiguration(pins.uart_rx_port, pins.uart_rx_pin, &rx_pin);
    //clear USART CR1 register
    init->uart->CR1 = 0x00;
    //enable usart
    init->uart->CR1 |= USART_CR1_UE;
    //select word length
    init->uart->CR1 |= (uint32_t) init->wordLength << USART_CR1_M_Pos;
    //select parity control
    init->uart->CR1 |= (uint32_t) init->parityControl << USART_CR1_PCE_Pos;
    if(init->parityControl == UART_PARITY_CONTROL_ENABLED)
        init->uart->CR1 |= (uint32_t) init->paritySelection << USART_CR1_PS_Pos;
    //select oversampling mode
    init->uart->CR1 |= (uint32_t) init->oversampling << USART_CR1_OVER8_Pos;
    //set transmitter/receiver mode
    init->uart->CR1 |= (uint32_t) init->mode << USART_CR1_RE_Pos;
    volatile uint32_t pclk;
    //set the baudrate
    if(init->uart == USART2) {
        pclk = 42000000;
    }
    else {
        pclk = 84000000;
    }
		//calculate appropriate mantisa and fraction of baudRate according to selected oversampling mode (calculated using method from HAL USART library)
    uint32_t div = (uint32_t)(pclk*25U/(2U*(2U-(uint8_t)init->oversampling)*init->baudRate));
    uint32_t mant = div/100U;
    uint32_t fraq = ((((div - (mant * 100U)) * 8U*(2U-(uint8_t)init->oversampling)) + 50U) / 100U);
		//set baudrate to its register
    if(init->oversampling)
        init->uart->BRR = ((mant << 4U) + ((fraq & 0xF8U) << 1U) + (fraq & 0x07U));
    else
        init->uart->BRR = ((mant << 4U) + (fraq & 0xF0U) + (fraq & 0x0FU));
}

void uart_sendChar(UART_initStruct* init, char c) {
    //load data to data register
    init->uart->DR = c;
    //wait for TC to be set (Transmission Complete)
    while(!(init->uart->SR & USART_SR_TC));
}

void uart_sendString(UART_initStruct* init, char *s) {
    while(*s) uart_sendChar(init, *s++);
}

uint8_t uart_getChar(UART_initStruct* init) {
    uint8_t res;
    while(!(init->uart->SR & USART_SR_RXNE));
    res = (uint8_t) init->uart->DR;
    return res;
}

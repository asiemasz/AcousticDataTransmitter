#ifndef UART_H
#define UART_H

#include "stm32f401xe.h"
#include "gpio.h"
#include "assert.h"

struct uart_pins {
	GPIO_TypeDef* uart_tx_port;
	GPIO_TypeDef* uart_rx_port;
	uint8_t uart_tx_pin;
	uint8_t uart_rx_pin;
	enum GPIO_ALTERNATE_FUNCTION uart_tx_af;
	enum GPIO_ALTERNATE_FUNCTION uart_rx_af;
};

static struct uart_pins uart1_pins = {GPIOA, GPIOA, PIN9, PIN10, GPIO_AF7, GPIO_AF7};
static struct uart_pins uart2_pins = {GPIOA, GPIOA, PIN2, PIN3, GPIO_AF7, GPIO_AF7};
static struct uart_pins uart6_pins = {GPIOA, GPIOA, PIN11, PIN12, GPIO_AF8, GPIO_AF8};

enum UART_OVERSAMPLING_MODE {
    UART_OVERSAMPLING_BY_16,
    UART_OVERSAMPLING_BY_8
};

enum UART_WORD_LENGTH {
    UART_WORD_LENGTH_8,
    UART_WORD_LENGTH_9
};

enum UART_PARITY_CONTROL {
    UART_PARITY_CONTROL_DISABLED,
    UART_PARITY_CONTROL_ENABLED
};

enum UART_PARITY_SELECTION {
    UART_PARITY_SELECTION_EVEN,
    UART_PARITY_SELECTION_ODD
};

enum UART_STOP_BITS {
    UART_STOP_BITS_1,
    UART_STOP_BITS_0_5,
    UART_STOP_BITS_2,
    UART_STOP_BITS_1_5
};

enum UART_MODE {
    UART_RECEIVER_ONLY = 1,
    UART_TRANSMITTER_ONLY,
    UART_TRANSMITTER_AND_RECEIVER,
};

typedef struct UART_initStruct {
    USART_TypeDef* uart;
    enum UART_MODE mode;
    uint32_t baudRate;
    enum UART_WORD_LENGTH wordLength;
    enum UART_STOP_BITS stopBits;
    enum UART_PARITY_CONTROL parityControl;
    enum UART_PARITY_SELECTION paritySelection;
    enum UART_OVERSAMPLING_MODE oversampling;
} UART_initStruct;

void uart_init(UART_initStruct* init);

void uart_sendChar(UART_initStruct* init, char c);

void uart_sendString(UART_initStruct* init, char *s);

uint8_t uart_getChar(UART_initStruct* init);

#endif

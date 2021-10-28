#include "spi.h"
#include <assert.h>

static void spi_gpioInit(SPI_initStruct* init) {
    gpio_init(init->pinout.CLK_Port);

    GPIO_pinConfigStruct spi_pin = {
        .mode = GPIO_MODE_ALTERNATE_FUNCTION,
        .outType = GPIO_OUT_TYPE_PUSH_PULL,
        .outSpeed = GPIO_OUT_SPEED_VERY_HIGH,
        .alternateFunction = init->pinout.af,
        .pullConfiguration = GPIO_NO_PULL
    };   

    gpio_setPinConfiguration(init->pinout.CLK_Port, init->pinout.CLK_Pin, &spi_pin);

    if(!init->halfDuplex) {
        gpio_init(init->pinout.MISO_Port); gpio_init(init->pinout.MOSI_Port);
        gpio_setPinConfiguration(init->pinout.MISO_Port, init->pinout.MISO_Pin, &spi_pin);
        gpio_setPinConfiguration(init->pinout.MOSI_Port, init->pinout.MOSI_Pin, &spi_pin);
    }
    else {
        if(init->pinout.MISO_Port) {
            gpio_init(init->pinout.MISO_Port); 
            gpio_setPinConfiguration(init->pinout.MISO_Port, init->pinout.MISO_Pin, &spi_pin);
        } else if(init->pinout.MOSI_Port) {
            gpio_init(init->pinout.MOSI_Port);
            gpio_setPinConfiguration(init->pinout.MOSI_Port, init->pinout.MOSI_Pin, &spi_pin);            
        }
        else
            assert(0);
    }
}

void spi_init(SPI_initStruct* init) {
    //enable SPI clock
    if(init->spi == SPI1) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    } else if(init->spi == SPI2) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    } else if(init->spi == SPI3) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    } else {
        assert(0);
    }

    //select SPI clock mode
    init->spi->CR1 |= init->clockMode << SPI_CR1_CPHA_Pos;

    //select SPI mode
    init->spi->CR1 |= init->mode << SPI_CR1_MSTR_Pos;

    //select baudrate
    init->spi->CR1 |= (init->clkPrescaller << SPI_CR1_BR_Pos) & SPI_CR1_BR_Msk;

    //select data bits order
    init->spi->CR1 |= init->dataOrder << SPI_CR1_LSBFIRST_Pos;

    //select data frame length
    init->spi->CR1 |= init->dataLength << SPI_CR1_DFF_Pos;

    //select Slave Management mode
    init->spi->CR1 |= init->slaveSelectMode << SPI_CR1_SSM_Pos;
    
    if(init->slaveSelectMode) {
        init->spi->CR1 |= SPI_CR1_SSI;
    }

    if(init->halfDuplex) {
        init->spi->CR1 |= SPI_CR1_BIDIMODE; //enable one wire mode
        init->spi->CR1 |= init->mode << SPI_CR1_BIDIOE_Pos;//select receive or transmit only mode in one wire mode
    }

    spi_gpioInit(init);

    //enable spi
    init->spi->CR1 |= SPI_CR1_SPE;
}

void spi_write(SPI_initStruct *init, uint8_t *data, uint16_t size) {
    while(SPI_BUSY(init->spi)); //wait for TXE bit to be set (buffer empty)

    uint16_t i;
    for (i = 0; i < size; i++) {
        init->spi->DR = data[i];
        while(SPI_BUSY(init->spi)); //wait for TXE bit to be set (buffer empty)
        (void) init->spi->DR;
    }
}

void spi_read(SPI_initStruct *init, uint8_t* data, uint16_t size) {
    uint16_t i;
        while(SPI_BUSY(init->spi)); //wait for TXE bit to be set (buffer empty)

    for(i = 0; i < size; i++) {
        init->spi->DR = 0xFF; //Dummy fill
        while(SPI_BUSY(init->spi)); //wait for TXE bit to be set (buffer empty)
        data[i] = init->spi->DR;
    }
}

void spi_transmitReceive(SPI_initStruct* init, uint8_t* dataOut, uint8_t* dataIn, uint16_t size) {
    uint16_t i;
    while(SPI_BUSY(init->spi)); //wait for TXE bit to be set (buffer empty)

    for(i = 0; i < size; i++) {
        init->spi->DR = dataOut[i];
        while(SPI_BUSY(init->spi)); //wait for TXE bit to be set (buffer empty)
        dataIn[i] = init->spi->DR;
    }
}
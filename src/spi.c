#include "spi.h"
#include <assert.h>

void spi_init(SPI_initStruct* init) {
    //enable SPI clock
    if(init->spi == SPI1) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    } else if(init->spi == SPI2) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    } else if(init->spi == SPI3) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    } else if(init->spi == SPI4) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
    }
    else {
        assert(0);
    }

    //select SPI clock mode
    init->spi->CR1 |= init->clockMode & (SPI_CR1_CPHA_Msk | SPI_CR1_CPOL_Msk);

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

    //enable spi
    init->spi->CR1 |= SPI_CR1_SPE;

}

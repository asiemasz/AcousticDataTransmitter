#ifndef SPI_H
#define SPI_H

#include "stm32f401xe.h"

enum SPI_DATA_FRAME_LENGTH {
    SPI_DATA_FRAME_FORMAT_8_BITS,
    SPI_DATA_FRAME_FORMAT_16_BITS
};

enum SPI_DATA_FRAME_ORDER {
    SPI_DATA_FRAME_FORMAT_MSB_FIRST,
    SPI_DATA_FRAME_FORMAT_LSB_FIRST
};

enum SPI_FRAME_FORMAT {
    SPI_FRAME_FORMAT_MOTOROLA_MODE,
    SPI_FRAME_FORMAT_TI_MODE
};

enum SPI_MODE {
    SPI_MODE_MASTER,
    SPI_MODE_SLAVE
};

enum SPI_HALF_DUPLEX {
    SPI_HALF_DUPLEX_DISABLE,
    SPI_HALF_DUPLEX_ENABLE
};

enum SPI_MASTER_CLOCK_PRESCALLER {
    SPI_MASTER_CLOCK_PRESCALLER_2,
    SPI_MASTER_CLOCK_PRESCALLER_4,
    SPI_MASTER_CLOCK_PRESCALLER_8,
    SPI_MASTER_CLOCK_PRESCALLER_16,
    SPI_MASTER_CLOCK_PRESCALLER_32,
    SPI_MASTER_CLOCK_PRESCALLER_64,
    SPI_MASTER_CLOCK_PRESCALLER_128,
    SPI_MASTER_CLOCK_PRESCALLER_256
};

/*  This should be set according to the slave requirement!
    Clock polarity - 0: CK to 0 when idle, 1: CK to 1 when idle
   Clock phase - 0: The first clock transition is the first data capture edge, 1: The second clock transition is the first data capture edge
 */
enum SPI_CLOCK_MODE {
    SPI_CLOCK_MODE_POLARITY_0_PHASE_0, 
    SPI_CLOCK_MODE_POLARITY_0_PHASE_1,
    SPI_CLOCK_MODE_POLARITY_1_PHASE_0,
    SPI_CLOCK_MODE_POLARITY_1_PHASE_1
};

enum SPI_SLAVE_SELECT_MODE {
    SPI_SOFTWARE_NSS_MANAGEMENT,
    SPI_HARDWARE_NSS_MANAGEMENT
};

typedef struct SPI_pinout{
    GPIO_TypeDef*   MOSI_Port;
    enum GPIO_PIN   MOSI_Pin;
    GPIO_TypeDef*   MISO_Port;
    enum GPIO_PIN   MISO_Pin;
    GPIO_TypeDef*   SS_Port; //Slave Select 
    enum GPIO_PIN   SS_Pin; 
} SPI_pinout; //For full duplex (for half duplex either mosi or miso pin can be selected - the one that's not empty by default)

typedef struct SPI_initStruct {
    SPI_TypeDef*                     spi;
    enum SPI_MODE                    mode;
    enum SPI_DATA_FRAME_LENGTH       dataLength;
    enum SPI_DATA_FRAME_ORDER        dataOrder;
    enum SPI_FRAME_FORMAT            dataFrameFormat;
    enum SPI_HALF_DUPLEX             halfDuplex;
    enum SPI_MASTER_CLOCK_PRESCALLER clkPrescaller;
    enum SPI_CLOCK_MODE              clockMode;
    enum SPI_SLAVE_SELECT_MODE       slaveSelectMode;
    SPI_pinout                       pinout;
} SPI_initStruct;

#endif
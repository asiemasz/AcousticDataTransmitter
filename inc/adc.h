#ifndef ADC_H
#define ADC_H

#include <stm32f401xe.h>

enum ADC_RESOLUTION {
    ADC_RESOLUTION_12,
    ADC_RESOLUTION_10,
    ADC_RESOLUTION_8,
    ADC_RESOLUTION_6
};

enum ADC_CLOCK_PRESCALLER {
    ADC_CLOCK_PRESCALLER_2,
    ADC_CLOCK_PRESCALLER_4,
    ADC_CLOCK_PRESCALLER_6,
    ADC_CLOCK_PRESCALLER_8
};

enum ADC_DATA_ALIGNMENT {
    ADC_DATA_ALIGNMENT_right,
    ADC_DATA_ALIGNMENT_left
};
/*
enum ADC_SCAN_CONVERSION_MODE {
    
};

enum ADC_CONTINUOUS_CONVERSION_MODE {
    
};

enum ADC_DISCONTINUOUS_CONVERSION_MODE {

};

enum ADC_DMA_CONTINUOUS_REQUESTS {

};

enum ADC_sampleTime {

};
*/


typedef struct ADC_initStruct {
    enum ADC_CLOCK_PRESCALLER clockPrescaller;
    enum ADC_RESOLUTION resolution;
    enum ADC_DATA_ALIGNMENT dataAlignment;
} ADC_initStruct;

void adc_init(ADC_initStruct adc);

#endif
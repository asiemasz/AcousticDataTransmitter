#ifndef ADC_H
#define ADC_H

#include <stm32f401xe.h>
#include <assert.h>
#include "gpio.h"

#define ADC_REGULAR_SEQUENCE_MAX_LENGTH 16
#define ADC_INJECTED_SEQUENCE_MAX_LENGTH 16

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
    ADC_DATA_ALIGNMENT_RIGHT,
    ADC_DATA_ALIGNMENT_LEFT
};

enum ADC_SAMPLING_TIME {
    ADC_SAMPLING_TIME_3CYCL,
    ADC_SAMPLING_TIME_15CYCL,
    ADC_SAMPLING_TIME_28CYCL,
    ADC_SAMPLING_TIME_56CYCL,
    ADC_SAMPLING_TIME_84CYCL,
    ADC_SAMPLING_TIME_112CYCL,
    ADC_SAMPLING_TIME_144CYCL,
    ADC_SAMPLING_TIME_480CYCL
};

#define ADC_REAL_SAMPLING_TIME(samplingTime, resolution) (samplingTime + (12 - 2 * resolution))


enum ADC_SCAN_CONVERSION_MODE {
    ADC_SCAN_CONVERSION_MODE_DISABLED,
    ADC_SCAN_CONVERSION_MODE_ENABLED
};

enum ADC_CONTINUOUS_CONVERSION_MODE {
    ADC_CONTINUOUS_CONVERSION_MODE_DISABLED,
    ADC_CONTINUOUS_CONVERSION_MODE_ENABLED
};

enum ADC_DISCONTINUOUS_CONVERSION_MODE {
    ADC_DISCONTINUOUS_CONVERSION_MODE_DISABLED,
    ADC_DISCONTINUOUS_CONVERSION_MODE_ENABLED
};

enum ADC_EXTERNAL_TRIG {
    ADC_EXTERNAL_TRIG_DISABLED
};
/*
enum ADC_DMA_CONTINUOUS_REQUESTS {

};


*/


typedef struct ADC_initStruct {
    enum ADC_CLOCK_PRESCALLER clockPrescaller;
    enum ADC_RESOLUTION resolution;
    enum ADC_DATA_ALIGNMENT dataAlignment;
    enum ADC_CONTINUOUS_CONVERSION_MODE continuous;
    enum ADC_DISCONTINUOUS_CONVERSION_MODE discontinuous;
    enum ADC_SCAN_CONVERSION_MODE scan;
    enum ADC_EXTERNAL_TRIG ext_trig;
		uint16_t maxValue;
		float refVoltage;
        uint8_t conversionNumber;
} ADC_initStruct;

typedef struct ADC_channel {
    uint8_t number;
    GPIO_TypeDef* GPIO_port;
    enum GPIO_PIN GPIO_pin;
} ADC_channel;

void adc_init(ADC_initStruct* adc);

void adc_start(void);

void adc_configureChannel(ADC_initStruct* adc, ADC_channel* channel, uint8_t order, enum ADC_SAMPLING_TIME samplingTime);

void adc_pollForConversion(void);

uint16_t adc_getValue(void);

float adc_getVoltage(ADC_initStruct* adc);
#endif

#include "../inc/adc.h"

void adc_init(ADC_initStruct adc) {
		//discontin. mode can be enabled only if scan mode is enabled
		//disc. and cont. mode cant be enabled at the same time (add assertions)
		assert((uint8_t)adc.discontinuous <= (uint8_t)adc.scan);
    assert(!(adc.discontinuous == 1 && adc.continuous == 1));
		assert(!(adc.clockPrescaller == ADC_CLOCK_PRESCALLER_2 && SystemCoreClock > 36000000)); 

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    //set adc clock prescaler (PCLK2 / value) (PCLK2 = 84 MHz) 
    ADC1_COMMON->CCR |= (uint32_t) adc.clockPrescaller << ADC_CCR_ADCPRE_Pos;

    //set adc resolution
    ADC1->CR1 |= (uint32_t) adc.resolution << ADC_CR1_RES_Pos;

    //set adc conversion data alignment
    ADC1->CR2 |= (uint32_t) adc.dataAlignment << ADC_CR2_ALIGN_Pos;

    //set adc continuous or discontinuous mode bit
    if(adc.continuous) {
      ADC1->CR2 |= ADC_CR2_CONT;
    }
    else if(adc.discontinuous) {
      ADC1->CR1 |= ADC_CR1_DISCEN;
    }
    //enable scan mode
    if(adc.scan) {
      ADC1->CR1 |= ADC_CR1_SCAN;
    }

    //enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;  
}

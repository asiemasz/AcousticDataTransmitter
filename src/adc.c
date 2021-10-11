#include "../inc/adc.h"

void adc_init(ADC_initStruct *adc)
{
  //discontin. mode can be enabled only if scan mode is enabled
  //disc. and cont. mode cant be enabled at the same time (add assertions)
  assert((uint8_t)adc->discontinuous <= (uint8_t)adc->scan);
  assert(!(adc->discontinuous == 1 && adc->continuous == 1));
  assert(!(adc->clockPrescaller == ADC_CLOCK_PRESCALLER_2 && SystemCoreClock > 36000000));
  assert(adc->resolution >= 0 && adc->resolution <= 3);
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  //set adc clock prescaler (PCLK2 / value) (PCLK2 = 84 MHz)
  ADC1_COMMON->CCR |= (uint32_t)adc->clockPrescaller << ADC_CCR_ADCPRE_Pos;

  //set adc resolution
  ADC1->CR1 |= (uint32_t)adc->resolution << ADC_CR1_RES_Pos;

  //set adc conversion data alignment
  ADC1->CR2 |= (uint32_t)adc->dataAlignment << ADC_CR2_ALIGN_Pos;

  //set adc continuous or discontinuous mode bit
  if (adc->continuous)
  {
    ADC1->CR2 |= ADC_CR2_CONT;
  }
  else if (adc->discontinuous)
  {
    ADC1->CR1 |= ADC_CR1_DISCEN;
  }
  //enable scan mode
  if (adc->scan)
  {
    ADC1->CR1 |= ADC_CR1_SCAN;
  }

  //enable ADC
  ADC1->CR2 |= ADC_CR2_ADON;

  switch (adc->resolution)
  {
  case ADC_RESOLUTION_6:
    adc->maxValue = 63;
    break;
  case ADC_RESOLUTION_8:
    adc->maxValue = 255;
    break;
  case ADC_RESOLUTION_10:
    adc->maxValue = 1023;
    break;
  case ADC_RESOLUTION_12:
    adc->maxValue = 4095;
    break;
  }

  adc->refVoltage = (float)3.3;
}

void adc_start()
{
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while(!(ADC1->SR & ADC_SR_STRT));
}

uint16_t adc_poll(uint8_t channel) {
	ADC1->SQR3 = ADC_SQR3_SQ1_Msk && channel;
  adc_start();
  while(!(ADC1->SR & ADC_SR_EOC));
  return ADC1->DR;
}



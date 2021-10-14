#include "../inc/adc.h"

void adc_init(ADC_initStruct *adc)
{
  //discontin. mode can be enabled only if scan mode is enabled
  //disc. and cont. mode cant be enabled at the same time (add assertions)
  assert((uint8_t)adc->discontinuous <= (uint8_t)adc->scan);
  assert(!(adc->discontinuous == 1 && adc->continuous == 1));
  assert(!(adc->clockPrescaller == ADC_CLOCK_PRESCALLER_2 && SystemCoreClock > 36000000));
  assert(adc->resolution >= 0 && adc->resolution <= 3);
  assert(adc->conversionNumber >= 1 && adc->conversionNumber <= 16);

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

  //set adc conversion number
  ADC1->SQR1 |= (uint32_t) (adc->conversionNumber - 1) << ADC_SQR1_L_Pos;

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

  adc->refVoltage = (float)3.3; //TODO: sprawdzić w datasheecie jaka faktyczna
}

void adc_start()
{
  //generate start condition
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while(!(ADC1->SR & ADC_SR_STRT));
}

void adc_pollForConversion() {
  adc_start();
  //wait until conversion end
  while(!(ADC1->SR & ADC_SR_EOC));
}

void adc_configureChannel(ADC_initStruct* adc, ADC_channel* channel, uint8_t order, enum ADC_SAMPLING_TIME samplingTime) {
  assert(order >= 1 && order <= adc->conversionNumber);
  //configure gpio pin as analog input
  gpio_init(channel->GPIO_port);
  gpio_set_pin_mode(channel->GPIO_port, channel->GPIO_pin, GPIO_MODE_ANALOG);
  gpio_set_pin_pull(channel->GPIO_port, channel->GPIO_pin, GPIO_NO_PULL);

  //set sampling time 
  if(channel->number < 10) {
    ADC1->SMPR2 |= (uint32_t) ((uint32_t)samplingTime << (uint8_t)channel->number * 3);
  }
  else {
    ADC1->SMPR1 |= (uint32_t) ((uint32_t)samplingTime << ((uint8_t)channel->number-10) * 3);
  }
  //set conversion sequency order number
  if(order <= 6) {
    ADC1->SQR3 |= (uint32_t) channel->number << (order-1) * 5; 
  }
  else if(order > 6 && order <= 12) {
    ADC1->SQR2 |= (uint32_t) channel->number << (order-7) * 5; 
  }
  else {
    ADC1->SQR1 |= (uint32_t) channel->number << (order-13) * 5; 
  }
}

void adc_startDMA(ADC_initStruct* adc, void* buf, uint8_t size) {
  dma_init(DMA2);
  DMA_requestStruct request;
  request.channel = DMA_CHANNEL_0;
  request.direction = DMA_DIRECTION_PERIPHERAL_TO_MEMORY;
  request.memInc = DMA_INC_ENABLE;
  request.periphDataSize = DMA_DATA_SIZE_HALF_WORD;
  request.memoryDataSize = DMA_DATA_SIZE_HALF_WORD;
  request.mode = DMA_CIRCULAR_MODE;
  request.fifoMode = DMA_FIFO_MODE_DISABLED;
	request.fifoThreshold = 0;
  request.priority = DMA_PRIORITY_LEVEL_HIGH;
  request.dataNumber = size;
  request.peripheralAddress = &(ADC1->DR);
  request.memory0Address = (uint32_t *)buf;

  dma_streamConfig(DMA2_Stream4, &request);
  ADC1->CR2 |= ADC_CR2_EOCS;
  ADC1->CR2 |= ADC_CR2_DMA;
  if(adc->dma) {
    ADC1->CR2 |= ADC_CR2_DDS;
  }
	adc_start();
}


uint16_t adc_getValue(void) {
  adc_pollForConversion();
  return (uint16_t) ADC1->DR;
}

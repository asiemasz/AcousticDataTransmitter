#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "i2c.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"

static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

static volatile uint16_t buffer[40000];
volatile uint8_t dataReady;
int main() {

	uart2.baudRate = 115200;
	uart2.mode = UART_TRANSMITTER_ONLY;
	uart2.oversampling = UART_OVERSAMPLING_BY_16;
	uart2.parityControl = UART_PARITY_CONTROL_DISABLED;
	uart2.stopBits = UART_STOP_BITS_1;
	uart2.wordLength = UART_WORD_LENGTH_8;
	uart2.uart = USART2;
		
	uart_init(&uart2);

	adc.clockPrescaller = ADC_CLOCK_PRESCALLER_4;
	adc.continuous = ADC_CONTINUOUS_CONVERSION_MODE_DISABLED;
	adc.ext_mode = ADC_EXTERNAL_TRIG_MODE_RISING_EDGE;
	adc.ext_trig = ADC_EXTERNAL_TRIG_TIMER2_TRGO;
	adc.resolution = ADC_RESOLUTION_12;
	adc.dma = ADC_DMA_CONTINUOUS_REQUEST_ENABLED;
	adc.conversionNumber = 1;
	
	chan0.number = 1;
	chan0.GPIO_pin = PIN1;
	chan0.GPIO_port = GPIOA;
	
	tim2.tim = TIM2;
	tim2.direction = TIMER_COUNTER_DIRECTION_DOWN;
	tim2.prescaler = 1;
	tim2.autoReload = 1050;

	timer_init(&tim2);
	timer_selectTRGOUTEvent(&tim2, TIMER_TRGOUT_EVENT_UPDATE);
	timer_enableIT(&tim2, TIMER_IT_TRGOUT);

	adc_init(&adc);
	adc_configureChannel(&adc, &chan0, 1, ADC_SAMPLING_TIME_144CYCL); //max. sampling time for 400 kHz sampling rate
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	adc_startDMA(&adc, (uint32_t *)buffer,(uint16_t) 40000, DMA_CIRCULAR_MODE);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

	timer_start(&tim2);	

	while(1) {
		if(dataReady) {
			dataReady = 0;
		}
	}
		
}
void DMA2_Stream4_IRQHandler() {

	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
		if(!dataReady) {
				dataReady = 0x1;
		}
	}

	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
		if(!dataReady) {
				dataReady = 0x1;
		}
	}
}

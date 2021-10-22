#include "../inc/gpio.h"
#include "../inc/i2c.h"
#include "../inc/timer.h"
#include "../inc/adc.h"
#include "../inc/uart.h"

static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

static volatile uint16_t buffer[2000];
volatile uint32_t i = 0;

int main() {
	gpio_init(GPIOA);
	GPIO_pinConfigStruct a10;
	a10.mode = GPIO_MODE_OUTPUT;
	a10.outSpeed = GPIO_OUT_SPEED_VERY_HIGH;
	
	gpio_setPinConfiguration(GPIOA, PIN10, &a10);

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
	tim2.autoReload = 210;

	timer_init(&tim2);
	timer_selectTRGOUTEvent(&tim2, TIMER_TRGOUT_EVENT_UPDATE);
	timer_enableIT(&tim2, TIMER_IT_TRGOUT);

	adc_init(&adc);
	adc_configureChannel(&adc, &chan0, 1, ADC_SAMPLING_TIME_3CYCL); //max. sampling time for 400 kHz sampling rate
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	adc_startDMA(&adc, (uint32_t *)buffer,(uint16_t) 2000, DMA_CIRCULAR_MODE);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER | DMA_IT_TRANSFER_COMPLETE);

	timer_start(&tim2);
	float x;
	char buf[20];
	
	while(1) {
		sprintf(buf, "%d \r\n", i);
		uart_sendString(&uart2, buf);
	}
		
}
void DMA2_Stream4_IRQHandler() {
	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER))
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE))
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);

	i = DMA2_Stream4->NDTR;
}

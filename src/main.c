#include "../inc/gpio.h"
#include "../inc/i2c.h"
#include "../inc/timer.h"
#include "../inc/adc.h"
#include "../inc/uart.h"
#include <arm_math.h>

static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

static volatile uint16_t buffer[4096];
volatile uint8_t dataReady;
float32_t buffer_input[2048], buffer_output[2048], buffer_output_mag[2048];
int main() {
	arm_rfft_instance_f32 S;
	arm_cfft_radix4_instance_f32 S_CFFT;
	uart2.baudRate = 115200;
	uart2.mode = UART_TRANSMITTER_ONLY;
	uart2.oversampling = UART_OVERSAMPLING_BY_16;
	uart2.parityControl = UART_PARITY_CONTROL_DISABLED;
	uart2.stopBits = UART_STOP_BITS_1;
	uart2.wordLength = UART_WORD_LENGTH_8;
	uart2.uart = USART2;
	
	arm_rfft_init_f32(&S, &S_CFFT, 2048, 0, 1);
	
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
	tim2.autoReload = 2100;

	timer_init(&tim2);
	timer_selectTRGOUTEvent(&tim2, TIMER_TRGOUT_EVENT_UPDATE);
	timer_enableIT(&tim2, TIMER_IT_TRGOUT);

	adc_init(&adc);
	adc_configureChannel(&adc, &chan0, 1, ADC_SAMPLING_TIME_144CYCL); //max. sampling time for 400 kHz sampling rate
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	adc_startDMA(&adc, (uint32_t *)buffer,(uint16_t) 4096, DMA_CIRCULAR_MODE);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

	timer_start(&tim2);	
	char buf[20];	
	float32_t maxVal;
	uint32_t maxValIndex;
	while(1) {
		if(dataReady) {
			arm_rfft_f32(&S, buffer_input, buffer_output);
			arm_cmplx_mag_f32(buffer_output, buffer_output_mag, 2048);
			arm_max_f32(buffer_output_mag, 2048, &maxVal, &maxValIndex);
			for( uint16_t i=0; i < 2048; i++){
				buffer_output_mag[i] = 100*buffer_output_mag[i]/maxVal;
				//sprintf(buf, "%.4f \r\n", buffer_output_mag[i]);
				//uart_sendString(&uart2, buf);
			}	
			uart_sendString(&uart2, "fft ready\r\n");
			dataReady = 0;
		}
	}
		
}
void DMA2_Stream4_IRQHandler() {

	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
		if(!dataReady) {
			for(int i = 0; i < 2048; i++) {
				buffer_input[i] = buffer[i];
			}
			dataReady = 0x1;
			uart_sendString(&uart2, "half\r\n");
		}
	}
	else if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
		if(!dataReady) {
			for(int i = 2048; i < 4096; i++) {
				buffer_input[i] = buffer[i];
			}
			dataReady = 0x1;
			uart_sendString(&uart2, "full\r\n");
		}
	}
}

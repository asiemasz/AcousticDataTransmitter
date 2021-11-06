#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "i2c.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"

#define BLOCK_SIZE 32
#define NUM_TAPS_ARRAY_SIZE 29
#define NUM_TAPS 29

static float32_t firStateF32[2048 + NUM_TAPS - 1];

static uint16_t numBlocks = 2048/BLOCK_SIZE;

const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {-0.001238f, -0.002175f, -0.000845f, 0.003789f, 0.007679f, 0.002303f, -0.013385f,
 -0.022869f, -0.004360f, 0.038050f, 0.059609f, 0.006119f, -0.128249f, -0.275917f, 0.660179f, -0.275917f, -0.128249f, 
 0.006119f, 0.059609f, 0.038050f, -0.004360f, -0.022869f, -0.013385f, 0.002303f, 0.007679f, 0.003789f, -0.000845f,
  -0.002175f, -0.001238f}; //highpass filter coeffs (15kHz +)

static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

static volatile uint16_t dmaBuffer[4096];
float32_t buffer_input[2048], buffer_filtered[2048], buffer_output[2048], buffer_output_mag[2048], maxValue;
uint16_t maxValueIndex;
volatile uint8_t dataReady;

int main() {		
	//Peripherals initialization
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
	tim2.autoReload = 1905;

	timer_init(&tim2);
	timer_selectTRGOUTEvent(&tim2, TIMER_TRGOUT_EVENT_UPDATE);
	timer_enableIT(&tim2, TIMER_IT_TRGOUT);

	adc_init(&adc);
	adc_configureChannel(&adc, &chan0, 1, ADC_SAMPLING_TIME_144CYCL); //max. sampling time for 400 kHz sampling rate
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	adc_startDMA(&adc, (uint32_t *)dmaBuffer,(uint16_t) 4096, DMA_CIRCULAR_MODE);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

	//DSP objects
	arm_rfft_fast_instance_f32 S; 
	arm_fir_instance_f32 S_f;
	arm_cfft_radix4_instance_f32 S_CFFT;

	arm_rfft_init_f32(&S, &S_CFFT, 2048, 0, 1);
	arm_fir_init_f32(&S_f, NUM_TAPS, firCoeffs32, firStateF32, 2048);

	timer_start(&tim2);	
	char buf[40];
	while(1) {
		if(dataReady) {
			for (uint32_t i = 0; i < numBlocks; i++) {
				arm_fir_f32(&S_f, buffer_input + (i * BLOCK_SIZE), buffer_filtered + (i * BLOCK_SIZE), BLOCK_SIZE); //filter data
			}
			arm_rfft_f32(&S, buffer_filtered, buffer_output); //calculate DFT
			arm_cmplx_mag_f32(buffer_output, buffer_output_mag, 2048); //DFT modulus
			arm_max_f32(buffer_output_mag, 1024, &maxValue, &maxValueIndex);//find main peak within 0-(Fs/2) freq. range
			sprintf(buf, "Detected frequency: %.4f \r\n", maxValueIndex*44100.0f/2048.0f);
			uart_sendString(&uart2, buf);
			dataReady = 0;
		}
	}
		
}
void DMA2_Stream4_IRQHandler() {

	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
		if(!dataReady) {
			for(uint16_t i = 0; i < 2048; i++)
				buffer_input[i] = dmaBuffer[i];
			dataReady = 0x1;
		}
	}

	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
		if(!dataReady) {
			for(uint16_t i = 2048; i < 4096; i++)
				buffer_input[i] = dmaBuffer[i];
			dataReady = 0x1;
		}
	}
}

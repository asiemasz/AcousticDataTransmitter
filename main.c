#include "stm32f401xe.h"
#include "arm_math.h"
#include "gpio.h"
#include "i2c.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "SRRC_filter.h"
#include "BPSK.h"

#define SAMPLES 2048

#define FS 48000
#define FSystem 84000000
#define FC 17000
#define ROLLOVER_FACTOR 0.25
#define FSPAN 4
#define FB 1000
#define SPB (FS / FB)
#define N_BYTES 3

#define SYNC_PATTERN 170

#define SYNC_PATTERN_LENGTH (SPB*8)

//Filter
#define BLOCK_SIZE 32
#define NUM_TAPS_ARRAY_SIZE 29
#define NUM_TAPS 29
static float32_t firStateF32[SAMPLES + NUM_TAPS - 1];
static uint16_t numBlocks = SAMPLES/BLOCK_SIZE;

const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {-0.001238f, -0.002175f, -0.000845f, 0.003789f, 0.007679f, 0.002303f, -0.013385f,
 -0.022869f, -0.004360f, 0.038050f, 0.059609f, 0.006119f, -0.128249f, -0.275917f, 0.660179f, -0.275917f, -0.128249f, 
 0.006119f, 0.059609f, 0.038050f, -0.004360f, -0.022869f, -0.013385f, 0.002303f, 0.007679f, 0.003789f, -0.000845f,
  -0.002175f, -0.001238f}; //highpass filter coeffs (15kHz +)
static arm_fir_instance_f32 S_f;

//Matched filter and pattern symbol
float32_t coeffs[FSPAN*SPB + 1];
float32_t pattern[SYNC_PATTERN_LENGTH];

float32_t convRes[SAMPLES*2 - 1];

//Peripherals
static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

//data storage
static volatile uint16_t dmaBuffer[SAMPLES*2];
static float32_t buffer_input[SAMPLES], buffer_filtered[SAMPLES];
static volatile uint8_t dataReady;

int main() {

	SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, coeffs, FSPAN*SPB + 1);

	BPSK_parameters params = {
	.Fb = FB,
	.Fs = FS,
	.Fc = FC,
	.FSpan = FSPAN,
	.firCoeffs = coeffs,
	.firCoeffsLength = FSPAN*SPB + 1
	};

	uint8_t sync[1] = {SYNC_PATTERN};

	BPSK_getOutputSignal(&params, sync, 1, pattern, SYNC_PATTERN_LENGTH);

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
	tim2.autoReload = 1750;

	timer_init(&tim2);
	timer_selectTRGOUTEvent(&tim2, TIMER_TRGOUT_EVENT_UPDATE);
	timer_enableIT(&tim2, TIMER_IT_TRGOUT);

	adc_init(&adc);
	adc_configureChannel(&adc, &chan0, 1, ADC_SAMPLING_TIME_56CYCL);
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);

	adc_startDMA(&adc, (uint32_t *)dmaBuffer,(uint16_t) SAMPLES*2, DMA_CIRCULAR_MODE);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

	arm_fir_init_f32(&S_f, NUM_TAPS, firCoeffs32, firStateF32, SAMPLES);

	timer_start(&tim2);	
	char buf[40];
	while(1) {
		if(dataReady) {
			for (uint32_t i = 0; i < numBlocks; i++) {
				arm_fir_f32(&S_f, buffer_input + (i * BLOCK_SIZE), buffer_filtered + (i * BLOCK_SIZE), BLOCK_SIZE); //filter data
			}
			arm_correlate_f32(buffer_filtered, 2048, pattern, SYNC_PATTERN_LENGTH, convRes);
			dataReady = 0;
		}
	}
		
}
void DMA2_Stream4_IRQHandler() {
	char buf[20];
	if(dataReady) {
		sprintf(buf, "Overrun \r\n\n");
		uart_sendString(&uart2, buf);
		Default_Handler();
	}

	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
		if(!dataReady) {
			for(uint16_t i = 0; i < SAMPLES; i++)
				buffer_input[i] = dmaBuffer[i];
			dataReady = 0x1;
		}
	}



	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
		if(!dataReady) {
			for(uint16_t i = SAMPLES; i < 2*SAMPLES; i++)
				buffer_input[i] = dmaBuffer[i];
			dataReady = 0x1;
		}
	}
}

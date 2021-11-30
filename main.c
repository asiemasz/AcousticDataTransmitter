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

#define NUM_TAPS_ARRAY_SIZE 30

static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {-0.001238f, -0.002175f, -0.000845f, 0.003789f, 0.007679f, 0.002303f, -0.013385f,
 -0.022869f, -0.004360f, 0.038050f, 0.059609f, 0.006119f, -0.128249f, -0.275917f, 0.660179f, -0.275917f, -0.128249f, 
 0.006119f, 0.059609f, 0.038050f, -0.004360f, -0.022869f, -0.013385f, 0.002303f, 0.007679f, 0.003789f, -0.000845f,
  -0.002175f, -0.001238f, 0}; //highpass filter coeffs (15kHz +)

static q15_t firCoeffs_q15[NUM_TAPS_ARRAY_SIZE];
static q15_t temp[2*SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
//Matched filter and pattern symbol
static float32_t coeffs[FSPAN*SPB + 1];
static float32_t pattern[SYNC_PATTERN_LENGTH];
static q15_t     pattern_q15[SYNC_PATTERN_LENGTH];

static q15_t corrRes[2*SAMPLES*2 - 1];
static q15_t result[SPB*8*3];
static float32_t result_f[SPB*8*3];
static uint8_t data[3];

//Peripherals
static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

//data storage
static volatile uint16_t dmaBuffer[SAMPLES*2];
static q15_t buffer_input[2*SAMPLES], buffer_filtered[2*SAMPLES];
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

	arm_float_to_q15(firCoeffs32, firCoeffs_q15, NUM_TAPS_ARRAY_SIZE);
	float32_t maxVal;
	uint16_t maxIdx;
	arm_max_f32(pattern, SYNC_PATTERN_LENGTH, &maxVal, &maxIdx);
	for(uint16_t i = 0; i < SYNC_PATTERN_LENGTH; i++) {
		pattern_q15[i] = (pattern[i]/maxVal) * 1000;
	}

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

	adc_startDMA(&adc, (uint32_t *)dmaBuffer,(uint16_t) SAMPLES*2, DMA_DIRECT_MODE);
	//dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
	dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

	timer_start(&tim2);	
	char buf[40];
	while(1) {
		if(dataReady) {
			q15_t max;
			uint16_t idx;
			arm_conv_fast_q15(buffer_input, 2*SAMPLES, firCoeffs_q15, 30, temp);
			arm_copy_q15(temp + NUM_TAPS_ARRAY_SIZE - 1, buffer_filtered, 2*SAMPLES);
			
			arm_correlate_fast_q15(buffer_filtered, 2*SAMPLES, pattern_q15, SYNC_PATTERN_LENGTH, corrRes);
			arm_max_q15(corrRes + 2*SAMPLES, 2*SAMPLES, &max, &idx);

			if(idx < 4096 - 1152 - SYNC_PATTERN_LENGTH)
				arm_copy_q15((buffer_filtered + idx + SYNC_PATTERN_LENGTH), result ,1152);
			else 
				arm_copy_q15((buffer_filtered + idx - 1152), result, 1152);

			arm_q15_to_float(result, result_f, 1152);

			BPSK_demodulateSignal(&params, result_f, 1152, data, 3);


			//not very elegant, but useful for future changes approving(smaller buffer etc)
			if(data[0] == 25 && data[1] == 161 && data[2] == 149)
				uart_sendString(&uart2, "ok\r\n");
			else
				uart_sendString(&uart2, "nie ok\r\n"); 

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

	/*if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
		if(!dataReady) {
			for(uint16_t i = 0; i < SAMPLES; i++)
				buffer_input[i] = dmaBuffer[i];
			dataReady = 0x1;
		}
	}*/

	if(dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
		dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
		if(!dataReady) {
			for(uint16_t i = 0; i < 2*SAMPLES; i++) {
				buffer_input[i] = dmaBuffer[i];
				sprintf(buf, "%d \r\n", buffer_input[i]);
				uart_sendString(&uart2, buf);
			}
			//dataReady = 0x1;
		}
	}
}

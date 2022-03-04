#include "BPSK.h"
#include "IIR_filter.h"
#include "SRRC_filter.h"
#include "adc.h"
#include "arm_math.h"
#include "gpio.h"
#include "i2c.h"
#include "stm32f401xe.h"
#include "timer.h"
#include "uart.h"

#define SAMPLES 2048

#define FS 48000
#define FSystem 84000000
#define FC 8000
#define ROLLOVER_FACTOR 0.25f
#define FSPAN 4
#define SPB 48
#define N_BYTES 1

#define NUM_TAPS_ARRAY_SIZE 30

static const float32_t lpIIRCoeffsA[2] = {1.0f, -0.5095254494944288f};
static const float32_t lpIIRCoeffsB[2] = {0.2452372752527856f,
                                          0.2452372752527856f};

/*
static const float32_t lpIIRCoeffsA[3] = {1.000000f, -0.000000f, 0.171573f};
static const float32_t lpIIRCoeffsB[3] = {0.292893f, 0.585786f, 0.292893f};
*/
/*
static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    -0.001238f, -0.002175f, -0.000845f, 0.003789f,
    0.007679f,  0.002303f,  -0.013385f, -0.022869f,
    -0.004360f, 0.038050f,  0.059609f,  0.006119f,
    -0.128249f, -0.275917f, 0.660179f,  -0.275917f,
    -0.128249f, 0.006119f,  0.059609f,  0.038050f,
    -0.004360f, -0.022869f, -0.013385f, 0.002303f,
    0.007679f,  0.003789f,  -0.000845f, -0.002175f,
    -0.001238f, 0}; // highpass filter coeffs (15kHz +)*/
static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    -0.010641f, -0.008923f, 0.012742f, 0.034371f, 0.021928f, -0.026645f,
    -0.063340f, -0.036643f, 0.041265f, 0.091514f, 0.049803f, -0.053188f,
    -0.112052f, -0.058111f, 0.059353f, 0.119577f, 0.059353f, -0.058111f,
    -0.112052f, -0.053188f, 0.049803f, 0.091514f, 0.041265f, -0.036643f,
    -0.063340f, -0.026645f, 0.021928f, 0.034371f, 0.012742f, -0.008923f,
    -0.010641f};

static q15_t firCoeffs_q15[NUM_TAPS_ARRAY_SIZE];
static q15_t temp_[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
// Matched filter and pattern symbol
static float32_t coeffs[FSPAN * SPB + 1];

static q15_t corrRes[2 * SAMPLES * 2 - 1];

// Peripherals
static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

// data storage
static volatile uint16_t dmaBuffer[SAMPLES * 2];

static q15_t buffer_input[2 * SAMPLES], buffer_filtered[2 * SAMPLES];
static float32_t buffer_filtered_f32[2 * SAMPLES];
static volatile uint8_t dataReady;

uint8_t data[2 * SAMPLES / 624];
uint16_t idx[2 * SAMPLES / 624];
uint16_t foundFrames;
float32_t bufA_I, bufA_Q, bufB_I, bufB_Q;

int main() {
  SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, coeffs, FSPAN * SPB + 1);

  IIR_filter filterI =
      IIR_filter_init(lpIIRCoeffsA, 2, lpIIRCoeffsB, 2, &bufA_I, &bufB_I);
  IIR_filter filterQ =
      IIR_filter_init(lpIIRCoeffsA, 2, lpIIRCoeffsB, 2, &bufA_Q, &bufB_Q);

  costasLoop_parameters costas = {.alpha = 0.1f,
                                  .beta = 0.1f * 0.1f / 4.0f,
                                  .LP_filterI = &filterI,
                                  .LP_filterQ = &filterQ};

  BPSK_parameters BPSK_params = {.Fb = FS / SPB,
                                 .Fs = FS,
                                 .Fc = FC,
                                 .FSpan = FSPAN,
                                 .matchedFilterCoeffs = coeffs,
                                 .matchedFilterCoeffsLength = FSPAN * SPB + 1,
                                 .frameLength = SPB * 8,
                                 .samplesPerBit = SPB,
                                 .costas = &costas,
                                 .gardner = &gardner,
                                 .differential = false,
                                 .prefix = false};

  BPSK_init(&BPSK_params);

  int16_t barker[5] = {1, 1, 1, -1, 1};
  float32_t preamble[5 * SPB];

  BPSK_setPreamble(&BPSK_params, barker, 5, preamble, 5 * SPB);

  arm_float_to_q15(firCoeffs32, firCoeffs_q15, NUM_TAPS_ARRAY_SIZE);

  // Peripherals initialization
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

  adc_startDMA(&adc, (uint32_t *)dmaBuffer, (uint16_t)SAMPLES * 2,
               DMA_DIRECT_MODE);
  // dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
  dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

  timer_start(&tim2);
  char buf[40];
  while (1) {
    if (dataReady) {
      arm_conv_fast_q15(buffer_input, 2 * SAMPLES, firCoeffs_q15, 30, temp_);
      arm_copy_q15(temp_ + NUM_TAPS_ARRAY_SIZE / 2, buffer_filtered,
                   2 * SAMPLES);

      arm_q15_to_float(buffer_filtered, buffer_filtered_f32, 2 * SAMPLES);
      float32_t maxVal;
      arm_max_f32(buffer_filtered_f32, 2 * SAMPLES, &maxVal, &idx);
      uart_sendString(&uart2, "\r\n\r\n Buffer after filter: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        buffer_filtered_f32[i] = buffer_filtered_f32[i] / maxVal;
        sprintf(buf, "%f \r\n", buffer_filtered_f32[i]);
        uart_sendString(&uart2, buf);
      }

      BPSK_syncSignalCarrier(&BPSK_params, buffer_filtered_f32, 2 * SAMPLES);

      float32_t temp[2 * SAMPLES + BPSK_params.matchedFilterCoeffsLength - 1];

      arm_conv_f32(buffer_filtered_f32, 2 * SAMPLES,
                   BPSK_params.matchedFilterCoeffs,
                   BPSK_params.matchedFilterCoeffsLength, temp);

      uart_sendString(&uart2, "\r\n\r\n Buffer after conv: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        sprintf(
            buf, "%f \r\n",
            *(temp + BPSK_params.samplesPerBit * BPSK_params.FSpan / 2 + i));
        uart_sendString(&uart2, buf);
      }

      float32_t buffer_[2 * SAMPLES];
      for (uint16_t i = BPSK_params.samplesPerBit * BPSK_params.FSpan / 2;
           i < 2 * SAMPLES + BPSK_params.samplesPerBit * BPSK_params.FSpan / 2;
           ++i) {
        if (temp[i] > 0) {
          buffer_[i - BPSK_params.samplesPerBit * BPSK_params.FSpan / 2] = 1.0f;
        } else {
          buffer_[i - BPSK_params.samplesPerBit * BPSK_params.FSpan / 2] =
              -1.0f;
        }
      }

      uart_sendString(&uart2, "\r\n\r\n Decode: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; ++i) {
        sprintf(buf, "%f \r\n", buffer_[i]);
        uart_sendString(&uart2, buf);
      }

      BPSK_findSymbolsStarts(&BPSK_params, buffer_, 2 * SAMPLES, idx,
                             &foundFrames);
      uint16_t good = 0;
      for (uint16_t i = 0; i < foundFrames; ++i) {
        BPSK_demodulateSignal(&BPSK_params, buffer_ + idx[i],
                              BPSK_params.frameLength, data + i, 1);

        for (uint16_t j = 0; j < 8; ++j) {
          if ((data[i] & (1U << j)) == (109 & (1U << j))) {
            good++;
          }
        }
      }
      dataReady = 0;
    }
  }
}

void DMA2_Stream4_IRQHandler() {
  char buf[20];
  if (dataReady) {
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

  if (dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
    dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
    if (!dataReady) {
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        buffer_input[i] = dmaBuffer[i];
      }
      dataReady = 0x1;
    }
  }
}

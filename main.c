#include "adc.h"
#include "arm_math.h"
#include "gpio.h"
#include "i2c.h"
#include "stm32f401xe.h"
#include "timer.h"
#include "uart.h"

#include "BPSK.h"
#include "IIR_filter.h"
#include "SRRC_filter.h"

#define SAMPLES 2048

#define FS 48000
#define FSystem 84000000
#define FC 8000
#define ROLLOVER_FACTOR 0.25f
#define FSPAN 2
#define SPB 24
#define N_BYTES 1
#define COSTAS_LPF_ORDER 4

#define NUM_TAPS_ARRAY_SIZE 31

static const float32_t lpIIRCoeffsA[4] = {1.0000f, -1.9630f, 1.4000f, -0.3464f};
static const float32_t lpIIRCoeffsB[4] = {0.0113f, 0.0340f, 0.0340f, 0.0113f};

// static const float32_t lpIIRCoeffsA[2] = {1.0f, -0.5095254494944288f};
// static const float32_t lpIIRCoeffsB[2] = {0.2452372752527856f,
//                                          0.2452372752527856f};

/* static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    -0.010641f, -0.008923f, 0.012742f, 0.034371f, 0.021928f, -0.026645f,
    -0.063340f, -0.036643f, 0.041265f, 0.091514f, 0.049803f, -0.053188f,
    -0.112052f, -0.058111f, 0.059353f, 0.119577f, 0.059353f, -0.058111f,
    -0.112052f, -0.053188f, 0.049803f, 0.091514f, 0.041265f, -0.036643f,
    -0.063340f, -0.026645f, 0.021928f, 0.034371f, 0.012742f, -0.008923f,
    -0.010641f}; */
static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    -0.000319f, -0.000771f, 0.001450f, 0.006804f, 0.006916f, -0.009589f,
    -0.032568f, -0.025144f, 0.029760f, 0.081756f, 0.052462f, -0.056917f,
    -0.133670f, -0.074220f, 0.076056f, 0.156148f, 0.076056f, -0.074220f,
    -0.133670f, -0.056917f, 0.052462f, 0.081756f, 0.029760f, -0.025144f,
    -0.032568f, -0.009589f, 0.006916f, 0.006804f, 0.001450f, -0.000771f,
    -0.000319f};

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

uint8_t data[2 * SAMPLES / 312];
uint16_t idx[2 * SAMPLES / 312];
uint16_t foundFrames;
float32_t bufA_I[COSTAS_LPF_ORDER - 1], bufA_Q[COSTAS_LPF_ORDER - 1],
    bufB_I[COSTAS_LPF_ORDER - 1], bufB_Q[COSTAS_LPF_ORDER - 1];

int main() {
  SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, coeffs, FSPAN * SPB + 1);

  IIR_filter filterI =
      IIR_filter_init(lpIIRCoeffsA, COSTAS_LPF_ORDER, lpIIRCoeffsB,
                      COSTAS_LPF_ORDER, bufA_I, bufB_I);
  IIR_filter filterQ =
      IIR_filter_init(lpIIRCoeffsA, COSTAS_LPF_ORDER, lpIIRCoeffsB,
                      COSTAS_LPF_ORDER, bufA_Q, bufB_Q);

  costasLoop_parameters costas = {.alpha = 0.1f,
                                  .beta = 0.1f * 0.1f / 40.0f,
                                  .LP_filterI = &filterI,
                                  .LP_filterQ = &filterQ};

  gardnerTimingRecovery_parameters gardner = {.loop_gain = 0.35,
                                              .max_error = 5};

  int8_t barker[5] = {1, 1, 1, -1, 1};
  float32_t preamble[5 * SPB];

  BPSK_parameters BPSK_params = {.Fb = FS / SPB,
                                 .Fs = FS,
                                 .Fc = FC,
                                 .FSpan = FSPAN,
                                 .matchedFilterCoeffs = coeffs,
                                 .matchedFilterCoeffsLength = FSPAN * SPB + 1,
                                 .frameLength = 8,
                                 .samplesPerBit = SPB,
                                 .costas = &costas,
                                 .gardner = &gardner,
                                 .differential = false,
                                 .preambleCode = barker,
                                 .preambleCodeLength = 5,
                                 .prefix = false};

  BPSK_reset(&BPSK_params);

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

      BPSK_carrierRecovery(&BPSK_params, buffer_filtered_f32, 2 * SAMPLES);

      float32_t temp[2 * SAMPLES + BPSK_params.matchedFilterCoeffsLength - 1];

      arm_conv_f32(buffer_filtered_f32, 2 * SAMPLES,
                   BPSK_params.matchedFilterCoeffs,
                   BPSK_params.matchedFilterCoeffsLength, temp);

      arm_max_f32(temp, 2 * SAMPLES + BPSK_params.matchedFilterCoeffsLength - 1,
                  &maxVal, &idx);

      uart_sendString(&uart2, "\r\n\r\n Buffer after conv: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        temp[BPSK_params.samplesPerBit * BPSK_params.FSpan / 2 + i] /= maxVal;
        sprintf(
            buf, "%f \r\n",
            *(temp + BPSK_params.samplesPerBit * BPSK_params.FSpan / 2 + i));
        uart_sendString(&uart2, buf);
      }

      int8_t buffer_[2 * SAMPLES / SPB];

      BPSK_timingRecovery(&BPSK_params,
                          temp +
                              BPSK_params.samplesPerBit * BPSK_params.FSpan / 2,
                          2 * SAMPLES, buffer_, 2 * SAMPLES / SPB);

      BPSK_findSymbolsStarts_decimated(&BPSK_params, buffer_, 2 * SAMPLES / SPB,
                                       idx, &foundFrames);

      uart_sendString(&uart2, "\r\n\r\n Decode: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES / SPB; ++i) {
        sprintf(buf, "%d \r\n", buffer_[i]);
        uart_sendString(&uart2, buf);
      }
      uint8_t out_data[foundFrames];

      BPSK_demodulateSignal_decimated(&BPSK_params, buffer_, 2 * SAMPLES / SPB,
                                      idx, foundFrames, out_data, foundFrames);

      uart_sendString(&uart2, "\r\n\r\n Data: \r\n");
      for (uint16_t i = 0; i < foundFrames; ++i) {
        sprintf(buf, "%d\r\n", out_data[i]);
        uart_sendString(&uart2, buf);
      }
      dataReady = 0;
    }
    //   uart_sendString(&uart2, "ok");
  }
}

void DMA2_Stream4_IRQHandler() {
  char buf[20];
  if (dataReady) {
    sprintf(buf, "Error! Overrun \r\n");
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

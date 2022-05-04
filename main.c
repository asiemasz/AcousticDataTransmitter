#include "adc.h"
#include "arm_math.h"
#include "gpio.h"
#include "i2c.h"
#include "stm32f401xe.h"
#include "timer.h"
#include "uart.h"

#include "BPSK.h"
#include "FIR_filter.h"
#include "IIR_filter.h"
#include "SRRC_filter.h"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)                                                   \
  (byte & 0x80 ? '1' : '0'), (byte & 0x40 ? '1' : '0'),                        \
      (byte & 0x20 ? '1' : '0'), (byte & 0x10 ? '1' : '0'),                    \
      (byte & 0x08 ? '1' : '0'), (byte & 0x04 ? '1' : '0'),                    \
      (byte & 0x02 ? '1' : '0'), (byte & 0x01 ? '1' : '0')

#define SAMPLES 2048

#define FS 48000
#define FSystem 84000000
#define FC 8000
#define ROLLOVER_FACTOR 0.5f
#define FSPAN 2
#define SPB 24
#define COSTAS_LPF_ORDER 11

#define NUM_TAPS_ARRAY_SIZE 42

//#define DEBUG

static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    0.018192f,  0.000047f,  -0.010678f, -0.005610f, 0.000000f,  -0.007013f,
    -0.014393f, -0.000048f, 0.030547f,  0.039173f,  0.000080f,  -0.056329f,
    -0.064752f, -0.000087f, 0.079936f,  0.086631f,  0.000066f,  -0.097063f,
    -0.100767f, -0.000025f, 0.104480f,  0.104480f,  -0.000025f, -0.100767f,
    -0.097063f, 0.000066f,  0.086631f,  0.079936f,  -0.000087f, -0.064752f,
    -0.056329f, 0.000080f,  0.039173f,  0.030547f,  -0.000048f, -0.014393f,
    -0.007013f, 0.000000f,  -0.005610f, -0.010678f, 0.000047f,  0.018192f};

float32_t lpFIRCoeffs[COSTAS_LPF_ORDER] = {
    0.078989f, 0.085868f, 0.091518f, 0.095721f, 0.098311f, 0.099186f,
    0.098311f, 0.095721f, 0.091518f, 0.085868f, 0.078989f,
};

static q15_t temp_[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
static float32_t temp_f32[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
static volatile uint16_t time = 0;
static uint16_t start, end;
// Matched filter and pattern symbol
static float32_t matchedCoeffs[FSPAN * SPB + 1];

// Peripherals structs
static const ADC_initStruct adc = {
    .clockPrescaller = ADC_CLOCK_PRESCALLER_4,
    .continuous = ADC_CONTINUOUS_CONVERSION_MODE_DISABLED,
    .ext_mode = ADC_EXTERNAL_TRIG_MODE_RISING_EDGE,
    .ext_trig = ADC_EXTERNAL_TRIG_TIMER2_TRGO,
    .resolution = ADC_RESOLUTION_12,
    .dma = ADC_DMA_CONTINUOUS_REQUEST_ENABLED,
    .conversionNumber = 1};

static const TIMER_initStruct tim2 = {.tim = TIM2,
                                      .direction = TIMER_COUNTER_DIRECTION_DOWN,
                                      .prescaler = 1,
                                      .autoReload = FSystem / FS};

static ADC_channel chan0 = {.number = 1, .GPIO_pin = PIN1, .GPIO_port = GPIOA};

static const UART_initStruct uart2 = {.baudRate = 115200,
                                      .mode = UART_TRANSMITTER_ONLY,
                                      .oversampling = UART_OVERSAMPLING_BY_16,
                                      .parityControl =
                                          UART_PARITY_CONTROL_DISABLED,
                                      .stopBits = UART_STOP_BITS_1,
                                      .wordLength = UART_WORD_LENGTH_8,
                                      .uart = USART2};

// Data storage
static volatile uint16_t dmaBuffer[SAMPLES * 2];
static float32_t buffer_filtered_f32[SAMPLES];
static float32_t buffer_input_f32[SAMPLES];
static volatile uint8_t dataReady;

float32_t buffer_LP_costas_I[COSTAS_LPF_ORDER],
    buffer_LP_costas_Q[COSTAS_LPF_ORDER];

int main() {
  /// Peripherals initialization ///
  uart_init(&uart2);

  timer_init(&tim2);
  timer_selectTRGOUTEvent(&tim2, TIMER_TRGOUT_EVENT_UPDATE);
  timer_enableIT(&tim2, TIMER_IT_TRGOUT);

  adc_init(&adc);
  adc_configureChannel(&adc, &chan0, 1, ADC_SAMPLING_TIME_144CYCL);
  NVIC_EnableIRQ(DMA2_Stream4_IRQn);

#ifndef DEBUG
  adc_startDMA(&adc, (uint32_t *)dmaBuffer, (uint16_t)SAMPLES * 2,
               DMA_CIRCULAR_MODE);
#else
  adc_startDMA(&adc, (uint32_t *)dmaBuffer, (uint16_t)SAMPLES * 2,
               DMA_DIRECT_MODE);
#endif
  dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
  dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

  /// Demodulator system components initialization ///
  SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, matchedCoeffs, FSPAN * SPB + 1,
                    0.5f);

  FIR_filter filterI =
      FIR_filter_init(lpFIRCoeffs, COSTAS_LPF_ORDER, buffer_LP_costas_I);
  FIR_filter filterQ =
      FIR_filter_init(lpFIRCoeffs, COSTAS_LPF_ORDER, buffer_LP_costas_Q);

  costasLoop_parameters costas = {.alpha = 0.1f,
                                  .beta = 0.1f * 0.1f / 4.0f,
                                  .LP_filterI = &filterI,
                                  .LP_filterQ = &filterQ};

  gardnerTimingRecovery_parameters gardner = {.Kp = -0.0042797f,
                                              .Ki = -3.17255e-06f};

  const int8_t barker[5] = {1, 1, 1, -1, 1};

  BPSK_parameters BPSK_params = {
      .Fb = FS / SPB,
      .Fs = FS,
      .Fc = FC,
      .FSpan = FSPAN,
      .matchedFilterCoeffs = matchedCoeffs,
      .matchedFilterCoeffsLength = FSPAN * SPB + 1,
      .frameLength = 8,
      .samplesPerBit = SPB,
      .costas = &costas,
      .gardner = &gardner,
      .preambleCode = barker,
      .preambleCodeLength = 5,
  };

  BPSK_reset(&BPSK_params);

  SysTick_Config(84000000 / 1000);

  char buf[40];
  timer_start(&tim2);

  while (1) {
    if (dataReady) {
      uint16_t foundFrames = 0U;

      arm_conv_f32(buffer_input_f32, SAMPLES, firCoeffs32, NUM_TAPS_ARRAY_SIZE,
                   temp_f32);
      arm_copy_f32(temp_f32 + NUM_TAPS_ARRAY_SIZE / 2, buffer_filtered_f32,
                   SAMPLES);
      float32_t maxVal;
      uint16_t maxIdx;
      arm_max_f32(buffer_filtered_f32, SAMPLES, &maxVal, &maxIdx);
#ifdef DEBUG
      uart_sendString(&uart2, "\r\n\r\n Buffer after filter: \r\n ");
#endif
      for (uint16_t i = 0; i < SAMPLES; i++) {
        buffer_filtered_f32[i] = buffer_filtered_f32[i] / maxVal * 1.5f;
#ifdef DEBUG
        sprintf(buf, "%f \r\n", buffer_filtered_f32[i]);
        uart_sendString(&uart2, buf);
#endif
      }

      float32_t locked =
          BPSK_carrierRecovery(&BPSK_params, buffer_filtered_f32, SAMPLES);

      float32_t temp[SAMPLES + BPSK_params.matchedFilterCoeffsLength - 1];

      arm_conv_f32(buffer_filtered_f32, SAMPLES,
                   BPSK_params.matchedFilterCoeffs,
                   BPSK_params.matchedFilterCoeffsLength, temp);

      arm_max_f32(temp, SAMPLES + BPSK_params.matchedFilterCoeffsLength - 1,
                  &maxVal, &maxIdx);
#ifdef DEBUG
      uart_sendString(&uart2, "\r\n\r\n Buffer after conv: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        sprintf(
            buf, "%f \r\n",
            *(temp + BPSK_params.samplesPerBit * BPSK_params.FSpan / 2 + i));
        uart_sendString(&uart2, buf);
      }
#endif
      int8_t buffer_[SAMPLES / SPB + 1];

      BPSK_timingRecovery(&BPSK_params,
                          temp +
                              BPSK_params.samplesPerBit * BPSK_params.FSpan / 2,
                          SAMPLES, buffer_, SAMPLES / SPB + 1);

      uint16_t idx[SAMPLES / 312 + 2];
      BPSK_findSymbolsStarts(&BPSK_params, buffer_, SAMPLES / SPB, idx,
                             &foundFrames);
#ifdef DEBUG
      uart_sendString(&uart2, "\r\n\r\n Decode : \r\n ");
      for (uint16_t i = 0; i < 2 * SAMPLES / SPB; ++i) {
        sprintf(buf, "%d \r\n", buffer_[i]);
        uart_sendString(&uart2, buf);
      }
#endif
      uint8_t out_data[foundFrames + 1];

      BPSK_demodulateSignal(&BPSK_params, buffer_, 2 * SAMPLES / SPB, idx,
                            foundFrames, out_data, foundFrames);
#ifdef DEBUG
      uart_sendString(&uart2, "\r\n\r\n Data: \r\n");
      for (uint16_t i = 0; i < foundFrames; ++i) {
        sprintf(buf, "%d %c%c%c%c%c%c%c%c \r\n", out_data[i],
                BYTE_TO_BINARY(out_data[i]));
        uart_sendString(&uart2, buf);
      }
#endif
      uint16_t correct = 0;
      for (uint8_t i = 0; i < foundFrames; i++) {
        if (out_data[i] == 109 || out_data[i] == 222 || out_data[i] == 11)
          correct++;
      }

      dataReady = 0;
      end = time - start;
      sprintf(buf, "\r\n %ld %d %d %f\r\n", time, correct, foundFrames, locked);
      uart_sendString(&uart2, buf);
    }
  }
}

void DMA2_Stream4_IRQHandler() {
  char buf[20];
  if (dataReady) {
    sprintf(buf, "Error! Overrun \r\n");
    uart_sendString(&uart2, buf);
    __DMB();
    __DMB();
    __DMB();

    Default_Handler();
  }

  if (dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER)) {
    dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_HALF_TRANSFER);
    if (!dataReady) {
      for (uint16_t i = 0; i < SAMPLES; i++)
        buffer_input_f32[i] = (float32_t)dmaBuffer[i] / 4096.0f * 3.3f;
      dataReady = 0x1;
    }
  }

  if (dma_streamGetITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE)) {
    dma_streamClearITFlag(DMA2, 4, DMA_IT_FLAG_TRANSFER_COMPLETE);
    if (!dataReady) {
#ifdef DEBUG
      uart_sendString(&uart2, "\r\n Buffer ADC: \r\n");
#endif
      for (uint16_t i = SAMPLES; i < 2 * SAMPLES; i++) {
        buffer_input_f32[i - SAMPLES] =
            (float32_t)dmaBuffer[i] / 4096.0f * 3.3f;
#ifdef DEBUG
        sprintf(buf, "%f \r\n", buffer_input_f32[i]);
        uart_sendString(&uart2, buf);
#endif
      }
      dataReady = 0x1;
      start = time;
    }
  }
}

void SysTick_Handler(void) { time++; }
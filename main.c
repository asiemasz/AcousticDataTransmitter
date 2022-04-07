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
#define ROLLOVER_FACTOR 0.25f
#define FSPAN 2
#define SPB 24
#define COSTAS_LPF_ORDER 5

#define NUM_TAPS_ARRAY_SIZE 41

//#define DEBUG

/* static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] =
    { // 3kHz width, Kaiser
        -0.008380f, -0.006318f, 0.008362f,  0.020471f,  0.011854f,
-0.013133f, -0.027991f, -0.014370f, 0.014195f,  0.026845f,  0.012019f,
-0.009967f, -0.014530f, -0.003931f, -0.000000f, -0.008949f, -0.009423f,
0.014759f, 0.040768f,  0.026189f,  -0.032055f, -0.075718f, -0.043476f,
0.048781f, 0.107312f,  0.057990f,  -0.061683f, -0.129299f, -0.066819f,
0.068142f, 0.137173f,  0.068142f,  -0.066819f, -0.129299f, -0.061683f,
0.057990f, 0.107312f,  0.048781f,  -0.043476f, -0.075718f, -0.032055f,
0.026189f, 0.040768f,  0.014759f,  -0.009423f, -0.008949f, -0.000000f,
-0.003931f, -0.014530f, -0.009967f, 0.012019f,  0.026845f,  0.014195f,
-0.014370f, -0.027991f, -0.013133f, 0.011854f,  0.020471f,  0.008362f,
-0.006318f, -0.008380f}; static const float32_t
firCoeffs32[NUM_TAPS_ARRAY_SIZE] = { 0.000000f,  -0.000102f, -0.000330f,
-0.000175f, 0.000122f,  -0.000000f, -0.000191f, 0.000446f,  0.001509f,
0.001099f,  -0.001459f, -0.003616f, -0.002117f, 0.002355f,  0.004985f,
0.002501f,  -0.002357f, -0.004081f, -0.001542f, 0.000859f,  -0.000000f,
-0.001015f, 0.002154f,  0.006749f, 0.004623f,  -0.005837f, -0.013888f,
-0.007868f, 0.008531f,  0.017703f, 0.008755f,  -0.008172f, -0.014085f,
-0.005321f, 0.002978f,  -0.000000f, -0.003603f, 0.007802f,  0.025089f,
0.017757f,  -0.023347f, -0.058400f, -0.035189f, 0.041174f,  0.094010f,
0.052530f,  -0.057598f, -0.124125f, -0.065790f, 0.068661f,  0.141154f,
0.068661f,  -0.065790f, -0.124125f, -0.057598f, 0.052530f,  0.094010f,
0.041174f,  -0.035189f, -0.058400f, -0.023347f, 0.017757f,  0.025089f,
0.007802f,  -0.003603f, -0.000000f, 0.002978f,  -0.005321f, -0.014085f,
-0.008172f, 0.008755f,  0.017703f, 0.008531f,  -0.007868f, -0.013888f,
-0.005837f, 0.004623f,  0.006749f, 0.002154f,  -0.001015f, -0.000000f,
0.000859f,  -0.001542f, -0.004081f, -0.002357f, 0.002501f,  0.004985f,
0.002355f,  -0.002117f, -0.003616f, -0.001459f, 0.001099f,  0.001509f,
0.000446f,  -0.000191f, -0.000000f, 0.000122f,  -0.000175f, -0.000330f,
-0.000102f, 0.000000f};

static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] =
    { // Czebyszew, 101, 3kHz width, bardzo silnie obcina (ponizej -100 dB)
        0.000002f,  -0.000004f, -0.000012f, -0.000009f, 0.000012f,  0.000028f,
        0.000013f,  -0.000007f, 0.000011f,  0.000029f,  -0.000065f, -0.000233f,
        -0.000184f, 0.000267f,  0.000724f,  0.000461f,  -0.000553f, -0.001246f,
        -0.000652f, 0.000617f,  0.000991f,  0.000264f,  0.000098f,  0.001203f,
        0.001251f,  -0.002035f, -0.005851f, -0.003877f, 0.004822f,  0.011352f,
        0.006334f,  -0.006680f, -0.013183f, -0.005944f, 0.004623f,  0.005070f,
        -0.000389f, 0.004177f,  0.017636f,  0.014248f,  -0.020357f, -0.053963f,
        -0.033920f, 0.040932f,  0.095513f,  0.054123f,  -0.059765f, -0.128878f,
        -0.067938f, 0.070104f,  0.141675f,  0.070104f,  -0.067938f, -0.128878f,
        -0.059765f, 0.054123f,  0.095513f,  0.040932f,  -0.033920f, -0.053963f,
        -0.020357f, 0.014248f,  0.017636f,  0.004177f,  -0.000389f, 0.005070f,
        0.004623f,  -0.005944f, -0.013183f, -0.006680f, 0.006334f,  0.011352f,
        0.004822f,  -0.003877f, -0.005851f, -0.002035f, 0.001251f,  0.001203f,
        0.000098f,  0.000264f,  0.000991f,  0.000617f,  -0.000652f, -0.001246f,
        -0.000553f, 0.000461f,  0.000724f,  0.000267f,  -0.000184f, -0.000233f,
        -0.000065f, 0.000029f,  0.000011f,  -0.000007f, 0.000013f,  0.000028f,
        0.000012f,  -0.000009f, -0.000012f, -0.000004f, 0.000002f};
*/

static const float32_t firCoeffs32[NUM_TAPS_ARRAY_SIZE] = {
    0.013841f,  -0.013403f, -0.026630f, -0.012246f, 0.009481f,  0.012862f,
    0.002252f,  0.002807f,  0.017553f,  0.015587f,  -0.022095f, -0.059555f,
    -0.037635f, 0.044112f,  0.102915f,  0.058158f,  -0.062931f, -0.135504f,
    -0.071261f, 0.073017f,  0.147602f,  0.073017f,  -0.071261f, -0.135504f,
    -0.062931f, 0.058158f,  0.102915f,  0.044112f,  -0.037635f, -0.059555f,
    -0.022095f, 0.015587f,  0.017553f,  0.002807f,  0.002252f,  0.012862f,
    0.009481f,  -0.012246f, -0.026630f, -0.013403f, 0.013841f,
};

/*
float32_t lpFIRCoeffs[COSTAS_LPF_ORDER] = {0.117181f, 0.142181f, 0.158530f,
                                       0.164215f, 0.158530f, 0.142181f,
                                       0.117181f}; */

static const float32_t lpFIRCoeffs[COSTAS_LPF_ORDER] = {
    0.072255f, 0.074308f, 0.075000f, 0.074308f, 0.072255f};

static q15_t temp_[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
static float32_t temp_f32[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
static volatile uint16_t time = 0;
static uint16_t start, end;
// Matched filter and pattern symbol
static float32_t matchedCoeffs[FSPAN * SPB + 1];

// Peripherals structs
static ADC_initStruct adc;
static TIMER_initStruct tim2;
static ADC_channel chan0;
static UART_initStruct uart2;

// Data storage
static volatile uint16_t dmaBuffer[SAMPLES * 2];
static float32_t buffer_filtered_f32[2 * SAMPLES];
static float32_t buffer_input_f32[2 * SAMPLES];
static volatile uint8_t dataReady;

float32_t buffer_LP_costas_I[COSTAS_LPF_ORDER],
    buffer_LP_costas_Q[COSTAS_LPF_ORDER];

int main() {
  /// Peripherals initialization ///
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
  tim2.autoReload = FSystem / FS;

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
  // dma_streamITEnable(DMA2_Stream4, DMA_IT_HALF_TRANSFER);
  dma_streamITEnable(DMA2_Stream4, DMA_IT_TRANSFER_COMPLETE);

  /// Demodulator system components initialization ///
  SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, matchedCoeffs,
                    FSPAN * SPB + 1);

  FIR_filter filterI =
      FIR_filter_init(lpFIRCoeffs, COSTAS_LPF_ORDER, buffer_LP_costas_I);
  FIR_filter filterQ =
      FIR_filter_init(lpFIRCoeffs, COSTAS_LPF_ORDER, buffer_LP_costas_Q);

  costasLoop_parameters costas = {.Kp = 0.25f,
                                  .Ki = 0.25f * 0.25f / 4.0f,
                                  .LP_filterI = &filterI,
                                  .LP_filterQ = &filterQ};

  gardnerTimingRecovery_parameters gardner = {.loop_gain = 0.35,
                                              .max_error = 10};

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

      arm_conv_f32(buffer_input_f32, 2 * SAMPLES, firCoeffs32,
                   NUM_TAPS_ARRAY_SIZE, temp_f32);
      arm_copy_f32(temp_f32 + NUM_TAPS_ARRAY_SIZE / 2, buffer_filtered_f32,
                   2 * SAMPLES);
      float32_t maxVal;
      uint16_t maxIdx;
      arm_max_f32(buffer_filtered_f32, 2 * SAMPLES, &maxVal, &maxIdx);
#ifdef DEBUG
      uart_sendString(&uart2, "\r\n\r\n Buffer after filter: \r\n ");
#endif
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        buffer_filtered_f32[i] = buffer_filtered_f32[i] / maxVal * 1.5f;
#ifdef DEBUG
        sprintf(buf, "%f \r\n", buffer_filtered_f32[i]);
        uart_sendString(&uart2, buf);
#endif
      }

      BPSK_carrierRecovery(&BPSK_params, buffer_filtered_f32,
                           2 * SAMPLES); // to dużo czasu!

      float32_t temp[2 * SAMPLES + BPSK_params.matchedFilterCoeffsLength - 1];

      arm_conv_f32(buffer_filtered_f32, 2 * SAMPLES,
                   BPSK_params.matchedFilterCoeffs,
                   BPSK_params.matchedFilterCoeffsLength, temp);

      arm_max_f32(temp, 2 * SAMPLES + BPSK_params.matchedFilterCoeffsLength - 1,
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
      int8_t buffer_[2 * SAMPLES / SPB];

      BPSK_timingRecovery(&BPSK_params,
                          temp +
                              BPSK_params.samplesPerBit * BPSK_params.FSpan / 2,
                          2 * SAMPLES, buffer_, 2 * SAMPLES / SPB);

      uint16_t idx[2 * SAMPLES / 312];
      BPSK_findSymbolsStarts_decimated(&BPSK_params, buffer_, 2 * SAMPLES / SPB,
                                       idx, &foundFrames);
#ifdef DEBUG
      uart_sendString(&uart2, "\r\n\r\n Decode : \r\n ");
      for (uint16_t i = 0; i < 2 * SAMPLES / SPB; ++i) {
        sprintf(buf, "%d \r\n", buffer_[i]);
        uart_sendString(&uart2, buf);
      }
#endif
      uint8_t out_data[foundFrames];

      BPSK_demodulateSignal_decimated(&BPSK_params, buffer_, 2 * SAMPLES / SPB,
                                      idx, foundFrames, out_data, foundFrames);
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
      sprintf(buf, "\r\n %ld %d %d\r\n", time, correct, foundFrames);
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
      //  uart_sendString(&uart2, "\r\n Buffer ADC: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        // buffer_input[i] = dmaBuffer[i];
        buffer_input_f32[i] = (float32_t)dmaBuffer[i] / 4096.0f * 3.3f;
        //   sprintf(buf, "%f \r\n", buffer_input_f32[i]);
        //   uart_sendString(&uart2, buf);
      }
      dataReady = 0x1;
      start = time;
    }
  }
}

void SysTick_Handler(void) { time++; }
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
#define COSTAS_LPF_ORDER 7

#define NUM_TAPS_ARRAY_SIZE 101

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
*/
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

static q15_t firCoeffs_q15[NUM_TAPS_ARRAY_SIZE];
static q15_t temp_[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];
static float32_t temp_f32[2 * SAMPLES + NUM_TAPS_ARRAY_SIZE - 1];

// Matched filter and pattern symbol
static float32_t matchedCoeffs[FSPAN * SPB + 1];

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
static float32_t buffer_input_f32[2 * SAMPLES];
static volatile uint8_t dataReady;

uint8_t data[2 * SAMPLES / 312];
uint16_t idx[2 * SAMPLES / 312];
uint16_t foundFrames;

float32_t buffer_LP_costas_I[COSTAS_LPF_ORDER],
    buffer_LP_costas_Q[COSTAS_LPF_ORDER];
float32_t lpFIRCoeffs[COSTAS_LPF_ORDER] = {0.117181f, 0.142181f, 0.158530f,
                                           0.164215f, 0.158530f, 0.142181f,
                                           0.117181f};

int main() {
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

  // Demodulator system components initialization
  SRRC_getFIRCoeffs(FSPAN, SPB, ROLLOVER_FACTOR, matchedCoeffs,
                    FSPAN * SPB + 1);

  FIR_filter filterI =
      FIR_filter_init(matchedCoeffs, COSTAS_LPF_ORDER, buffer_LP_costas_I);
  FIR_filter filterQ =
      FIR_filter_init(matchedCoeffs, COSTAS_LPF_ORDER, buffer_LP_costas_Q);

  costasLoop_parameters costas = {.alpha = 0.2f,
                                  .beta = 0.2f * 0.2f / 4.0f,
                                  .LP_filterI = &filterI,
                                  .LP_filterQ = &filterQ};

  gardnerTimingRecovery_parameters gardner = {.loop_gain = 0.35,
                                              .max_error = 10};

  int8_t barker[5] = {1, 1, 1, -1, 1};
  float32_t preamble[5 * SPB];

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

  arm_float_to_q15(firCoeffs32, firCoeffs_q15, NUM_TAPS_ARRAY_SIZE);

  timer_start(&tim2);

  char buf[40];
  while (1) {
    if (dataReady) {
      /*  arm_conv_fast_q15(buffer_input, 2 * SAMPLES, firCoeffs_q15,
                          NUM_TAPS_ARRAY_SIZE, temp_);
        arm_copy_q15(temp_ + NUM_TAPS_ARRAY_SIZE / 2, buffer_filtered,
                     2 * SAMPLES);

        arm_q15_to_float(buffer_filtered, buffer_filtered_f32, 2 * SAMPLES); */

      arm_conv_f32(buffer_input_f32, 2 * SAMPLES, firCoeffs32,
                   NUM_TAPS_ARRAY_SIZE, temp_f32);
      arm_copy_f32(temp_f32 + NUM_TAPS_ARRAY_SIZE / 2, buffer_filtered_f32,
                   2 * SAMPLES);
      float32_t maxVal;
      arm_max_f32(buffer_filtered_f32, 2 * SAMPLES, &maxVal, &idx);
      uart_sendString(&uart2, "\r\n\r\n Buffer after filter: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        buffer_filtered_f32[i] = buffer_filtered_f32[i] / maxVal * 1.5f;
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
        sprintf(buf, "%d %c%c%c%c%c%c%c%c \r\n", out_data[i],
                BYTE_TO_BINARY(out_data[i]));
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
      uart_sendString(&uart2, "\r\n Buffer ADC: \r\n");
      for (uint16_t i = 0; i < 2 * SAMPLES; i++) {
        buffer_input[i] = dmaBuffer[i];
        buffer_input_f32[i] = (float32_t)dmaBuffer[i] / 4096.0f * 3.3f;
        sprintf(buf, "%f \r\n", buffer_input_f32[i]);
        uart_sendString(&uart2, buf);
      }
      dataReady = 0x1;
    }
  }
}

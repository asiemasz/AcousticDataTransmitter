#ifndef TIMER_H
#define TIMER_H

#include <stm32f401xe.h>
#include <assert.h>
#include <stdint.h>

enum TIMER_COUNTER_DIRECTION {
    TIMER_COUNTER_DIRECTION_UP,
    TIMER_COUNTER_DIRECTION_DOWN
};

enum TIMER_IT {
    TIMER_IT_UPDATE_EVENT,
    TIMER_IT_CC1,
    TIMER_IT_CC2,
    TIMER_IT_CC3,
    TIMER_IT_CC4,
    TIMER_IT_TRGOUT = 6
};

enum TIMER_DMA_REQUEST {
    TIMER_DMA_REQUEST_UPDATE_EVENT,
    TIMER_DMA_REQUEST_CC1,
    TIMER_DMA_REQUEST_CC2,
    TIMER_DMA_REQUEST_CC3,
    TIMER_DMA_REQUEST_CC4,
    TIMER_DMA_REQUEST_TRGOUT = 6
};

enum TIMER_TRGOUT_EVENT {
    TIMER_TRGOUT_EVENT_RESET,
    TIMER_TRGOUT_EVENT_ENABLE,
    TIMER_TRGOUT_EVENT_UPDATE,
    TIMER_TRGOUT_EVENT_COMPARE_PULSE,
    TIMER_TRGOUT_EVENT_COMPARE_OC1REF,
    TIMER_TRGOUT_EVENT_COMPARE_OC2REF,
    TIMER_TRGOUT_EVENT_COMPARE_OC3REF,
    TIMER_TRGOUT_EVENT_COMPARE_OC4REF,
};

typedef struct TIMER_initStruct {
    TIM_TypeDef* tim;
    uint16_t prescaler;
    uint32_t autoReload;
    enum TIMER_COUNTER_DIRECTION direction;
} TIMER_initStruct;

void timer_init(TIMER_initStruct* init);

void timer_start(TIMER_initStruct* init);

void timer_clearITflag(TIMER_initStruct* init);

void timer_enableIT(TIMER_initStruct* init, uint16_t its);

void timer_enableDMA(TIMER_initStruct* init, uint16_t dmas);

void timer_selectTRGOUTEvent(TIMER_initStruct* init, enum TIMER_TRGOUT_EVENT event);

uint32_t timer_getCounterVal(TIMER_initStruct* init);

#endif

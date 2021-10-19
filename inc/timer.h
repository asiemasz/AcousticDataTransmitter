#ifndef TIMER_H
#define TIMER_H

#include <stm32f401xe.h>
#include <assert.h>

enum TIMER_COUNTER_DIRECTION {
    TIMER_COUNTER_DIRECTION_UP,
    TIMER_COUNTER_DIRECTION_DOWN
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

uint32_t timer_getCounterVal(TIMER_initStruct* init);

#endif

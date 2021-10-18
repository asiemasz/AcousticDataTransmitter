#ifndef TIMER_H
#define TIMER_H

#include <stm32f4xx.h>

enum TIMER_COUNTER_DIRECTION {
    TIMER_COUNTER_DIRECTION_UPCOUNTER,
    TIMER_COUNTER_DIRECTION_DOWNCOUNTER
};

typedef struct TIMER_initStruct {
    TIM_TypeDef* tim;
    uint32_t counter;
    uint16_t prescaler;
    uint32_t autoReload;
    enum TIMER_COUNTER_DIRECTION direction;
} TIMER_initStruct;

void timer_init(TIMER_initStruct init);

#endif

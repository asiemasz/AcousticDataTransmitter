#include "../inc/timer.h"

void timer_init(TIMER_initStruct* init) {
		assert(init->prescaler > 0);

    if(init->tim == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    } else if (init->tim == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    } else if (init->tim == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    } else if (init->tim == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    } else if (init->tim == TIM5) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    } else if (init->tim == TIM9) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    } else if (init->tim == TIM10) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
    } else if (init->tim == TIM11) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
    } else {
        assert(0);
    }

	//set counter direction
    init->tim->CR1 |= (uint32_t) init->direction << TIM_CR1_DIR_Pos;
		
	//set the timer clock prescaller
    init->tim->PSC = init->prescaler - (uint8_t) 1;

    //set auto reload value
    init->tim->ARR = init->autoReload - (uint8_t) 1;
}

void timer_start(TIMER_initStruct* init) {
	init->tim->CR1 |= TIM_CR1_CEN;
	init->tim->SR = 0x0;
}

void timer_stop(TIMER_initStruct* init) {
	init->tim->CR1 &= ~TIM_CR1_CEN;
}

void timer_clearITflag(TIMER_initStruct* init) {
	init->tim->SR = 0x0;
	//while(init->tim->SR);
}

void timer_enableIT(TIMER_initStruct* init, uint16_t its) {
	init->tim->DIER |= its;
}

void timer_enableDMA_IT(TIMER_initStruct* init, uint32_t dmas) {
    timer_enableIT(init, dmas);
}

void timer_selectTRGOUTEvent(TIMER_initStruct* init, enum TIMER_TRGOUT_EVENT event) {
    init->tim->CR2 |= ((uint16_t) event << TIM_CR2_MMS_Pos);
}

uint32_t timer_getCounterVal(TIMER_initStruct* init) {
    return init->tim->CNT;
}

uint8_t timer_getITFlag(TIMER_initStruct* init, enum TIMER_IT_FLAG flag) {
	return ((init->tim->SR & (1U << flag)) > 0);
}

void timer_setReloadVal(TIMER_initStruct* init, uint32_t reload) {
    timer_stop(init);
    init->tim->ARR = reload;
    timer_start(init);
}


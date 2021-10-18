#include "../inc/timer.h"

void timer_init(TIMER_initStruct* init) {
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
    init->tim->PSC = init->prescaler;

    //set auto reload value
    init->tim->ARR = init->autoReload;
}

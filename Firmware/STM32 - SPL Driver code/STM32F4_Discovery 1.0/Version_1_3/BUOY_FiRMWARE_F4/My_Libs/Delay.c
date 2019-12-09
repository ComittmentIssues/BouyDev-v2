/*
 * Delay.c
 *
 *  Created on: Jun 16, 2019
 *      Author: Jamie
 */

#include "Delay.h"

void init_Delay(void)
{
	RCC_DelayPeriphClockCmd(RCC_Delay_Periph,ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure; //create a 1ms delaay

	timerInitStructure.TIM_Prescaler = PSC_Base;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period= 4999;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(Delay_Timer, &timerInitStructure);

	/* Prevent interrupt from triggering*/
	TIM_ClearITPendingBit(Delay_Timer,TIM_IT_Update);
	TIM_UpdateRequestConfig(TIM2,TIM_UpdateSource_Regular);
	TIM_ITConfig(Delay_Timer, TIM_IT_Update,ENABLE);

	NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = Delay_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);
	timeout = 0;
	//enable interrupts
}

void Delay_begin_Timeout(uint32_t time)//time in ms
{
	//clear any active flags
	timeout = 0;
	float ms = (float)time/1000;
	//set prescaler to 100
	uint32_t arr_value = ms*clock_frequency/(PSC_Base+1) -1;
	Delay_Timer->ARR = arr_value;
	TIM_Cmd(Delay_Timer, ENABLE);
	TIM_ITConfig(Delay_Timer,TIM_IT_Update,ENABLE);
}
void Delay_IRQHandler(void)
{
	//set timeout flag
	timeout = 1;
	//disable timer
	TIM_ClearITPendingBit(Delay_Timer, TIM_IT_Update);
	Delay_Disable();
}

void Delay_Disable(void)
{
	//turn off timer
	TIM_ITConfig(Delay_Timer,TIM_IT_Update,DISABLE);
	TIM_Cmd(Delay_Timer, DISABLE);
	//clear counter
	assert_param(IS_TIM_ALL_PERIPH(Delay_Timer));
	Delay_Timer->CNT = 0;

}


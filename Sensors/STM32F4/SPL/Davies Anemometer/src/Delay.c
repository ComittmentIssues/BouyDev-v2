/*
 * Delay.c
 *
 *  Created on: Jun 16, 2019
 *      Author: Jamie
 */

#include "Delay.h"

RCC_ClocksTypeDef rcc;

void init_Delay(void)
{
	RCC_DelayPeriphClockCmd(RCC_Delay_Periph,ENABLE);


	RCC_GetClocksFreq(&rcc);
	uint32_t psc = 2*rcc.PCLK2_Frequency/(65536);
	uint32_t arr = 2*rcc.PCLK2_Frequency/(psc+1);
	TIM_TimeBaseInitTypeDef timerInitStructure; //create a 1ms delaay

	timerInitStructure.TIM_Prescaler = psc;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period= arr;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;

	/* Prevent interrupt from triggering*/
	TIM_ClearITPendingBit(Delay_Timer,TIM_IT_Update);
	TIM_UpdateRequestConfig(Delay_Timer,TIM_UpdateSource_Regular);
	TIM_ITConfig(Delay_Timer, TIM_IT_Update,ENABLE);

	TIM_TimeBaseInit(Delay_Timer, &timerInitStructure);
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
	float seconds = (float)time/1000;
	//set prescaler to 100
	uint32_t arr_value = 4*seconds*rcc.PCLK2_Frequency/(Delay_Timer->PSC +1) -1;
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


/*
 * Delay.h
 *
 *  Created on: Jun 16, 2019
 *      Author: Jamie
 */

#include "stm32f4xx.h"
#include "stm32f4xx_tim.h"
#define Delay_Timer TIM2
#define RCC_DelayPeriphClockCmd RCC_APB1PeriphClockCmd
#define RCC_Delay_Periph RCC_APB1Periph_TIM2
#define Delay_IRQHandler TIM2_IRQHandler
#define Delay_IRQn  TIM2_IRQn
#define clock_frequency 84000000
#define PSC_Base 4200
uint32_t ticks;      //updated by irq handler
uint32_t timeout;    //set by user
uint32_t delay;      //set by user
uint8_t status_flag; // 0 - nothing, 1 - timeout, 2 - delay
uint8_t delay_flag;
#ifndef DELAY_H_
#define DELAY_H_

void Delay_Disable(void);
void Delay_Enable(void);
void Delay_begin_Timeout(uint32_t time);
void init_Delay(void);
#endif /* DELAY_H_ */

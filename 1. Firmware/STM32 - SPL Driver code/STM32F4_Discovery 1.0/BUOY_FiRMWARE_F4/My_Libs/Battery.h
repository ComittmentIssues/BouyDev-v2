/*
 * Battery.h
 *
 *  Created on: Jul 8, 2019
 *      Author: Jamie
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#define VBAT_PIN GPIO_Pin_1
#define VBAT_CTRLPIN GPIO_Pin_2
#define VBAT_PINSource GPIO_PinSource1
#define VBAT_GPIO GPIOA
#define VBAT_ADC ADC1
#define VBAT_ADCChannel ADC_Channel_1
#define RCC_GPIOPeriph RCC_AHB1Periph_GPIOA
#define RCC_ADCPeriph RCC_APB2Periph_ADC1
#define RCC_GPIO_PeriphEnable RCC_AHB1PeriphClockCmd
#define RCC_ADC_PeriphEnable  RCC_APB2PeriphClockCmd
#endif /* BATTERY_H_ */

uint8_t init_Battery_ADC(void);
void deinit_Battery_ADC(void);
uint8_t Sample_ADC(void);

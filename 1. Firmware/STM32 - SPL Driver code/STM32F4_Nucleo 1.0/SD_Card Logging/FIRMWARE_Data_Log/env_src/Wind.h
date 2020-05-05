/*
 * Wind.h
 *
 *  Created on: Oct 7, 2019
 *      Author: jamie
 */
#include <stdint.h>
#include "stm32f4xx.h"
#ifndef WIND_H_
#define WIND_H_

#define USE_RES_12Bit

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
	float windspeed;
	float winddir;
}Wind_t;

/* Private define ------------------------------------------------------------*/
#define ADC_AN ADC1
#define ADC_AN_Channel ADC_Channel_0
#define RCC_ADCPeriph RCC_APB2Periph_ADC1
#define GPIO_AN GPIOA
#define Weather_Vane_Pin GPIO_Pin_0
#define RCC_WVPeriph RCC_AHB1Periph_GPIOA

#define DIR_OFFSET 0
#define SPEED_OFFSET 0
#ifdef USE_RES_12Bit
#define MAX_VAL 4095
#endif
#ifdef USE_RES_10Bit
#define MAX_VAL 1023
#endif

#ifdef USE_RES_8Bit
#define MAX_VAL 255
#endif
// Wind SPeed Macros
#define TIM_AN TIM2
#define TIM_AN_Channel TIM_Channel_2
#define Wind_Speed_Pin GPIO_Pin_1
#define PSC 5
#define W_ARR 55999
#define IC_F 250
#define WIND_SPEED_MIN 0.5
#define WIND_SPEED_MAX 89
#define WIND_SPEED_FMin 1
#define WIND_SPEED_FMax 112
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

float windspeed;
float winddir;
/* Private function prototypes -----------------------------------------------*/
void init_ADC(ADC_TypeDef* ADCx,uint32_t pin);
uint16_t get_Wind_Direction(void);
uint16_t get_ADC_Val(void);
uint16_t get_Wind_Speed(void);
void init_InputCapture(uint32_t pin);


#endif /* WIND_H_ */

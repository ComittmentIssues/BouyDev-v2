/*
 * Wind.c
 *
 *  Created on: Oct 7, 2019
 *      Author: jamie
 */

#include "Wind.h"


/* Private variables ---------------------------------------------------------*/

static uint32_t capture1 = -1; //no value
static uint32_t capture2 = -1;

/* Private functions ---------------------------------------------------------*/
void init_ADC(ADC_TypeDef* ADCx,uint32_t pin)
{
	//enable peripherals
	RCC_APB2PeriphClockCmd(RCC_ADCPeriph, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_GPIOPeriph,ENABLE);

	GPIO_InitTypeDef GPIO_Initstruct;
	GPIO_Initstruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Initstruct.GPIO_Pin = pin;
	GPIO_Initstruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIO_AN,&GPIO_Initstruct);
	//ADC initialisation
	ADC_InitTypeDef ADC_Initstruct;
	ADC_Initstruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_Initstruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_Initstruct.ADC_ContinuousConvMode = DISABLE;
	ADC_Initstruct.ADC_NbrOfConversion = 1;
	ADC_Initstruct.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
	ADC_Initstruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADCx,&ADC_Initstruct);
	ADC_Cmd(ADCx,ENABLE);
	//channel config
	ADC_RegularChannelConfig(ADC_AN,ADC_AN_Channel,1,ADC_SampleTime_84Cycles);
}

uint16_t get_ADC_Val(void)
{
	//start conversion
	ADC_SoftwareStartConv(ADC_AN);
	//wait for completion
	while(!ADC_GetFlagStatus(ADC_AN, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC_AN);
}

float get_Wind_Direction(void)
{
	uint16_t ad_val = get_ADC_Val() >>4;
	return (((float)ad_val)/MAX_VAL)*360 + DIR_OFFSET;
}

void init_InputCapture(uint32_t pin)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_GPIOPeriph,ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Pin = pin;
	GPIO_Init(GPIO_AN,&GPIO_InitStruct);
	//AF Mapping
	GPIO_PinAFConfig(GPIO_AN,GPIO_PinSource1,GPIO_AF_TIM2);
	//init base clock with a counter frequency of 200Hz

	//configure input capture parameters
	TIM_ICInitTypeDef TIM_ICInitStruct;
	TIM_ICInitStruct.TIM_Channel = TIM_AN_Channel;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM_AN,&TIM_ICInitStruct);
	TIM_CCxCmd(TIM_AN,TIM_Channel_2,TIM_CCx_Enable);
	TIM_Cmd(TIM_AN,ENABLE);
	//enable interrupt
	TIM_ITConfig(TIM_AN,TIM_IT_CC2,ENABLE);
	TIM_ClearITPendingBit(TIM_AN,TIM_IT_CC2);

	NVIC_InitTypeDef NVIC_Initstruct;
	NVIC_Initstruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_Initstruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Initstruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Initstruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_Initstruct);


}

void TIM2_IRQHandler()
{

	if(capture1 == -1)
	{
		capture1 = TIM_GetCapture2(TIM_AN);
	}else
	{
		capture2 = TIM_GetCapture2(TIM_AN);
		uint32_t clks;
		//compute elapsed clock cycles
		if(capture2 > capture1)
		{
			clks = capture2 -capture1;
		}
		else if(capture2 < capture1)
		{
			clks = ((0xFFFF-capture1) + capture2);
		}else
		{
			clks = 0;
		}
		//calculate frequency
		RCC_ClocksTypeDef rcc;
		RCC_GetClocksFreq(&rcc);
		float frequency = (float)rcc.PCLK1_Frequency/clks;
		//calculate revolutions per second
		float rph = frequency*3600; //convert to rph 1600 rph = 1 mph

		windspeed = (rph/1600) +SPEED_OFFSET;
		//reset capture values
		capture1 = -1;
	}

	//clear pending bit
	TIM_ClearITPendingBit(TIM_AN,TIM_IT_CC2);
}

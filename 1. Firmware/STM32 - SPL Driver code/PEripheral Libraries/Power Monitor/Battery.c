/*
 * Battery.c
 *
 *  Created on: Jul 8, 2019
 *      Author: Jamie
 */


#include "Battery.h"

uint8_t init_Battery_ADC(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	ADC_InitTypeDef ADC_InitStruct;

	RCC_GPIO_PeriphEnable(RCC_GPIOPeriph,ENABLE);
	RCC_ADC_PeriphEnable(RCC_ADCPeriph,ENABLE);

	/* Init GPIO Pin to Analog input*/
	GPIO_InitStruct.GPIO_Pin = VBAT_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(VBAT_GPIO,&GPIO_InitStruct);

	/* Configure ADC */
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_Init(VBAT_ADC, &ADC_InitStruct);

	//Channel select
	ADC_Cmd(VBAT_ADC, ENABLE);
	ADC_RegularChannelConfig(VBAT_ADC,VBAT_ADCChannel,1,ADC_SampleTime_144Cycles);

	return 0;
}

void deinit_Battery_ADC(void)
{
	ADC_DeInit();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_StructInit(&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = VBAT_PIN;
	GPIO_Init(VBAT_GPIO,&GPIO_InitStruct);
	//Disable clock
	RCC_GPIO_PeriphEnable(RCC_GPIOPeriph,DISABLE);
	RCC_ADC_PeriphEnable(RCC_ADCPeriph,DISABLE);
}
uint8_t Sample_ADC(void)
{
	ADC_SoftwareStartConv(VBAT_ADC);
	while(ADC_GetFlagStatus(VBAT_ADC,ADC_FLAG_EOC) == 0);
	return ADC_GetConversionValue(VBAT_ADC);

}

/**
  ******************************************************************************
  * @file    Anemometer Test
  * @author  Jamie Jacobson
  * @version V1.0.1
  * @date    30-08-2019
  * @brief   Main program body
  ******************************************************************************
  *This project tests the code developed for the Vanatage Pro 2 Weather Vane
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "math.h"
#define USE_RES_12Bit

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define ADC_AN ADC1
#define ADC_AN_Channel ADC_Channel_0
#define RCC_ADCPeriph RCC_APB2Periph_ADC1
#define GPIO_AN GPIOA
#define Weather_Vane_Pin GPIO_Pin_0
#define RCC_GPIOPeriph RCC_AHB1Periph_GPIOA


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
#define ARR 55999
#define IC_F 250
#define WIND_SPEED_MIN 0.5
#define WIND_SPEED_MAX 89
#define WIND_SPEED_FMin 1
#define WIND_SPEED_FMax 112
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t capture1 = -1; //no value
static uint32_t capture2 = -1;
float windspeed;
/* Private function prototypes -----------------------------------------------*/
void init_ADC(ADC_TypeDef* ADCx,uint32_t pin);
float get_Wind_Direction(void);
uint16_t get_ADC_Val(void);

void init_InputCapture(uint32_t pin);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	//TODO: User code
  init_ADC(ADC_AN,Weather_Vane_Pin);
  init_InputCapture(Wind_Speed_Pin);
  float winddir;

  while (1)
  {
	  winddir = get_Wind_Direction();

  }
}

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
	uint16_t ad_val = get_ADC_Val();
	return (((float)ad_val)/MAX_VAL)*360;
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
		uint32_t frequency = SystemCoreClock/clks;
		windspeed = 0.1*frequency +0.5;
		//reset capture values
		capture1 = -1;
	}

	//clear pending bit
	TIM_ClearITPendingBit(TIM_AN,TIM_IT_CC2);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************END OF FILE****************************/

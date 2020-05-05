/*
 * IMU.c
 *
 *  Created on: Jul 13, 2019
 *      Author: Jamie
 */

#include "IMU.h"
#include "eeprom.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
/* Private Variables*/

TM_MPU6050_t MPU6050_Data0;
TM_LIS302DL_LIS3DSH_t LIS3DSH_Data0;
int16_t count = 0;

uint16_t VirtAddVarTab[6] = {0x01,0x0961,0x12C1,0x2581,0x2EE1,0x3841};

/* Private Function Definitions*/
void init_Timer(void)
{

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure; //create a 1ms delay
		RCC_ClocksTypeDef rcc;
		RCC_GetClocksFreq(&rcc);

		timerInitStructure.TIM_Prescaler = Sample_PSC;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period= rcc.HCLK_Frequency/(2*SAMPLE_RATE*Sample_PSC);
		timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timerInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(Sample_Timer, &timerInitStructure);

		/* Prevent interrupt from triggering*/
		TIM_ClearITPendingBit(Sample_Timer,TIM_IT_Update);
		TIM_UpdateRequestConfig(TIM2,TIM_UpdateSource_Regular);
		TIM_ITConfig(Sample_Timer, TIM_IT_Update,ENABLE);

		NVIC_InitTypeDef nvicStructure;
	    nvicStructure.NVIC_IRQChannel = TIM7_IRQn;
		nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
		nvicStructure.NVIC_IRQChannelSubPriority = 1;
		nvicStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvicStructure);
		TIM_Cmd(Sample_Timer,ENABLE);
		//enable interrupts

}

uint8_t init_IMU(void)
{
	uint8_t flag = 0;
#ifdef USE_EXT_IMU
	flag = (TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_250s) == TM_MPU6050_Result_Ok);
	if(flag)
	{
		STM_EVAL_LEDOn(LED3);
	}else
	{
		STM_EVAL_LEDOn(LED4);
	}

	#else
	if(TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS302DL)
	{
		TM_LIS302DL_LIS3DSH_Init(TM_LIS302DL_Sensitivity_2_3G, TM_LIS302DL_Filter_2Hz);
		flag = 1;
		STM_EVAL_LEDOn(LED4);
	}if(TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS3DSH)
	{
		TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_4G, TM_LIS3DSH_Filter_50Hz);
		STM_EVAL_LEDOn(LED5);
		flag = 1;
	}else
	{
		flag = 0;

	}
#endif
	return flag;
}

void read_IMU_Data(void)
{
#ifdef USE_EXT_IMU
	TM_MPU6050_ReadAccelerometer(&MPU6050_Data0);
	TM_MPU6050_ReadGyroscope(&MPU6050_Data0);
#else
	TM_LIS302DL_LIS3DSH_ReadAxes(&LIS3DSH_Data0);
#endif

}
void deinit_Timer(void)
{
	TIM_Cmd(Sample_Timer,DISABLE);
	TIM_ITConfig(Sample_Timer, TIM_IT_Update,DISABLE);
	TIM_DeInit(TIM7);
	NVIC_DisableIRQ(TIM7_IRQn);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,DISABLE);
}
/****** IRQ HANDLER **************/
void TIM7_IRQHandler(void)
{
	STM_EVAL_LEDToggle(LED6);
	read_IMU_Data();
	/* Store Data in Flash */
#ifdef USE_EXT_IMU
	EE_WriteVariable(VirtAddVarTab[0],MPU6050_Data0.Accelerometer_X);
	EE_WriteVariable(VirtAddVarTab[1],MPU6050_Data0.Accelerometer_Y);
	EE_WriteVariable(VirtAddVarTab[2],MPU6050_Data0.Accelerometer_Z);
	EE_WriteVariable(VirtAddVarTab[3],MPU6050_Data0.Gyroscope_X);
	EE_WriteVariable(VirtAddVarTab[4],MPU6050_Data0.Gyroscope_Y);
#elif
	EE_WriteVariable(VirtAddVarTab[0],LIS3DSH_Data0.X);
	EE_WriteVariable(VirtAddVarTab[1],LIS3DSH_Data0.Y);
	EE_WriteVariable(VirtAddVarTab[2],LIS3DSH_Data0.Z);
#endif
	/* End */

	if(count == __numSamples())
	{
		sample_finished = 1;
		deinit_Timer();
	}else
	{
		count++;
	}
	TIM_ClearITPendingBit(Sample_Timer,TIM_IT_Update);

}

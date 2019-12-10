/*
 * IMU.c
 *
 *  Created on: Jul 13, 2019
 *      Author: Jamie
 */

#include "IMU.h"
#include "eeprom.h"
#include "stm32f4xx.h"

/* Private Variables*/

TM_MPU6050_t MPU6050_Data0;
int16_t count = 0;
uint16_t VirtIMUAdd[6] = {0x04,0x0964,0x12C4,0x1C24,0x2584,0x2EE4};
uint16_t VirtAddVarTab;
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
	flag = (TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_250s) == TM_MPU6050_Result_Ok);

	return flag;
}

void deinit_IMU(void)
{
	I2C_DeInit(I2C1);
	GPIO_DeInit(GPIOB);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,DISABLE);

}
void read_IMU_Data(void)
{
	TM_MPU6050_ReadAccelerometer(&MPU6050_Data0);
	TM_MPU6050_ReadGyroscope(&MPU6050_Data0);
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
	read_IMU_Data();
	/* Store Data in Flash */

	EE_WriteVariable(VirtIMUAdd[0],MPU6050_Data0.Accelerometer_X);
	EE_WriteVariable(VirtIMUAdd[1],MPU6050_Data0.Accelerometer_Y);
	EE_WriteVariable(VirtIMUAdd[2],MPU6050_Data0.Accelerometer_Z);
	EE_WriteVariable(VirtIMUAdd[3],MPU6050_Data0.Gyroscope_X);
	EE_WriteVariable(VirtIMUAdd[4],MPU6050_Data0.Gyroscope_Y);
	EE_WriteVariable(VirtIMUAdd[5],MPU6050_Data0.Gyroscope_Y);

	/* End */

	if(count == __numSamples())
	{
		sample_finished = 1;
		TIM_ClearITPendingBit(Sample_Timer,TIM_IT_Update);
		TIM_Cmd(TIM7,DISABLE);

	}else
	{
		count++;
	}
	TIM_ClearITPendingBit(Sample_Timer,TIM_IT_Update);

}

/*
 * IMU.c
 *
 *  Created on: Jul 13, 2019
 *      Author: Jamie
 */

#include "IMU.h"
#include "../My_Libs/eeprom.h"
#include "stm32f4xx.h"

/* Private Variables*/

TM_MPU6050_t MPU6050_Data0;
TM_LIS302DL_LIS3DSH_t LIS3DSH_Data0;
int16_t count = 0;

uint16_t VirtIMUAdd[6] = {0xD0,0x1391,0x2652,0x3913,0x4BD4,0x5E95};

/* Private Function Definitions*/
void init_Timer(void)
{

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);

		TIM_TimeBaseInitTypeDef timerInitStructure; //create a 1ms delay
		RCC_ClocksTypeDef rcc;
		RCC_GetClocksFreq(&rcc);
		int PSC = rcc.PCLK1_Frequency*SAMPLE_RATE/65536;
		timerInitStructure.TIM_Prescaler = PSC;
		timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timerInitStructure.TIM_Period= rcc.PCLK1_Frequency/(PSC+1);
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
//Note: Internal IMU only available on STM32f4 Discovery
	#else
	if(TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS302DL)
	{
		TM_LIS302DL_LIS3DSH_Init(TM_LIS302DL_Sensitivity_2_3G, TM_LIS302DL_Filter_2Hz);
		flag = 1;
	}if(TM_LIS302DL_LIS3DSH_Detect() == TM_LIS302DL_LIS3DSH_Device_LIS3DSH)
	{
		TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_4G, TM_LIS3DSH_Filter_50Hz);
		flag = 1;
	}else
	{
		flag = 0;

	}
#endif
	return flag;
}

void deinit_IMU(void)
{
	I2C_DeInit(I2C3);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOA);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,DISABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,DISABLE);

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

	read_IMU_Data();
	/* Store Data in Flash */
	FLASH_Unlock();
#ifdef USE_EXT_IMU
	EE_WriteVariable(VirtIMUAdd[0]+count,MPU6050_Data0.Accelerometer_X);
	EE_WriteVariable(VirtIMUAdd[1]+count,MPU6050_Data0.Accelerometer_Y);
	EE_WriteVariable(VirtIMUAdd[2]+count,MPU6050_Data0.Accelerometer_Z);
	EE_WriteVariable(VirtIMUAdd[3]+count,MPU6050_Data0.Gyroscope_X);
	EE_WriteVariable(VirtIMUAdd[4]+count,MPU6050_Data0.Gyroscope_Y);
	EE_WriteVariable(VirtIMUAdd[5]+count,MPU6050_Data0.Gyroscope_Z);
#elif
	EE_WriteVariable(VirtIMUAdd[0],LIS3DSH_Data0.X);
	EE_WriteVariable(VirtIMUAdd[1],LIS3DSH_Data0.Y);
	EE_WriteVariable(VirtIMUAdd[2],LIS3DSH_Data0.Z);
#endif
	/* End */
	FLASH_Lock();
	if(count == __numSamples())
	{
		sample_finished = 1;
		TIM_ClearITPendingBit(Sample_Timer,TIM_IT_Update);
		deinit_Timer();
		TIM_Cmd(TIM7,DISABLE);

	}else
	{
		count++;
	}
	TIM_ClearITPendingBit(Sample_Timer,TIM_IT_Update);

}

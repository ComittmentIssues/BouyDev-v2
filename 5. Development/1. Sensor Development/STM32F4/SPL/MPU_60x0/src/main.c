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
#include "tm_stm32f4_mpu6050.h"
#include "IMU.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum{
	OFF,
	ON
}led_t;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t VIA[6] = {0x04,0x0964,0x12C4,0x1C24,0x2584,0x2EE4};
/* Private function prototypes -----------------------------------------------*/
void init_LED();
void set_LED(led_t val);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	init_LED();
int16_t imu_val[3][__numSamples()];
if(init_IMU())
{
	FLASH_Unlock();
	EE_Init();
	init_Timer();
	set_LED(ON);

	//wait for imu to finish sampling
	while(!sample_finished);
	//read

	for (int var = 0; var < __numSamples(); ++var)
	{
		uint16_t temp[6];
		EE_ReadVariable(VIA[0],&temp[0]);
		EE_ReadVariable(VIA[1],&temp[1]);
		EE_ReadVariable(VIA[2],&temp[2]);
		EE_ReadVariable(VIA[3],&temp[3]);
		EE_ReadVariable(VIA[4],&temp[4]);
		EE_ReadVariable(VIA[5],&temp[5]);
		imu_val[0][var] = (int16_t)temp[0];
		imu_val[1][var] = (int16_t)temp[1];
		imu_val[2][var] = (int16_t)temp[2];
		imu_val[3][var] = (int16_t)temp[3];
		imu_val[4][var] = (int16_t)temp[4];
		imu_val[5][var] = (int16_t)temp[5];
	}
}
	while(1)
  {

  }
}

void init_LED()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef led;
	led.GPIO_Mode = GPIO_Mode_OUT;
	led.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA,&led);
}

void set_LED(led_t val)
{
	if(val)
	{
		GPIOA->ODR |= GPIO_ODR_ODR_5;
	}else
	{
		GPIOA->ODR &= ~GPIO_ODR_ODR_5;
	}
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

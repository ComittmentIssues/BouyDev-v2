/**
  ******************************************************************************
  * @file    I2C Library
  * @author  Jamie Jacobson
  * @version V1.0.1
  * @date    30-08-2019
  * @brief   Main program body
  ******************************************************************************
  * Based on the library created by  Erwin Ouyang
  * http://www.handsonembedded.com/stm32f103-spl-tutorial-6/
  * This library contains the functions necessary for I2C
  * communications
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
#include "BMP280.h"
#include "Delay.h"
#include "Wind.h"
#include "string.h"
#include "stdio.h"
/* Debug ---------------------------------------------------------------------*/
#include "defines.h"
#include "tm_stm32f4_usart.h"

/* Private typedef -----------------------------------------------------------*/



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	init_LED();

	//init_Delay();
	//DEBUG TO LAPTOP via usart

	char buff[100];
	if(BMP280_Begin() == BMP_OK)
	{

		GPIO_WriteBit(GPIOA,GPIO_Pin_5,SET);
		//get calibration data
		BMP280_GetCoeff(&bmp);
		//configure ctrl measurement register
		BMP280_Configure_CTRLMEAS(BMP280_CTRLMEAS_OSRST_OS_1,BMP280_CTRLMEAS_OSRSP_OS_1,BMP280_CTRLMEAS_MODE_SLEEP);
		//configure filter, i2c mode, odr
		BMP280_Configure_Config(BMP280_CONFIG_tsb_1000,BMP280_CONFIG_FILTER_COEFF_OFF,BMP280_CONFIG_SPI3_DIS);
		double temp,press;
		TM_USART_Init(USART2,TM_USART_PinsPack_1,115200);
		uint8_t string[] = "Starting BMP280\r\n";
		uint8_t length = sizeof(string)/sizeof(string[0])-1;
		TM_USART_Send(USART2,string,length);
		while(1)
		{
			BMP280_Force_Measure(&temp,&press);
			int t[2] = {(int)temp, (int)((temp - (int) temp)*10000)};
			int p[2] = {(int)press, (int)((press - (int) press)*10000)};
			sprintf(buff,"Temp = %d.%d °C\t Press = %d.%d Pa\r\n",t[0],t[1],p[0],p[1]);
			TM_USART_Send(USART2,buff,strlen((char*)buff));
			DelayMs(1000);
			__NOP();
		}

	}
	while (1)
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

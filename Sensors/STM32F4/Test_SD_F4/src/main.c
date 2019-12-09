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
#include "../SD_CARD/tm_stm32f4_fatfs.h"
#include "../SD_CARD/tm_stm32f4_delay.h"
/* Private typedef -----------------------------------------------------------*/
/*
 * Abstract flag definition I2C_PinMap_t allows for simple selection of
 * I2C pins corresponding to I2C peripherals in a simple manner.
 */

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void init_SD(void);

  /* @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  int i = 0;
  FATFS fatfs;
  FIL file;
  uint32_t total, free;
  TM_DELAY_Init();
  if(f_mount(&fatfs,"",1) == FR_OK)
  {
	  //try to open file
	  if(f_open(&file,"test.txt",FA_OPEN_APPEND | FA_WRITE) == FR_OK)
	  {
		  char buff[60];
		  f_puts("Accelerometer Data Reading 09/23/2019\n\r",&file);
		  sprintf(buff,"ACC:\t Ax\t Ay\t Az GYR:\t Gx\t Gy\t Gz\n\r");
		  f_puts((char *)buff,&file);

	  }
	  f_close(&file);
	  f_mount(0,"",1);
  }
  while(1)
  {
	  i++;
  }
}


/*
 * Note for SD card with SPI
 * 1. must provide idle clocks into the device send 0xFF untill it sends it back to you
 * 2. before you ask the chip to come out of idle state, give it 80 clocks with CS High
 * 3. ONLY SEND 0xFF when clocking data out the chip
 * - from https://ralimtek.com/stm32/firmware/stm32_spi_sd/
 * The regulator can be disabled to turn off the microSD card
 * and save power by driving the EN pin low.
 */



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

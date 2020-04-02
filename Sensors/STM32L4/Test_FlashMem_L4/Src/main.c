/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "DATAFLASH.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
uint8_t Data_t256 [256];
uint8_t Data_t264[264];
uint8_t* Data_p = 0x00;
uint8_t Data_r[256] = {0x00};
uint8_t Data_Register[2] = {0x00};
int FLAG = 0;
int COMP = 0;
int EPE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void FLASH_Get_ID(int ChipNumber,uint8_t* id);
uint8_t FLASH_Is_Online(int ChipNumber);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Takes pointer returned from READ operations and populates the Data array from DataFlash memory
void PopulateReadArray(uint8_t* Data_p)
{
	for(int i = 0; i<264;i++){
		Data_r[i] = *(Data_p+i);
	}
}

//Takes pointer returned from GetRegister operation and populates the Register array from DataFlash register
void PopulateRegisterArray(uint8_t* Data_p)
{
	for(int i = 0; i<2;i++){
		Data_Register[i] = *(Data_p+i);
	}
}

void TEST1()
{
	  //Gets the Status register from chip 2
	  Data_p = FLASH_GetStatusRegister(2);
	  PopulateRegisterArray(Data_p);
}

void TEST2()
{
	//CONFIGURE PAGE SIZE
	FLASH_CONFIG_PageSize(2, 264);

	//POPULATE DATA
	for(int i = 0; i <264; i++){
	  Data_t264[i] = 0x0F;
	}

	//GET STATUS REGISTER
	Data_p = FLASH_GetStatusRegister(2);
	PopulateRegisterArray(Data_p);

	//ERASE PAGE, SET EPE AND READ PAGE TO SHOW PROPER ERASE RESULT
	EPE = FLASH_ERASE_Page(2);
	Data_p = FLASH_READ_Page(2);
	PopulateReadArray(Data_p);

	//WRITE PAGE AND SET EPE AND READ PAGE TO SHOW PROPER WRITE RESULT
	EPE = FLASH_WRITE_Page(2, BUFFER2, Data_t264);
	Data_p = FLASH_READ_Page(2);
	PopulateReadArray(Data_p);

	//POPULATE ARBIRTARILY SIZED ARRAY, SET EPE AND PERFORM READ MODIFY WRITE OPERATION
	uint8_t Data[5] = {0x00,0x01,0x02,0x03,0x04};
	EPE = FLASH_WRITE_ReadModifyWrite(2, BUFFER2, Data, 5);

	//READ DATA TO SHOWCASE READMODIFY WRITE
	Data_p = FLASH_READ_Page(2);
	PopulateReadArray(Data_p);

	//CONFIGURE PAGE SIZE BACK TO 256 FOR REST OF DEMO
	FLASH_CONFIG_PageSize(2, 256);
}

void TEST3()
{
	  //POPULATE DATA
	  for(int i = 0; i <256; i++){
		 Data_t256[i] = 0x0F;
	  }

	  //SET MAX ADDRESS FLAG, SET ADDRESS TO PAGE BEFORE MAX ADDRESS ADDRESS, WRITE AND READ PAGE
	  FLAG = FLASH_GetMaxAddressFlag();
	  FLASH_SetAddress((FLASH_MAX_ADDRESS&BYTE_Mask_MSB)>>16,(FLASH_MAX_ADDRESS&BYTE_Mask_MID)>>8,0x00);
	  EPE = FLASH_WRITE_Page(2, BUFFER2, Data_t256);
	  Data_p = FLASH_READ_Page(2);
	  PopulateReadArray(Data_p);

	  //INCREMENT ADDRESS, GET ADDRESS FLAG, POPULATE ARRAY, RESET FLAG, WRITE AND READ PAGE
	  FLASH_IncAddress(256);
	  FLAG = FLASH_GetMaxAddressFlag();
	  for(int i = 0; i <256; i++){
	  	  Data_t256[i] = 0x03;
	  }
	  FLASH_ResetMaxAddressFlag();
	  EPE = FLASH_WRITE_Page(2, BUFFER2, Data_t256);
	  Data_p = FLASH_READ_Page(2);
	  PopulateReadArray(Data_p);

	  //SET ADDRESS BACK AND READ TO SHOW THAT ADDRESSES HAVE CHANGED
	  FLASH_SetAddress((FLASH_MAX_ADDRESS&BYTE_Mask_MSB)>>16,(FLASH_MAX_ADDRESS&BYTE_Mask_MID)>>8,0x00);
	  Data_p = FLASH_READ_Page(2);
	  PopulateReadArray(Data_p);

}

void TEST4()
{
	//RESET ADDRESS AND POPULATE ARBITRARILY SIZED ARRAYS AND WRITE TO BOTH BUFFERS
	FLASH_SetAddress(0x00, 0x00, 0x00);
	uint8_t Data[10] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};
	EPE = FLASH_WRITE_Buffer(2, BUFFER2, Data, 10);
	uint8_t Data2[10] = {0x09,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00};
	EPE = FLASH_WRITE_Buffer(2, BUFFER1, Data2, 10);

	//DEMONSTRATE BUFFER READ AT HF
	Data_p = FLASH_READ_BufferHF(2, BUFFER1);
	PopulateReadArray(Data_p);

	//DEMONSTRATE BUFFER READ AT LF
	Data_p = FLASH_READ_BufferLF(2, BUFFER2);
	PopulateReadArray(Data_p);

	//DEMOSNTRATE BUFFER TO PAGE
	EPE = FLASH_WRITE_BufferToPage(2, BUFFER2);
	Data_p = FLASH_READ_Page(2);
	PopulateReadArray(Data_p);

}

void TEST5()
{
	//DEMONSTRATE READY BIT, DELAY AND ERASE CHIP
	FLASH_SetAddress(0x00, 0x00, 0x00);
	EPE = FLASH_ERASE_Chip(2);

	//SHOW READY BIT
	Data_p = FLASH_GetStatusRegister(2);
	PopulateRegisterArray(Data_p);

	//SHOW SUCCESSFUL ARRAY
	Data_p = FLASH_READ_Page(2);
	PopulateReadArray(Data_p);
}

void TEST6()
{
	//ERASE AND SUSPEND ERASE OPERATION
	EPE = FLASH_ERASE_Sector(2);
	FLASH_SUSPEND(2);

	//SHOW ERASE SUSPEND BIT
	Data_p = FLASH_GetStatusRegister(2);
	PopulateRegisterArray(Data_p);

	//RESUME ERASE
	FLASH_RESUME(2);

	//SHOW SUCCESSFUL ERASE
	Data_p = FLASH_GetStatusRegister(2);
	PopulateRegisterArray(Data_p);
}

void TEST7()
{
	//POPULATE DATA, WRITE PAGE, REWRITE PAGE
	for(int i = 0; i <256; i++){
		Data_t256[i] = 0x0F;
	}
	EPE = FLASH_WRITE_Page(2, BUFFER1, Data_t256);
	FLASH_ADDITIONAL_AutoPageRewrite(2, BUFFER1);

	//SHOW SUCCESSFUL WRITE REWRITE
	Data_p = FLASH_READ_Page(2);
	PopulateReadArray(Data_p);

	//COPY PAGE TO BUFFER AND READ BUFFER
	FLASH_ADDITIONAL_PageToBuffer(2, BUFFER2);
	Data_p = FLASH_READ_BufferHF(2, BUFFER2);
	PopulateReadArray(Data_p);

	//SET UP BUFFER PAGE COMPARISON TEST
	for(int i = 0; i <256; i++){
		Data_t256[i] = 0x03;
	}
	EPE = FLASH_WRITE_Buffer(2, BUFFER1, Data_t256, 256);
	COMP = FLASH_ADDITIONAL_PageBufferCompare(2, BUFFER1);
	HAL_Delay(10);
	Data_p = FLASH_GetStatusRegister(2);
	PopulateRegisterArray(Data_p);

}

HAL_StatusTypeDef Init_Flash_Chips(void)
{
	//control Pin init
	MX_GPIO_Init();
	//SPI int
	if(MX_SPI_Init(SPI2,&hspi2)		!= HAL_OK) return HAL_ERROR;
	// INIT UART Peripherals for testing
//	if(MX_UART_Init(UART4,&huart4) 	!= HAL_OK) return HAL_ERROR;
//	if(MX_UART_Init(USART2,&huart2)	!= HAL_OK) return HAL_ERROR;
	//check the status of each chip
	for (int chipnumber = 1; chipnumber < 5; ++chipnumber)
	{
		if(!FLASH_Is_Online(chipnumber))
		{
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

void FLASH_Get_ID(int ChipNumber,uint8_t* id)
{

	uint8_t cmd = 0x9F;
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);
	HAL_SPI_Transmit(&hspi2,&cmd,1,100);
	HAL_SPI_Receive(&hspi2,id,5,100);
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);
	__NOP();


}
uint8_t FLASH_Is_Online(int ChipNumber)
{
	uint8_t id[5] = {0};
	FLASH_Get_ID(ChipNumber,id);
	return (id[0] == 0x1f)&&(id[1] == 0x28)&&(id[2] == 0)&&(id[3] == 1)&&(id[4] == 0);

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Init_Flash_Chips();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

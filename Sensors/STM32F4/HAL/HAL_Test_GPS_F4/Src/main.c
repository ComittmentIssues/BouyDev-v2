/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HAL_GPS.h"
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
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  //enable Idle Line detection
  __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_TC);

  //send acknowledgement
 UBX_MSG_t ack_state = UBX_Send_Ack();
 if(ack_state == UBX_ACK_ACK)
 {
	 HAL_GPIO_WritePin(GPIOA,LD2_Pin, GPIO_PIN_SET);
 }
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

UBX_MSG_t UBX_Send_Ack(void)
{
	uint8_t ubx_ack_string[] = {0xB5 ,0x62 ,0x06 ,0x09 ,0x0D ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFF ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x17 ,0x31 ,0xBF };
	 int size = (sizeof(ubx_ack_string)/sizeof(*ubx_ack_string));
	 for (int i = 0; i < size ; ++i)
	 {
		DMA_TX_Buffer[i] = ubx_ack_string[i];
	 }

	 HAL_UART_Transmit_DMA(&huart4,DMA_TX_Buffer,size);
	 HAL_UART_Receive_DMA(&huart4,DMA_RX_Buffer,DMA_RX_BUFFER_SIZE);

	 while(!RX_COMPLETE_FLAG);
	 //wait for Rx to complete
	 char msg [10];
	 for (int i = 0; i < 10; ++i)
	 {
	 	 msg[i] = DMA_RX_Buffer[i];
	 }
	 UBX_MSG_t GPS_Acknowledgement_State;
	 uint16_t header = ((uint16_t)msg[0]<<8) | ((uint16_t)msg[1]);
	 if(header == 0xb562)
	 {
		 uint8_t ck_A =0, ck_B =0;
		 for (int i = 2; i < 8; ++i)
		 {
		 	ck_A += (uint8_t)msg[i];
		 	ck_B += ck_A;
		 }
		 if((ck_A == msg[8])&& (ck_B == msg[9]))
		 {
		 	//acknowledgement
		 	if(msg[2] == 0x05)
		 	{
		 		switch (msg[3])
		 		{
		 			case 0:
		 			GPS_Acknowledgement_State = UBX_ACK_NACK;
		 			break;
		 			case 1:
		 			GPS_Acknowledgement_State = UBX_ACK_ACK;
		 			break;
		 		}
		 	}
		 }
		 else
		 {
		 	GPS_Acknowledgement_State = UBX_ERROR;
		 }
	 }
	 return GPS_Acknowledgement_State;
}
void USART_clear_Buffer(uint8_t* buffer, uint32_t size)
{
	for (int i = 0; i < size; ++i)
	{
		buffer[i] = 0;
	}
}

void USART_GPS_IRQHandler( UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma )
{
	if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
	{
		//clear the register, disable the stream
		uint32_t temp = huart->Instance->DR;
		temp = huart->Instance->SR;
		(void)temp;
		//disable the stream
		hdma->Instance->CR &= ~DMA_SxCR_EN;

	}
}

void DMA_Rx_IRQHandler( DMA_HandleTypeDef* hdma, UART_HandleTypeDef* huart )
{
	typedef struct
		{
			__IO uint32_t ISR;   /*!< DMA interrupt status register */
			__IO uint32_t Reserved0;
			__IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
		} DMA_Base_Registers;

	if(__HAL_DMA_GET_IT_SOURCE(hdma,DMA_IT_TC))
	{
		DMA_Base_Registers *reg  = (DMA_Base_Registers *)hdma->StreamBaseAddress;
		//clear Transfer complete flag
		__HAL_DMA_CLEAR_FLAG(hdma,DMA_Rx_Flag_TCF);
		//get position
		gnss_length = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hdma);

		/*****************************************************************/
		/*    	     TODO: Additional processing HERE    				 */
		/*****************************************************************/
		RX_COMPLETE_FLAG = 1;

		/*****************************************************************/
		/*    	     					end				   				 */
		/*****************************************************************/

		/* Method to prepare for next DMA transfer*/
		reg->IFCR = 0x3FU << hdma->StreamIndex; // clear all interrupts
		hdma->Instance->M0AR = (uint32_t)DMA_RX_Buffer; //reset the pointer
		hdma->Instance->NDTR = DMA_RX_BUFFER_SIZE; //set the number of bytes to expect
		hdma->Instance->CR |= DMA_SxCR_EN;            /* Start DMA transfer */

	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{	//disable  TX transfer stream

	//clear TX Buffer
	USART_clear_Buffer(DMA_TX_Buffer,DMA_TX_BUFFER_SIZE);

	//disable Tx stream
	huart->hdmatx->Instance->CR &= ~DMA_SxCR_EN;
}
void DMA_Tx_IRQHandler(DMA_HandleTypeDef* hdma, UART_HandleTypeDef* huart)
{
	typedef struct
			{
				__IO uint32_t ISR;   /*!< DMA interrupt status register */
				__IO uint32_t Reserved0;
				__IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
			} DMA_Base_Registers;

		if(__HAL_DMA_GET_IT_SOURCE(hdma,DMA_IT_TC))
		{
			DMA_Base_Registers *reg  = (DMA_Base_Registers *)hdma->StreamBaseAddress;
			//clear Transfer complete flag
			__HAL_DMA_CLEAR_FLAG(hdma,DMA_Rx_Flag_TCF);

			//set Rx flag to show ready for data recieve
			GPS_tx_Complete = 1;

			/* Method to prepare for next DMA transfer*/
			reg->IFCR = 0x3FU << hdma->StreamIndex; // clear all interrupts
			hdma->Instance->M0AR = (uint32_t)DMA_TX_Buffer; //reset the pointer
			hdma->Instance->NDTR = DMA_RX_BUFFER_SIZE; //set the number of bytes to expect
			//hdma->Instance->CR |= DMA_SxCR_EN;            /* Start DMA transfer */
		}

}
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(uint32_t baud);
static void MX_TIM5_Init(void);
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

  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  //calculate timeout
  MX_DMA_Init();
  MX_UART4_Init(9600);
  UBX_MSG_t ack_state = UBX_Send_Ack();
   if(ack_state == UBX_ACK_ACK)
   {
	  Clear_Buffer(DMA_RX_Buffer,DMA_RX_BUFFER_SIZE);
	  Clear_Buffer(DMA_TX_Buffer,DMA_TX_BUFFER_SIZE);
	 //GPS is configured for 9600, change baud to 115200
	 uint8_t ubx_baude_rate_config[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E};
	 uint32_t size =  sizeof(ubx_baude_rate_config)/sizeof(ubx_baude_rate_config[0]);
	 memcpy(DMA_TX_Buffer,ubx_baude_rate_config,size);
	 //Send poll request
	 HAL_USART_Error_Handle(&huart4);
	 if(HAL_UART_Transmit_DMA(&huart4,DMA_TX_Buffer,size) == HAL_OK)
	 {
		 //disable
		 while(!__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TXE));
		 __HAL_UART_DISABLE_IT(&huart4, UART_IT_TC);
		 __HAL_UART_DISABLE(&huart4);
		 HAL_UART_DeInit(&huart4);
		 MX_UART4_Init(115200);
		 HAL_USART_Error_Handle(&huart4);
		 ack_state = UBX_Send_Ack();

	 }
   }else if(ack_state == UBX_TIMEOUT_Rx)
	 {
		 //deinit
	   	while(!__HAL_UART_GET_FLAG(&huart4,UART_FLAG_TXE));
	  	__HAL_UART_DISABLE(&huart4);
	  	__HAL_UART_DISABLE_IT(&huart4,UART_IT_TC);
		 HAL_UART_DeInit(&huart4);
		 MX_UART4_Init(115200);
		 HAL_USART_Error_Handle(&huart4);
		 ack_state = UBX_Send_Ack();

	 }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i;
  if(ack_state == UBX_ACK_ACK)
  {
	__HAL_DMA_ENABLE_IT(&hdma_uart4_rx, DMA_IT_TC);
	uint8_t gll[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x01,0xFC,0x12};
	int len = sizeof(gll)/sizeof(gll[0]);
	HAL_UART_Transmit_DMA(&huart4,gll,len);
	uint8_t gsa[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x01,0xFD,0x14};
	uint8_t zda[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x08,0x01,0x03,0x20};
	HAL_UART_Transmit_DMA(&huart4,gsa,len);
	HAL_UART_Transmit_DMA(&huart4,zda,len);
	//set device ready to recieve GPS
	Recieve_GPS_Data = 1;
	Ack_message = 0;
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart4,DMA_RX_Buffer,DMA_RX_BUFFER_SIZE);
	while(!RX_COMPLETE_FLAG){};
	 HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
  }
  while (1)
  {
    /* USER CODE END WHILE */
	  i++;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 23999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 480;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(uint32_t baud)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */
  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = baud;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  // clear any additional errors
  		if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_PE))
  		{
  			__HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_PE);
  		}
  		if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_NE))
  		{
  			__HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_NE);
  		}
  		if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_FE))
  		{
  			__HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_FE);
  		}
  		if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_ORE))
  		{
  			__HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_ORE);
  		}
  		__HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_IDLE);
  		USART_TX_Ready = 1;
  /* USER CODE END UART4_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void  USART_GPS_IRQHandler( UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma )
{
	if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
	{

		//clears IDLE AND OVERRUN ERROR FLAG
		uint32_t temp  = huart->Instance->RDR;
		temp = huart->Instance->ISR;
		(void)temp;
		 HAL_USART_Error_Handle(huart);
		//clear DMA stream
		//This code overcomes the errata in the DMA where
		//all three Transfer flags active causes the DMA Channel
		//To become disabled
		 if(hdma->State == HAL_DMA_STATE_BUSY)
		 {
				hdma->Instance->CCR |= DMA_CCR_EN;
				hdma->DmaBaseAddress->ISR &= ~(DMA_Rx_ISR_HTF| DMA_Rx_ISR_TE);
				hdma->DmaBaseAddress->ISR |= DMA_Rx_ISR_TCF;
				while(!HAL_IS_BIT_SET(hdma->DmaBaseAddress->ISR,DMA_Rx_ISR_TCF))
				{
					 HAL_USART_Error_Handle(huart);
				}

				hdma->Instance->CCR &= ~DMA_CCR_EN;
		 	 }
	}
	//Tx USART_Handler
	if(__HAL_UART_GET_IT(huart,UART_IT_TC))
	{
		//read from TDR
		if(USART_TX_Ready)
		{
			uint32_t temp = huart->Instance->TDR;
			(void)temp;
			//clear flag
			__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TC |UART_FLAG_TXE);
			//disable DMA
			TX_COMPLETE_FLAG = 1;
			Clear_Buffer(DMA_TX_Buffer,DMA_TX_BUFFER_SIZE);
			CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
			huart->gState = HAL_UART_STATE_READY;
		}else
		{
			__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TC |UART_FLAG_TXE);
			__HAL_UART_DISABLE_IT(huart, UART_IT_TC);

		}
	}

}

void DMA_Rx_IRQHandler( DMA_HandleTypeDef* hdma, UART_HandleTypeDef* huart )
{


	if(__HAL_DMA_GET_IT_SOURCE(hdma,DMA_IT_TC))
	{
		//clear Transfer complete flag
		__HAL_DMA_CLEAR_FLAG(hdma,DMA_Rx_Flag_TCF);
		//get position

		/*****************************************************************/
		/*    	     TODO: Additional processing HERE    				 */
		/*****************************************************************/
		RX_COMPLETE_FLAG = 1;
		//stop timer
		HAL_TIM_Base_Stop_IT(&htim5);
		/*****************************************************************/
		/*    	     					end				   				 */
		/*****************************************************************/

		/* Method to prepare for next DMA transfer*/
		hdma->DmaBaseAddress->IFCR = 0x3FU << hdma->ChannelIndex; // clear all interrupts
		hdma->Instance->CMAR = (uint32_t)DMA_RX_Buffer; //reset the pointer
		hdma->Instance->CNDTR = DMA_RX_BUFFER_SIZE; //set the number of bytes to expect
	}

}

void HAL_USART_Error_Handle(UART_HandleTypeDef *huart)
{
				uint32_t temp  = READ_REG(huart->Instance->ISR);
		 		temp =  READ_REG(huart->Instance->ISR);
		 		(void)temp;
	 			__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
	 			CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	 			CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
	 			huart4.Instance->ICR = 0xFFFF;
	 			if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE))
	 			{
	 				__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_ORE);
	 			}
	 			// clear any additional errors
	 			if(__HAL_UART_GET_FLAG(huart,UART_FLAG_PE))
	 			{
	 				__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_PE);
	 			}
	 			if(__HAL_UART_GET_FLAG(huart,UART_FLAG_NE))
	 			{
	 				__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_NE);
	 			}
	 			if(__HAL_UART_GET_FLAG(huart,UART_FLAG_FE))
	 			{
	 				__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_FE);
	 			}
}
/********************* END OF PERIPHERAL FUNCTIONS ********************************/

/********************* START OF UBX FUNCTIONS ************************************/
UBX_MSG_t UBX_Send_Ack(void)
{
	  __HAL_UART_ENABLE_IT(&huart4, UART_IT_TC);
	uint8_t ubx_ack_string[] = {0xB5 ,0x62 ,0x06 ,0x09 ,0x0D ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFF ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x17 ,0x31 ,0xBF };
	 int size = (sizeof(ubx_ack_string)/sizeof(*ubx_ack_string));

	 for (int i = 0; i < size ; ++i)
	 {
		DMA_TX_Buffer[i] = ubx_ack_string[i];
	 }

	 if(HAL_UART_Transmit_DMA(&huart4,DMA_TX_Buffer,size)== HAL_OK)
	 {
		  __HAL_DMA_ENABLE_IT(&hdma_uart4_rx, DMA_IT_TC);
		  __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
		 HAL_UART_Receive_DMA(&huart4,DMA_RX_Buffer,DMA_RX_BUFFER_SIZE);
		 USART_Begin_Timeout(&htim5,10000);
	 }else
	 {
		 return UBX_TIMEOUT_Tx;
	 }

	 while(!RX_COMPLETE_FLAG)
	 {
		 if(RX_TIMEOUT_FLAG)
		 {
			 RX_TIMEOUT_FLAG = 0;
			 return UBX_TIMEOUT_Rx;
		 }
	 }
	 RX_COMPLETE_FLAG = 0;
	 //wait for Rx to complete
	 //find first occurance of 0xB5
	 	 char val = (char) 0xB5;
	 	 int index = (int)(strchr((char*)DMA_RX_Buffer,val))-(int)DMA_RX_Buffer;
	 	 if((index < 0) || (index > DMA_RX_BUFFER_SIZE))
	 	 {
	 		 return UBX_ERROR;
	 	 }
	 	 char msg [10];
	 	 for (int i = 0; i < 10; ++i)
	 	 {
	 		 msg[i] = DMA_RX_Buffer[i+index];
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

void USART_GPS_Timout_Handler(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart)
{
	//disable Timer
	HAL_TIM_Base_Stop_IT(htim);
	RX_TIMEOUT_FLAG = 1;
	//disable DMA
	HAL_UART_DMAStop(huart);
}

void USART_Begin_Timeout(TIM_HandleTypeDef *htim,uint32_t ms)
{
	  uint32_t time = ms; //ms
	  //clear any pending updates
	  __HAL_TIM_SET_AUTORELOAD(htim,time);
	  __HAL_TIM_CLEAR_IT(htim,TIM_IT_UPDATE);
	  HAL_TIM_Base_Start_IT(htim);
}

void Clear_Buffer(uint8_t* buffer,int size)
{
	for (int i = 0; i < size; ++i)
	{
		buffer[i] = 0;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/*
 * HAL_Iridium.c
 *
 *  Created on: Mar 28, 2020
 *      Author: jamie
 */

#include "HAL_Iridium.h"

static HAL_StatusTypeDef MX_GPIO_Init(void);
static HAL_StatusTypeDef MX_DMA_Init(void);
static HAL_StatusTypeDef MX_UART5_Init(void);
static HAL_StatusTypeDef MX_TIM2_Init(void);

//------------------------------------------------------------------
//					Peripheral Init Functions
//------------------------------------------------------------------

static HAL_StatusTypeDef MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65536;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1152000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
	  return HAL_ERROR;
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  __HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
  /* USER CODE END TIM2_Init 2 */
  return HAL_OK;

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static HAL_StatusTypeDef MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 19200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */
  return HAL_OK;
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = Iridium_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(Iridium_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Iridium_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(Iridium_RX_GPIO_Port, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA2_Channel2;
    hdma_uart5_rx.Init.Request = DMA_REQUEST_2;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_NORMAL;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
    	HAL_DMA_DeInit(&hdma_uart5_rx);
    }else
    {
    	__HAL_LINKDMA(huart,hdmarx,hdma_uart5_rx);

    	    /* UART5 interrupt Init */
    	HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
        /* USER CODE BEGIN UART5_MspInit 1 */
    	CLEAR_REG(huart->Instance->CR1);
   		CLEAR_REG(huart->Instance->CR2);
   		CLEAR_REG(huart->Instance->CR3);
   	    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
   	    HAL_NVIC_EnableIRQ(UART5_IRQn);
   	    /* USER CODE END UART5_MspInit 1 */
    }
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(Iridium_TX_GPIO_Port, Iridium_TX_Pin);

    HAL_GPIO_DeInit(Iridium_RX_GPIO_Port, Iridium_RX_Pin);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);

    /* UART5 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }

}
/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel2
  */
static HAL_StatusTypeDef MX_DMA_Init(void)
{
  if(DMA2_Channel2->CCR != 0)
  {
   //clear channel to reset state
   hdma_uart5_rx.Instance = DMA2_Channel2;
   hdma_uart5_rx.DmaBaseAddress->ISR = DMA2->ISR;
   hdma_uart5_rx.ChannelIndex = 2;
   HAL_DMA_DeInit(&hdma_uart5_rx);
  }
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel2 on DMA1_Channel2 */
  hdma_memtomem_dma1_channel2.Instance = DMA1_Channel2;
  hdma_memtomem_dma1_channel2.Init.Request = DMA_REQUEST_0;
  hdma_memtomem_dma1_channel2.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel2.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel2.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel2.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel2.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel2.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel2) != HAL_OK)
  {
	  return HAL_ERROR;
  }

  /* DMA interrupt init */
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

  return HAL_OK;

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */

static HAL_StatusTypeDef MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA,IR_OnOff_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin IR_OnOff_Pin */
  GPIO_InitStruct.Pin =  IR_OnOff_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_RIng_Pin IR_NetAv_Pin */
  GPIO_InitStruct.Pin = IR_RIng_Pin|IR_NetAv_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  return HAL_OK;

}

//------------------------------------------------------------------
//					Utility Functions
//------------------------------------------------------------------

void  Clear_Buffer(uint8_t *buffer,uint32_t size)
{
	memset(buffer,0,size);
}

uint16_t calculate_checkSum(uint8_t* messagebuff, uint8_t size)
{
	uint32_t sum = 0;
	for (int i = 0; i < size; ++i)
	{
		sum+= messagebuff[i];
	}
	//return last 16 bits
	return (uint16_t)(sum & 0xFFFF);
}

//------------------------------------------------------------------
//					Command Functions
//------------------------------------------------------------------

IR_Status_t IR_Init_Module(void)
{
	 if(MX_GPIO_Init() != HAL_OK){return IR_Pin_CFG_Error;}
	 if(MX_DMA_Init()  != HAL_OK){return IR_Pin_CFG_Error;}
	 if(MX_UART5_Init()!= HAL_OK){return IR_Pin_CFG_Error;}
	 if(MX_TIM2_Init()!= HAL_OK){return IR_Pin_CFG_Error;}

	  //send acknowledgement
	 char* msg;
	 if(send_AT_CMD("AT\r")== IR_OK)
	 {
	 	msg = strtok((char*)(&RM_Buffer[2]),"\r");
	 	if(strcmp(msg,(char*)"OK") != 0)
	 	{
	 	  return IR_Ack_Error;
	 	}
	 }else
	 {
	 	  return IR_Ack_Error;
	 }
	 	  //analyse message
	 	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	 	  return IR_OK;
}

IR_Status_t get_Signal_Strength(uint8_t* signal_Strength)
{
	  char* msg;
	  if(send_AT_CMD("AT\r")== IR_OK)
	  {
		  msg = strtok((char*)(&RM_Buffer[2]),"\r");
		  if(strcmp(msg,(char*)"OK") != 0)
		  {
		    return IR_Ack_Error;
		  }
	  }else
	  {
		  return IR_Ack_Error;
	  }
	  //analyse message
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( send_AT_CMD("AT&K0\r") == IR_OK)
	  {
			msg = strtok((char*)(&RM_Buffer[2]),"\r");
			if(strcmp(msg,(char*)"OK") != 0)
			{
				return IR_CFG_Error;
			}
	  }
	  else
	  {
		return IR_CFG_Error;
	  }
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  Session_Flag = SBDIX;
	  if(send_AT_CMD("AT+CSQ\r")!= IR_OK)
	  {
		  return IR_Data_Error;
	  }
	  char* sig = strtok((char*)&RM_Buffer[7],"\r\n");
	  msg =  strtok(NULL,"\r\n");
	  if(strcmp(msg,"OK") != 0)
	  {
		  return IR_CSQ_Ack_Error;
	  }

	  *signal_Strength = atoi(sig);
	  return IR_OK;
}

IR_Status_t start_SBD_Session(SBDX_Status_t* sbd)
{
	//increase prescaler to lengthen timeout
	htim2.Instance->PSC = 750;
	Session_Flag = SBDIX;
	char* cmd = "AT+SBDIX\r";
	int size = strlen(cmd);
		memcpy(TX_Buffer,cmd,size);
		if(HAL_UART_Transmit(&huart5,TX_Buffer,size,100) != HAL_OK)
		{
			return IR_Tx_Error;
		}
		__HAL_DMA_ENABLE_IT(&hdma_uart5_rx,DMA_IT_TC);
		if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE))
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		}
		__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart5,RX_Buffer,RX_BUFFER_SIZE);
		__HAL_TIM_DISABLE_IT(&htim2,TIM_IT_CC1);
		__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);
		HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
		__HAL_TIM_ENABLE(&htim2);
		while(RX_Flag == RESET);
		if(RX_Flag == -2)
		{
			return IR_Rx_Timeout;
		}if(RX_Flag == -1)
		{
			return IR_Rx_Error;
		}
		RX_Flag = RESET;
		//decode SBD Message
		char* status = strtok((char*)&RM_Buffer[2],"\r\n");
		char* msg = strtok(NULL,"\r\n");
		if(strcmp(msg,"OK") != 0)
		{
			return IR_SBDIX_SESSION_ERROR;
		}
		char* temp = strtok(&status[7],", ");
		int temp_sbd[6] = {atoi(temp),0};
		int count = 1;
		while(temp != NULL)
		{
			temp = strtok(NULL, ", ");
			temp_sbd[count++] = atoi(temp);
		}
		sbd->MO_Status= temp_sbd[0];
		sbd->MO_MSN = temp_sbd[1];
		sbd->MT_Status = temp_sbd[2];
		sbd->MT_MSN = temp_sbd[3];
		sbd->MT_length = temp_sbd[4];
		sbd->MT_Queued = temp_sbd[5];
		//reset prescaler
		htim2.Instance->PSC = 1;
		return IR_OK;

}

IR_Status_t send_Bin_String(uint8_t* bin_string,uint32_t len)
{

	  char* msg;
	  if(send_AT_CMD("AT\r")== IR_OK)
	  {
		  msg = strtok((char*)(&RM_Buffer[2]),"\r");
		  if(strcmp(msg,(char*)"OK") != 0)
		  {
		    return IR_Ack_Error;
		  }
	  }else
	  {
		  return IR_Ack_Error;
	  }
	  //analyse message
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( send_AT_CMD("AT&K0\r") == IR_OK)
	  {
			msg = strtok((char*)(&RM_Buffer[2]),"\r");
			if(strcmp(msg,(char*)"OK") != 0)
			{
				return IR_CFG_Error;
			}
	  }
	  else
	  {
		return IR_CFG_Error;
	  }
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  //prepare Iridium for binary message reception
	  sprintf((char*)TX_Buffer,"AT+SBDWB=%lu\r",len);
	  if(send_AT_CMD((char*)TX_Buffer) == IR_OK)
	  {
		  Clear_Buffer(TX_Buffer,TX_BUFFER_SIZE);
		  msg = strtok((char*)(&RM_Buffer[2]),"\r");
		  if(strcmp(msg,(char*)"READY") != 0)
		  {
		  	return IR_CFG_Error;
		  }
	  }
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  //create a binary message complete with checksum
	  memcpy(TX_Buffer,bin_string,len);
	  uint16_t temp = calculate_checkSum(bin_string,len);
	  uint8_t check_sum[3]  = {(uint8_t)((temp&0xFF00)>>8),(uint8_t)temp&0xFF,0x0d};
	  memcpy(&TX_Buffer[len],check_sum,3);
	  //upload to message buffer
	  Session_Flag = SBDWB;
	  send_AT_CMD((char*)TX_Buffer);
	  Session_Flag = NONE;
	  msg = strtok((char*)(&RM_Buffer[2]),"\r\n");
	  int8_t ret_val = *(msg) -48;
	  msg = strtok(NULL,"\r\n");
	  if(ret_val < 0 || ret_val > 9)
	  {
		  return IR_SBDWB_STATUS_ERROR;
	  }
	  if(strcmp(msg,(char*)"OK") == 0)
	 {
		  switch(ret_val)
	  	  {
	  	  	  case 0:
	  		  	  return IR_MSG_UPLOAD_OK;
	  	  	  case 1:
	  		  	  return IR_SBDWB_TIMEOUT;
	  	  	  case 2:
	  		  	  return IR_SBDWB_CHECKSUM_ERROR;
	  	  	  case 3:
	  		  	  return IR_SBDWB_MSGOVERRUN_ERROR;
	  	  }
	 	 }
	  //decode return pack
	  //return status

	  return IR_MSG_UPLOAD_ERROR;
}

IR_Status_t send_String(char* string)
{
	  uint32_t len = strlen(string);
	  char* msg;
	  if(send_AT_CMD("AT\r")== IR_OK)
	  {
		  msg = strtok((char*)(&RM_Buffer[2]),"\r");
		  if(strcmp(msg,(char*)"OK") != 0)
		  {
		    return IR_Ack_Error;
		  }
	  }else
	  {
		  return IR_Ack_Error;
	  }
	  //analyse message
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( send_AT_CMD("AT&K0\r") == IR_OK)
	  {
			msg = strtok((char*)(&RM_Buffer[2]),"\r");
			if(strcmp(msg,(char*)"OK") != 0)
			{
				return IR_CFG_Error;
			}
	  }
	  else
	  {
		return IR_CFG_Error;
	  }
		Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
		//create string with message
	    memcpy(TX_Buffer,(const char*)"AT+SBDWT=",ASCII_MSG_BYTE_LEN);
		memcpy(&TX_Buffer[ASCII_MSG_BYTE_LEN],string,len);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0xFFFFFFFF);
		if(send_AT_CMD((char*)TX_Buffer) == IR_OK)
		{
			msg = strtok((char*)(&RM_Buffer[2]),"\r");
			if(strcmp(msg,(char*)"OK") != 0)
			{
				return IR_MSG_UPLOAD_ERROR;
			}
	}
	  return IR_MSG_UPLOAD_OK;
}

IR_Status_t recieve_String(uint8_t* MSG_Buff,uint32_t MSG_BUFF_SIZE, uint16_t *num_messages)
{
	  char* msg;
	  if(send_AT_CMD("AT\r")== IR_OK)
	  {
		  msg = strtok((char*)(&RM_Buffer[2]),"\r");
		  if(strcmp(msg,(char*)"OK") != 0)
		  {
		    return IR_Ack_Error;
		  }
	  }else
	  {
		  return IR_Ack_Error;
	  }
	  //analyse message
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( send_AT_CMD("AT&K0\r") == IR_OK)
	  {
			msg = strtok((char*)(&RM_Buffer[2]),"\r");
			if(strcmp(msg,(char*)"OK") != 0)
			{
				return IR_CFG_Error;
			}
	  }
	  else
	  {
		return IR_CFG_Error;
	  }
	  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	SBDX_Status_t sbd;
	IR_Status_t flag = start_SBD_Session(&sbd);

	if(flag != IR_OK)
	{
		return flag;
	}
	//check SBDIX return status of mobile Terminated buffer
	switch(sbd.MT_Status)
	{
	case 0:
		return IR_SBDIX_NO_NEW_MESSAGE;
	case 2:
		return IR_SBDIX_MAIL_CHECK_ERROR;
	default:
		break;
	}
	// Download Message to your controller
	*num_messages = sbd.MT_Queued;
	Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	Session_Flag = SBDRT;
	if(send_AT_CMD("AT+SBDRT\r") != IR_OK)
	{
		return IR_SBDRT_Rx_Error;
	}
	// SBDRT return type : +SBDRT\r\n<msg>\r\n<return_status>
	char* temp = strtok((char*)&RM_Buffer[10],"\r\n");
	msg = strtok(NULL,"\r\n");
	 if(strcmp(msg,(char*)"OK") != 0)
	 {
		return IR_Ack_Error;
	 }
	memcpy(MSG_Buff,temp,strlen(temp));
 	return IR_OK;
}

void DMA_Iridium_Periph_IRQHandler(UART_HandleTypeDef *huart)
{

	huart->hdmarx->DmaBaseAddress->IFCR |= (DMA_IFCR_CTCIF2|DMA_IFCR_CGIF2);
	HAL_TIM_Base_Stop_IT(&htim2);
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1|TIM_IT_UPDATE);

	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	__HAL_UART_DISABLE_IT(&huart5,UART_IT_IDLE);
	__HAL_TIM_DISABLE_IT(&htim2,TIM_IT_CC1);

	if(__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_CC1))
	{
		__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
	}
	//begin transfer of collected data to memory
    uint8_t* ind = (uint8_t*)strchr((char*)RX_Buffer,'\r')+1;
	int len = (ind - RX_Buffer)+1; // chope off the \0
	msg_len = gnss_length -len-1;
	if(len > 0)
	{
	   	__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel2,DMA_IT_TC);
	   	HAL_DMA_Start(&hdma_memtomem_dma1_channel2,(uint32_t)(&RX_Buffer[len]),(uint32_t)RM_Buffer,msg_len);
	}
	TIM_IDLE_Timeout = RESET;
}

void DMA_Iridium_MEM_IRQHandler(DMA_HandleTypeDef *hdma_mem)
{
	//clear the rx buffer
	Clear_Buffer(RX_Buffer,RX_BUFFER_SIZE);
	msg_len = strlen((char*)RM_Buffer);
	//check message to see if valid
	//valid messages follow the format "\r\nMSG_STRING\r\n"
	if((RM_Buffer[0] == '\r') && (RM_Buffer[1] =='\n') &&(RM_Buffer[msg_len - 2] == '\r') && (RM_Buffer[msg_len -1] == '\n' ))
	{
		RX_Flag = SET;
	}else
	{
		//invalid message returned
		RX_Flag = -1;
	}
	__HAL_DMA_CLEAR_FLAG(hdma_mem,DMA_FLAG_TC2);
	if(__HAL_DMA_GET_FLAG(hdma_mem,DMA_FLAG_HT2))
	{
		__HAL_DMA_CLEAR_FLAG(hdma_mem,DMA_FLAG_HT2);
	}
}

void USART_RTO_IRQHandler(TIM_HandleTypeDef *htim)
{
	if(__HAL_TIM_GET_IT_SOURCE(htim,TIM_IT_CC1))
	{
		//clear interrupt
		htim->Instance->CR1 &= ~TIM_CR1_CEN;
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1|TIM_IT_UPDATE);
		//set reciever timeout flag
		TIM_IDLE_Timeout = 1;
		//disable timer
		HAL_TIM_Base_Stop_IT(htim);
		HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(htim,0);
		USART_Iridium_IRQHandler(&huart5);
	}
	if(__HAL_TIM_GET_IT_SOURCE(htim,TIM_IT_UPDATE))
	{
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1|TIM_IT_UPDATE);
		if(Session_Flag == SBDIX)
		{
			Session_Flag = NONE;
		}else
		{
			TIM_IDLE_Timeout = 1;
			HAL_TIM_Base_Stop_IT(htim);
			USART_Iridium_IRQHandler(&huart5);
		}
		__NOP();

	}
}

void USART_Iridium_IRQHandler(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
	{
		uint32_t temp = huart->Instance->ISR;
		temp = huart->Instance->RDR;
		(void)temp;
		//check for reciever timeout
		if(TIM_IDLE_Timeout)
		{
			//check data counter
			HAL_UART_DMAStop(huart);
			gnss_length = (sizeof(RX_Buffer)/sizeof(RX_Buffer[0])) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
			if(gnss_length > 0)
			{
				// transfer incomplete, move transfered data to message buffer
				uint8_t* ind;
				int len;
				if(Session_Flag == SBDWB)
				{
					ind = (uint8_t*)strchr((char*)RX_Buffer,'\r');
					len = strlen((char*)ind);
				}else if (Session_Flag == SBDRT)
				{
					ind = (uint8_t*)strchr((char*)RX_Buffer,'\r') + 1 ;
					len = strlen((char*)ind);
				}
				else
				{
					if(strcmp((char*)RX_Buffer,"AT+SBDIX\r") != 0)
					{
						ind = (uint8_t*)strchr((char*)RX_Buffer,'\r')+1;
						len = (ind - RX_Buffer) -1; // chope off the \0
						msg_len = gnss_length -len-1;
					}else
					{
						len = 0;
					}

				}

			    if(len > 0)
			    {
			    	__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel2,DMA_IT_TC);
			    	if(Session_Flag == SBDWB || Session_Flag == SBDRT)
			    	{
			    		HAL_DMA_Start(&hdma_memtomem_dma1_channel2,(uint32_t)(ind),(uint32_t)RM_Buffer,len);
			    	}else
			    	{
			    		HAL_DMA_Start(&hdma_memtomem_dma1_channel2,(uint32_t)(&RX_Buffer[len+1]),(uint32_t)RM_Buffer,msg_len);
			    	}
			    }else
			    {
			    	RX_Flag = -1;
			    }
			    (void)ind;

			}else
			{
				//reciever timeout
				RX_Flag = -2;
			}
			TIM_IDLE_Timeout = 0;
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			__HAL_UART_DISABLE_IT(huart,UART_IT_IDLE);
		}
		__HAL_UART_CLEAR_IDLEFLAG(huart);
	}
	if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_ERR))
	{
		//clear framing error
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_FE) == SET)
		{
			__HAL_UART_CLEAR_FEFLAG(huart);
		}
		//clear noise error
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_NE) == SET)
		{
			__HAL_UART_CLEAR_NEFLAG(huart);
		}
		//clear overun error
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) == SET)
		{
			uint8_t temp = huart->Instance->RDR;
			(void)temp;
			__HAL_UART_CLEAR_OREFLAG(huart);
		}
		//clear parity errors
		if(__HAL_UART_GET_FLAG(huart,UART_FLAG_PE) == SET)
		{
			__HAL_UART_CLEAR_PEFLAG(huart);
		}
	}
}

void Iridium_ControlPin_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(IR_RIng_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(IR_RIng_Pin);
		//download messages
	}

	if(__HAL_GPIO_EXTI_GET_IT(IR_NetAv_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(IR_NetAv_Pin);

	}
}

IR_Status_t send_AT_CMD(char* cmd)

{
	int size = strlen(cmd);
	memcpy(TX_Buffer,cmd,size);
	if(HAL_UART_Transmit(&huart5,TX_Buffer,size,100) != HAL_OK)
	{
		return IR_Tx_Error;
	}
	__HAL_DMA_ENABLE_IT(&hdma_uart5_rx,DMA_IT_TC);
	if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	}
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart5,RX_Buffer,RX_BUFFER_SIZE);
	__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	while(RX_Flag == RESET);
	if(RX_Flag == -2)
	{
		return IR_Rx_Timeout;
	}if(RX_Flag == -1)
	{
		return IR_Rx_Error;
	}
	RX_Flag = 0;
	return IR_OK;
}


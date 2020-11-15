/*
 * HAL_Iridium.c
 *
 *  Created on: Mar 28, 2020
 *      Author: Jamie Jacobson
 *      Student No: JCBJAM007
 *      For: University of Cape Town
 */

//============================= 1. Includes ==============================================

#include "HAL_Iridium.h"


//======================= 2. Function Prototypes ==================================

/*
 * Function Name void  Clear_Buffer(uint8_t *buffer,uint32_t size);
 *
 * @brief: deletes all data in a specified array
 *
 * @param: buffer - pointer to uint8_t array
 *
 * @param: size - size of array
 *
 * @return: void
 */
void  IR_Clear_Buffer(uint8_t *buffer,uint32_t size);

/*
 * Function Name uint16_t calculate_checkSum(uint8_t* messagebuff, uint8_t size);
 *
 * @brief: calculates a pair of checksum bytes for an SBD Binary Message.
 *
 * @param: messagebuff - pointer to data array
 *
 * @param: size - number of bytes in message
 *
 * @return: uint16_t checksum bytes
 */
uint16_t IR_Calculate_Checksum(uint8_t* messagebuff, uint8_t size);

/*
 * Function Name static HAL_StatusTypeDef MX_GPIO_Init(void);
 *
 * @brief: Initialize GPIO Control Pins
 *
 * @param: void
 *
 * @return: HAL_StatusTypeDef - Hal return val showing status of initialization for error handling
 */
static HAL_StatusTypeDef MX_GPIO_Init(void);

/*
 * Function Name static HAL_StatusTypeDef MX_DMA_Init(void);
 *
 * @brief: Function to initialize DMA channel for memory to memory data streaming
 *
 * @param: void
 *
 * @return: HAL_StatusTypeDef - Hal return val showing status of initialization for error handling
 */
static HAL_StatusTypeDef MX_DMA_Init(void);

/*
 * Function Name static HAL_StatusTypeDef MX_UART5_Init(void);
 *
 * @brief: Initialize USART Communications on UART5
 *
 * @param: void
 *
 * @return: HAL_StatusTypeDef - Hal return val showing status of initialization for error handling
 */
static HAL_StatusTypeDef MX_UART5_Init(void);

/*
 * Function Name static HAL_StatusTypeDef MX_TIM3_Init(void);
 *
 * @brief: Initialize Slave Reset using External Trigger on TIM3
 *
 * @param: void
 *
 * @return: HAL_StatusTypeDef - Hal return val showing status of initialization for error handling
 */
static HAL_StatusTypeDef MX_TIM3_Init(void);

//======================= 3. Static Function Definition ==================================

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static HAL_StatusTypeDef MX_UART5_Init(void)
{

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

  return HAL_OK;
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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /*Configure GPIO pin Output Level */


  /*Configure GPIO pins : IR_OnOff_Pin */
  GPIO_InitStruct.Pin =  IR_OnOff_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IR_OnOff_GPIO_Port, &GPIO_InitStruct);
  /* Disable Internal Pull Down Resistor*/


  /*Configure GPIO pins : IR_RIng_Pin IR_NetAv_Pin */
  GPIO_InitStruct.Pin = IR_Ring_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IR_Ring_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = IR_NetAv_Pin;
  HAL_GPIO_Init(IR_NetAv_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/

  HAL_NVIC_SetPriority(IR_NetAv_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(IR_NetAv_EXTI_IRQn);


  return HAL_OK;

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static HAL_StatusTypeDef MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    return HAL_ERROR;
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    return HAL_ERROR;
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIMEx_RemapConfig(&htim3, TIM_TIM3_ETR_COMP1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 11520;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

//======================= 4. MSP Function Definition ==================================

/* MSP FUNCTIONS:
 *
 *  Functions designed to replace the _weak MSP peripheral initialization
 *  function definitions in the HAL Library files.
 *
 *  NB!!!! before running the code do the following:
 *
 *  1. Uncomment the desired MSP Function
 *
 *  2. Cut the function and paste it in the stm32l4xx_hal_msp.c file
 *
 *  3. In the stm32l4xx_hal_msp.c file, include the header "HAL_Iridium.h"
 *
 *  If an MSP function already exists, then instead of copying over the function
 *  Select the if statement with the desired instance and copy that accross to
 *  inside the Pre existing MSP function
 */



//======================= 5. Utility Function Definition ==================================

void  IR_Clear_Buffer(uint8_t *buffer,uint32_t size)
{
	memset(buffer,0,size);
}

uint16_t IR_Calculate_Checksum(uint8_t* messagebuff, uint8_t size)
{
	uint32_t sum = 0;
	for (int i = 0; i < size; ++i)
	{
		sum+= messagebuff[i];
	}
	//return last 16 bits
	return (uint16_t)(sum & 0xFFFF);
}

//======================= 6. Iridium Module Function Definition ==================================

/*
 * @brief: Intialize device and send acknowledgment
 *
 * @param: none
 *
 * @retval: IR_Status_t
 */
IR_Status_t IR_Init_Module(void)
{
	 if(MX_GPIO_Init() != HAL_OK){return IR_Pin_CFG_Error;}
	 HAL_GPIO_WritePin(IR_OnOff_GPIO_Port,IR_OnOff_Pin,SET);
	 if(MX_DMA_Init()  != HAL_OK){return IR_Pin_CFG_Error;}
	 if(MX_UART5_Init()!= HAL_OK){return IR_Pin_CFG_Error;}
	 if(MX_TIM3_Init()!= HAL_OK){return IR_Pin_CFG_Error;}

	  //send acknowledgement
	 char* msg;
	 if(IR_send_AT_CMD("AT\r")== IR_OK)
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
	 	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	 	  return IR_OK;
}

/*
 * @brief: Deinitializes Peripherals and puts the modem to sleep
 * @param: none
 * @retval: IR_Status_t
 *
 * NB: Ensure that HAL_TIM_BASE_DeInit calls the user defined MSP DeInit otherwise the peripheral will not
 * 	   be cleared
 */
IR_Status_t IR_DeInit_Module(void)
{
	/* Deinitialize TIMx Peripheral*/
	if(htim3.Instance != IR_TIM_PORT)
	{
		htim3.Instance = IR_TIM_PORT;
	}
	HAL_TIM_Base_Stop_IT(&htim3);
	if(HAL_TIM_Base_DeInit(&htim3) != HAL_OK)
	{
		return IR_CFG_Error;
	}
	/* Deinitialize UARTx Peripheral*/
	if(huart5.Instance != UART5)
	{
		huart5.Instance = IR_USART_PORT;
	}
	if(HAL_UART_DeInit(&huart5) != HAL_OK)
	{
		return IR_CFG_Error;
	}

	/* Deinitialize DMA MEM Channel*/
	if(hdma_memtomem_dma1_channel2.Instance != DMA1_Channel2)
	{
		hdma_memtomem_dma1_channel2.Instance = DMA1_Channel2;
	}
	if(HAL_DMA_DeInit(&hdma_memtomem_dma1_channel2) != HAL_OK)
	{
		return IR_CFG_Error;
	}
	//set On/Off Pin to low and enable pull down resistor
	HAL_GPIO_WritePin(IR_OnOff_GPIO_Port,IR_OnOff_Pin,RESET);
	HAL_PWREx_EnableGPIOPullDown(IR_OnOff_PWR_GPIO_Port,IR_OnOff_Pin);
	HAL_PWREx_EnablePullUpPullDownConfig();

	return IR_OK;
}

IR_Status_t IR_get_Signal_Strength(uint8_t* signal_Strength)
{
	  char* msg;
	  if(IR_send_AT_CMD("AT\r")== IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( IR_send_AT_CMD("AT&K0\r") == IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  Session_Flag = CSQ;
	  __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
	  if(IR_send_AT_CMD("AT+CSQ\r")!= IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  return IR_OK;
}

IR_Status_t IR_send_AT_CMD(char* cmd)

{
	int size = strlen(cmd);
	memcpy(IR_TX_Buffer,cmd,size);
	if(HAL_UART_Transmit(&huart5,IR_TX_Buffer,size,100) != HAL_OK)
	{
		return IR_Tx_Error;
	}
	__HAL_DMA_ENABLE_IT(&hdma_uart5_rx,DMA_IT_TC);
	if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	}
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart5,IR_RX_Buffer,RX_BUFFER_SIZE);
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	while(IR_RX_Flag == RESET);
	if(IR_RX_Flag == -2)
	{
		return IR_Rx_Timeout;
	}if(IR_RX_Flag == -1)
	{
		return IR_Rx_Error;
	}
	IR_RX_Flag = 0;
	return IR_OK;
}

IR_Status_t IR_send_AT_CMD_Bin(uint8_t* cmd,size_t len)
{
	memcpy(IR_TX_Buffer,cmd,len);
	if(HAL_UART_Transmit(&huart5,IR_TX_Buffer,len,100) != HAL_OK)
	{
		return IR_Tx_Error;
	}
	__HAL_DMA_ENABLE_IT(&hdma_uart5_rx,DMA_IT_TC);
	if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	}
	__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart5,IR_RX_Buffer,RX_BUFFER_SIZE);
	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	while(IR_RX_Flag == RESET);
	if(IR_RX_Flag == -2)
	{
		return IR_Rx_Timeout;
	}if(IR_RX_Flag == -1)
	{
		return IR_Rx_Error;
	}
	IR_RX_Flag = 0;
	return IR_OK;
}
//======================= 7. Transmit/Recieve Function Definition ==================================

IR_Status_t IR_start_SBD_Session(SBDX_Status_t* sbd)
{
	//increase prescaler to lengthen timeout
	htim3.Instance->PSC = 5;
	Session_Flag = SBDIX;
	char* cmd = "AT+SBDIX\r";
	int size = strlen(cmd);
		memcpy(IR_TX_Buffer,cmd,size);
		if(HAL_UART_Transmit(&huart5,IR_TX_Buffer,size,100) != HAL_OK)
		{
			return IR_Tx_Error;
		}
		__HAL_DMA_ENABLE_IT(&hdma_uart5_rx,DMA_IT_TC);
		if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE))
		{
			__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		}
		__HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart5,IR_RX_Buffer,RX_BUFFER_SIZE);
		__HAL_TIM_DISABLE_IT(&htim3,TIM_IT_CC1);
		__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
		HAL_NVIC_ClearPendingIRQ(TIM3_IRQn);
		__HAL_TIM_ENABLE(&htim3);
		while(IR_RX_Flag == RESET);
		if(IR_RX_Flag == -2)
		{
			IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
			return IR_Rx_Timeout;
		}if(IR_RX_Flag == -1)
		{
			IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
			return IR_Rx_Error;
		}
		IR_RX_Flag = RESET;
		//decode SBD Message
		char* status = strtok((char*)&RM_Buffer[2],"\r\n");
		char* msg = strtok(NULL,"\r\n");
		if(strcmp(msg,"OK") != 0)
		{
			IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
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
		htim3.Instance->PSC = 0;
		IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
		return IR_OK;

}

IR_Status_t IR_send_Bin_String(uint8_t* bin_string,uint32_t len)
{

	  char* msg;
	  if(IR_send_AT_CMD("AT\r")== IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( IR_send_AT_CMD("AT&K0\r") == IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  //prepare Iridium for binary message reception
	  sprintf((char*)IR_TX_Buffer,"AT+SBDWB=%lu\r",len);
	  if(IR_send_AT_CMD((char*)IR_TX_Buffer) == IR_OK)
	  {
		  IR_Clear_Buffer(IR_TX_Buffer,TX_BUFFER_SIZE);
		  msg = strtok((char*)(&RM_Buffer[2]),"\r");
		  if(strcmp(msg,(char*)"READY") != 0)
		  {
		  	return IR_CFG_Error;
		  }
	  }
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  //create a binary message complete with checksum
	  memcpy(IR_TX_Buffer,bin_string,len);
	  uint16_t temp = IR_Calculate_Checksum(bin_string,len);
	  uint8_t check_sum[3]  = {(uint8_t)((temp&0xFF00)>>8),(uint8_t)temp&0xFF,0x0d};
	  memcpy(&IR_TX_Buffer[len],check_sum,3);
	  //upload to message buffer
	  Session_Flag = SBDWB;
	  IR_send_AT_CMD_Bin(IR_TX_Buffer,len+3);
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

IR_Status_t IR_send_String(char* string)
{
	  uint32_t len = strlen(string);
	  char* msg;
	  if(IR_send_AT_CMD("AT\r")== IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( IR_send_AT_CMD("AT&K0\r") == IR_OK)
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
		IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
		//create string with message
	    memcpy(IR_TX_Buffer,(const char*)"AT+SBDWT=",ASCII_MSG_BYTE_LEN);
		memcpy(&IR_TX_Buffer[ASCII_MSG_BYTE_LEN],string,len);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0xFFFFFFFF);
		if(IR_send_AT_CMD((char*)IR_TX_Buffer) == IR_OK)
		{
			msg = strtok((char*)(&RM_Buffer[2]),"\r");
			if(strcmp(msg,(char*)"OK") != 0)
			{
				return IR_MSG_UPLOAD_ERROR;
			}
	}
	  return IR_MSG_UPLOAD_OK;
}

IR_Status_t IR_recieve_String(uint8_t* MSG_Buff,uint32_t MSG_BUFF_SIZE, uint16_t *num_messages)
{
	  char* msg;
	  if(IR_send_AT_CMD("AT\r")== IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	  if( IR_send_AT_CMD("AT&K0\r") == IR_OK)
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
	  IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	SBDX_Status_t sbd;
	IR_Status_t flag = IR_start_SBD_Session(&sbd);

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
	IR_Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
	Session_Flag = SBDRT;
	if(IR_send_AT_CMD("AT+SBDRT\r") != IR_OK)
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

//======================= 8. Handler Function Definition ==================================

void DMA_Iridium_Periph_IRQHandler(UART_HandleTypeDef *huart)
{

	huart->hdmarx->DmaBaseAddress->IFCR |= (DMA_IFCR_CTCIF2|DMA_IFCR_CGIF2);
	HAL_TIM_Base_Stop_IT(&htim3);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1|TIM_IT_UPDATE);

	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	__HAL_UART_DISABLE_IT(&huart5,UART_IT_IDLE);
	__HAL_TIM_DISABLE_IT(&htim3,TIM_IT_CC1);

	if(__HAL_TIM_GET_FLAG(&htim3,TIM_FLAG_CC1))
	{
		__HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC1);
	}
	//begin transfer of collected data to memory
    uint8_t* ind = (uint8_t*)strchr((char*)IR_RX_Buffer,'\r')+1;
	int len = (ind - IR_RX_Buffer)+1; // chope off the \0
	msg_len = IR_length -len-1;
	if(len > 0)
	{
	   	__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel2,DMA_IT_TC);
	   	HAL_DMA_Start(&hdma_memtomem_dma1_channel2,(uint32_t)(&IR_RX_Buffer[len]),(uint32_t)RM_Buffer,msg_len);
	}
	IR_TIM_IDLE_Timeout = RESET;
}

void DMA_Iridium_MEM_IRQHandler(DMA_HandleTypeDef *hdma_mem)
{
	//clear the rx buffer
	IR_Clear_Buffer(IR_RX_Buffer,RX_BUFFER_SIZE);
	msg_len = strlen((char*)RM_Buffer);
	//check message to see if valid
	//valid messages follow the format "\r\nMSG_STRING\r\n"
	if((RM_Buffer[0] == '\r') && (RM_Buffer[1] =='\n') &&(RM_Buffer[msg_len - 2] == '\r') && (RM_Buffer[msg_len -1] == '\n' ))
	{
		IR_RX_Flag = SET;
	}else
	{
		//invalid message returned
		IR_RX_Flag = -1;
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
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1|TIM_IT_UPDATE);
		if(Session_Flag != CSQ)
		{
		htim->Instance->CR1 &= ~TIM_CR1_CEN;

			//set reciever timeout flag
			IR_TIM_IDLE_Timeout = 1;
			//disable timer
			HAL_TIM_Base_Stop_IT(htim);
			HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim,0);
		}
		else
		{
			__NOP();
		}

	}
	if(__HAL_TIM_GET_IT_SOURCE(htim,TIM_IT_UPDATE))
	{
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1|TIM_IT_UPDATE);
		if(Session_Flag == SBDIX || Session_Flag ==CSQ)
		{
			Session_Flag = NONE;
		}else
		{
			IR_TIM_IDLE_Timeout = 1;
			HAL_TIM_Base_Stop_IT(htim);
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
		if(IR_TIM_IDLE_Timeout)
		{
			//check data counter
			HAL_UART_DMAStop(huart);
			IR_length = (sizeof(IR_RX_Buffer)/sizeof(IR_RX_Buffer[0])) - __HAL_DMA_GET_COUNTER(huart->hdmarx);
			if(IR_length > 0)
			{
				// transfer incomplete, move transfered data to message buffer
				uint8_t* ind;
				int len;
				if(Session_Flag == SBDWB)
				{
					ind = (uint8_t*)strchr((char*)IR_RX_Buffer,'\r');
					len = strlen((char*)ind);
				}else if (Session_Flag == SBDRT)
				{
					ind = (uint8_t*)strchr((char*)IR_RX_Buffer,'\r') + 1 ;
					len = strlen((char*)ind);
				}
				else
				{
					if(strcmp((char*)IR_RX_Buffer,"AT+SBDIX\r") != 0)
					{
						ind = (uint8_t*)strchr((char*)IR_RX_Buffer,'\r')+1;
						len = (ind - IR_RX_Buffer) -1; // chope off the \0
						msg_len = IR_length -len-1;
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
			    		HAL_DMA_Start(&hdma_memtomem_dma1_channel2,(uint32_t)(&IR_RX_Buffer[len+1]),(uint32_t)RM_Buffer,msg_len);
			    	}
			    }else
			    {
			    	IR_RX_Flag = -1;
			    }
			    (void)ind;

			}else
			{
				//reciever timeout
				IR_RX_Flag = -2;
			}
			IR_TIM_IDLE_Timeout = 0;
			__HAL_UART_CLEAR_IDLEFLAG(huart);

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
	if(__HAL_GPIO_EXTI_GET_IT(IR_Ring_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(IR_Ring_Pin);
		//download messages
	}

	if(__HAL_GPIO_EXTI_GET_IT(IR_NetAv_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(IR_NetAv_Pin);

	}
}


//------------------------------ MOVE TO stn32l4xx_it.c --------------------------------//


/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
//void DMA1_Channel2_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
//
//  /* USER CODE END DMA1_Channel2_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_memtomem_dma1_channel2);
//  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
//  DMA_Iridium_MEM_IRQHandler(&hdma_memtomem_dma1_channel2);
//  /* USER CODE END DMA1_Channel2_IRQn 1 */
//}
//
///**
//  * @brief This function handles TIM2 global interrupt.
//  */
//void TIM3_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM2_IRQn 0 */
//	USART_RTO_IRQHandler(&htim3);
//  HAL_TIM_IRQHandler(&htim3);
//}
//
///**
//  * @brief This function handles EXTI line[15:10] interrupts.
//  */
//void EXTI15_10_IRQHandler(void)
//{

//  Iridium_ControlPin_IRQHandler();

//}
//
///**
//  * @brief This function handles UART5 global interrupt.
//  */
//void UART5_IRQHandler(void)
//{
//  /* USER CODE BEGIN UART5_IRQn 0 */
//  USART_Iridium_IRQHandler(&huart5);
//}
//
///**
//  * @brief This function handles DMA2 channel2 global interrupt.
//  */
//void DMA2_Channel2_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */
//
//  /* USER CODE END DMA2_Channel2_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_uart5_rx);
//  /* USER CODE BEGIN DMA2_Channel2_IRQn 1 */
//  DMA_Iridium_Periph_IRQHandler(&huart5);
//  /* USER CODE END DMA2_Channel2_IRQn 1 */
//}

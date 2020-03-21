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
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	IR_OK,
	IR_Rx_Error,
	IR_Tx_Error,
	IR_Rx_Incplt,
	IR_Rx_Timeout,
	IR_Data_Error,
	IR_Ack_Error,
	IR_CFG_Error,
	IR_MSG_UPLOAD_ERROR,
	IR_MSG_UPLOAD_OK,
	IR_SBDWB_STATUS_ERROR,
	IR_SBDWB_TIMEOUT,
	IR_SBDWB_MSGOVERRUN_ERROR,
	IR_SBDWB_CHECKSUM_ERROR,
	IR_SBDIX_SESSION_ERROR
} IR_Status_t;

typedef enum
{
	NONE,
	SBDWB,
	SBDIX

}Session_t;
typedef struct
{
	uint8_t MO_Status;
	uint32_t MO_MSN;
	uint8_t MT_Status;
	uint32_t MT_MSN,MT_length,MT_Queued;

} SBDX_Status_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUFFER_SIZE 2046
#define RX_BUFFER_SIZE 2046
#define RM_BUFFER_SIZE 500
#define ASCII_MSG_BYTE_LEN 9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel2;
/* USER CODE BEGIN PV */
int8_t RX_Flag,TIM_IDLE_Timeout;
Session_t Session_Flag;
uint8_t RX_Buffer[RX_BUFFER_SIZE];
uint8_t TX_Buffer[TX_BUFFER_SIZE];
uint8_t RM_Buffer[RM_BUFFER_SIZE];
uint32_t gnss_length;
uint32_t msg_len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void DMA_Iridium_Periph_IRQHandler(UART_HandleTypeDef *huart);
void USART_Iridium_IRQHandler(UART_HandleTypeDef *huart);
IR_Status_t send_AT_CMD(char* cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
		int count = 0;
		while(temp != NULL)
		{
			temp = strtok(NULL, ", ");
			temp_sbd[count++] = atoi(temp);
		}
		sbd->MO_MSN = temp_sbd[0];
		sbd->MO_Status = temp_sbd[1];
		sbd->MT_MSN = temp_sbd[2];
		sbd->MT_Status = temp_sbd[3];
		sbd->MT_length = temp_sbd[4];
		sbd->MT_Queued = temp_sbd[5];
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
void USART_TIM_RTO_Handler(TIM_HandleTypeDef *htim)
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
				}else
				{
					ind = (uint8_t*)strchr((char*)RX_Buffer,'\r')+1;
					len = (ind - RX_Buffer) -1; // chope off the \0
					msg_len = gnss_length -len-1;
				}

			    if(len > 0)
			    {
			    	__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel2,DMA_IT_TC);
			    	if(Session_Flag == SBDWB)
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
  	  if(DMA2_Channel2->CCR != 0)
  	  {
  		  //clear channel to reset state
  		  hdma_uart5_rx.Instance = DMA2_Channel2;
  		  hdma_uart5_rx.DmaBaseAddress->ISR = DMA2->ISR;
  		  hdma_uart5_rx.ChannelIndex = 2;
  		  HAL_DMA_DeInit(&hdma_uart5_rx);
  	  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t msg[24] = "This is A Test in Binary";
  if(send_Bin_String(msg,24) == IR_MSG_UPLOAD_OK)
  {
	  SBDX_Status_t sbd;
	  while(1)
	  {
		  if(start_SBD_Session(&sbd) == IR_OK)
		 {
		 		  //check return status
		 		  if(sbd.MO_Status == 0)
		 		  {
		 			  //message sent
		 			  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
		 			  break;
		 		  }
		 		  Clear_Buffer(RM_Buffer,RM_BUFFER_SIZE);
		 }
	  }


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
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
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1152000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  __HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
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
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel2
  */
static void MX_DMA_Init(void) 
{

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
    Error_Handler( );
  }

  /* DMA interrupt init */
  HAL_NVIC_ClearPendingIRQ(DMA2_Channel2_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA2_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
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

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
#define _XOPEN_SOURCE // Definition for time.h to work
#define ARM_MATH_CM4  // Also defined for the time.h embedded implementation
//======================================================================================

/* Includes */
#include <stm32l4xx.h> //StandardPeriph Driver Header
#include "string.h"	   // for string handlings
#include "stdio.h"
#include "stdlib.h"
#include <math.h>  //Header for additional Math Functions
#include <time.h>		// contains function to convert UTC to Epoch Time
//=======================================================================================

#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	UBX_OK = 2,
	UBX_ACK_ACK = 1,
	UBX_ACK_NACK = 0,
	UBX_ERROR = -1,
	UBX_TIMEOUT_Tx = -2,
	UBX_TIMEOUT_Rx = -3,

} UBX_MSG_t;

typedef enum{
	GPS_Init_OK = 1,
	GPS_Init_Ack_Error = 2,
	GPS_Init_Baud_Config_Error = 3,
	GPS_Init_MSG_Config_Error = 4,
	GPS_Init_Ack_Tx_Error = 5

}GPS_Init_msg_t;

/* private structs */

/*
 * Coordinate Object
 *
 * Stores the Cordinates of GPS in the form DDMM.mmmm
 * where DD - Degrees
 * 	     MM - Minutes
 * 	   mmmm - Fractional minutes
 * Variables:	Name.............Type.................................Description
 * 				lat..............float32_t............................GPS Lattitude
 * 				longi............float32_t............................GPS Longitude
 */
typedef struct
{
	float_t lat;
	float_t longi;
}Coord_t;
/*
 * DOP Object
 *
 * Dilation of Precision is a metric of how the elevation, satelite number and satelite spread
 * affects the accuracy of the signal. This object allows for the conversion and storage of a
 * 32 bit float value representing a DOP into an 8 bit digit and 8 bit precision the range of
 * values outputted from the GPS is between 0,0 and 99.9. This will allow for accuracy up to 2
 * decimal places.
 *
 * Variables:	Name.............Type.................................Description
 * 				digit............uint8_t..............................8 bit representation of the whole number
 * 				precision........uint8_t..............................2 significant places representation of the fraction
 *
 */
typedef struct
{
	uint8_t digit;
	uint8_t precision;
}DOP_t;

/*
 * Diagnostic Object
 *
 * Structure to Hold the GPS data signal diagnostics
 *
 * Variables:	Name.............Type.................................Description
 * 				PDOP.............DOP_t................................Positional Dilation of Precision (3D)
 * 				HDOP.............DOP_t................................Horizontal Dilation of Precision
 * 				VDOP.............DOP_t................................Vertical   Dilation of Precision
 * 				num_sats.........uint8_t..............................Number of Satelites used to obtain positional Fix
 * 				fix_type.........uint8_t..............................number between 1-3 describing the type of fix obtained
 *
 * Fix types
 * 1 - No Fix
 * 2 - 2D Fix (No altitude)
 * 3 - 3D Fix
 */
typedef struct
{
	DOP_t PDOP;
	DOP_t HDOP;
	DOP_t VDOP;
	uint8_t num_sats;
	uint8_t fix_type;
}Diagnostic_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_TX_BUFFER_SIZE 2048
#define DMA_RX_BUFFER_SIZE 2048
#define GNSS_BUFFER_SIZE 2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

/* USER CODE BEGIN PV */
Coord_t GPS_coord;		//storage variable for coordinates from GPS
uint32_t eTime;			//storage variable for Epoch Time
Diagnostic_t diag;		//storage variable for Diagnostic information

uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
uint8_t GNSS_Buffer[GNSS_BUFFER_SIZE];
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];
uint8_t M2M_Txfer_Cplt, TIM_IDLE_Timeout,TX_Cplt;
int gnss_length;
uint8_t packet_full;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//USART CONFIG FUNCTIONS
HAL_StatusTypeDef USART_Set_Baudrate(UART_HandleTypeDef* huart, TIM_HandleTypeDef *htim, uint32_t baud);

//CUSTOM IRQ Handle Functions
void USART_GPS_IRQHandler(UART_HandleTypeDef *huart);
void DMA_GNSS_MEM_IRQHandler(DMA_HandleTypeDef *hdma);
void DMA_GNSS_Periph_IRQHandler(DMA_HandleTypeDef *hdma_periph, DMA_HandleTypeDef *hdma_mem);
//GPS FUNCTIONS
GPS_Init_msg_t init_GPS(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
UBX_MSG_t UBX_Send_Ack(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim);
UBX_MSG_t UBX_Configure_Baudrate(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
UBX_MSG_t UBX_Configure_Messages(UART_HandleTypeDef *huart);
//utility Functions
void  Clear_Buffer(uint8_t *buffer,uint32_t size);
uint8_t char_to_hex(char c);
uint8_t is_valid(char* nmeamsg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void  Clear_Buffer(uint8_t *buffer,uint32_t size)
{
	memset(buffer,0,size);
}

HAL_StatusTypeDef USART_Set_Baudrate(UART_HandleTypeDef* huart, TIM_HandleTypeDef *htim, uint32_t baud)
{
	//disable UART peripheral and change baud rate
 	 huart->Instance->CR1 &= ~USART_CR1_UE;
	 huart->Init.BaudRate = baud;
	 if(HAL_UART_Init(huart) != HAL_OK)
	 {
		return HAL_ERROR;
	 }
	 huart->Instance->CR1 |= USART_CR1_UE;
	 //clear all errors
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
	 	uint8_t temp = huart4.Instance->RDR;
	 	(void)temp;
	 	__HAL_UART_CLEAR_OREFLAG(huart);
	 }
	 //clear parity errors
	 if(__HAL_UART_GET_FLAG(huart,UART_FLAG_PE) == SET)
	 {
	 	__HAL_UART_CLEAR_PEFLAG(huart);
	 }
	 //clear hanging idle flag
	 if(__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE) == SET)
	 {
	  	__HAL_UART_CLEAR_IDLEFLAG(huart);
     }
	 //increase Timeout value to allow for longer waits
	 __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,1152000);
	 return HAL_OK;
}


void DMA_Test_Mem2MeM(void)
{
	  int i;
	  for (i = 0; i < 10; ++i)
	  {
		DMA_RX_Buffer[i] = i;
	  }
	  __HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel1,DMA_IT_TC);
	  HAL_DMA_Start(&hdma_memtomem_dma1_channel1,(uint32_t)DMA_RX_Buffer,(uint32_t)GNSS_Buffer,i);
	  while(M2M_Txfer_Cplt != SET);
}

void USART_TIM_RTO_Handler(TIM_HandleTypeDef *htim)
{
	if(__HAL_TIM_GET_IT_SOURCE(&htim2,TIM_IT_CC1))
	{
		//clear interrupt
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1|TIM_IT_UPDATE);
		//set reciever timeout flag
		TIM_IDLE_Timeout = 1;
		//disable timer
		HAL_TIM_Base_Stop_IT(htim);

	}
}
void DMA_GNSS_MEM_IRQHandler(DMA_HandleTypeDef *hdma)
{
		M2M_Txfer_Cplt = SET;

}

void DMA_GNSS_Periph_IRQHandler(DMA_HandleTypeDef *hdma_periph, DMA_HandleTypeDef *hdma_mem)
{
	if(__HAL_DMA_GET_IT_SOURCE(hdma_periph,DMA_IT_TC))
	{
		__HAL_DMA_CLEAR_FLAG(hdma_periph,DMA_FLAG_TC5);
		//stop timer and reset flag
		HAL_TIM_Base_Stop(&htim2);
		__HAL_TIM_DISABLE_IT(&htim2,TIM_IT_CC1);
		if(__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_CC1))
		{
			__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
		}
		TIM_IDLE_Timeout = RESET;

		//begin a Memory to Memory PEripheral transfer
		__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel1,DMA_IT_TC);
		HAL_DMA_Start(hdma_mem,(uint32_t)DMA_RX_Buffer,(uint32_t)GNSS_Buffer,DMA_RX_BUFFER_SIZE);

	}if(__HAL_DMA_GET_IT_SOURCE(hdma_periph,DMA_IT_HT))
	{
		__HAL_DMA_CLEAR_FLAG(hdma_periph,DMA_IT_HT);
		__HAL_DMA_DISABLE_IT(hdma_periph,DMA_IT_HT);
	}
	if(__HAL_DMA_GET_IT_SOURCE(hdma_periph,DMA_IT_TE))
	{
		__HAL_DMA_CLEAR_FLAG(hdma_periph,DMA_IT_TE);
		__HAL_DMA_DISABLE_IT(hdma_periph,DMA_IT_TE);
	}
}

void USART_GPS_IRQHandler(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_IDLE))
	{
		uint32_t temp = huart->Instance->ISR;
		temp = huart->Instance->RDR;
		(void)temp;
		/* Case 1: gnss_length < DMA_RX BUFFER SIZE
		 * 		   disable Periph-Mem stream and
		 * 		   begin Mem - Mem transfer of known data
		 *
		 */
		//check flag in TIM2
		if(TIM_IDLE_Timeout == SET)
		{
			gnss_length = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
			//Disable DMA and unlink from UART
			HAL_UART_DMAStop(huart);
			//Timeout case: USART has recieved no data, Reciever timeout

			if(gnss_length > 0)
			{
				//begin transfer from mem to mem
				__HAL_DMA_ENABLE_IT(&hdma_memtomem_dma1_channel1,DMA_IT_TC);
				HAL_DMA_Start(&hdma_memtomem_dma1_channel1,(uint32_t)DMA_RX_Buffer,(uint32_t)GNSS_Buffer,gnss_length);

			}else
			{
			/*
			 * Case 2: gnss_length == 0;
			 *
			 * Reciever has recieved no data and has thus timed out.
			 */
				M2M_Txfer_Cplt = HAL_TIMEOUT;
			}
			//clear tim flag
			TIM_IDLE_Timeout = 0;
			__HAL_UART_DISABLE_IT(huart,UART_IT_IDLE);
		}

		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_IDLE);
	} if(__HAL_UART_GET_IT_SOURCE(huart,UART_IT_TC))
	{

		HAL_UART_AbortTransmit_IT(huart);
		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_IDLE);
		__HAL_UART_DISABLE_IT(huart,UART_IT_IDLE);
		TX_Cplt = 1;

	}
	// additional error handling
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

UBX_MSG_t UBX_Send_Ack(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim)
{
	 uint8_t ubx_ack_string[] = {0xB5 ,0x62 ,0x06 ,0x09 ,0x0D ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFF ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x17 ,0x31 ,0xBF };
	 int size = (sizeof(ubx_ack_string)/sizeof(*ubx_ack_string));
	 for (int i = 0; i < size ; ++i)
	 {
	  	DMA_TX_Buffer[i] = ubx_ack_string[i];
	 }
	 TX_Cplt = 0;
	 if(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC))
	 {
		 __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TC);
	 }
	 __HAL_UART_ENABLE_IT(&huart4,UART_IT_TC);
	 if( HAL_UART_Transmit_DMA(&huart4,DMA_TX_Buffer, size) == HAL_OK)
	 {
	  //begin DMA Reception
	 while(TX_Cplt != SET);
	 TX_Cplt = 0; //clear flag
	 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
	 __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_TC);
	 if(__HAL_TIM_GET_FLAG(htim,TIM_FLAG_CC1) == SET)
	 {
		 __HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC1);
	 }
	 M2M_Txfer_Cplt = 0;
	 HAL_UART_Receive_DMA(huart,DMA_RX_Buffer, DMA_RX_BUFFER_SIZE);
	 __HAL_TIM_ENABLE_IT(htim,TIM_IT_CC1);
	 HAL_TIM_OC_Start_IT(htim, TIM_CHANNEL_1);
	 HAL_TIM_Base_Start_IT(htim);
	 }
	  while(M2M_Txfer_Cplt != SET)
	  {
		  //TODO: SET DEVICE TO LOW POWER MODE WHILE DMA TRASNFER OCCURS
		  if(M2M_Txfer_Cplt == HAL_TIMEOUT)
		  {
			  return UBX_TIMEOUT_Rx;
		  }
	  }
	  M2M_Txfer_Cplt = RESET;
	  char val = (char) 0xB5;
	  int index = (int)(strchr((char*)GNSS_Buffer,val))-(int)GNSS_Buffer;
	  UBX_MSG_t GPS_Acknowledgement_State;
	  if((index < 0) || (index >GNSS_BUFFER_SIZE))
	  {

	  }else{
	  uint8_t msg[10] = {0};
	  memcpy(msg,&GNSS_Buffer[index],10);

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
	  }
	  return GPS_Acknowledgement_State;
}

UBX_MSG_t UBX_Configure_Baudrate(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{

	//GPS is configured for 9600, change baud to 115200
	uint8_t ubx_baude_rate_config[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E};
	uint32_t size =  sizeof(ubx_baude_rate_config)/sizeof(ubx_baude_rate_config[0]);
	memcpy(DMA_TX_Buffer,ubx_baude_rate_config,size);
	if(HAL_UART_Transmit_DMA(huart,DMA_TX_Buffer,size) == HAL_OK)
	{
		 while(TX_Cplt != SET);
		 Clear_Buffer(DMA_TX_Buffer,DMA_TX_BUFFER_SIZE);
		 if(USART_Set_Baudrate(huart,htim,115200) != HAL_OK)
		 {
			 return UBX_ERROR;
		 }
		 return UBX_Send_Ack(huart,htim);
	}
	return UBX_TIMEOUT_Tx;
}

UBX_MSG_t UBX_Configure_Messages(UART_HandleTypeDef *huart)
{
	//clear all active/useless messages
	uint8_t NMEA_Clear_buffer[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x0A, 0x00, 0x04, 0x23, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x09, 0x00, 0x03, 0x21, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x0D, 0x00, 0x07, 0x29, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x0F, 0x00, 0x09, 0x2D, 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19} ;
	uint32_t size = sizeof(NMEA_Clear_buffer)/sizeof(NMEA_Clear_buffer[0]);
	 if(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC))
	 {
		__HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TC);
	 }
	__HAL_UART_ENABLE_IT(huart,UART_IT_TC);
	if(HAL_UART_Transmit_DMA(huart,NMEA_Clear_buffer,size) != HAL_OK)
	{
		return UBX_ERROR;
	}
	while(TX_Cplt != SET);
	TX_Cplt = 0;
	(void)NMEA_Clear_buffer;
	//enable messages GLL ZDA GSA
	uint8_t NMEA_msgs[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x01,0xFC,0x12,0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x01,0xFD,0x14,0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x08,0x01,0x03,0x20};
	size = sizeof(NMEA_msgs)/sizeof(NMEA_msgs[0]);
	 if(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC))
	 {
		 __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_TC);
	 }
	 __HAL_UART_ENABLE_IT(huart,UART_IT_TC);
	if(HAL_UART_Transmit_DMA(huart,NMEA_msgs,size) == HAL_OK)
	{
		while(TX_Cplt != SET);
		TX_Cplt = 0;
		return UBX_OK;
	}

	return UBX_ERROR;

}

GPS_Init_msg_t init_GPS(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma)
{
	//send acknowledgement
	UBX_MSG_t GPS_Acknowledgement_State = UBX_Send_Ack(huart,htim);
	if(GPS_Acknowledgement_State == UBX_ACK_ACK)
	{
		Clear_Buffer(DMA_TX_Buffer,DMA_TX_BUFFER_SIZE);
		Clear_Buffer(GNSS_Buffer,GNSS_BUFFER_SIZE);
		if( UBX_Configure_Baudrate(huart, htim) != UBX_ACK_ACK)
		{
			return GPS_Init_Baud_Config_Error;
		}

	}else if(GPS_Acknowledgement_State == UBX_TIMEOUT_Rx)
	{
		/*
		 * If Not recieving Ack-Ack on 115200, it could be possible that the device is
		 * already configured. change baud rate and try again
		 */
		//configure baud rate to 115200 and try again
		if(USART_Set_Baudrate(huart,htim,115200) == HAL_OK)
		{
			GPS_Acknowledgement_State = UBX_Send_Ack(huart,htim);
		}


	}else if(GPS_Acknowledgement_State == UBX_TIMEOUT_Tx)
	{
		return GPS_Init_Ack_Tx_Error;
	}
	//configure message buffer
	if( UBX_Configure_Messages(huart) != UBX_OK )
	{
		return GPS_Init_MSG_Config_Error;
	}
	return GPS_Init_OK;
}

uint8_t is_valid(char* nmeamsg)
{
	uint8_t flag = 0;

	char msg[4] = {0};
	for (int i = 0; i < 3; ++i)
	{
		msg[i] = *(nmeamsg+2+i);
	}
	if((strcmp((char*)msg,"GLL") != 0))
	{

		if (strcmp((char*)msg,"ZDA") != 0)
		{
			if(strcmp((char*)msg,"GSA") != 0)
			{
				return -1;
			}
			else
			{
				flag = 2;
			}
		}else
		{
			flag = 3;
		}

	}
	else
	{
		flag = 1;
	}
	/* check sum */
	uint16_t checksum = 0;
	while(*nmeamsg != '*')
	{
		checksum^= *nmeamsg;
		nmeamsg++;
	}
	uint8_t h = char_to_hex(*(++nmeamsg))*16;
	uint8_t l = char_to_hex(*(++nmeamsg));
	uint16_t checkbyte= h+l;

	if(!(checksum == checkbyte))
	{
		return -1;
	}

	return flag;
}
/*
 * Flag keeps track of successful coordinate retrievals
 */

uint8_t char_to_hex(char c)
{
	if(c == '\0')
	{
		return 0;
	}
	if ((c >='0') &&(c <='9'))
	{
		return (uint8_t)c - 48;
	}
	if((c >='a') &&(c <='f'))
	{
		return (uint8_t)c - 87;
	}
	if((c >='A') &&(c <='F'))
	{
		return (uint8_t)c - 55;
	}
	return -1; //invalid charachter
}

/*
 * Parser Functions:
 * Extract key data from NMEA message strings.
 */
uint8_t parse_ZDA(char* ZDAstring)
{
	time_t t;
	struct tm *timepointer;

	t = time(NULL);
	timepointer = localtime(&t);
	/* Get UTC time*/
	while(*ZDAstring++ != ',');
	char* temp = ZDAstring++;
	for (int i = 0; i < strlen(ZDAstring); ++i)
	{
		if ((ZDAstring[i]==',')&&(ZDAstring[i+1] == ','))
		{
			/* Data is invalid*/
			return -1;
		}
	}
	timepointer->tm_hour = (temp[0]-48)*10+ (temp[1]-48)+2;
	timepointer->tm_min = (temp[2]-48)*10+ (temp[3]-48)-1;
	timepointer->tm_sec = (temp[4]-48)*10+ (temp[5]-48) -1 ;
	while(*ZDAstring++ != ',');
	temp = ZDAstring;
	timepointer->tm_mday = (temp[0]-48)*10+ (temp[1]-48);
	timepointer->tm_mon = (temp[3]-48)*10+ (temp[4]-48)-1;
	timepointer->tm_year = (temp[6]-48)*1000 +(temp[7]-48)*100 + (temp[8]-48)*10 +(temp[9]-48)-1900;
	time_t result;

	 result = mktime(timepointer);
	 eTime =  result;
	return 0;
}

uint8_t Parse_GLL(char* GLLstring)
{
	/* Extract latitude*/
	uint8_t flag = 0;
	GLLstring+= 6;
	char* temp = GLLstring;
	uint8_t count = 0;
	while(*GLLstring++ != ',')
	{
		count++;
	}
	if((count > 0))
	{
		temp[count] = '\0';
		int8_t sign = 1 -2*( temp[++count] =='S');
		GPS_coord.lat = sign*atof(temp);
		flag++;
	}

	/* Extract longitude */
	while(*GLLstring++ !=',');
	temp = GLLstring;
	count = 0;
	while(*GLLstring++ != ',')
	{
			count++;
	}
	if((count > 0))
	{
			temp[count] = '\0';
			int8_t sign = 1 -2*( temp[++count] =='W');
			GPS_coord.longi = sign*atof(temp);
			flag++;

	}

return flag;

}

uint8_t parse_GSA(char* GSA_string)
{
	/* Isolate Dilation of Precisions*/
	uint8_t count = 0;
	char* t = GSA_string;
	while(count < 2)
	{
		if(*t++==',')count++;
	}
	diag.fix_type = (*t++-48);

	//field 3 - 15 indicate satelites
	uint8_t numsats = 0;
	uint8_t numfields = 0;
	while(numfields < 12)
	{
		uint8_t count = 0;
		while(*++t !=',')count++;
		if(count > 0)
		{
			numsats++;
		}
		numfields++;

	}
	diag.num_sats = numsats;
	DOP_t dop[3] = {0};
	for (int i = 0; i < 3; ++i)
	{
		//get digit
		while(*++t != '.')
		{
			dop[i].digit = dop[i].digit*10 +(*t-48);
		}
		while(*++t != ',')
		{
			dop[i].precision = dop[i].precision*10+(*t-48);
		}
	}
	/*
	 * If successful, add to diagnostic struct
	 *
	 */
diag.HDOP = dop[0];
diag.PDOP = dop[1];
diag.VDOP = dop[2];
	return 0;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  //initialise Peripherals
  	  MX_GPIO_Init();
  	  MX_DMA_Init();
  	  MX_UART4_Init();
  	  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if(init_GPS(&huart4,&htim2,&hdma_memtomem_dma1_channel1)== GPS_Init_OK)
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
  __HAL_DMA_ENABLE_IT(huart4.hdmarx, DMA_IT_TC);
  if(__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_CC1) == SET)
  {
   __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
  }
  M2M_Txfer_Cplt = 0;
  __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  while (1)
  {
    /* USER CODE END WHILE */
	//sample GPS
	__HAL_TIM_ENABLE_IT(&htim2,TIM_IT_CC1);
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_DMA(&huart4,DMA_RX_Buffer,DMA_RX_BUFFER_SIZE);
	while(M2M_Txfer_Cplt != SET);
	M2M_Txfer_Cplt = RESET;
	char* msg = strtok((char*)GNSS_Buffer, "$");
	while(msg != NULL)
	{
		switch(is_valid(msg))
		{
		  case 1:
			if(Parse_GLL(msg) == 2)
			{
				packet_full |= 0b1;
			}
			break;
		  case 2:
			if(parse_GSA(msg) == 0)
			{
				packet_full |= 0b10;
			}
			break;
	      case 3:
	    	if(parse_ZDA(msg) == 0)
	    	{
	    		packet_full |= 0b100;
	    	}
	    	break;
		  default:
			// invalid case
			break;
		}
		msg = strtok(NULL,"$");
	}
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
  htim2.Init.Period = 65536;
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
  sConfigOC.Pulse = CCR1_VAL;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */
	/*
	 * Configure USART for auto baud rate detection as per Application Note 4908
	 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
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

  /* USER CODE END UART4_Init 2 */

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_0;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  CLEAR_REG(hdma_uart4_rx.DmaBaseAddress->ISR);
  CLEAR_REG(hdma_uart4_tx.DmaBaseAddress->ISR);
  CLEAR_REG(hdma_memtomem_dma1_channel1.DmaBaseAddress->ISR);

  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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

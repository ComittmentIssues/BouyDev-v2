/*
 * HAL_GPS.c
 *
 *  Created on: Mar 11, 2020
 *      Author: jamie
 */

#include "HAL_GPS.h"

void  Clear_Buffer(uint8_t *buffer,uint32_t size)
{
	memset(buffer,0,size);
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
	 	uint8_t temp = huart->Instance->RDR;
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
	 __HAL_UART_ENABLE_IT(huart,UART_IT_TC);
	 if( HAL_UART_Transmit_DMA(huart,DMA_TX_Buffer, size) == HAL_OK)
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

void USART_TIM_RTO_Handler(TIM_HandleTypeDef *htim)
{
	if(__HAL_TIM_GET_IT_SOURCE(htim,TIM_IT_CC1))
	{
		//clear interrupt
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1|TIM_IT_UPDATE);
		//set reciever timeout flag
		TIM_IDLE_Timeout = 1;
		//disable timer
		HAL_TIM_Base_Stop_IT(htim);

	}
}

void DMA_GNSS_MEM_IRQHandler(DMA_HandleTypeDef *hdma, TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart)
{

		M2M_Txfer_Cplt = SET;
		if(log_gps)
		{
			Clear_Buffer(DMA_RX_Buffer,DMA_RX_BUFFER_SIZE);
			//reset pointer
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
			if(__HAL_TIM_GET_IT_SOURCE(htim,TIM_IT_CC1))
			{
				__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC1);
				htim->Instance->CNT = 0;
			}
			huart->hdmarx->DmaBaseAddress->IFCR = 0x3FU << huart->hdmarx->ChannelIndex; // clear all interrupts
			huart->hdmarx->Instance->CMAR = (uint32_t)DMA_RX_Buffer; //reset the pointer
			huart->hdmarx->Instance->CNDTR = DMA_RX_BUFFER_SIZE; //set the number of bytes to expect
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
			if(packet_full != 7)
			{
				__HAL_DMA_ENABLE(huart->hdmarx);
				HAL_UART_DMAResume(huart);
			}
		}

}

void DMA_GNSS_Periph_IRQHandler(DMA_HandleTypeDef *hdma_periph, DMA_HandleTypeDef *hdma_mem, TIM_HandleTypeDef *htim)
{
	if(__HAL_DMA_GET_IT_SOURCE(hdma_periph,DMA_IT_TC))
	{
		__HAL_DMA_CLEAR_FLAG(hdma_periph,DMA_FLAG_TC5);
		//stop timer and reset flag
		HAL_TIM_Base_Stop(htim);
		__HAL_TIM_DISABLE_IT(htim,TIM_IT_CC1);
		if(__HAL_TIM_GET_FLAG(htim,TIM_FLAG_CC1))
		{
			__HAL_TIM_CLEAR_FLAG(htim,TIM_FLAG_CC1);
		}
		TIM_IDLE_Timeout = RESET;

		//begin a Memory to Memory PEripheral transfer
		__HAL_DMA_ENABLE_IT(hdma_mem,DMA_IT_TC);
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

void USART_GPS_IRQHandler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_mem)
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
			if(log_gps)
			{
				HAL_UART_DMAPause(huart);
				huart->hdmarx->Instance->CCR &= ~DMA_CCR_EN;
				__NOP();

			}else
			{
				HAL_UART_DMAStop(huart);
			}
			//Timeout case: USART has recieved no data, Reciever timeout

			if(gnss_length > 0)
			{
				//begin transfer from mem to mem
				__HAL_DMA_ENABLE_IT(hdma_mem,DMA_IT_TC);
				HAL_DMA_Start(hdma_mem,(uint32_t)DMA_RX_Buffer,(uint32_t)GNSS_Buffer,gnss_length);

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


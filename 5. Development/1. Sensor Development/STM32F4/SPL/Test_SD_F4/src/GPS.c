/*
 * GPS.c
 *
 *  Created on: Jun 20, 2019
 *      Author: Jamie
 */


#include "GPS.h"



void init_USART_GPS(void)
{

		//init GPIOB for AF
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Pin = (GPIO_USART1_RX | GPIO_USART1_TX);
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOB,&GPIO_InitStruct);
		//AF MAPPing
		GPIO_PinAFConfig(GPIOB,GPIO_USART1_RX_SRC,GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB,GPIO_USART1_TX_SRC,GPIO_AF_USART1);

		//USART CONFIG
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
		USART_InitTypeDef USART_InitStructure;
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_BaudRate = GPS_BAUDRATE;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART1, &USART_InitStructure);

		//configure USART for idle interrupt
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
		//enable USART
		USART_Cmd(USART1,ENABLE);

		//DMA init
#ifdef STM32_GNSS_USE_DMA
		DMA_InitTypeDef DMA_InitStructure;
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

		/* De-initialize DMA RX & TX Stream */
			DMA_DeInit(DMA2_Stream2);
			while (DMA_GetCmdStatus(DMA2_Stream2 ) != DISABLE) { ; }
			DMA_DeInit(DMA2_Stream7);
			while (DMA_GetCmdStatus(DMA2_Stream7 ) != DISABLE) { ; }
			DMA_DeInit(DMA2_Stream0); //DMA2 Stream 0 for memory streaming
			while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){;}

		/* CONFIGURE DMA STREAM on DMA 2
		 *  Stream 2: RX
		 *  Stream 7: TX
		 *  Stream 0: MEM
		 */
			/* shared DMA configuration values */
				DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
				DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
				DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
				DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

				DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_RX_Buffer; //feeds into buffer direct
				DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
				DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
				DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

				DMA_InitStructure.DMA_Channel = DMA_Channel_4;
				DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
				DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
				DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
				DMA_InitStructure.DMA_BufferSize = DMA_RX_BUFFER_SIZE;

				DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
				DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;

				DMA_Init(DMA2_Stream2, &DMA_InitStructure);

				// enable the interrupt in the NVIC
				NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
				DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);
#ifdef STM32_GMEM_USE_DMA
	/* UART - MEM */
	/* shared DMA configuration values */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DMA_RX_Buffer;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)GNSS_LOG_Buffer;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_BufferSize = GNSS_LOG_BUFFER_SIZE;

	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;

	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	// enable the interrupt in the NVIC
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

#endif
DMA_Cmd(DMA2_Stream2, ENABLE);
while (DMA_GetCmdStatus(DMA2_Stream2 ) != ENABLE) { ; }
#endif

}
void deinit_USART_GPS(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	USART_DeInit(USART1);
	USART_Cmd(USART1, DISABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);

	/* All SPI-Pins to input with weak internal pull-downs */
	GPIO_InitStructure.GPIO_Pin = GPIO_USART1_RX|GPIO_USART1_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//==========================================================================
/* Communication Commands */

void USART_transmit_byte(uint8_t byte)
{
	USART1->DR = (uint16_t)byte;
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) != SET);
}
void USART_transmit_string(unsigned char* data)
{
	uint16_t length = sizeof(data)/sizeof(data[0]);

	for (int i = 0; i < length; i++)
	{
		USART_transmit_byte(*data++);
	}

}
unsigned char USART_receive_byte(void)
{
	while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) != SET);
	return (uint8_t)USART_ReceiveData(USART1);

}

//==========================================================================
/* USART BUFFER FUNCTIONS*/
void zero_dma_gnss_memory(void)
{
		for (int i = 0; i < DMA_RX_BUFFER_SIZE; i++)
		{
			DMA_RX_Buffer[i] = 0;
		}
}
void zero_gnss_memory(void)
	{
		for (int i = 0; i < GNSS_LOG_BUFFER_SIZE; i++){
			GNSS_LOG_Buffer[i] = 0;
		}
	}

//==========================================================================
/* IRQ HANDLERS*/
/*
 * called when USART has finished receiving
 * disables the stream
 */
void USART1_IRQHandler(void)
{
	//idle line detection
	if(USART_GetFlagStatus(USART1,USART_FLAG_IDLE) != RESET)
	{
		//clear reg
		RX_COMPLETE_FLAG = 0;
		volatile uint32_t tmp;
		tmp = USART_GetITStatus(USART1, USART_IT_IDLE);
		tmp = USART_ReceiveData(USART1);
		(void)tmp;

		DMA_Cmd(DMA2_Stream2,DISABLE);
		while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);

	}

}

/* UART-MEM IRQ HANDLER */
#ifdef STM32_GNSS_USE_DMA
void DMA2_Stream2_IRQHandler(void)
{
	//check transfer complete flag
	if(DMA_GetFlagStatus(DMA2_Stream2,DMA_FLAG_TCIF2) == SET)
	{

		//set log to off
		RX_COMPLETE_FLAG = 0;
		//get data that still needs to be transferred
		gnss_length = DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream2);
		//zero_gnss_memory();

		//clear USART-DMA Transfer bit
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);

		/* Enable DMA transfer to memory */
		#ifdef STM32_GMEM_USE_DMA
		DMA_Cmd(DMA2_Stream0, ENABLE);
		while (DMA_GetCmdStatus(DMA2_Stream0) != ENABLE) { ; }

		#else
		DMA_Cmd(DMA2_Stream2, ENABLE);
		while (DMA_GetCmdStatus(DMA_Stream_USART_GNSS_RX ) != ENABLE) { ; }
		#endif
	}

}
#endif
/* MEM _ MEM IRQ HANDLER */
#ifdef STM32_GMEM_USE_DMA
void DMA2_Stream0_IRQHandler (void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0) != RESET)
	{

		RX_COMPLETE_FLAG = 1;
		// transfer message to buffer
		char* msg;
		msg = strtok((char*)DMA_RX_Buffer,"$");
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
		zero_dma_gnss_memory();
		zero_gnss_memory();
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		/* Enable DMA transfer */
		DMA_Cmd(DMA2_Stream2, ENABLE);
		while (DMA_GetCmdStatus(DMA2_Stream2 ) != ENABLE);

	}
}
#endif

//==========================================================================

/*
 * checks if the data is valid
 * Criteria: 1 - acceptable msg
 * Criteria: 2 - valid checksum
 */

/*
 * Return status:
 *  -1 invalid
 *   1 GLL msg
 *   2 GGA msg
 *   3 ZDA msg
 */
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

//==========================================================================

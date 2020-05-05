/*
 * GPS.c
 *
 *  Created on: Jun 20, 2019
 *      Author: Jamie
 */

/* Includes */

#include "GPS.h" //Import macros and functions from GPS.h header

//======================================================================================================================

/* private flags */

static UBX_MSG_t GPS_Acknowledgement_State; // flag showing the result of the received UBX message

//======================================================================================================================

/* private Functions Prototypes */

/*
 * FUNCTION : UBX_Send_Ack(char* msg);
 * return type: UBX_MSG_t
 *
 * Paramters:	NAME..............TYPE............ DESCRIPTION
 * 				msg...............char*........... UBX message string from GPS
 *
 * Description: Function Sends a UBX CFG-CFG command (as per u-blox7-V14_ReceiverDescriptionProtocolSpec documentation)
 * 				And determines the state of the GPS based on the message recieved. GPS is on and working if the message
 * 				is read and understood by the GPS, it will return an acknowledgement. If no Acknowledgement is recieved,
 * 				the function will timeout and return an ERROR. If the program recieved the command but does not understand it, it will return a NACK.
 *
 *
 * Return Type: NAME........................TYPE............................VALUE
 *  			ACKNOWLEDGEMENT.............UBX_ACK_ACK.....................1
 *  			NO Acknowledgment...........UBX_ACK_NACK....................0
 *  			ERROR.......................UBX_ERROR......................-1
 */

static UBX_MSG_t UBX_Send_Ack(void);

/*
 * Description: Function To Configure The GPS Baudrate.
 * 				When plugged in, The GPS has a factory set baud rate
 * 				of 9600 bit/s. To speed up data transfer, this needs to be increased to 1152000
 * 				The function sends a UBX Binary message to configure the baud rate. The function
 * 				will update the USART peripheral to match the baud rate and will attempt to recieve
 * 				an acknolwedgement from the device
 *
 * 	parameters: void
 *
 * Return Type: NAME........................TYPE............................VALUE
 *  			ACKNOWLEDGEMENT.............UBX_ACK_ACK.....................1
 *  			NO Acknowledgment...........UBX_ACK_NACK....................0
 *  			ERROR.......................UBX_ERROR......................-1
 */
static UBX_MSG_t UBX_Configure_BaudRate(void);

//====================================================================================================

/*  GPS Initialization Functions */

void init_USART_GPS(uint32_t bauderate)
{

		//init GPIOB for AF
		RCC_AHB1PeriphClockCmd(RCC_GPIOPeriph,ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Pin = (GPIO_USART_RX | GPIO_USART_TX);
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIO_GPS,&GPIO_InitStruct);
		//AF MAPPing
		GPIO_PinAFConfig(GPIO_GPS,GPIO_USART_RX_SRC,USART_AF);
		GPIO_PinAFConfig(GPIO_GPS,GPIO_USART_TX_SRC,USART_AF);

		//USART CONFIG
#ifdef USE_APB2
		RCC_APB2PeriphClockCmd(RCC_USARTPeriph,ENABLE);
#endif

#ifdef USE_APB1
		RCC_APB1PeriphClockCmd(RCC_USARTPeriph,ENABLE);
#endif
		/*
		 * Configurations:
		 *  MODE = Tx & Rx
		 *  Word Length = 8 bits
		 *  Stop Bits = 1
		 *  Parity = None
		 *  Flow Control: Off
		 */
		USART_InitTypeDef USART_InitStructure;
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_BaudRate = bauderate; 	//parameter of function
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART_GPS, &USART_InitStructure);

		//configure USART for idle interrupt
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USART_GPS_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART_GPS, USART_IT_IDLE, ENABLE);
		//enable USART
		USART_Cmd(USART_GPS,ENABLE);

		//DMA init
#ifdef STM32_GNSS_USE_DMA
		DMA_InitTypeDef DMA_InitStructure;
		USART_DMACmd(USART_GPS, USART_DMAReq_Rx, ENABLE);

		RCC_AHB1PeriphClockCmd(RCC_USART_DMAPeriph,ENABLE);

		/* De-initialize DMA RX & TX Stream */
			DMA_DeInit(USART_DMA_RxStream);
			while (DMA_GetCmdStatus(USART_DMA_RxStream ) != DISABLE) { ; }
			DMA_DeInit(USART_DMA_TxStream);
			while (DMA_GetCmdStatus(USART_DMA_TxStream) != DISABLE) { ; }
			DMA_DeInit(DMA2_Stream0); //DMA2 Stream 0 for memory streaming
			while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){;}

		/* CONFIGURE DMA STREAM on DMA 1
		 *  Stream 2: RX
		 *  Stream 4: TX
		 *  Stream 0: MEM
		 */
			/* shared DMA configuration values */
				DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART_GPS->DR));
				DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
				DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
				DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

				DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DMA_RX_Buffer; //feeds into buffer direct
				DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
				DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
				DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

				DMA_InitStructure.DMA_Channel = USART_DMA_RxChannel;
				DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
				DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
				DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
				DMA_InitStructure.DMA_BufferSize = DMA_RX_BUFFER_SIZE;

				DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
				DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;

				DMA_Init(USART_DMA_RxStream, &DMA_InitStructure);

				// enable the interrupt in the NVIC when DMA detects transfer complete
				NVIC_InitStructure.NVIC_IRQChannel = DMA_Rx_IRQn;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
				DMA_ITConfig(USART_DMA_RxStream, DMA_IT_TC, ENABLE);
#ifdef STM32_GMEM_USE_DMA
	/* UART - MEM */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
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
DMA_Cmd(USART_DMA_RxStream, ENABLE);
while (DMA_GetCmdStatus(USART_DMA_RxStream ) != ENABLE) { ; }
#endif

}

void deinit_USART_GPS(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* De-initialize DMA RX & TX Stream */
	DMA_DeInit(USART_DMA_RxStream);
	while (DMA_GetCmdStatus(USART_DMA_RxStream ) != DISABLE) { ; }
	DMA_DeInit(USART_DMA_TxStream);
	while (DMA_GetCmdStatus(USART_DMA_TxStream) != DISABLE) { ; }
	DMA_DeInit(DMA2_Stream0); //DMA2 Stream 0 for memory streaming
	while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){;}
	RCC_AHB1PeriphClockCmd(RCC_USART_DMAPeriph,DISABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,DISABLE);

	USART_DeInit(USART_GPS);
	USART_Cmd(USART_GPS, DISABLE);
#ifdef USE_APB2
		RCC_APB2PeriphClockCmd(RCC_USARTPeriph,DISABLE);
#endif

#ifdef USE_APB1
		RCC_APB1PeriphClockCmd(RCC_USARTPeriph,DISABLE);
#endif

	/* All SPI-Pins to input with weak internal pull-downs */
	GPIO_InitStructure.GPIO_Pin = GPIO_USART_RX|GPIO_USART_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIO_GPS, &GPIO_InitStructure);
}

uint8_t init_GPS(void)
{
	init_USART_GPS(9600);
	//send acknowledgement
	UBX_MSG_t flag = UBX_Send_Ack();
	if(flag == UBX_ACK_ACK) //indicates that system is online and functioning
	{
		//configure baude rate
		flag = UBX_Configure_BaudRate();

	} else if(flag == UBX_ERROR) // try different baude rate
	{
		deinit_USART_GPS();
		//it may be that the GPS has already been configured in which case, de init and try again with different baude rate
		init_USART_GPS(115200);
		if (UART4->SR & USART_SR_ORE) // Overrun Error
		UART4->SR &= ~USART_SR_ORE;
		if (UART4->SR & USART_SR_NE) // Noise Error
		UART4->SR &= ~USART_SR_NE;
		if (UART4->SR & USART_SR_FE) // Framing Error
		UART4->SR &= ~USART_SR_FE;
		USART_Cmd(USART_GPS,ENABLE);
		//no config needed
		flag = UBX_Send_Ack(); //test if gps acknowledges on new baudrate

	}
	if(flag == UBX_ERROR) //if there is still no response, initialization unsuccessful
	{
		deinit_USART_GPS();
		return 0;
	}
	 //enable messages GLL ZDA GSA
	 uint8_t gll[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x01,0xFC,0x12};
	 int len = sizeof(gll)/sizeof(gll[0]);
	 USART_transmit_data(gll,len);
	 uint8_t gsa[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x01,0xFD,0x14};
	 uint8_t zda[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x08,0x01,0x03,0x20};
	 USART_transmit_data(gsa,len);
	 USART_transmit_data(zda,len);
	 //set device ready to recieve GPS
	 Recieve_GPS_Data = 1;
	 Ack_message = 0;
	 return 1;
}

//==========================================================================
/* Communication Commands */

void USART_transmit_byte(uint8_t byte)
{
	USART_GPS->DR = (uint16_t)byte;
	while(USART_GetFlagStatus(USART_GPS,USART_FLAG_TXE) != SET);
}

void USART_transmit_data(uint8_t* data,int len)
{

	for (int i = 0; i < len; i++)
	{
		USART_transmit_byte(*data++);
	}

}

unsigned char USART_receive_byte(void)
{
	while(USART_GetFlagStatus(USART_GPS,USART_FLAG_RXNE) != SET);
	return (uint8_t)USART_ReceiveData(USART_GPS);

}

//==========================================================================
/* GPS Configuration Commands */

UBX_MSG_t UBX_Send_Ack(void)
{
	Ack_message = 1;
	GPS_Acknowledgement_State = 0;
	uint8_t ubx_ack_string[] = {0xB5 ,0x62 ,0x06 ,0x09 ,0x0D ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0xFF ,0xFF ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x17 ,0x31 ,0xBF };
	int size = sizeof(ubx_ack_string)/sizeof(ubx_ack_string[0]);
	USART_transmit_data(ubx_ack_string,size);
	//wait for acknowledge
	Delay_begin_Timeout(100); //set function to timeout if no response recieved
	while(Ack_message)
	{
		if(timeout)
		{
			return UBX_ERROR;
		}
	}

	return GPS_Acknowledgement_State;
}

UBX_MSG_t UBX_Configure_BaudRate(void)
{
	uint8_t ubx_baude_rate_config[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0xC0,0x7E};
	uint8_t ubx_poll_config[] = {0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};
	int size = sizeof(ubx_baude_rate_config)/sizeof(ubx_baude_rate_config[0]);
	USART_transmit_data(ubx_baude_rate_config,size);
	size = sizeof(ubx_poll_config)/sizeof(ubx_poll_config[0]);
	USART_transmit_data(ubx_baude_rate_config,size);

	//disable UART and reconfigure with new bauderate
	while(USART_GetFlagStatus(USART_GPS,USART_FLAG_TXE) != SET);
	deinit_USART_GPS();
	init_USART_GPS(115200);
	//clear Overun and framing Errors
	if (UART4->SR & USART_SR_ORE) // Overrun Error
	UART4->SR &= ~USART_SR_ORE;

	if (UART4->SR & USART_SR_NE) // Noise Error
	UART4->SR &= ~USART_SR_NE;

	if (UART4->SR & USART_SR_FE) // Framing Error
	UART4->SR &= ~USART_SR_FE;

	USART_Cmd(USART_GPS,ENABLE);

	return UBX_Send_Ack();
}

//==============================================================================

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

/* IRQ Handlers */

/*
 * These Functions handle the Transfer of ALL messages from the
 * GPS to the micro. Data is transmitted constantly from the GPS.
 * When a block of data is being recieved, the end of transfer is
 * marked by the Rx line going idle. When this happens, the USART_IRQ
 * will Clear the data register and Disable The Peripheral To Memory Stream
 *
 * The DMA Peripheral- Memory IRQ will fire when the system has detected that
 * the Peripheral - Memory Transfer is completed. IT will then determine how much data has been transfered into the buffer
 * Finally, it will enable the Memory - Memory Data Stream
 *
 * The DMA Memory - Memory IRQ will fire when data transfer to the memory address has been completed.
 * Here the handler will process the data depending on what the status of the flags are.
 *
 */

void USART_GPS_IRQHandler(void)
{
	//idle line detection
	if(USART_GetFlagStatus(USART_GPS,USART_FLAG_IDLE) != RESET)
	{
		//clear reg
		RX_COMPLETE_FLAG = 0;
		volatile uint32_t tmp;
		tmp = USART_GetITStatus(USART_GPS, USART_IT_IDLE);
		tmp = USART_ReceiveData(USART_GPS);
		(void)tmp;

		DMA_Cmd(USART_DMA_RxStream,DISABLE);
		while(DMA_GetCmdStatus(USART_DMA_RxStream) != DISABLE);

	}

}

/* UART-MEM IRQ HANDLER */
#ifdef STM32_GNSS_USE_DMA
void DMA_Rx_IRQHandler(void)
{
	//check transfer complete flag
	if(DMA_GetFlagStatus(USART_DMA_RxStream,DMA_Rx_Flag_TCF) == SET)
	{

		RX_COMPLETE_FLAG = 0;
		//get data that still needs to be transferred
		gnss_length = DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(USART_DMA_RxStream);
		//zero_gnss_memory();

		//clear USART-DMA Transfer bit
		DMA_ClearITPendingBit(USART_DMA_RxStream, DMA_Rx_IT_TCF);

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
		if(Ack_message) // system expecting a UBX acknowledgement
		{
			Ack_message = 0; //system no longer waiting on response
			//find the UBX message
			char val = (char) 0xB5;
			int index = (int)(strchr((char*)DMA_RX_Buffer,val))-(int)DMA_RX_Buffer;
			char msg[10];
			memcpy(msg,&DMA_RX_Buffer[index],10);
			//Header
			uint16_t header = ((uint16_t)msg[0]<<8) | ((uint16_t)msg[1]);
			if(header == 0xb562)
			{
				//checksum calculation
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
			else
			{
				GPS_Acknowledgement_State = UBX_ERROR;
			}

		}

		//system Expecting NMEA message
		if(Recieve_GPS_Data)
		{
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
		}
		zero_dma_gnss_memory();
		zero_gnss_memory();
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);

		/* Enable DMA transfer */
		DMA_Cmd(USART_DMA_RxStream, ENABLE);
		while (DMA_GetCmdStatus(USART_DMA_RxStream ) != ENABLE);

	}
}
#endif

//==========================================================================

/*  Data Processing Functions */

/*
 * checks if the data is valid
 * Criteria: 1 - acceptable msg
 * Criteria: 2 - valid checksum
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

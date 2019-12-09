/*
 * Iridium.c
 *
 *  Created on: Jun 11, 2019
 *      Author: Jamie
 */

#include "Iridium.h" //function definitions, macros and state flags

//====================================================================

/* Initialization Functions */

/*
 * USART Parameters
 *
 *  baudrate: 19200
 *  8 bits
 *  1 stop bit
 *  No Parity
 *
 */

void init_Iridium_USART(void)
{
	// Initialization structs

	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART5_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;


	//clear flags
	IR_Rx_done = 0;
	status_Received = 0;
	bin_message_received = 0;
	network_available = 0;

	/* ENABLE RCC */
	Iridium_USART_PeriphClockCommand(Iridium_USART_RCCPeriph ,ENABLE);

#ifdef USE_2_GPIO_Ports
	Iridium_GPIO_PeriphClockCommand(Iridium_GPIO_RCCPeriph| Iridium_GPIO_RCCPeriph_2,ENABLE);
#else
	Iridium_GPIO_PeriphClockCommand(Iridium_GPIO_RCCPeriph,ENABLE);
#endif

	/* Configure GPIO to alternate function Open Drain Pull UP */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

#ifdef USE_2_GPIO_Ports
	//first Tx Pin then RX pin
	GPIO_InitStruct.GPIO_Pin = Iridium_USART_TX;
	GPIO_Init(Iridium_GPIO,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = Iridium_USART_RX;
	GPIO_Init(Iridium_GPIO_2,&GPIO_InitStruct);
	GPIO_PinAFConfig(Iridium_GPIO_2,Iridium_USART_RXsrc,Iridium_GPIO_AF);
	GPIO_PinAFConfig(Iridium_GPIO,Iridium_USART_TXsrc,Iridium_GPIO_AF);
#else
	GPIO_InitStruct.GPIO_Pin = (Iridium_USART_RX | Iridium_USART_TX);
	GPIO_Init(Iridium_GPIO,&GPIO_InitStruct);
	GPIO_PinAFConfig(Iridium_GPIO,Iridium_USART_RXsrc,Iridium_GPIO_AF);
	GPIO_PinAFConfig(Iridium_GPIO,Iridium_USART_TXsrc,Iridium_GPIO_AF);
#endif

	//AF Mapping
	/* Configure USART	*/
	USART5_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART5_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART5_InitStruct.USART_StopBits = USART_StopBits_1;
	USART5_InitStruct.USART_Parity = USART_Parity_No;
	USART5_InitStruct.USART_BaudRate = Iridium_Baudrate;
	USART5_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(Iridium_USART, &USART5_InitStruct);

	/* Configure NVIC for interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = Iridium_USART_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);
	USART_ITConfig(Iridium_USART, USART_IT_IDLE,ENABLE);
	USART_Cmd(Iridium_USART,ENABLE);


#ifdef IRIDIUM_Periph_Use_DMA
	DMA_InitTypeDef DMA_InitStructure;
	USART_DMACmd(Iridium_USART,USART_DMAReq_Rx,ENABLE); //enable USART DMA
	DMA_AHB1PeriphClockCmd(DMA_AHB1Periph,ENABLE);		//Turn on Clock to DMA
#ifdef IRIDIUM_MEM_Use_DMA
	DMA_AHB1PeriphClockCmd(DMA2_AHB1Periph,ENABLE);
#endif
	/* De-initialize DMA RX & TX Stream */ 								//important so no residual data flows in
	DMA_DeInit(Iridium_DMA_RX_Stream);
	while (DMA_GetCmdStatus(Iridium_DMA_RX_Stream ) != DISABLE) { ; }
	DMA_DeInit(Iridium_DMA_MEM_Stream);
	while (DMA_GetCmdStatus(Iridium_DMA_MEM_Stream) != DISABLE) { ; }

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(Iridium_USART->DR)); //set start address to USART Data REgister
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(Iridium_Rx_Buff);     // destination address: Iridium_Rx_Buff
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

	DMA_InitStructure.DMA_Channel = Iridium_DMA_RX_Channel;					//configured to UART5 Data Channel
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;					//Set PEripheral - Memory transfer
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_BufferSize = Iridium_RX_Buffsize;					//Important - size of buffer = number of bytes expected to transfer

	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_Init(Iridium_DMA_RX_Stream, &DMA_InitStructure);

	// enable the interrupt in the NVIC
	NVIC_InitStruct.NVIC_IRQChannel = DMA_USART_RX_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(Iridium_DMA_RX_Stream, DMA_IT_TC, ENABLE);
#ifdef IRIDIUM_MEM_Use_DMA

			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(Iridium_Rx_Buff);	//set start address to Iridium_Rx_Buff
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)message_buff;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

			DMA_InitStructure.DMA_Channel = Iridium_DMA_MEM_Channel;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;					//set transfer direction memory to memory
			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
			DMA_InitStructure.DMA_BufferSize = Iridium_message_Buffsize;

			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;

			DMA_Init(Iridium_DMA_MEM_Stream, &DMA_InitStructure);

			// enable the interrupt in the NVIC
			NVIC_InitStruct.NVIC_IRQChannel = DMA_USART_MEM_IRQn;
			NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 00;
			NVIC_InitStruct.NVIC_IRQChannelSubPriority = 01;
			NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStruct);
			DMA_ITConfig(Iridium_DMA_MEM_Stream, DMA_IT_TC, ENABLE);
#endif
	DMA_Cmd(Iridium_DMA_RX_Stream, ENABLE);									//enable Periph - Memory Stram and begin transfer
	while (DMA_GetCmdStatus(Iridium_DMA_RX_Stream) != ENABLE);				//enable not instantanious
#endif
}

/*configure Control pins */

/*
 * Note: The NetAv pin exti line needs additional configuration
 * for the purposes of this project the control pins arent neccessary
 * and therefore have not been initialized
 *
 * NetAV- GPIO Input -> External Interrupt
 * ONOFF - GPIO Digital Output
 *
 */
void init_Control_Pins(void)
{
	//using port C
	Iridium_GPIO_PeriphClockCommand(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);			//enable EXTI  through SYSCFG

	GPIO_InitTypeDef GPIO_InitStruct;
	/* Configure Wake up Pin
	 * Mode = Out
	 * PUPD = Pull Down
	 * Speed = 100MHz
	 */
	GPIO_InitStruct.GPIO_Pin = Iridium_Wakeup_Pin;
	GPIO_InitStruct.GPIO_Mode =GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Iridium_GPIO,&GPIO_InitStruct);
	GPIO_WriteBit(Iridium_GPIO,Iridium_Wakeup_Pin,RESET);
	/* Network available */
	/*
	 * Mode = Input
	 * Otype = Push Pull
	 * PUPD = DOWN
	 * SPEED = 100MHz
	 *
	 * 	3.3V Digital
	 *	Available = high
	 *	Not available= low
	 */
	GPIO_InitTypeDef GPIO_NETAV;
	GPIO_NETAV.GPIO_Pin = GPIO_Pin_13;
	GPIO_NETAV.GPIO_Mode =GPIO_Mode_IN;
	GPIO_NETAV.GPIO_OType = GPIO_OType_PP;
	GPIO_NETAV.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_NETAV.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(Iridium_GPIO,&GPIO_NETAV);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource13);
	//configure NetAv pin to EXTI line
	EXTI_InitTypeDef EXTI_InitStruct;

	EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode =EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //when high
	EXTI_Init(&EXTI_InitStruct);
	NVIC_InitTypeDef  NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		// needs to be reconfigured
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(EXTI15_10_IRQn);								//needs to be reconfigured

}

/*
 * Iridium Init: Acknowledgement Command AT\r expecting OK\r
 */

int8_t init_Iridium_Module(void)
{
	init_Iridium_USART();
	//init_Control_Pins();
	init_Rx_Buff();
	//wait for module to fully power out
	uint8_t flag = send_ATcmd("AT\r",1000);
	if(flag == -2)
	{
		//timeout
		return flag;
	}
	if(strcmp((char*)temp_buff,"OK") ==0)
	{
		//VALID RESPONSE
		return 0;
	}
	//INVALID RESPONSE
	return -1;

}

/*
 * Peripheral Destructor
 */
void deinit_Iridium_Module(void)
{
	 //Clear Data Register and disable
	 Iridium_USART->DR = 0;
	 USART_DeInit(Iridium_USART);
	 USART_ITConfig(Iridium_USART,USART_IT_RXNE,DISABLE);
	 NVIC_DisableIRQ(Iridium_USART_IRQn);

	 RCC_APB1PeriphClockCmd(Iridium_USART_RCCPeriph,DISABLE);

	 // deinit pins and place in weak pull up
	 GPIO_InitTypeDef  GPIO_InitStructure;
	 GPIO_InitStructure.GPIO_Pin = Iridium_USART_RX|Iridium_USART_TX;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_Init(Iridium_GPIO, &GPIO_InitStructure);

}

//=========================================================================================

/* Buffer Functions - instead of rewriting the algorithm many times */

//rx buffer clear
void init_Rx_Buff(void)
{
	Iridium_data_length = 0;
	for (int i = 0; i < Iridium_RX_Buffsize; ++i)
	{
		Iridium_Rx_Buff[i] = 0;
	}
}

//message buffer clear
void init_message_buff()
{
	for (int i = 0; i < Iridium_message_Buffsize; ++i)
	{
		message_buff[i] = 0;
		temp_buff[i] =0;
	}
}
//==========================================================================================

/* USART Transmission Commands */

//Simple USART string Data Transfer

void transmit_data(char* tx_buff,size_t len)
{
	//TODO: timeout and error checking
	/* get size of pointer*/
	for (int i = 0; i < len; ++i)
	{

		USART_SendData(Iridium_USART,*(tx_buff++));
		Delay_begin_Timeout(100);
		while(!USART_GetFlagStatus(Iridium_USART,USART_FLAG_TXE))
		{
			if(timeout)
			{
				break;
			}
		}
	}
}

// Simple Binary Data Transfer

void transmit_bin_Data(uint8_t* buff,size_t len)
{
	for (int i = 0; i < len; ++i)
	{
		USART_SendData(Iridium_USART,*(buff++));
		while(!USART_GetFlagStatus(Iridium_USART,USART_FLAG_TXE));
	}
}

//==========================================================================================

/* Short Burst Data Commands */

/*
 *  The modem uses Short Burst Data (SBD) protocols which allow for band-limited transmission of data
 *  The modem can transmit a message 340 bytes long and receive a message 270 bytes long
 */

/*
 *  Function to initiate an SBDquery session
 *  NOTE:
 	 *Data must be less than 340 bytes in length
 	 *Network availability does not result in success
 	 *  Successful query will return: +SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MT queued>
		 * MO status:
		 	 * 0 - 2 successful transmit
		 	 * 32 - no network available
		 	 * anything else is a failure
		 *MOMSN:
		 	 *Mobile Originated Message Sequence Number
		 	 *incremented each time successful
		 *MT Status:
		 	 * 0 - no message to receive
		 	 * 1 - message successfully received
		 	 * 2 - error checking mailbox
		 *MTMSN:
		 	 *The Mobile Terminated Message Sequence Number
		 *MT length
		 	 *Number of bytes received
		 *Count of mobile terminated SBD messages waiting at the GSS to be transferred to the ISU.
		 	 */

int8_t create_SBD_Session(void)
{
	/* Initiate SBD session */
	session_flag = 1;
	int8_t flag = send_ATcmd("AT+SBDIX\r",50000);
		//error = wait a few seconds then try again
	if(flag == -2)
	{
		//timeout
		return -1;
	}
	//check SBD session
	switch(SBDIX_status[0])
	{
	case 0:
	case 1:
	case 2:
		if(strcmp((char*)temp_buff,"OK") == 0)
		{
			break;
		}
		return -1;
	case 32:
		return -2;
	default:
		return -3;
	}
	return 0;
}

//status check for SBD session function

void get_status(char* cmd)
{
	uint8_t count = 0;

	for(int j = 0; j < 6; j++)
	{
		uint8_t numcount =0 ;
		uint len = strlen(cmd);
		while(cmd[count] != ',')
		{
			if((count) == len)
			{
				break;
			}
			if(cmd[count] >= '0' && cmd[count] <= '9')
			{
				numcount++;
			}

			count++;
		}
		//combine numbers
		int16_t temp = 0;

		for (int i = 0; i < numcount; ++i)
		{
			temp = temp*10+ (cmd[count -numcount+i ]-48);
			if (cmd[count-numcount -1] == '-')
			{
				temp = temp*(-1);
			}
		}
		SBDIX_status[j] = temp;
		count++;
	}
}

// clear session

void clear_Status(void)
{
	for (int i = 0; i < 6; ++i)
	{
		SBDIX_status[i] = 0;
	}
}

/* Retrieve the Iridium response only from Rx buffer */

char* get_AT_response(void)
{
	//clear flag
	char* tmp = (char*)Iridium_Rx_Buff;
	if(status_Received== 1)
	{
		//get transmission status
		tmp+=2;
		bin_message_received = *tmp;
		status_Received = 0;
		tmp+=2;
	}
	tmp = strtok((char*)Iridium_Rx_Buff,"\r\n");
	if(strlen(tmp) == 0)
	{
		return NULL;
	}
	if(session_flag == 1)
	{
		//SBD session: update status buffer
		clear_Status();
		char* msg = strtok(NULL,"\r\n");
		get_status(tmp);
		tmp = msg;
		session_flag = 0;
	}else
	{
		tmp = strtok(NULL,"\r\n");
	}
	init_message_buff();
	memcpy(temp_buff,tmp,strlen(tmp));
	init_Rx_Buff();
	return temp_buff;
}

/* Sends an AT command over USART returns the response */
int8_t send_ATcmd(char* cmd,uint32_t delay)
{
	transmit_data(cmd,strlen(cmd));
	//timeout after 60 seconds of no activity
	Delay_begin_Timeout(delay);
	while((IR_Rx_done != 1))
	{
		if(timeout)
		{
			timeout= 0;
			//Disable Interrupt
			return -2;
		}
	}
	IR_Rx_done = 0;
	return 0;
}

uint8_t send_ASCII_Message(char* msg)
{
	/* Set Flow control off*/
	int8_t flag = send_ATcmd("AT&K0\r",100);
	if(flag == -2)
	{
		if(strcmp(msg,(char*)"TIMEOUT")==0)
		{
			return -1;
		}
		else return -2;
	}

	/*create message string*/
	size_t len = strlen(msg)+strlen("AT+SBDWT=\r");
	char* cmd = (char*)message_buff;
	char atarr[len];
	memset(atarr,0,len);
	strcat(atarr,"AT+SBDWT=");
	strcat(atarr,msg);
	send_ATcmd(atarr,10000);
	if(strcasecmp(cmd,(char*)"OK") !=0)
	{
		if(strcmp(msg,(char*)"TIMEOUT")==0)
		{
			//timeout during transmission
			free(atarr);
			return -3;

		}else
		{
			//module did not recieve correctly
			free(atarr);
			return -4;
		}
	}
	//begin SBD extended session
	free(atarr);
	return 0;
}

uint8_t send_Binary_Message(uint8_t *msg, uint16_t size)
{

	/* Set Flow control off*/
	int8_t flag = send_ATcmd("AT&K0\r",1000);
	if(flag != 0)
	{
		//Timeout
		return -2;
	}else
	{
		if(strcmp((char*)temp_buff,"OK") != 0)
		{
			//Error message
			return -3;
		}
	}
	/* Tell Iridium data is binary*/
	char atarr [12];
	sprintf(atarr,"AT+SBDWB=%d\r",size);
	send_ATcmd(atarr,1000);
	//check for ready message
	char* imsg= (char*)temp_buff;
	imsg = strtok((char*)temp_buff,"\r\n");
	uint8_t msg_ready = 0;
	while(imsg != NULL)
	{
		if(strcmp((char*)msg,"READY"))
		{
			msg_ready = 1;
			break;
		}
		imsg = strtok(NULL,"\r\n");
	}
	if(!msg_ready)
	{
		return -3;
	}

	/* create message string*/
	uint16_t temp = calculate_checkSum(msg,size);
	uint8_t check_sum[3]  = {(uint8_t)((temp&0xFF00)>>8),(uint8_t)temp&0xFF,0x0d};
	//transmit message
	transmit_bin_Data(msg,size);
	transmit_bin_Data(check_sum,3);
	//wait for response
	status_Received = 1;
	while(IR_Rx_done != 1);
	IR_Rx_done= 0;
	switch ((bin_message_received-48)) {
		case 0:
			//check if status ok
			if(strcmp((char*)temp_buff,"OK") != 0)
			{
				return -3;
			}
			break;
		case 1:
			//timeout
			return -1;
		case 2:
			//invalid checksum
			return -4;
		default:
			break;
	}
	//begin SBDIX session
	return 0;
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

//==========================================================================
/** IRQ HANDLERS **/
void Iridium_USART_IRQHandler(void)
{
	//TODO: USART pin handler
	if (USART_GetFlagStatus(Iridium_USART,USART_FLAG_IDLE) != RESET)
	{
		/* Clear USART registers */
		volatile uint32_t tmp;
		tmp = USART_GetITStatus(Iridium_USART, USART_IT_IDLE);
		tmp = USART_ReceiveData(Iridium_USART);
		(void)tmp;
		/* Disable DMA RX Stream */
		DMA_Cmd(Iridium_DMA_RX_Stream, DISABLE);
		while (DMA_GetCmdStatus(Iridium_DMA_RX_Stream) != DISABLE) { ; }
	}
}

#ifdef IRIDIUM_Periph_Use_DMA
void DMA_USART_RX_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
		if (DMA_GetFlagStatus(Iridium_DMA_RX_Stream, DMA_FLAG_TCIF0) != RESET)
		{
			Iridium_data_length = Iridium_RX_Buffsize - DMA_GetCurrDataCounter(Iridium_DMA_RX_Stream);

			/* Clear DMA Stream Transfer Complete interrupt pending bit */
			DMA_ClearITPendingBit(Iridium_DMA_RX_Stream, DMA_IT_TCIF0);

			/* Enable DMA transfer */
			#ifdef IRIDIUM_MEM_Use_DMA
			DMA_Cmd(Iridium_DMA_MEM_Stream, ENABLE);
			while (DMA_GetCmdStatus(Iridium_DMA_MEM_Stream ) != ENABLE) { ; }

			#else
			DMA_Cmd(Iridium_DMA_RX_Stream, ENABLE);
			while (DMA_GetCmdStatus(Iridium_DMA_RX_Stream) != ENABLE) { ; }
			#endif
		}
}
#endif

#ifdef IRIDIUM_MEM_Use_DMA
void DMA_USART_MEM_IRQHandler(void)
{
	if (DMA_GetFlagStatus(Iridium_DMA_MEM_Stream, DMA_FLAG_TCIF1) != RESET)
		{

			//set flag

			if(session_flag)
			{
				if(strlen((char*)Iridium_Rx_Buff)> 9)
				{
					IR_Rx_done = 1;
					//Get Message
					get_AT_response();
				}
			}else
			{
				IR_Rx_done = 1;
				//Get Message
				get_AT_response();
			}
			/* Enable DMA transfer */
			DMA_Cmd(Iridium_DMA_RX_Stream, ENABLE);
			while (DMA_GetCmdStatus(Iridium_DMA_RX_Stream) != ENABLE) { ; }
			/* Clear DMA Stream Transfer Complete interrupt pending bit */
			DMA_ClearITPendingBit(Iridium_DMA_MEM_Stream, DMA_IT_TCIF1);
		}
}
#endif

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		if(GPIO_ReadInputDataBit(Iridium_GPIO,Iridium_NetAv_Pin) == SET)
		{
			network_available = 1;
		}
		if(GPIO_ReadInputDataBit(Iridium_GPIO,Iridium_NetAv_Pin) == RESET)
		{
			network_available = 0;
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}



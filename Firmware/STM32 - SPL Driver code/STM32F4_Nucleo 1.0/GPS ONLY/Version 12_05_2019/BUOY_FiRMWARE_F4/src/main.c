/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 7.1.1   2019-07-07

The MIT License (MIT)
Copyright (c) 2009-2017 Atollic AB

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
This is the main program to be run on the SHARC BUOY.
******************************************************************************
*/

/* Includes */
#include <stm32f4xx.h>			//Main SPL HEADER

#include "../My_Libs/RTC.h"		//For Sleep Mode and Wake UP Timing
#include "../My_Libs/eeprom.h"	//For Temporary offline storage of variables
#include "../My_Libs/Delay.h" 	//For Accurate Timing and Delay functions
#include "../My_Libs/GPS.h"		//Functions for connecting to The GPS VIA USART
#include "../My_Libs/Iridium.h" //custom Function for connecting to the IRIDIUM 9603 MODEM via usart

//===================================================================================================

/* Debug Includes*/
#include "tm_stm32f4_usart.h"	//TM Library for easy initialization of UART for DATA Output to Serial

//====================================================================================================

/* Private Structs*/

/*
 * PACKET OBJECT
 *
 * Variables:	Name.............Type.................................Description
 * 				ID...............uint8_t..............................value of 1-4 unique identifier for a packet in a single sample period
 * 				coord............{float32_t,float 32_t}...............Storage object for sampled GPS Coordinates
 * 				Etime............uint32_t.............................Stores the Epoch Time in Seconds from the GPS
 * 				Diagnostic_t.....{DOP_t,DOP_t,DOP_t,uint8_t,uint8_t}..Stores signal diagnostics from GSA message string *DOP_t is a uint8_t array storing the Integer and precision (to 2 decimal places)
 *
 */
typedef struct		//Object to store GPS data in an Organised Format
{
	uint8_t ID;
	Coord_t coord;
	uint32_t Etime;
	Diagnostic_t diag;
} Packet;

//=====================================================================================================

/* Private macro */
#define length(x) sizeof(x)/sizeof(*x)		//simple array length function instead of typing this out multipe time
#define IRIDIUM_TIMEOUT_ATTEMPTS 5			//Number of attempts program will make to recieve an acknowledgement from the Iridium modem
#define USART_DEBUG USART2					//MACRO to select the USARTPeriphal used for output to Serial

//=====================================================================================================

/* Private Typedefs */
RTC_TimeTypeDef rtc_time;					//Struct used to set the alarm. Stores HH, MM, SS and Time Format
RTC_AlarmTypeDef RTC_alarma;				//Alarm Object used to set the Wake up time
Packet packet;								//global packet object that can be accessed by all functions

//=====================================================================================================


/* private Data Buffers*/
uint8_t packet_buff[25];		//buffer to hold compressed GPS Data before storage in EEPROM

//=====================================================================================================

/* Private Virtual address:*/

/*
 * EEPROM EMULATION:
 *
 * Flash Sectors Configured in a Circular Buffer.
 * A single 32 bit register stores a 16 bit virtual address and a 16 bit uint8_t variable
 *
 * The following variables:
 *
 * VirtAddVarTab
 * VirtPacketAdd
 * VirtCounterAdd
 *
 * are virtual addresses ranging between 1 and 0xFFFE (0 and 0xFFFF are illegal)
 *
 * The following varuables
 * VarDataTab[4]
 * nextPacketID
 * routineCountNo
 *
 * are regular data variables
 *
 * Address Range:.........Data
 * 0x01 - 0x0A: System State Variables
 * 0x0B - 0x30: GPS Packet 1
 * 0x31 - 0x56: GPS Packet 2
 * 0x57 - 0x7C: GPS Packet 3
 * 0x7D - 0xA2: GPS Packet 4
 * 0xA3 - 0xFFFE: IMU DATA
 */

/*

 */
uint16_t VirtAddVarTab[4] = {0x0B,0x31,0x57,0x7D}; 	// Compressed GPS Data Base virtual addresses. Marks the starting location of each variable being stored for each packet
uint16_t nextPacketID = 0;						   	//Counter variable to determine what the systems next action is: 1- 3: sample GPS, 4: Sample and Transmit all data. Value resets to 1 after 4
uint16_t routineCountNo = 0;						//History variable to determine how many wake up cycles/ routines the system has previously executed for extra-synchronous functionality
uint16_t VirtPacketAdd = 0x0001;					//virtual Address for nextPacketID
uint16_t VirtCounterAdd	= 0x0002;					//Virtual Address for RoutineCount No

//=======================================================================================

/* private state Variables */
/*
 * State Variables determine what the current state of the system is and what the next action is
 * USED TO ACTIVATE AND DEACTIVATE ROUTINES DURING A CYCLE
 *
 * Variable Name..................ON STATE FUNCTION...............................................................ACTIVATED BY
 * init_State.....................Intialise system variables to 0,Set nextPacketID to 2 and Start GPS Sampling....Power up for the First Time
 * log_State......................Turn On GPS Sampling Routines, prepare EEPROM for Variable Storage//............init_State Functions & Wake Up from Sleep
 * Transmit State.................Turn On Iridium Transmission Routine Set NExtPAcketID to 1......................NextPAcketID = 4
 */
uint8_t init_State, log_State, Transmit_State,Deployment_State;

//========================================================================================

/* private peripheral flags */

/*
 *  These variables are set during peripheral initilzations.
 *  Each veriable represents the state of a peripheral. If an initialization was successful
 *  the flag is set to 1 and the system runs the routines
 *  If an error occurs, the system ignores all functions associated with that device
 *  These checks occur every cycle and are very specific to the System State
 *
 *  State:............Variables Initialized
 *  Init..............NONE
 *  Log...............GPS_On
 *  Transmission......Iridium_On
 */
uint8_t GPS_On, Iridium_On;

/* Private function prototypes */
//===================================================================================

/* RCC and System Functions */

/*
 * Description: This functions sets the system clock to 24MHz
 * 				Sets the Clock source to HSE (or to HSI if unavailable ) oscillators
 * 				and sets The PLL multipliers
 *
 * parameters:  void
 *
 * return: 		void
 */
void init_RCC_Clock(void);

/* Data Processing functions*/

/*
 * Description: Initializes the global pack_buff array to 0
 *
 * parameters:  void
 *
 * return type: void
 */
void clear_packet(void);

/*
 * Description: Compression Algorythm for GPS Data
 * 				Data compression is optimized for Iridium transmission bandwidth (340 bytes)
 * 				and RockBlock Data rates (50 bytes for 1 credit) Data is condensed into a packet
 * 				of 25 bytes. 4 x 25 bytes = 100 bytes (2 credits). Floats are converted to Unsigned
 * 				char arrays of 4 bytes. All data is broken into uint8_t bytes and stored in BIG Endian
 * 				Format
 *
 * parameters: Packet - struct containing uncompressed GPS data, ID - Number 1 - 4 indicating the packet's sample order
 *
 * return:     void
 */
void to_binary_format(Packet packet, uint8_t ID);

/*
 * Description: Stores an Array of Data in EEPROM at a specified address
 * 				The Base address is given and each variable in the array
 * 				is stored in each virtual address starting from the base
 *
 * parameters: data - array of unsigned 8 bit integers, length - length of the array to be stored
 * 			   virtualAddressBase - The Virtual Address of the First Variable
 */
void save_Data(uint8_t* data, int length, uint16_t virtualAddressBase);

/*
 * Description: Initializes the Nucleo User LED on GPIO port A pin 5 for Debugging purposes
 *
 * parameters:  void
 *
 * return: 		void
 */
void init_LED(void);



/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{

  /********************** Base Initializations *****************************/
  init_RCC_Clock();
  init_RTC();
  init_Delay();
  init_State = 0;
  log_State = 0;
  Transmit_State = 0;
  Deployment_State = 0;
/******************** DEBUGGING INITS **********************************/
 /*
  * NOTE: DELETE BEFORE Deployment
  */
  init_LED();
  TM_USART_Init(USART_DEBUG,TM_USART_PinsPack_1,115200);
  TM_USART_Send(USART_DEBUG,(uint8_t*)"System On. Starting Load...\r\n",31);


  /*********************** System State Check ******************************/


  /* Check if wake up came from power or external*/
  if(PWR_GetFlagStatus(PWR_FLAG_WU) == SET)
  {
	  /* Disable Power*/
	  RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR,DISABLE);
	  /* Enable access to back up registers*/
	  PWR_BackupAccessCmd(ENABLE);

	  /* Check if packet was saved successfully*/
	  uint8_t address_empty = EE_ReadVariable(VirtPacketAdd,&nextPacketID);
	  /* If empty, first data needs to be read*/
	  if(address_empty)
	  {
		  nextPacketID = 1;
		  log_State = 1;
	  }else
	  {
		  //read the last ID
		  if(nextPacketID == 4)
		  {
			  /* signal for transmission after last packet*/
			  Transmit_State = 1;
			  log_State = 1;

		  }

		  else if( nextPacketID > 4)
		  {
			  nextPacketID = 0;
		  }
		  else
		  {

			  log_State = 1;
		  }
	  }
	  /*	Read Number of routines	   */
	  uint8_t routine_empty = EE_ReadVariable(VirtCounterAdd,&routineCountNo);
	  if(!routine_empty)
	  {
		 routineCountNo++;
	  }
	  /* update next packet ID*/
	  packet.ID = nextPacketID++;
	  FLASH_Unlock();
	  EE_Init();
	  EE_WriteVariable(VirtPacketAdd,nextPacketID);
	  EE_WriteVariable(VirtCounterAdd,routineCountNo);
	  /* Clear Warm up flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

  }else
  {
	  /*
	   * First Time Start up Routine:
	   *
	   * System boots up and starts logging at the begining
	   * System state is set to Init and log
	   * System will start logging GPS data to packet 1
	   * next Packet is saved to EEPROM
	   */
	  /* Power on */
	  init_State = 1;
	  /* Init ID*/
	  FLASH_Unlock();
	  EE_Init();
	  nextPacketID = 2;
	  log_State = 1;

	  EE_WriteVariable(VirtPacketAdd,nextPacketID);
	  EE_WriteVariable(VirtCounterAdd,0);


	  /*
	   * LED BOOT SEQUENCE
	   */
	  for (int var = 0; var < 3; ++var)
	{
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,SET);
		Delay_begin_Timeout(500);
		while(!timeout);
		GPIO_WriteBit(GPIOA,GPIO_Pin_5,RESET);
		Delay_begin_Timeout(500);
		while(!timeout);
	}
  }

  /********************* Peripheral initializations ***********************/
if(log_State)
{
  /*
   * 1. GPS initialation
   */
	GPS_On = init_GPS();

	/*
	 * Additional preipherals can be added here as desired
	 */

  /*************** Routine 1: Collect GPS and store in a packet ***********/

if(log_State)
{
	if(GPS_On)
	{
		/* Acquire GPS signal*/
		GPIO_WriteBit(GPIOA,GPIO_PIN_5,SET);
		TM_USART_Send(USART_DEBUG,(uint8_t*)"GPS Online!\r\nSearching For Signal...\r\n",40);
		Delay_begin_Timeout(300000);
		while(!timeout)
		{
			if(packet_full == 7)
			{
				/* Disable delay and end comms*/
				Delay_Disable();
				deinit_USART_GPS();
				TM_USART_Send(USART_DEBUG,(uint8_t*)"Signal Locked on...\r\n",21);
				break;
			}
		}
		if(packet_full < 7)
		{
			TM_USART_Send(USART_DEBUG,(uint8_t*)"Failed To Find signal...\r\n",26);
		}
		/* If GPS is available, acquire signal and store data*/
		/* Transfer Data to packet*/
		packet.Etime = eTime;
		packet.coord = GPS_coord;
		packet.diag = diag;
	}
	else{
		TM_USART_Send(USART_DEBUG,(uint8_t*)"Failed to Find GPS...\r\n",24);
	}
	/* Save and store in */
	to_binary_format(packet, packet.ID);
	FLASH_Unlock();
	EE_Init();
	save_Data(packet_buff,length(packet_buff), VirtAddVarTab[packet.ID - 1]);
}



}

/********************** Routine 2: Iridium Transmit *****************************/

  if(Transmit_State)
  {
	  if(!Deployment_State)
	  {
		  //DEBUG Routine
		  /*
		   * USED TO Test the integrity of the data packets coming through
		   * Using the TM USART libraries, initialise USART and print the
		   * data to a terminal on the computer.
		   *
		   * Note: You need to download and install a terminal program
		   * I Recommend Realterm
		   *
		   * Ensure the following configurations?
		   *
		   * bauderate: 115200 bit/s
		   * 8 bits
		   * 1 stop bit
		   * No Parity
		   * No Flow Control
		   */
		  TM_USART_Init(USART_DEBUG,TM_USART_PinsPack_1,115200);
		  TM_USART_Send(USART_DEBUG,(uint8_t*)"Beginning Transmission Routine...\r\nData: ",43);
		  for (int i = 0; i < length(VirtAddVarTab); ++i)
		 {
			  uint8_t temp[length(packet_buff)];
		 	 /* load Data from FLASH into data buffer*/
			  load_Data(temp,VirtAddVarTab[i],length(packet_buff));
			  TM_USART_Send(USART_DEBUG,temp,length(packet_buff));

		 }
		  TM_USART_Send(USART_DEBUG,(uint8_t*)"\r\n",2);
	  }

	 /* Initialise Iridium Module */
	  Iridium_On = 0;
	  for (int i = 0; i < IRIDIUM_TIMEOUT_ATTEMPTS; ++i)
	  {

		  uint8_t ir_flag = init_Iridium_Module();
		 if (ir_flag ==0 )
		 {

		 	Iridium_On = 1;
		 	break;
		 }else
		 {
		 	deinit_Iridium_Module();
		 }
		 /* Delay for 3 seconds*/
		 Delay_begin_Timeout(3000);
		 while(!timeout);

	  }
	  /* If Communications was successful*/
	  if(Iridium_On)
	  {

		  /* Create a message buffer for all packets*/
		  clear_packet();
		  uint32_t size = length(packet_buff);
		  uint32_t numpackets = length(VirtAddVarTab);
		  uint8_t temp_log[size*numpackets];
		 for (int i = 0; i < length(VirtAddVarTab); ++i)
		 {
			 /* load Data from FLASH into data buffer*/
			load_Data(&temp_log[i*size],VirtAddVarTab[i],size);

		 }
		 /* Upload data to modem*/
		 send_Binary_Message(temp_log,length(temp_log));
		 /* Wait for network availability*/
		 uint8_t retry = 0;
		 Delay_Disable();
		 Delay_begin_Timeout(100000);
		 timeout = 0;
		 while(!timeout)
		 {

				 /* Create SBD Session*/

				 Delay_Disable();
				 int flag = create_SBD_Session();
				 if(flag == 0)
				 {
					 /* Flush the Iridium message queue*/

					 send_ATcmd("AT+SBDD0\r",1000);
					 break;
				 }
				 /* If not successful, set delay for another 3 seconds*/
				 if(flag == -2)
				 {

					 if (retry < 8)
					 {
						 retry++;
					 }else
					 {
d						 break;
					 }
				 }
		 }
		 /* Turn Off Module when done */
		 deinit_Iridium_Module();
		 nextPacketID = 0;
	  }

  	}

/******************************************************************************************************/
	/* SHUT DOWN ROUTINE */
  	  TM_USART_Send(USART_DEBUG,(uint8_t*)"System Going To Sleep\r\n",25);
	RTC_alarma.RTC_AlarmMask = RTC_AlarmMask_All&(~RTC_AlarmMask_Minutes);	//set the alarm mask to trigger wake up from minutes
	RTC_GetTime(RTC_Format_BIN,&rtc_time);									//get current System Time
	rtc_time.RTC_Minutes += 30;												//Increment by amount of time to sleep
	set_RTCAlarm_A(&rtc_time,&RTC_alarma);									//Set The Alarm
/******************************************************************************************************/
	/* Main Function, enter standby mode untill wake up*/
	while (1)
	{
		set_StdBy_Mode();
	}
}

void init_RCC_Clock(void)
{
	/* Set RCC value to default state*/
	RCC_DeInit();
	/* Enable external Crystal Oscilator*/
	RCC_HSEConfig(RCC_HSE_ON);
	ErrorStatus errorstatus = RCC_WaitForHSEStartUp();
	if(errorstatus == SUCCESS)
	{
		/* Configure PLL clock for 48 MHz*/
		RCC_PLLConfig(RCC_PLLSource_HSE,4,192,8,8);
		/* enable pll and wait until ready*/
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	}else
	{
			//Set clock source to default config
			RCC_HSEConfig(RCC_HSE_OFF);
			RCC_DeInit();
			RCC_HSICmd(ENABLE);
			RCC_PLLConfig(RCC_PLLSource_HSI,8,96,4,4);
			RCC_PLLCmd(ENABLE);
			while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);


	}
	/* Set Flash Latency*/
	FLASH_SetLatency(FLASH_Latency_1);
	/* Set AHB prescaler*/
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	/* Set APB1 Scaler */
	RCC_PCLK1Config(RCC_HCLK_Div2);
	/* Set APB2 Scaler*/
	RCC_PCLK2Config(RCC_HCLK_Div1);
	/* Set Clock source to PLL*/
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	SystemCoreClockUpdate();
}
/*
 * @brief: Takes a float value and converts to a char array by storing
 * 		   Data in a global Buffer. The char array is uint8_t
 */


void to_binary_format(Packet packet,uint8_t ID)
{
	packet_buff[0] = ID; // ID
	packet_buff[1] = (packet.Etime&0xFF000000)>>24;
	packet_buff[2] = (packet.Etime&0x00FF0000)>>16;
	packet_buff[3] = (packet.Etime&0x0000FF00)>>8;
	packet_buff[4] = (packet.Etime&0x000000FF);
	/* Add coordinates bytes 1 - 5 = lat, bytes 6 - 10 = long*/
	union
	{
		float a;
		unsigned char bytes[4];
	} coord_int;
	coord_int.a = packet.coord.lat;
	for (int i = 5; i < 9; ++i)
	{
		packet_buff[i] = coord_int.bytes[i-5];
	}
	coord_int.a = packet.coord.longi;

	for (int i = 9; i < 13; ++i)
	{
			packet_buff[i] =  coord_int.bytes[i-9];
	}
	/* break time down into MSB and LSB and store as as 2 unsigned bytes in big endian */

	//convert HDOP,VDOP, PDOP to 2 bytes big endian
	packet_buff[13] = packet.diag.HDOP.digit;
	packet_buff[14] = packet.diag.HDOP.precision;
	packet_buff[15] = packet.diag.VDOP.digit;
	packet_buff[16] = packet.diag.VDOP.precision;
	packet_buff[17] = packet.diag.PDOP.digit;
	packet_buff[18] = packet.diag.PDOP.precision;
	packet_buff[19] = (packet.diag.num_sats);
	packet_buff[19] = packet_buff[19]<<2;
	packet_buff[19] |= packet.diag.fix_type;
	packet_buff[24] = 0xd; //end of packet character

}

void clear_packet(void)
{
	for (int i = 0; i < length(packet_buff); ++i)
	{
		 packet_buff[i]=0;
	}
}

void save_Data(uint8_t* data, int length, uint16_t virtualAddressBase)
{
	FLASH_Unlock();
	for (uint16_t var = 0; var < length; ++var)
	{
		EE_WriteVariable(virtualAddressBase+var, (uint16_t)data[var]);
	}
	FLASH_Lock();
}

/*
 * Note: This function detects the presence of a reed switch
 * if a magnet is connected the system will not transmit data
 * to iridium
 */

void init_LED(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Pin = GPIO_PIN_5;
		GPIO_Init(GPIOA,&GPIO_InitStruct);
}



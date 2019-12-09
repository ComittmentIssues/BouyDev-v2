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
*/

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "../My_Libs/RTC.h"
#include "../My_Libs/eeprom.h"
#include "../My_Libs/Delay.h"
#include "../My_Libs/GPS.h"
#include "../My_Libs/Iridium.h"
#include "../My_Libs/Battery.h"
#include "../My_Libs/tm_stm32f4_ds18b20.h"
#include "../My_Libs/tm_stm32f4_onewire.h"
#include "../IMU_SRC/IMU.h"
//for debugging purposes
#include "tm_stm32f4_usart.h"
/* Private Structs*/
typedef struct
{
	uint8_t ID;
	Coord_t coord;
	uint32_t Etime;
	Diagnostic_t diag;
	float temp;
	uint8_t battery_voltage;
} Packet;

/* Private macro */
#define length(x) sizeof(x)/sizeof(*x)
#define IRIDIUM_TIMEOUT_ATTEMPTS 5
#define Temp_GPIO GPIOB
#define One_Wire_Pin GPIO_PIN_8
#define USART_DEBUG USART2
/* Private Typedefs */
RTC_TimeTypeDef rtc_time;
RTC_AlarmTypeDef RTC_alarma;
Packet packet;
TM_OneWire_t oneWire;

/* private Variables*/
float temp;

/* private Data Buffers*/
char fbuff[60];
uint8_t packet_buff[25];

/*
 * private virtual address variables for storage in emulated eeprom
 */
uint16_t VirtAddVarTab[4] = {0x1000,0x2000,0x3000,0x4000}; //base addresses for GPS data packets
uint16_t VarDataTab[4] = {0, 0, 0};
uint16_t nextPacketID = 0;
uint16_t routineCountNo = 0;
uint16_t VirtPacketAdd = 0x0001;
uint16_t VirtCounterAdd	= 0x0002;
uint8_t device[8];

/* private state flags */
uint8_t init_State, log_State, Transmit_State, IMU_State;

/* private peripheral flags */
uint8_t temp_On,VBat_On,GPS_On,Iridium_On,IMU_On;
/* Private function prototypes */

/* RCC and System Functions */
void init_RCC_Clock(void);
void init_All_Pins(void);
/* Data Processing functions*/
void ftoa(float f);
int32_t float_to_int(float f);
void clear_packet(void);
void to_binary_format(Packet packet, uint8_t ID);
void save_Data(uint8_t* data, int length, uint16_t virtualAddressBase);

/* Temp Sensor Functions */
uint8_t init_temp_sensor(void);
void get_Temp(float* temp);

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	/******************** DEBUGGING INITS **********************************/
	/*
	 * NOTE: DELETE BEFORE Deployment
	 */

	//indicator LEDs
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);


  /********************** Base Initializations *****************************/
  init_RCC_Clock();
  init_RTC();
  init_Delay();
  init_All_Pins();
  init_State = 0;
  log_State = 0;
  Transmit_State = 0;
  IMU_State = 0;

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
	  //Update Status LED to indicate stage in cycle
	  switch(packet.ID)
	  {
	  case 1:
		  STM_EVAL_LEDOn(LED3);
		  break;
	  case 2:
		  STM_EVAL_LEDOn(LED4);
		  //every half an hour sample IMU
		  IMU_State = 1;
		  break;
	  case 3:
		  STM_EVAL_LEDOn(LED5);
		  break;
	  case 4:
		  STM_EVAL_LEDOn(LED6);
		  break;
	  }
	  /* Clear Warm up flag*/
	  PWR_ClearFlag(PWR_FLAG_WU);

  }else
  {
	  /* Power on */
	  init_State = 1;
	  /* Init ID*/
	  FLASH_Unlock();
	  EE_Init();
	  nextPacketID = 2;
	  log_State = 1;
	  STM_EVAL_LEDOn(LED3);
	  EE_WriteVariable(VirtPacketAdd,nextPacketID);
	  EE_WriteVariable(VirtCounterAdd,0);
	  //UPDATE LED STATUS TO INIT ROUTINE
  }
  /********************* Peripheral initializations ***********************/
if(log_State)
{
  /*
   * 1. GPS
   */
  	  init_USART_GPS();
  /*
   * 2. Temp Sensor
   */
  	  temp_On = init_temp_sensor();

  /*
   * 3. Battery Monitor
   */
  	  VBat_On = init_Battery_ADC()&0b1;

  /*************** Routine 1: Collect GPS and store in a packet ***********/

if(log_State)
{
	/* Acquire GPS signal*/
	Delay_begin_Timeout(300);//000);
	GPS_On = 0;
	while(!timeout)
	{
		if(packet_full == 7)
		{
			/* Disable delay and end comms*/
			Delay_Disable();
			deinit_USART_GPS();
			GPS_On = 1;
			break;
		}
	}

	/* If Temp Sensor Available, get Temperature */
	if(temp_On)
	{
		get_Temp(&temp);
		packet.temp = temp;
	}
	/* If Battery Monitor Available Get Battery Voltage*/
	if(VBat_On)
	{
		/* Convert divided voltage to nominal voltage value*/
		packet.battery_voltage = 2*Sample_ADC();
		deinit_Battery_ADC();
	}

	/* If GPS is available, acquire signal and store data*/
	if(GPS_On)
	{
		/* Transfer Data to packet*/
		packet.Etime = eTime;
		packet.coord = GPS_coord;
		packet.diag = diag;
	}

	/* Save and store in */
	to_binary_format(packet, packet.ID);
	FLASH_Unlock();
	EE_Init();
	save_Data(packet_buff,length(packet_buff), VirtAddVarTab[packet.ID - 1]);
}



}

/********************** Routine 2:IMU Sample ************************************/
/*
 * Debug version:
 * Sample IMU at a known rate. Store data in EEPROM memory at known virtual address
 * When finished, read all data and transmit via UASART 2  PA2, PA3
 */
if(IMU_State)
{
	/*
	 * Initialise I2C command and
	 */
	IMU_On = init_IMU();
	//Communication link to check data
	TM_USART_Init(USART2,TM_USART_PinsPack_1,115200);
	//Communication link to check data
	if(IMU_On)
	{
		uint8_t string[] = "IMU FOUND!..Sampling\n\r";
		TM_USART_Send(USART2,string,length(string));
		init_Timer();
		while(!sample_finished);
		//read off data
		uint16_t VirtIMUAdd[6] = {0x04,0x0964,0x12C4,0x1C24,0x2584,0x2EE4};
		for (int i = 0; i < __numSamples(); ++i)
		{
			uint16_t temp_axes[6] = {0};

			EE_ReadVariable(VirtIMUAdd[0]++,&temp_axes[0]);
			EE_ReadVariable(VirtIMUAdd[1]++,&temp_axes[1]);
			EE_ReadVariable(VirtIMUAdd[2]++,&temp_axes[2]);
			EE_ReadVariable(VirtIMUAdd[3]++,&temp_axes[3]);
			EE_ReadVariable(VirtIMUAdd[4]++,&temp_axes[4]);
			EE_ReadVariable(VirtIMUAdd[5]++,&temp_axes[5]);

			char strbuf[100] = {0};
			sprintf(strbuf,"Ax = %d, Ay =%d, Az = %d || Gx = %d, Gy = %d, Gz = %d\n\r",(int16_t)temp_axes[0],(int16_t)temp_axes[1],(int16_t)temp_axes[2],(int16_t)temp_axes[3],(int16_t)temp_axes[4],(int16_t)temp_axes[5]);
			TM_USART_Send(USART2,(uint8_t*)strbuf,strlen((char*)strbuf));

		}
	}else
	{
		uint8_t string[] = "NO IMU FOUND\n\r";
		TM_USART_Send(USART2,string,length(string));
	}



}

/********************** Routine 3: Iridium Transmit *****************************/

  if(Transmit_State)
  {
	  STM_EVAL_LEDOn(LED3);
	  STM_EVAL_LEDOff(LED4);
	  STM_EVAL_LEDOff(LED5);
	  STM_EVAL_LEDOff(LED6);
	 /* Initialise Iridium Module */
	  Iridium_On = 0;
	  for (int i = 0; i < IRIDIUM_TIMEOUT_ATTEMPTS; ++i)
	  {

		  uint8_t ir_flag = init_Iridium_Module();
		 if (ir_flag ==0 )
		 {
		 	STM_EVAL_LEDOn(LED3);
		 	Iridium_On = 1;
		 	break;
		 }else
		 {
		 	deinit_Iridium_Module();
		 }
		 /* Delay for 3 seconds*/
		 Delay_begin_Timeout(3000);
		 while(!timeout);
		 STM_EVAL_LEDToggle(LED3);
	  }
	  /* If Communications was successful*/
	  if(Iridium_On)
	  {
		  STM_EVAL_LEDOn(LED4);
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
					 STM_EVAL_LEDOn(LED6);
					 send_ATcmd("AT+SBDD0\r",1000);
					 break;
				 }
				 /* If not successful, set delay for another 3 seconds*/
				 if(flag == -2)
				 {
					 STM_EVAL_LEDToggle(LED5);
					 if (retry < 8)
					 {
						 retry++;
					 }else
					 {
						 break;
					 }
				 }
		 }
		 /* Turn Off Module when done */
		 deinit_Iridium_Module();
		 nextPacketID = 0;
	  }

  	}
	/* SHUT DOWN ROUTINE */
	RTC_alarma.RTC_AlarmMask = RTC_AlarmMask_All;//&(~RTC_AlarmMask_Minutes);
	RTC_GetTime(RTC_Format_BIN,&rtc_time);
	rtc_time.RTC_Minutes += 30;
	set_RTCAlarm_A(&rtc_time,&RTC_alarma);

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

	}else
	{
			//Set clock source to default config
			RCC_DeInit();
	}
	SystemCoreClockUpdate();
}
/*
 * @brief: Takes a float value and converts to a char array by storing
 * 		   Data in a global Buffer. The char array is uint8_t
 */
void ftoa(float f)
{
	//get int;
	int num = (int)f;
	f -= (float)num;
	int dec = f*10000;
	sprintf(fbuff,"%d.%d\n\r",num,dec);

}

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

	/*Temperature - 2 bytes (dec), (precision ) - 2 decimal places */
	if(temp_On)
	{
		ftoa(packet.temp);
		char* t = strtok(fbuff,".");
		packet_buff[20] = atoi(t);
		uint16_t tempdec = 0;
		t = strtok(NULL,"\n");
		tempdec = atoi(t);
		packet_buff[21] = (tempdec&0xFF00)>>8;
		packet_buff[22] = (tempdec&0xFF);
	}
	/* Battery Conversion: 2 bytes */
	if(VBat_On)
	{
		packet_buff[22] = packet.battery_voltage;
	}
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
 * Function to set all pins to analog out
 */

void init_All_Pins(void)
{
	GPIO_InitTypeDef GPIO_Reset_InitStruct;

	GPIO_Reset_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	for (int i = 0; i < 16; ++i)
	{
		GPIO_Reset_InitStruct.GPIO_Pin = 0b1<<i;
		GPIO_Init(GPIOB,&GPIO_Reset_InitStruct);
	}
}
uint8_t init_temp_sensor(void)
{
	TM_DELAY_Init();
	/* Initialize OneWire on pin PD0 */
	TM_OneWire_Init(&oneWire, Temp_GPIO, One_Wire_Pin);
	if(TM_OneWire_First(&oneWire))
	{
		TM_OneWire_GetFullROM(&oneWire,device);
		return 1;
	}
	return 0;
}

void get_Temp(float* temp)
{
	TM_DS18B20_Start(&oneWire,device);
	while(!TM_DS18B20_AllDone(&oneWire));
	TM_DS18B20_Read(&oneWire,device,temp);
}

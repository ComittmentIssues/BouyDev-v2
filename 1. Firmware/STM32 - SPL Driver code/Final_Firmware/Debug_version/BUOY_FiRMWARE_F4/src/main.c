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
#include "../My_Libs/RTC.h"
#include "../My_Libs/eeprom.h"
#include "../My_Libs/Delay.h"
#include "../My_Libs/GPS.h"
#include "../My_Libs/Iridium.h"
#include "../IMU_SRC/IMU.h"
#include "../env_src/BMP280.h"
#include "../env_src/Wind.h"
//for debugging purposes
#include "tm_stm32f4_usart.h"
/* Private Structs*/
typedef struct
{
	uint8_t ID;
	Coord_t coord;
	uint32_t Etime;
	Diagnostic_t diag;
	BMP280_data_t bmp;
	uint16_t wind_dir;
	uint16_t wind_speed;
	float temp;
	float Pressure;
	uint16_t battery_voltage;
	uint16_t battery_current;

} Packet;

/* Private macro */
#define length(x) sizeof(x)/sizeof(*x)
#define IRIDIUM_TIMEOUT_ATTEMPTS 5

#define USART_DEBUG USART2
/* Private Typedefs */
RTC_TimeTypeDef rtc_time;
RTC_AlarmTypeDef RTC_alarma;
Packet packet;
BMP280_data_t BMPData;

/* private Variables*/
float temp;

/* private Data Buffers*/
char fbuff[60];
uint8_t packet_buff[37];

/*
 * private virtual address variables for storage in emulated eeprom
 */
uint16_t VirtAddVarTab[4] = {0x0B,0x31,0x57,0x7D}; //base addresses for GPS data packets
uint16_t VarDataTab[4] = {0, 0, 0};
uint16_t nextPacketID = 0;
uint16_t routineCountNo = 0;
uint16_t VirtPacketAdd = 0x0001;
uint16_t VirtCounterAdd	= 0x0002;
uint8_t device[8];

/* private state flags */
uint8_t init_State, log_State, Transmit_State, IMU_State,Deployment_State;

/* private peripheral flags */
uint8_t temp_On,VBat_On,GPS_On,Iridium_On,IMU_On,WeatherVane_On;
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
void init_LED(void);
void init_Reed_Switch(void);


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
	init_LED();



  /********************** Base Initializations *****************************/
  init_RCC_Clock();
  init_RTC();
  init_Delay();
  init_Reed_Switch();
  init_State = 0;
  log_State = 0;
  Transmit_State = 0;
  IMU_State = 0;
  Deployment_State = 0;
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

		  break;
	  case 2:
		  //every half an hour sample IMU
		  IMU_State = 1;
		  break;
	  case 3:

		  break;
	  case 4:

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

	  EE_WriteVariable(VirtPacketAdd,nextPacketID);
	  EE_WriteVariable(VirtCounterAdd,0);
	  //UPDATE LED STATUS TO INIT ROUTINE
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
  /*
   * Check to See if system is in deployment mode
   */
 // Deployment_State = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5);
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
//  	  if(BMP280_Begin() == BMP_OK)
//  	  {
//  		  //load trimming parameters for temp and pressure compensation
//  		  BMP280_GetCoeff(&bmp);
//  		  //configure ctrl measurement register
//  		  BMP280_Configure_CTRLMEAS(BMP280_CTRLMEAS_OSRST_OS_1,BMP280_CTRLMEAS_OSRSP_OS_1,BMP280_CTRLMEAS_MODE_SLEEP);
//  		  //configure filter, i2c mode, odr
//  		  BMP280_Configure_Config(BMP280_CONFIG_tsb_1000,BMP280_CONFIG_FILTER_COEFF_OFF,BMP280_CONFIG_SPI3_DIS);
//  		  temp_On = 1;
//  	  }

  /*
   * 3. Battery Monitor
   */
  	  //VBat_On = init_Battery_ADC()&0b1;

  /*
   * 4. Weather Vane
   */
  	  init_ADC(ADC_AN,Weather_Vane_Pin);
  	  init_InputCapture(Wind_Speed_Pin);
  	  WeatherVane_On = 1;

  /*************** Routine 1: Collect GPS and store in a packet ***********/

if(log_State)
{
	/* Acquire GPS signal*/
	Delay_begin_Timeout(300000);
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

	/* Temp and Pressure Algorithm */
	if(temp_On)
	{
		//force a measurement
		BMP280_Force_Measure(&BMPData.temp,&BMPData.press);
		packet.bmp.temp = BMPData.temp;
		packet.bmp.press = BMPData.press;

	}
	/* If Weather Vane Connected, sample Direction and Speed*/
	if(WeatherVane_On)
	{
		packet.wind_dir = get_Wind_Direction();
		packet.wind_speed = get_Wind_Speed();
	}
	/* If Battery Monitor Available Get Battery Voltage*/
	if(VBat_On)
	{
		/* Convert divided voltage to nominal voltage value*/

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
		uint16_t VirtIMUAdd[6] = {0xD0,0x1391,0x2652,0x3913,0x4BD4,0x5E95};



		/*
		 * check to see if the buoy has been set for deployment
		 */
		if(Deployment_State)
		{
			//init iridium comms
			 Iridium_On = 1;
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
					  //load 340 bytes into a buffer
					uint8_t packet_no = 0;
					uint8_t imu_buff[340] = {0};
					int count = 0;
					/*
					 * order byte 0 : IMU identifier i.e 2
					 * byte 1: the number of packet this is
					 * byte 2 - 337 |Ax|Ay|Az|Gx|Gy|Gz
					 * byte 338 - \n
					 * byte 339 - \r
					 */
					//start
					//loop until the total number of samples have been read
					while(count < __numSamples())
					{
						packet_no++;
						imu_buff[0] = 2;
						imu_buff[1] = packet_no;
						int i = 2;

						while( i < 336)
						{
							for (int j = 0; j < 6; ++j)
							{
							/*
							 * store data in LSB first
							 */
								uint16_t ax;
								EE_ReadVariable(VirtIMUAdd[j]+count,&ax);
								imu_buff[i++] = ax&0xFF;
								imu_buff[i++] = (ax&0xFF00)>>8;
							}
							count++;
						}
						imu_buff[338] = '\n';
						imu_buff[339] = '\r';
						//upload to buffer
						send_Binary_Message(imu_buff,length(imu_buff));

					}
				  }



		}
		else
		{
			for (int i = 0; i < __numSamples(); ++i)
			{
						uint16_t temp_axes[6][__numSamples()] = {0};
						EE_ReadVariable(VirtIMUAdd[0]++,&temp_axes[0][i]);
						EE_ReadVariable(VirtIMUAdd[1]++,&temp_axes[1][i]);
						EE_ReadVariable(VirtIMUAdd[2]++,&temp_axes[2][i]);
						EE_ReadVariable(VirtIMUAdd[3]++,&temp_axes[3][i]);
						EE_ReadVariable(VirtIMUAdd[4]++,&temp_axes[4][i]);
						EE_ReadVariable(VirtIMUAdd[5]++,&temp_axes[5][i]);

			//			char strbuf[100] = {0};
			//			sprintf(strbuf,"Ax = %d, Ay =%d, Az = %d || Gx = %d, Gy = %d, Gz = %d\n\r",(int16_t)temp_axes[0],(int16_t)temp_axes[1],(int16_t)temp_axes[2],(int16_t)temp_axes[3],(int16_t)temp_axes[4],(int16_t)temp_axes[5]);
			//			TM_USART_Send(USART2,(uint8_t*)strbuf,strlen((char*)strbuf));

			}
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
	RTC_alarma.RTC_AlarmMask = RTC_AlarmMask_All&(~RTC_AlarmMask_Minutes);
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
		//temp packets
		coord_int.a = packet.bmp.temp;
		packet_buff[20] = coord_int.bytes[3];
		packet_buff[21] = coord_int.bytes[2];
		packet_buff[22] = coord_int.bytes[1];
		packet_buff[23] = coord_int.bytes[0];
		//press
		coord_int.a = packet.bmp.press;
		packet_buff[24] = coord_int.bytes[3];
		packet_buff[25] = coord_int.bytes[2];
		packet_buff[26] = coord_int.bytes[1];
		packet_buff[27] = coord_int.bytes[0];

	}
	/* Battery Conversion: 2 bytes */
	if(WeatherVane_On)
	{
		packet_buff[28] = packet.wind_dir>>8;
		packet_buff[29] = packet.wind_dir&0xFF;
		packet_buff[30] = packet.wind_speed >>8;
		packet_buff[31] = packet.wind_speed&0xFF;
	}
	if(VBat_On)
	{
		packet_buff[32] = (packet.battery_voltage &0xFF00)>>8;
		packet_buff[33] = packet.battery_voltage &0xFF;
		packet_buff[34] = packet.battery_current >>8;
		packet_buff[35] = packet.battery_current&0xFF;
	}
	packet_buff[36] = 0xd; //end of packet character

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
void init_Reed_Switch(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_5;
	GPIO_Init(GPIOH,&GPIO_InitStruct);
}

void init_LED(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Pin = GPIO_PIN_5;
		GPIO_Init(GPIOA,&GPIO_InitStruct);
}
/*
 * Function to set all pins to analog out
 */


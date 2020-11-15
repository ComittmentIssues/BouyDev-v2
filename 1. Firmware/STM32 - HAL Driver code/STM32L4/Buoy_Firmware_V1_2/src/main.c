/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.0.0   2020-06-15

The MIT License (MIT)
Copyright (c) 2018 STMicroelectronics

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

#include "stm32l4xx_hal.h"	//HAL library

#include "Sharc_Frame.h"	//Power config, state machine, clock config for microcontroller

/* Sensor Includes */

#include "HAL_GPS.h"		//Ublox Neo GPS Library

#include "DATAFLASH.h"		//Flash Chips Library

#include "HAL_Iridium.h"	//Iridium satellite modem Library

#include "HAL_BMP280.h"		//Environmental Sensor Library

#include "HAL_INA219.h"		//Current Monitor Library

/* Private defines */

/* Private macro */

/* Private variables */

RTC_HandleTypeDef hrtc; //RTC Handler Instance

uint8_t GPS_On = 0; //flag to show if sensor is online

uint8_t IR_On = 0; //flag to show if Iridium is online
/* Private function prototypes */
/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void);

/**
  * @brief function to handle failures in Peripheral initialization functions
  * @param None
  * @retval None
  */
void Error_Handler(void);

/**
  * @brief Place code for IMU Event Detection here
  * @param None
  * @retval None
  */
static void Routine_ASYNC_IMU_EVENT(void);


/*
 * @brief Code for Reset State Routine
 * @param None
 * @retval None
 */
static void Routine_STATE_RESET(void);

/*
 * @brief Code for Sleep State Routine
 * @param None
 * @retval None
 */
static void Routine_STATE_SLEEP(void);

/*
 * @brief Code for Sample State Routine
 * @param None
 * @retval None
 */
static void Routine_STATE_SAMPLE(void);

/*
 * @brief Code for Transmit State Routine
 * @param None
 * @retval None
 */
static void Routine_STATE_TRANSMIT(void);
/*
 * @brief Code for Init State Routine
 * @param None
 * @retval None
 */
static void Routine_Init_STATE(void);
/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **  IMPORTANT NOTE!
 **  The symbol VECT_TAB_SRAM needs to be defined when building the project
 **  if code has been located to RAM and interrupts are used.
 **  Otherwise the interrupt table located in flash will be used.
 **  See also the <system_*.c> file and how the SystemInit() function updates
 **  SCB->VTOR register.
 **  E.g.  SCB->VTOR = 0x20000000;
 **
 **
 **===========================================================================
 **/

int main(void)
{
//======================== 1. SYSTEM INIT & CLOCK CONFIG ========================//

	HAL_Init();				//Init Flash prefetch, systick timer, NVIC and LL functions
	SystemClock_Config();	//configure clock
	GPIO_Set_Pin_LP();		//Configure all unused GPIO pins to low Power mode
	Init_Debug();			// initialize debug peripherals

//=================================== 1. END ====================================//

//======================= 2. POWER AND RESET STATE CHECK ========================//

/*
 * When system powers on, check for any asynchronous resets that
 * may have occured. Use this area to add in any reset handling
 */
	if(__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) == SET)
	{
	  BOR_Handler();
	}
	uint8_t flag = __HAL_RCC_GET_PORRST_FLAG();
	if(flag  == SET)
	{
	  printf("Software Reset Detected. Initializing main program...\r\n");
	  POR_Handler();
	}
//=================================== 2. END ====================================//


  /* Infinite loop START */
  while (1)
  {

//========================= 3. ASYNCHRONOUS STATE CHECK ===========================//

	  /*
	   * If an interrupt occurred while the device was sleeping, check the
	   * flags to determine if this occurred
	   */

	  //check for interrupts on wake up pins during deep sleep
	  __HAL_RCC_PWR_CLK_ENABLE();
	  if(__HAL_PWR_GET_FLAG(IMU_EVENT_WAKE_FLAG)|| __HAL_PWR_GET_FLAG(IRIDIUM_RING_WAKE_FLAG))
	  {
		  Current_State = __GET_PREV_STATE();			//get previous state from back up reg
		  if(__HAL_PWR_GET_FLAG(IMU_EVENT_WAKE_FLAG))
		  {
			  __HAL_PWR_CLEAR_FLAG(IMU_EVENT_WAKE_FLAG);	//clear  flag in PWR SR
			  Routine_ASYNC_IMU_EVENT();				//Perform IMU Event Detection Routine
		  }
		  if(__HAL_PWR_GET_FLAG(IRIDIUM_RING_WAKE_FLAG))	//clear  flag in PWR SR
		  {
			__HAL_PWR_CLEAR_FLAG(IRIDIUM_RING_WAKE_FLAG);
			Routine_ASYNC_IRIDIUM_RX();

		  }
		  if(Current_State == STATE_SLEEP)				//return to sleep if Interrupt event handled before wake up
		  {

			  printf("System Going Back To Sleep\r\n"); 	//check how long device was asleep for
			  set_WUP_Pin(IMU_EVENT_WAKE_PIN, MODE_WUP);	//reenable wake up pins
			  Go_To_Sleep(STDBY,T_SLEEP);						//return to sleep
		  }
		  	 else
		  {

		  	  printf("Going Back to Main Loop:\r\n");		//if come from wake mode
			  __SET_CURRENT_STATE(STATE_ASYNCINT);
		  }
	  }
//=================================== 3. END ====================================//

//=========================== 4. SYSTEM STATE CHECK =============================//
	  /*
	   * Main Loop state detection: use this area to implement state
	   * check.States are positive integers written to Back up Register
	   * 0. State functions include Reading/ Writing to Back Up register
	   * determing previous state before sleep and writing state to back up register
	   * states are defined in the enum Buoy_State_typedef. The state check block performs the following routine
	   *
	   */

	  //enable access to back up registers
	  switch(__GET_PREV_STATE())
	  {
	  	 case STATE_ASYNCINT:
	  	 case STATE_RESET:
	  	 //system encountered a power on reset, put peripherals here
	  	 Current_State = STATE_SAMPLE;
	  	 break;

	  	 case STATE_SAMPLE:
	  	 //check how many samples have been recorded by the Buoy
	  	 sample_count = __GET_SAMPLE_COUNT();
	  	 if(sample_count > 3)
	  	 {
	  		 //set next Buoy State to Transmit
	  		 Current_State = STATE_TRANSMIT;
	  	 }
	  	 // Set Buoy Next state to Sleep
	  	 else
	  	 {
	  		 Current_State = STATE_SLEEP;
	  		 __HAL_RCC_PWR_CLK_ENABLE();
	  		 __SET_CURRENT_STATE(Current_State);
	  		 __HAL_RCC_PWR_CLK_DISABLE();
	  	 }
	  	 break;

	  	 case STATE_SLEEP:
	  	hrtc.Instance = RTC;					 			//attach RTC instance to handler
	  	HAL_PWREx_DisableInternalWakeUpLine();				//clear wake up pending interrupt from internal wake up
	  	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);					//clear pending interrupt from ext wake up pins
	  	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		set_WUP_Pin(IMU_EVENT_WAKE_PIN,MODE_EXTI); 	  		//reconfigure wake up pins
	  	Current_State = STATE_SAMPLE;
	  	 break;

	  	 case STATE_TRANSMIT:
	  	 Current_State = STATE_SLEEP;
  		 __SET_CURRENT_STATE(Current_State);
	  	 break;

	  	 //default case: reset state
	  	 default:
	  	 Current_State = STATE_INIT;
	  }
	  __HAL_RCC_PWR_CLK_DISABLE();

//=================================== 4. END ====================================//

//============================== 5. STATE FUNCTIONS =============================//

	  /*
	   * Place Routine code Here
	   */
	  if(Current_State == STATE_INIT)
	  {
		  Routine_Init_STATE();
		  Current_State = STATE_RESET;
	  }

	  if(Current_State == STATE_RESET)
	  {
	  	Routine_STATE_RESET();
	  }
	  // SLEEP STATE
	  else if(Current_State == STATE_SLEEP)
	  {
		  Routine_STATE_SLEEP();
	  }
	  //RESET STATE

	  //SAMPLE STATE
	  else if(Current_State == STATE_SAMPLE)
	  {
		  Routine_STATE_SAMPLE();
	  }
	  //TRANSMIT STATE
	  else if(Current_State == STATE_TRANSMIT)
	  {
		  Routine_STATE_TRANSMIT();
	  }
	  // ADDITONAL STATE FUNCTIONS HERE:

//=================================== 5. END ====================================//

//========================= 6. END OF ROUTINE FUNCTION ==========================//
	  /*
	   * After each routine has run, save state to the back up registers
	   */
	  __HAL_RCC_PWR_CLK_ENABLE();
	  if(Current_State == STATE_SAMPLE)			//increment sample counter after each sampe
	  {
		  sample_count = __GET_SAMPLE_COUNT();
		  __SET_SAMPLE_COUNT(++sample_count);
	  }


	  __SET_CURRENT_STATE(Current_State);	    //write value to back up register
	  __HAL_RCC_PWR_CLK_DISABLE();

//=================================== 6. END ====================================//
  }
	/* Infinite loop END*/
}

/**
**=======================================================================================
**
**  								END OF MAIN PROGRAM
**
**=======================================================================================
*/

//==================== 7. Configuration & Initialization Functions ====================//

/* Private functions */

static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 57;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_MAY;
  sDate.Date = 8;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  //clear unwanted interrupts
  if(__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc,RTC_FLAG_WUTF))
  {
	  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc,RTC_FLAG_WUTF);
  }



}

//====================================== 7. END ======================================//

//==================================== 8. Handlers ===================================//

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

//===================================== 8. END ======================================//

//=============================== 9. Routines =======================================//

static void Routine_ASYNC_IMU_EVENT(void)
{
	 printf("IMU Event Detected: Sampling...");
	 for (int i = 0; i < 50; ++i)
	 {
	 	HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	 	HAL_Delay(100);
	 }
	 printf("Done\r\n");
}



static void Routine_STATE_RESET(void)
{
	 //initialise RTC
	 MX_RTC_Init();
	 //Enable Interrupt pins as EXTI Outputs
	 set_WUP_Pin(IMU_EVENT_WAKE_PIN,MODE_EXTI);
	 printf("Device Online!\r\n");
	 printf("Current State: RESET \t Next State: SAMPLE\r\n");
	 HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);


}

static void Routine_STATE_SLEEP(void)
{
	  printf("Current State: SLEEP \t Next State: SAMPLE\r\n");
	  printf("Good Night! \r\n");
	  set_WUP_Pin(IMU_EVENT_WAKE_PIN,MODE_WUP);
	  Go_To_Sleep(STDBY,T_SLEEP);
}

static void Routine_STATE_SAMPLE(void)
{
	  //GPS  Init Routine
	  GPS_Data_t Gdata;
	  sample_count = __GET_SAMPLE_COUNT();
	  if(sample_count < 3)
	  {
		  printf("Current State: SAMPLE \t Next State: SLEEP\r\n");
	  } else
	  {
		  printf("Current State: SAMPLE \t Next State: TRANS\r\n");
	  }

	  /* Attempt to initialize sensor within a number of retries */
	  uint8_t retries = 0;
	  do
	  {
		  //initialise gps peripherals and begin comms on sensor instance
		  if(init_GPS(&hgps)== GPS_Init_OK)
		  {
			 printf("GPS Online! Acquiring Signal...\r\n");
			 GPS_On = 1;
			 break;
		  }
		deinit_GPS(&hgps);
		retries++;
	  }
	  while (retries < GPS_INIT_RETRIES);

	  // Sample Routine

	  if(GPS_On)
	  {
		  	  int x = HAL_GetTick();
			  while(packet_full != 7)
			  {
				  GPS_Log_Begin();
				  int y = HAL_GetTick()- x;
				  if(y >  3000)
				  {
					  printf("Failed to Acquire Signal\r\n");
					  break;
				  }

			  }
			  GPS_Log_Stop();
			  printf("Logging Data...\r\n");
			  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
			  Gdata.coordinates = GPS_coord;
			  Gdata.Etime = eTime;
			  Gdata.diag = diag;
			  printf("local time: %lu, position: %f Lat, %f long\r\n", Gdata.Etime, Gdata.coordinates.lat, Gdata.coordinates.longi);
			  printf("HDOP = %d.%d, \t PDOP = %d.%d, VDOP = %d.%d\r\n",  Gdata.diag.HDOP.digit, Gdata.diag.HDOP.precision,  Gdata.diag.PDOP.digit, Gdata.diag.PDOP.precision,  Gdata.diag.VDOP.digit, Gdata.diag.VDOP.precision);
			  printf("Number of Satellites %d, Fix Type = %d\r\n", Gdata.diag.num_sats, Gdata.diag.fix_type);

		  deinit_GPS(&hgps);
	  }else
	  {
		  printf("Error GPS Not Found!\r\n");
	  }
	  // Environmental Sensor Init Routine
	  BMP_Init_Typedef BMP_InitStruct = {0};
	  //configure device for environmental sensing
	  BMP_InitStruct.BMP_Pressure_OverSample = BMP280_CTRLMEAS_OSRSP_OS_1;
	  BMP_InitStruct.BMP_Temperature_OverSample = BMP280_CTRLMEAS_OSRST_OS_1;
	  BMP_InitStruct.BMP_IIR_FILTER_COEFFICIENTS = BMP280_CONFIG_FILTER_COEFF_OFF;
	  BMP_InitStruct.BMP_Power_Mode = BMP280_CTRLMEAS_MODE_FORCED;
	  if(BMP280_Init(&BMP_InitStruct) == BMP_OK)
	  {
		  printf("Environmental Sensor Online!\r\n");
		  //create variables
		  uint32_t temp,press;
		  int32_t t_fine;
		  for (int i = 0; i < 5; ++i)
		  {
			BMP280_Force_Measure(&temp,&press);		//trigger conversion
			Gdata.env_Temp = BMP280_Compensate_Temp(temp,&t_fine,bmp.Factory_Trim);			//compensate temperature
			Gdata.atm_Press = BMP280_Compensate_Pressure(press,t_fine,bmp.Factory_Trim)/256;	//compensate pressure

		  }
		  printf("Temp = %ld�C \t Pressure = %lu Pa \r\n",Gdata.env_Temp,Gdata.atm_Press);
	  }

	  //INA219 Sample Routine
	  if(INA219_Init_Sensor() == INA_OK)
	  {
		  int16_t p_temp = 0;
		  printf("INA219 Ready! \r\n");
		  //Trigger a conversion
		  INA219_Trigger_Conversion(2);
		  //read registers
		  INA219_Get_Shunt_Voltage(&Gdata.shunt_v);
		  INA219_Get_Bus_Voltage(&Gdata.bus_v);
		  INA219_Get_Current(&Gdata.current);
		  INA219_Get_Power(&p_temp);
		  Gdata.power = (ina.INA219_P_LSB*(float)p_temp)*1000;
		  printf("V_Shunt %d mV V_Bus %d mV Current %d mA Power %d mW",Gdata.shunt_v,Gdata.bus_v, Gdata.current,Gdata.power);
	  }
	  //Init Flash Chips
	  uint8_t statusbyte;
	  uint8_t* buffer = to_binary_format(Gdata,__GET_SAMPLE_COUNT());
	  if(Init_Flash_Chips(&statusbyte)== HAL_OK)
	  {

		  uint8_t chipnumber = Get_Active_Chip();
		  //get current pointer to free memory in chip
		  uint8_t address[3] = {0};
		  Get_Current_Address_Pointer(chipnumber,address);
		  FLASH_SetAddress(address[2],address[1],address[0]);
		  //check if there is still space
		  if(!FLASH_Is_Available(chipnumber,get_driftBuffer_Size()))
		  {
			  //Routine to select next available chip for memory storage
			  printf("Warning! chip %d is at maximum capacity\r\n",chipnumber);
			  //set status to full
			  FLASH_SetAddress(0x00,0x00,0x00);
			  uint8_t chipstatus = Full;
			  FLASH_WRITE_ReadModifyWrite(chipnumber,BUFFER1,&chipstatus,1);
			  //get next active chip and set as active
			  chipnumber = Get_Next_Active_Chip();
			  chipstatus = Active;
			  FLASH_WRITE_ReadModifyWrite(chipnumber,BUFFER1,&chipstatus,1);
			  //Set Next Active Chip
			  Set_Active_Chip(chipnumber);
			  //Select inactive chip to be next for storage
			  //cycle through remaining chips
			  chipnumber = 0;
			  for (int i = Get_Next_Active_Chip(); i <FLASH_CHIPS; ++i)
			  {
				  if((statusbyte& 0b1<<i)? SET: RESET)
				  {
					  chipnumber = i+1;
					  break;
				  }

			  }
			  //set the status byte of the chip to next active
			  Set_Next_Active_Chip(chipnumber);
			  chipstatus = Next_Active;
			  FLASH_WRITE_ReadModifyWrite(chipnumber,BUFFER1,&chipstatus,1);
			  FLASH_IncAddress(1);
		  }
		  if(FLASH_WRITE_ReadModifyWrite(Get_Active_Chip(),BUFFER1,buffer,get_driftBuffer_Size()) != 1)
		 {
		 	printf("Successfully saved data to chip %d\r\n",Get_Active_Chip());
		 	//increment pointer to next available memory block
		 	FLASH_IncAddress(get_driftBuffer_Size());
		 	//save address to chip
		 	uint32_t temp = FLASH_GetAddress();
		 	address[2] = (temp & 0xFF0000)>>16;
		 	address[1] = (temp & 0xFF00)>>8;
		 	address[0] = (temp & 0xFF);
		 	Set_Current_Address_Pointer(chipnumber,address);
		 }
		 else
		 {
		   printf("Error Saving to Flash Chip\r\n");
		 }


	  }else
	  {
		  printf("Error! Memory Full");
	  }

}

static void Routine_STATE_TRANSMIT(void)
{
	  printf("Current State: TRANS \t Next State: SLEEP\r\n");

	  /* 1. Initialize Iridium Modem */
	  uint8_t chip = Get_Active_Chip();		 	 											//Get active chip
	  uint8_t Address[3] = {0};					 											//get current address pointer
	  Get_Current_Address_Pointer(chip,Address);
	  if(IR_Init_Module() == IR_OK)
	  {
		  uint32_t size = ((Address[2] - 0x00)<<16)|((Address[1] - 0x00)<<8)|(Address[0]-0x01); //calculate length of data
		  int num_pages = size/FLASH_GetPageSize() +1;											//calculate number of pages to be read
		  /* Load Data */
		  printf("Preparing To Transmit...");
		  /* 2. LOOP THROUGH NUMBER OF PAGES WITH DATA */
		  int payload_size = DRIFTBUFFER_SIZE*(__GET_SAMPLE_COUNT());
		  uint8_t packet [payload_size];

		  for (int i = 0; i < num_pages; ++i)
		  {
			  FLASH_SetAddress(0x00,0x00,0x01);														//read size variables from memory
			  uint8_t* buffer = FLASH_READ_BufferHF(chip,BUFFER1);
			  memcpy(packet,&buffer[1],payload_size);
			  IR_Status_t flag = IR_send_Bin_String(packet,payload_size);
		  	  if(flag == IR_MSG_UPLOAD_OK)
		  	  {
			  	  printf("Message Uploaded!\r\nTransmitting...");
			  	  //create SBD Session
			  	  SBDX_Status_t sbd;
			  	  if(IR_start_SBD_Session(&sbd)== IR_OK)
			  	  {

		 		  	  //check return status
		 		  	  if(sbd.MO_Status < 2)
		 		  	  {
		 			  	  //message sent
		 			  	  printf("Success!\r\n");
		 		  	  }else
		 		  	  {
		 		  		  printf("Failed!\r\n");
		 		  	  }
			  	  }

		  	  }
		  }

	  }
	  IR_DeInit_Module();

/* Reset Memory pointer*/
  //reset sample count
  __SET_SAMPLE_COUNT(0);
  //erase page and reset counter
  FLASH_ERASE_Page(chip);
  FLASH_SetAddress(0x00,0x00,0x00);
  uint8_t val = Active;
  FLASH_WRITE_PageOrByte_NoErase(chip,&val,1);
  FLASH_IncAddress(1);
  uint32_t temp = FLASH_GetAddress();
  Address[0] = temp&0xFF;
  Address[1] = (temp&0xFF00)>>8;
  Address[2] = (temp&0xFF0000)>>16;
  Set_Current_Address_Pointer(chip,Address);
}

static void Routine_Init_STATE(void)
{
	//Initialize and configure the flash chips
	printf("Setting Up Flash Chips...\r\n");
	uint8_t status;

	uint32_t backup_val;
	Init_Flash_Chips(&status);
	backup_val = status;
	//set address to point to statusbyte (memory location 0x00,0x00,0x00)

	//Find the bit set in the lowest bit of the status position

	uint8_t Active_Set = 0;			//flag to keep track of chip statuses
	printf("Allocating Chip Statuses...\r\n");
	for (int i = 0; i < 4; ++i)
	{
		uint8_t set = (status & 0b1<<i)>>i;
		if(set)
		{

			Chip_Status_t flashchips;
			if(Active_Set == 0)		// no chip set yet
			{
				flashchips = Active;
				backup_val |= ((i+1)<<8);
				Active_Set++;
			}else if(Active_Set == 1)//1st chip found
			{
				flashchips = Next_Active;
				backup_val |= (i+1)<<24;
				Active_Set++;
			}else
			{
				flashchips = Inactive;
			}
			FLASH_SetAddress(0x00,0x00,0x00);
			FLASH_ERASE_Page(i+1);
			FLASH_WRITE_PageOrByte_NoErase((i+1),&flashchips,1);
			uint8_t* buff;
			buff = FLASH_READ_Page(i+1);
			printf("Chip %d Status: %d\r\n",i+1,*buff);
			//increment current address for the chip and save to back up registers
			FLASH_IncAddress(1);
			uint32_t address = FLASH_GetAddress();
			Set_Current_Address_Pointer(i+1,(uint8_t*)&address);
		}else
		{
			printf("Chip %d Offline!\r\n",i+1);
		}
	}
	//Store Chip status variable in RTC->BCKUP byte 0 : chip status, byte 1 : active chip, byte 2: back up chip
	 __HAL_RCC_PWR_CLK_ENABLE();
	 RTC->BKP1R = backup_val;
	 __HAL_RCC_PWR_CLK_DISABLE();

	 /* Test Iridium Modem */
	 uint8_t retries = 0;
	 while(retries++ < 20)
	 {
		 if(IR_Init_Module() == IR_OK)
	 	 {
			 IR_On = 1;
			 break;
	 	 }
	 	 IR_DeInit_Module();
	 }
	 if(IR_On)
	 {
		 printf("Iridium Module online!\r\n");
	 }else
	 {
		 printf("Error Connecting To Modem\r\n");
	 }
	 IR_DeInit_Module();

	 /* Test BMP280 Sensor*/
	 BMP_Init_Typedef BMP_InitStruct = {0};
	 BMP280_Init_Preset_Mode(Weather_Monitoring,&BMP_InitStruct);
	 if(BMP280_Init(&BMP_InitStruct) == BMP_OK)
	 {
		 printf("Environmental Sensor Online!\r\n");
	 }else
	 {
		 printf("Environmental Sensor Offline!\r\n");
	 }

	 /* Test INA 219 Sensor */
	 if(INA219_Init_Sensor() == INA_OK)
	 {
		 printf("Current Monitor Online!");
	 } else
	 {
		 printf("Current Monitor Offline!\r\n");
	 }
}
//===================================== 9. END ======================================//

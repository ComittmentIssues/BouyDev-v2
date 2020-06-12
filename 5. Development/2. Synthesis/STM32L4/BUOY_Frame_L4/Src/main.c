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
#include "Sharc_Frame.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;


/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/


static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Test_BOR_PWR_RTC(void)
{
	  if(__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) == SET)
	  {
		  BOR_Handler();
	  }
	  uint8_t flag = __HAL_RCC_GET_PORRST_FLAG();
	  if(flag  == SET)
	  {
		  const char*  c = "Software Reset Detected. Initializing main program...\r\n";
		  HAL_UART_Transmit(&huart2,(uint8_t*)c,strlen(c),100);
		  POR_Handler();
	  }
	  //check wake up source
	  __HAL_RCC_PWR_CLK_ENABLE();

	  if(READ_REG( RTC->BKP0R) == 0b01)
	  {
		  //system has come from shut down
		  WRITE_REG(RTC->BKP0R,0);
		  /*
		   * On wake up from shut down, RTC is already initialised. Therefore,
		   * simply attach the instance to the handler and continue
		   */
		  hrtc.Instance = RTC;
		  //calculate how much time was spent in shutdown

		  //clear interrupts and disable line
		  HAL_PWREx_DisableInternalWakeUpLine();
		  /* Clear PWR wake up Flag */
		  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	  }
	  else
	  {
		  //system encountered a power on reset, put peripherals here
		  MX_RTC_Init();
		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
	  }
	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	  MX_RTC_Init();
	  /* USER CODE BEGIN 2 */
	  //go to sleep until recieved charachter from serial
	//	HAL_SuspendTick();
	//  	USART_Enter_Standby_for_data(&huart2);
	//  	SystemClock_Config();
	//  	HAL_ResumeTick();
	//  	printf("System Awake\r\n");
	 //System wakes up and resumes
	  	HAL_RTC_GetTime(&hrtc,&htime,RTC_FORMAT_BIN);
	    HAL_RTC_GetDate(&hrtc,&hdate,RTC_FORMAT_BCD);
	    //format into string
	    char buff[100] = {0};
	    sprintf(buff,"%02d:%02d:%02d %d-%d-%d\r",htime.Hours,htime.Minutes,htime.Seconds,hdate.Date,hdate.Month,(2000+hdate.Year));
	    HAL_UART_Transmit(&huart2,(uint8_t*)buff,strlen(buff),100);
	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  Go_To_Sleep(STDBY,__MINS_TO_SECS(1));
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

//=============== SYSTEM INIT & CLOCK CONFIG ===============//

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
  //set pin config t Analog mode for low power
  GPIO_Set_Pin_LP();
  Init_Debug();

  //disable wake up pin sources

  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
  //========================= END =========================//

  //============ POWER AND RESET STATE CHECK ==============//
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
  //========================= END =========================//
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //=============  ASYNCHRONOUS STATE CHECK ===============//
	  /*
	   * If an interrupt occured while the device was sleeping, check the
	   * flags to determine if this occured
	   */

	  //check for interrupts from Iridium
	  __HAL_RCC_PWR_CLK_ENABLE();
	  if(__HAL_PWR_GET_FLAG(IMU_EVENT_WAKE_FLAG)|| __HAL_PWR_GET_FLAG(IRIDIUM_RING_WAKE_FLAG))
	  {
		  Current_State = __GET_PREV_STATE();

	  //=============  ASYNCHRONOUS ROUTINES  ===============//

		  if(__HAL_PWR_GET_FLAG(IMU_EVENT_WAKE_FLAG))
		  {
			  __HAL_PWR_CLEAR_FLAG(IMU_EVENT_WAKE_FLAG);
			  printf("IMU Event Detected: Sampling...");
			  for (int i = 0; i < 50; ++i)
			 {
			 	HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
			 	HAL_Delay(100);
			 }
			 printf("Done\r\n");

		  }
		  if(__HAL_PWR_GET_FLAG(IRIDIUM_RING_WAKE_FLAG))
		  {
			__HAL_PWR_CLEAR_FLAG(IRIDIUM_RING_WAKE_FLAG);
			printf("Incoming Message from Satelite: Recieving...");
			for (int i = 0; i < 10; ++i)
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
				HAL_Delay(500);
			}
			printf("Message Recieved!\r\n");
		  }

	  //========================= END =========================//

		  if(Current_State == STATE_SLEEP)
		  {
			  //check how long device was asleep for
			  printf("System Going Back To Sleep\r\n");
			  //enable wake up pin
			  set_WUP_Pin(IMU_EVENT_WAKE_PIN, MODE_WUP);
			  set_WUP_Pin(IRIDIUM_RING_WAKE_PIN, MODE_WUP);
			  Go_To_Sleep(STDBY,10);
		  }
		  	 else
		  {
			  //if come from wake mode
		  	  printf("Going Back to Main Loop:\r\n");
			  __SET_CURRENT_STATE(STATE_ASYNCINT);
		  }
	  }

	  //========================= END =========================//

	  //================= SYSTEM STATE CHECK ==================//
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
	  	 //attach RTC instance to handler
	  	hrtc.Instance = RTC;
	  	//clear wake up pending interrupt
	  	HAL_PWREx_DisableInternalWakeUpLine();
	  	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	  	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	  	//disable external wake up pins
	  	HAL_PWR_DisableWakeUpPin(IRIDIUM_RING_WAKE_PIN);
	  	HAL_PWR_DisableWakeUpPin(IMU_EVENT_WAKE_PIN);
	  	//set Current State to Sample
	  	Current_State = STATE_SAMPLE;
	  	 break;

	  	 case STATE_TRANSMIT:
	  	 Current_State = STATE_SLEEP;
  		 __HAL_RCC_PWR_CLK_ENABLE();
  		 __SET_CURRENT_STATE(Current_State);
  		 __HAL_RCC_PWR_CLK_DISABLE();
	  	 break;

	  	 //default case: reset state
	  	 default:
	  	 Current_State = STATE_RESET;

	  }
	  __HAL_RCC_PWR_CLK_DISABLE();


	  //========================= END =========================//

	  //==================== STATE FUNCTIONS ==================//
	  /*
	   * Code that runs dependant on the Current state of the buoy
	   */

	  /* Initialize all configured peripherals */
	  if(Current_State == STATE_SLEEP)
	  {

		  printf("Current State: SLEEP \t Next State: SAMPLE\r\n");
		  printf("Good Night! \r\n");
		  set_WUP_Pin(IRIDIUM_RING_WAKE_PIN,MODE_WUP);
		  set_WUP_Pin(IMU_EVENT_WAKE_PIN,MODE_WUP);
		  Go_To_Sleep(STDBY,10);
	  }
	  if(Current_State == STATE_RESET)
	  {
		 MX_RTC_Init();
		 HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
		 printf("Current State: RESET \t Next State: SAMPLE\r\n");
	  }

	  if(Current_State == STATE_SAMPLE)
	  {
		  sample_count = __GET_SAMPLE_COUNT();
		  if(sample_count < 3)
		  {
			  printf("Current State: SAMPLE \t Next State: SLEEP\r\n");
		  } else
		  {
			  printf("Current State: SAMPLE \t Next State: TRANS\r\n");
		  }

		  //routine: Flash LED 3 times every 500 ms
		  for (int var = 0; var < 6; ++var)
		  {
			  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
			  HAL_Delay(250);
		  }
		  //increment sample counter
	  }
	  if(Current_State == STATE_TRANSMIT)
	  {
		  printf("Current State: TRANS \t Next State: SLEEP\r\n");

		  printf("Transmitting Package...");
		  for (int var = 0; var < 6; ++var)
		  {
			  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
			  HAL_Delay(500);
			  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,RESET);
			  HAL_Delay(250);
		  }
		  printf("Done!\r\n");
		  //reset sample count
		  __SET_SAMPLE_COUNT(0);
	  }
	  //========================= END =========================//

	  //================ END OF STATE FUNCTION ================//

	  //save state
	  __HAL_RCC_PWR_CLK_ENABLE();
	  if(Current_State == STATE_SAMPLE)
	  {
		  sample_count = __GET_SAMPLE_COUNT();
		  __SET_SAMPLE_COUNT(++sample_count);
	  }
	  __SET_CURRENT_STATE(Current_State);
	  __HAL_RCC_PWR_CLK_DISABLE();
	  //========================= END =========================//
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
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

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

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
  /* USER CODE BEGIN RTC_Init 2 */
  //Configure RTC_Wake up time for range 250ms - 36 Hours
  if(__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hrtc,RTC_FLAG_WUTF))
  {
	  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc,RTC_FLAG_WUTF);
  }

  /* USER CODE END RTC_Init 2 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

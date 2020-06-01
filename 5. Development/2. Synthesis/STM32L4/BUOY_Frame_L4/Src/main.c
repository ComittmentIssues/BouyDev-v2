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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	SHUTDOWN,
	STDBY
}PWR_MODE_t;

typedef enum
{
	STATE_RESET 	= 0b01,
	STATE_SAMPLE 	= 0b10,
	STATE_SLEEP 	= 0b11,
	STATE_TRANSMIT  = 0b110
}Buoy_State_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//The following lines of code allow for printf statements to output to serial via USART2. Remove code if not neccessary
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

#define RTC_WUCK_Period 1799 //s
/*
 * Debug Defines: These macros enable/ disable debuggin functions for the buoy such as testing the clock input
 *
 */

/*
 * @brief: enables function to map the Master Clock output to Pin PA8
 */
#define DEBUG_HSE_OUTPUT_ENABLE
/*
 * @brief: enables initialisation of USART2 to allow buoy to print data to  computer
 */
#define DEBUG_USART_ENABLE
/*
 * @brief: enables initialisation of LED1: on the NUCLEO-L4 development board
 */
#define DEBUG_LED1_ENABLE
/*
 * @brief: enables Debug for low power mode allowing the debugger to still work when mcu is in stop, sleep or stdby mode
 */
#define DEBUG_LP_ENABLE
#define RCC_FLAG_PORRST (0b101<<26)

#define __HAL_RCC_GET_PORRST_FLAG() ((READ_REG(RCC->CSR)&(RCC_FLAG_PORRST))>>26)&&0b111
//#define __RCC_GET_FLAG_PORRST()
#define __MINS_TO_SECS(x) (x)*60

#define __HRS_TO_SECS(x) (x)*3600

#define __DAYS_TO_SECS(x) (x)*__HRS_TO_SECS(24)


/*
 * @brief: state machine Macros
 */
//Returns the state the buoy was in prior to state check
#define __GET_PREV_STATE() (RTC->BKP0R)&0xFF

#define __SET_CURRENT_STATE(state) WRITE_REG(RTC->BKP0R,(RTC->BKP0R &0xFF00)|(state))

#define __GET_SAMPLE_COUNT()  ((RTC->BKP0R)&0xFF00)>>8

#define __SET_SAMPLE_COUNT(count)  WRITE_REG(RTC->BKP0R,(RTC->BKP0R &0xFF)|((count)<<8))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RTC_DateTypeDef hdate;
RTC_TimeTypeDef htime;
__IO uint8_t in_Shutdown = 0;
#ifdef DEBUG_USART_ENABLE
	UART_HandleTypeDef huart2;
#endif

uint32_t time;
Buoy_State_typedef Current_State;
uint8_t Sample_On,sample_count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/*
 * @brief: Function to initialise debugging capabilities based on macro definitions
 *
 * @param: void
 *
 * @return void
 */
static void Init_Debug(void);
static void GPIO_Set_Pin_LP(void);
static void USART_Enter_Standby_for_data(UART_HandleTypeDef *huart);
static void set_WUP_Pin(uint32_t Pin);
void POR_Handler(void);
void BOR_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Init_Debug(void)
{
	// set up clock output on GPIO Pin A8 for testing
#ifdef DEBUG_HSE_OUTPUT_ENABLE
	//configure pin
	/*Configure GPIO pin : PA8 */
	 GPIO_InitTypeDef GPIO_InitStruct;
	 GPIO_InitStruct.Pin = GPIO_PIN_8;
	 GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	 GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	 //map clock output to PIN PA8
	 HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
#endif
	//initialise USART 2 for USB comms
#ifdef DEBUG_USART_ENABLE
	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = 115200;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	  if (HAL_UART_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
#endif

#ifdef DEBUG_LED1_ENABLE
	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  /*Configure GPIO pin : PD2 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,SET);
#endif

#ifdef DEBUG_LP_ENABLE
	  HAL_DBGMCU_EnableDBGStandbyMode();
#endif
}

static void Go_To_Sleep(PWR_MODE_t mode, uint32_t seconds)
{
	//reset wake up pin interrupt
	__HAL_RCC_PWR_CLK_ENABLE();
	//enable pin for sleep
	//set_WUP_Pin(PWR_WAKEUP_PIN2);

	/* Enable Wake Up timer in interrupt mode */
	//set alarm
	 if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,(seconds-1),RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	  {
		  Error_Handler();
	  }
	 HAL_PWREx_EnableInternalWakeUpLine();
	 //if shutdown mode enabled
	 if(mode == SHUTDOWN)
	 {
		 HAL_PWREx_EnterSHUTDOWNMode();
	 }
	 else if(mode == STDBY)
	 {
		 HAL_PWR_EnterSTANDBYMode();
	 }


}

static void USART_Enter_Standby_for_data(UART_HandleTypeDef *huart)
{
	//step 1, wait for ongoing uart transmissions to finish
	printf("Heading to Sleep\r\n");

	//disable systick timer

	HAL_RCCEx_WakeUpStopCLKConfig(RCC_STOP_WAKEUPCLOCK_HSI);
	HAL_UARTEx_EnableClockStopMode(huart);
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_BUSY) == SET);
	//clear all interrupts
	while(__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET);
	UART_WakeUpTypeDef hwake;
	//configure wake up to wake up on start bit
	hwake.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;

	if(HAL_UARTEx_StopModeWakeUpSourceConfig(huart,hwake)!= HAL_OK)
	{
		//error setting sleep state
		Error_Handler();
	}
	//set device to generate interrupt on wake up
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);
	//configure wake up clock source to MSI and enable
	//enable stop mode
	HAL_UARTEx_EnableClockStopMode(huart);
	HAL_UARTEx_EnableStopMode(huart);
	HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
}

/*
 * @brief: This function initialises a pin to be used as an interrupt source to wake up the device from sleep mode
 * 			There are 5 wake up pins on the device. The following table shows the wake up pin mapping to GPIO pin port and
 * 			the input argument to the function to enable it
 * 			+------------------------------------------+
 * 			| Wake Up Pin | GPIO Pin | Argument        |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 1   | PA0      | PWR_WAKEUP_PIN1 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 2   | PC13     | PWR_WAKEUP_PIN2 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 3   | PE6      | PWR_WAKEUP_PIN3 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 4   | PA2      | PWR_WAKEUP_PIN4 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 5   | PC5      | PWR_WAKEUP_PIN5 |
 * 			+------------------------------------------+
 */

static void set_WUP_Pin(uint32_t Pin)
{
	GPIO_TypeDef *Pin_Port;
	IRQn_Type WUP_IRQn;
	GPIO_InitTypeDef GPIO_InitStruct;
	switch (Pin) {
		case PWR_WAKEUP_PIN1:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			Pin_Port = GPIOA;
			GPIO_InitStruct.Pin = GPIO_PIN_0;
			WUP_IRQn = EXTI0_IRQn;
			break;
		case PWR_WAKEUP_PIN2:
			__HAL_RCC_GPIOC_CLK_ENABLE();
			Pin_Port = GPIOC;
			GPIO_InitStruct.Pin = GPIO_PIN_13;
			WUP_IRQn = EXTI15_10_IRQn;
			break;
		case PWR_WAKEUP_PIN3:
			__HAL_RCC_GPIOE_CLK_ENABLE();
			Pin_Port = GPIOE;
			GPIO_InitStruct.Pin = GPIO_PIN_6;
			WUP_IRQn = EXTI9_5_IRQn;
			break;
		case PWR_WAKEUP_PIN4:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			GPIO_InitStruct.Pin = GPIO_PIN_2;
			Pin_Port = GPIOA;
			WUP_IRQn = EXTI2_IRQn;
			break;
		case PWR_WAKEUP_PIN5:
			__HAL_RCC_GPIOC_CLK_ENABLE();
			Pin_Port = GPIOC;
			GPIO_InitStruct.Pin = GPIO_PIN_5;
			WUP_IRQn = EXTI9_5_IRQn;
			break;
		default:
			break;
	}
	//configure pin for exti map


	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(Pin_Port,&GPIO_InitStruct);
	//set NVIC interrupt
    HAL_NVIC_SetPriority(WUP_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(WUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(WUP_IRQn);
    //enable wup in PWR register
    HAL_PWR_EnableWakeUpPin(Pin);
}

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
	  __HAL_RCC_PWR_CLK_ENABLE();
	  switch(__GET_PREV_STATE())
	  {
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
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 26;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2,(uint8_t*) &ch,1,0xFFFF);
	return ch;
}
static void GPIO_Set_Pin_LP(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  /*Configure GPIO pins : PC13 PC0 PC1 PC2
	                           PC3 PC4 PC5 PC6
	                           PC7 PC8 PC9 PC10
	                           PC11 PC12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
	                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
	                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
	                          |GPIO_PIN_11|GPIO_PIN_12;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA0 PA1 PA4 PA6
	                           PA7 PA9 PA10 PA11
	                           PA12 PA15 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_6
	                          |GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
	                          |GPIO_PIN_12|GPIO_PIN_15;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : LD2_Pin */
	  GPIO_InitStruct.Pin = LD2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PB0 PB1 PB2 PB10
	                           PB11 PB12 PB13 PB14
	                           PB15 PB3 PB4 PB5
	                           PB6 PB7 PB8 PB9 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
	                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
	                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
	                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void POR_Handler(void)
{
	  //clear flags
	  __HAL_RCC_CLEAR_RESET_FLAGS();
	  //clear the back up registers
	  HAL_PWR_EnableBkUpAccess();
	  __HAL_RCC_BACKUPRESET_FORCE();
	  __HAL_RCC_BACKUPRESET_RELEASE();
	  HAL_PWR_DisableBkUpAccess();
	  SystemClock_Config();
	  //deactivate and disable wake up timers
	  HAL_PWREx_DisableInternalWakeUpLine();
	  /* Clear PWR wake up Flag */
	 __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	 __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);


	  //reinitialise the clock
}

void BOR_Handler(void)
{
	  //clear flags
	  __HAL_RCC_CLEAR_RESET_FLAGS();
	  // transmit log to PC
	  char* msg= "Warning! Device encountered a Brown Out. Exiting Program...\r\n";
	  HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),100);
	  //perform system reset
	  POR_Handler();
	  HAL_NVIC_SystemReset();
	  while(1)
	  {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		  HAL_Delay(500);
	  }
}
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

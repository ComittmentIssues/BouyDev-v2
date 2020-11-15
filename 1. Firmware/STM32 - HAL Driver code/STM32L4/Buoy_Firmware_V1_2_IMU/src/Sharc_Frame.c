/*
 * Sharc_Frame.c
 *
 *  Created on: Jun 7, 2020
 *      Author: jamie
 */

#include "Sharc_Frame.h"



static uint8_t driftbuffer[DRIFTBUFFER_SIZE] = {0};

HAL_StatusTypeDef Init_Debug(void)
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
	    return HAL_ERROR;
	  }
	  setbuf(stdout, NULL);
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
	  HAL_DBGMCU_EnableDBGStandbyMode();	//enable for shutdown mode
	  HAL_DBGMCU_EnableDBGStopMode();

#endif

	  return HAL_OK;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
HAL_StatusTypeDef SystemClock_Config(void)
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
    return HAL_ERROR;
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
    return HAL_ERROR;
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
  return HAL_OK;
}

HAL_StatusTypeDef Go_To_Sleep(PWR_MODE_t mode, uint32_t seconds)
{
	//reset wake up pin interrupt
	__HAL_RCC_PWR_CLK_ENABLE();
	/* Enable Wake Up timer in interrupt mode */
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, GPIO_PIN_7);

	HAL_PWREx_EnablePullUpPullDownConfig();
	//set alarm
	if(seconds > 0)
	{
	 if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,(seconds-1),RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
	  {
		 return HAL_ERROR;
	  }
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
	 return HAL_OK;
}

HAL_StatusTypeDef USART_Enter_Standby_for_data(UART_HandleTypeDef *huart)
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
		return HAL_ERROR;
	}
	//set device to generate interrupt on wake up
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(huart,UART_IT_WUF);
	//configure wake up clock source to MSI and enable
	//enable stop mode
	HAL_UARTEx_EnableClockStopMode(huart);
	HAL_UARTEx_EnableStopMode(huart);
	HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
	return HAL_OK;
}

void set_WUP_Pin(uint32_t Pin, PinMode_typedef mode)
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
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(Pin_Port,&GPIO_InitStruct);
	//set NVIC interrupt
    HAL_NVIC_SetPriority(WUP_IRQn, 0x0F, 0);
    HAL_NVIC_EnableIRQ(WUP_IRQn);
    HAL_NVIC_ClearPendingIRQ(WUP_IRQn);
    //enable wup in PWR register
    __HAL_RCC_PWR_CLK_ENABLE();
    if(mode == MODE_WUP)
    {
    	__HAL_RCC_PWR_CLK_ENABLE();
    	HAL_PWR_EnableWakeUpPin(Pin);
    	//clear unwanted interrupts

    }else if (mode == MODE_EXTI)
    {
    	HAL_PWR_DisableWakeUpPin(Pin);
    }
   	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF4);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF5);
    __HAL_GPIO_EXTI_CLEAR_IT(EXTI_LINE_5);
    __HAL_GPIO_EXTI_CLEAR_IT(EXTI_LINE_13);
    __HAL_GPIO_EXTI_CLEAR_IT(EXTI_LINE_0);
   	__HAL_GPIO_EXTI_CLEAR_IT(EXTI_LINE_2);
   	__HAL_GPIO_EXTI_CLEAR_IT(EXTI_LINE_6);
	__HAL_RCC_PWR_CLK_DISABLE();
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2,(uint8_t*) &ch,1,0xFFFF);
	return ch;
}

void GPIO_Set_Pin_LP(void)
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

/*
 * @brief: Function code for returning Flash Chip address from storage
 *
 */
uint8_t Get_Current_Address_Pointer(uint8_t chip,uint8_t* address_Array)
{


	if(chip < 4 && chip > 0)
	{
		__HAL_RCC_PWR_CLK_ENABLE();
		//create pointer to register
		uint32_t* address = (uint32_t*)(&RTC->BKP2R) + (chip -1); //
		uint32_t temp = *address;
		//break up 24 bit number into 3 x 8 bit integer array
		for (int i = 0; i < 3; ++i)
		{
			address_Array[i] = (temp>>(i*8))&0xFF;
		}
		__HAL_RCC_PWR_CLK_DISABLE();
		return 1;
	}
	//chip value out of range
	return 0;
}

/*
 * @brief: Function code for storing Flash Chip address
 *
 */
uint8_t Set_Current_Address_Pointer(uint8_t chip,uint8_t* address_Array)
{
	if(chip <= 4 && chip > 0)
		{
			__HAL_RCC_PWR_CLK_ENABLE();
			//create pointer to register
			uint32_t* address = (uint32_t*)(&RTC->BKP2R) + (chip -1); //
			uint32_t temp = 0;
			//break up 24 bit number into 3 x 8 bit integer array
			for (int i = 0; i < 3; ++i)
			{
				temp |= address_Array[i] <<(8*i);
			}
			*address = temp;
			__HAL_RCC_PWR_CLK_DISABLE();
			return 1;
		}
		//chip value out of range
		return 0;
}

/*
 * @brief: Code for retrieving active chip from back up registers
 *
 */
uint8_t Get_Active_Chip(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	uint8_t chip = ((RTC->BKP1R)&0xFF00)>>8;
	__HAL_RCC_PWR_CLK_DISABLE();
	return chip;
}

void Set_Active_Chip(uint8_t chipnumber)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	uint32_t val = (RTC->BKP1R)&0xFFFF00FF;
	val |= (chipnumber<<8);
	RTC->BKP1R = val;
	__HAL_RCC_PWR_CLK_DISABLE();
}

uint8_t Get_Next_Active_Chip(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	uint8_t chip = ((RTC->BKP1R)&0xFF000000)>>24;
	__HAL_RCC_PWR_CLK_DISABLE();
	return chip;
}

void Set_Next_Active_Chip(uint8_t chipnumber)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	uint32_t val = (RTC->BKP1R)&0x00FFFFFF;
	val |= (chipnumber<<24);
	RTC->BKP1R = val;
	__HAL_RCC_PWR_CLK_DISABLE();
}

uint8_t* to_binary_format(GPS_Data_t gps_data ,uint8_t ID)
{
	//define union to convert 32 bit float to unsigned uint8_t array
	union
	{
		float a;					//for 32 bit floats
		unsigned char bytes[4];		//for 4 byte long unsigned integer arrays
		uint32_t num;				//for long numbers
	} byte_converter;
	//define union to convert uint16_t/int16_t into unsigned 8-bit byte arrays
	union
	{
		int16_t  s_halfword;
		unsigned char bytes[2];
	}halfword_converter;

	/*0. Byte 0: Packet ID*/
	*driftbuffer = ID;
	/*1. Bytes 1 - 4:  Epoch Time*/
	byte_converter.num = gps_data.Etime;
	memcpy(&driftbuffer[1],(byte_converter).bytes,4);

	/* Add coordinates bytes 1 - 5 = lat, bytes 6 - 10 = long*/
	byte_converter.a = gps_data.coordinates.lat;
	memcpy(&driftbuffer[5],byte_converter.bytes,4);
	byte_converter.a = gps_data.coordinates.longi;
	memcpy(&driftbuffer[9],byte_converter.bytes,4);


	//convert HDOP,VDOP, PDOP to 2 bytes big endian
	driftbuffer[13] = gps_data.diag.HDOP.digit;
	driftbuffer[14] = gps_data.diag.HDOP.precision;
	driftbuffer[15] = gps_data.diag.VDOP.digit;
	driftbuffer[16] = gps_data.diag.VDOP.precision;
	driftbuffer[17] = gps_data.diag.PDOP.digit;
	driftbuffer[18] = gps_data.diag.PDOP.precision;
	driftbuffer[19] = (gps_data.diag.num_sats);
	driftbuffer[19] = driftbuffer[19]<<2;
	driftbuffer[19] |= gps_data.diag.fix_type;

	//breakdown temperature into 4 unsigned bytes
	byte_converter.num = (uint32_t)gps_data.env_Temp;
	memcpy(&driftbuffer[20],byte_converter.bytes,sizeof(uint32_t));

	//break down pressure into 4 unsigned bytes
	byte_converter.num = gps_data.atm_Press;
	memcpy(&driftbuffer[24],byte_converter.bytes,sizeof(uint32_t));

	//shunt voltage
	halfword_converter.s_halfword = gps_data.shunt_v;
	memcpy(&driftbuffer[29],halfword_converter.bytes,sizeof(int16_t));

	//bus voltage
	halfword_converter.s_halfword = gps_data.bus_v;
	memcpy(&driftbuffer[31],halfword_converter.bytes,sizeof(int16_t));

	//current
	halfword_converter.s_halfword = gps_data.current;
	memcpy(&driftbuffer[33],halfword_converter.bytes,sizeof(int16_t));

	//power
	halfword_converter.s_halfword = gps_data.power;
	memcpy(&driftbuffer[35],halfword_converter.bytes,sizeof(int16_t));

	driftbuffer[get_driftBuffer_Size() -1] = 0xd; //end of packet character

	//return a pointer the the array driftbuffer;
	return driftbuffer;
}

uint8_t  get_driftBuffer_Size(void)
{
	return DRIFTBUFFER_SIZE;
}
/******************************* HANDLER FUNCTIONS *******************************/

/*
 * @brief: Routine for Ring Alert Detection on IRidium Ring Pin
 */

void Routine_ASYNC_IRIDIUM_RX(void)
{
	printf("Incoming Message from Satellite: Receiving...");
	//intialise modem
	if(IR_Init_Module() == IR_OK)
	{
		//recieve message
		uint8_t message[100] = {0};
		uint16_t num_messages = 0;
		IR_recieve_String(message,100,&num_messages);
	}
	printf("Message Received!\r\n");
}

/*
 * @brief: Routine for IMU event detection
 */
void Routine_Async_IMUevent_Sample(void)
{

		 printf("IMU Event Detected while awake: Sampling...");

		printf("Done\r\n");
		//ROUTINE END
}
/*
 * @brief: Power On Reset Handler for BUOY (CASE: NRST Line Pulled low)
 */
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
/*
 * @brief: Brown Out Reset Handler for BUOY (case Vbat < Vbrownoutthreshold)
 */
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
	  /*
	   * Failure to perform system reset causes device to enter an infinite loop
	   */
	  while(1)
	  {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
		  HAL_Delay(500);
	  }
}

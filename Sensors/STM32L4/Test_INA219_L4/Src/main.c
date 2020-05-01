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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	CONFIG_REG = 0x00,
	V_SHUNT_REG = 0x01,
	V_BUS_REG = 0x02,
	POWER_REG = 0x03,
	CURRENT_REG = 0x04,
	CALIBRATION_REG = 0x05
}INA_Register_t;

typedef enum
{
	INA_OK,
	INA_I2C_READ_ERROR,
	INA_I2C_WRITE_ERROR,
	INA_DEVICE_ONLINE,
	INA_DEVICE_OFFLINE,
} INA_Status_t;

typedef enum
{
 Default,
 Configured,
}INA_Config_Status;

typedef struct
{
	uint16_t INA_BUS_VOLTAGE_RANGE;
	uint16_t INA_SHUNT_PGA_RANGE;
	uint16_t INA_BUS_ADC_RESOLUTION;
	uint16_t INA_SHUNT_RESOLUTION;
	uint16_t INA_OPPERATING_MODE;
}INA219_Init_Typedef;

typedef struct
{
	INA219_Init_Typedef Init;
	uint16_t Config_val;
	I2C_HandleTypeDef ina_i2c;
	INA_Config_Status status;

	float INA219_I_LSB;
	float INA219_Vshunt_LSB;
	float INA219_I_Divider;
	float INA219_V_Divider;
	float INA219_P_LSB;


} INA219_Handle_Typedef;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_I2C_2 //change this to the corresponding I2C handler
//config register definitions
#define INA219_CONFIG_RESET 0b1<<15	//setting this bit causes device to reset
#define INA219_CONFIG_BRNG_32V_EN 0b1<<13
#define INA219_CONFIG_PG_2 0b01<<11
#define INA219_CONFIG_PG_4 0b10<<11
#define INA219_CONFIG_PG_8 0b11<<11
#define INA219_CONFIG_BADC_MODE_SAMPLE_9_BIT    0b0000<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_10_BIT   0b0001<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_11_BIT   0b0010<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_12_BIT   0b0011<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_12_BIT_2 0b1000<<7

#define INA219_CONFIG_BADC_MODE_SAMPLE_2_samples   0b1001<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_4_samples   0b1010<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_8_samples   0b1011<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_16_samples  0b1100<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_32_samples  0b1101<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_64_samples  0b1110<<7
#define INA219_CONFIG_BADC_MODE_SAMPLE_128_samples 0b1111<<7

#define INA219_CONFIG_SADC_MODE_SAMPLE_9_BIT    0b0000<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_10_BIT   0b0001<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_11_BIT   0b0010<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_12_BIT   0b0011<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_12_BIT_2 0b1000<<3

#define INA219_CONFIG_SADC_MODE_SAMPLE_2_samples   0b1001<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_4_samples   0b1010<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_8_samples   0b1011<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_16_samples  0b1100<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_32_samples  0b1101<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_64_samples  0b1110<<3
#define INA219_CONFIG_SADC_MODE_SAMPLE_128_samples 0b1111<<3

#define INA219_CONFIG_MODE_POWER_DOWN 			    0b000
#define INA219_CONFIG_MODE_SHUNT_TRIGGERERD 	    0b001
#define INA219_CONFIG_MODE_BUS_TRIGGERED 			0b010
#define INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED 		0b011
#define INA219_CONFIG_MODE_ADC_DIS 	 	 			0b100
#define INA219_CONFIG_MODE_SHUNT_CTS 	 			0b101
#define INA219_CONFIG_MODE_BUS_CTS 	 	 			0b110
#define INA219_CONFIG_MODE_SHUNT_BUS_CTS 			0b111

#define INA219_I2C_Address 0x80
#define INA219_DEFAULT_CONFIG 0x399F
#define INA219_R_SHUNT 0.1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
INA219_Handle_Typedef ina;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * INA config register defaults to 0x399F on power up or reset
 * If the device has been configured, the register will return a different value
 * in order to determine if the device is functioning, the updated register value must be provided using
 */
INA_Status_t INA219_Begin(void)
{
	//attach i2c object to ina object
#ifdef USE_I2C_1
	  ina.ina_i2c = hi2c1;
#endif
#ifdef USE_I2C_2
	  ina.ina_i2c = hi2c2;
#endif
#ifdef USE_I2C_3
	  ina.ina_i2c = hi2c3;
#endif

	//check if device is online
	uint8_t temp[2] = {0};
	if(HAL_I2C_IsDeviceReady(&ina.ina_i2c,INA219_I2C_Address,10,100) != HAL_OK)
	{
		return INA_DEVICE_OFFLINE;
	}

	if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,CONFIG_REG,2,temp,2,100) != HAL_OK)
	{
		return INA_I2C_READ_ERROR;
	}
	//check for previous configuration
	uint16_t checkbyte = INA219_DEFAULT_CONFIG;
	if(ina.status == Configured)
	{
		checkbyte = ina.Config_val;
	}
	uint16_t configbyte = (temp[0]<<8) | temp[1];
	if(configbyte == checkbyte)
	{
		return INA_DEVICE_ONLINE;
	}

	return INA_DEVICE_OFFLINE;
}

void INA219_Init(INA219_Handle_Typedef* hina,uint16_t config_reg)
{
	hina->Init.INA_BUS_VOLTAGE_RANGE = config_reg & (0b1<<13);
	hina->Init.INA_SHUNT_PGA_RANGE = config_reg & (0b11<<11);
	hina->Init.INA_BUS_ADC_RESOLUTION = config_reg & (0b1111<<7);
	hina->Init.INA_SHUNT_RESOLUTION = config_reg & (0b1111<<3);
	hina->Init.INA_OPPERATING_MODE = config_reg & 0b111;
}
INA_Status_t INA219_Get_Shunt_Voltage(int16_t *Shunt_Voltage)
{
	uint8_t temp[2] = {0};
	if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,V_SHUNT_REG,2,temp,2,100)!= HAL_OK)
	{
		return INA_I2C_READ_ERROR;
	}
	//convert to signed 16 bit word
	*Shunt_Voltage = (temp[0]<<8) | temp[1];
	return INA_OK;
}

INA_Status_t INA219_Get_Bus_Voltage(int16_t *Bus_Voltage)
{
	uint8_t temp[2] = {0};
	if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,V_BUS_REG,1,temp,2,100)!= HAL_OK)
	{
		return INA_I2C_READ_ERROR;
	}
	uint16_t u_temp = (temp[0]<<8) | temp[1];
	*Bus_Voltage = (int16_t)((u_temp >>3)*4); //remove conv and MOF flags
	return INA_OK;
}

INA_Status_t INA219_Get_Current(int16_t *current)
{
	uint8_t temp[2] = {0};
	if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,CURRENT_REG,1,temp,2,100)!= HAL_OK)
	{
		return INA_I2C_READ_ERROR;
	}
	*current = (temp[0]<<8) | temp[1];
	return INA_OK;
}

INA_Status_t INA219_Get_Power(int16_t *power)
{
	uint8_t temp[2] = {0};
	if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,POWER_REG,1,temp,2,100)!= HAL_OK)
	{
		return INA_I2C_READ_ERROR;
	}
	*power = (temp[0]<<8) | temp[1];
	return INA_OK;
}
/*
 * @brief: The following function writes a 16 bit value to the calibration register whicch
 * 			is used to adjust the current, bias voltage and power. Here, A LSB value is
 * 			calculated based on the user requirements and selected from a range. It would
 * 			be advisable to calculate the value manually and replace it in the funciton below
 * 			please note: the following funcitons has values calculated manually. THese can be
 * 			changed based on the configuration settings. For now, assuming default settings,
 * 			The following is assumed:
 * 			BRNG: 32V, PG: /8, shunt/bus continuous mode, 12 bit adc, No Overflow
 *
 * 			Step 1: Establish following parameters
 *
 * 			Vbus_Max = 32V
 * 			Vshunt_Max = 0.32V
 * 			Rshunt = 0.1 //Shunt resistor value on sharc buoy pcb
 *
 * 			Step 2: Calculate Max Current:
 *
 * 			I_Max = Vshunt_Max/Rshunt = 3.2A
 *
 * 			Step 3: assume a max expected current
 *
 * 			Iexpected_Max = 2A //absolute maximum, will be adjusted after testing
 *
 * 			Step 4: Calculate an LSB step size. Do this by computing the max and min values and
 * 					select a value thats nice between this range
 * 			Min_LSB  = Iexpected_Max/32767 = 61.62 uA
 * 			Max_LSB = Iexpected_Max/4096 = 488.23 uA
 *
 * 			Choose I_LSB = 100uA
 *
 * 			Step 5: compute calibration value
 *
 * 			cal = trunc(0.04096/(Current_LSB*Rshunt)
 *
 * 			Step 6: calculate the Power LSB
 * 					= 20*I_LSB
 *					PWR_LSB = 2 mW
 *
 *			Step 7: Max_Current = Current_LSB*32767. Check to see if this value
 *					is greater than the maximum possible current. IF it is, then
 *					the max current before overflow is the maximum possible current
 *
 *			Step 8: Calculate Max Power
 *@param I_MBO pointer to float to hold the result of the I_Max before overflow calculation
 *@param V_MBO pointer to float to hold the result of the V_Max before overflow calculation
 *@param P_max pointer to float to hold the result of the max power calculation
 *@
 *@return INA_Status_t integer to show the status of the function
 *
 */
INA_Status_t INA219_Calibrate_No_Overflow(float *I_MBO, float *V_MBO, float *P_Max)
{
	//calculate maximum current
	float I_LSB = 100.0/1000000.0;
	uint16_t I_cal_val = (uint16_t)(0.04096/(I_LSB*INA219_R_SHUNT));
	ina.INA219_P_LSB = 20*I_LSB;
	//calculate max current and shunt voltage values
	float I_max = I_LSB*32767;
	if(I_max  > 3.2) //max possible current
	{
		*I_MBO = 3.2;
	}else
	{
		*I_MBO = I_max;
	}
	float Vshunt_max = *I_MBO*INA219_R_SHUNT;
	if(Vshunt_max > 0.32)
	{
		*V_MBO = 0.32;
	}
	else
	{
		*V_MBO = Vshunt_max;
	}
	*P_Max = *I_MBO*32;

	//write I_Cal_val to register
	uint8_t temp[2] = {(I_cal_val&0xFF00)>>8,(I_cal_val&0x00FF)};
	if(HAL_I2C_Mem_Write(&ina.ina_i2c,INA219_I2C_Address,CALIBRATION_REG,1,temp,2,100) != HAL_OK)
	{
		return INA_I2C_WRITE_ERROR;
	}

	return INA_OK;

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  INA219_Init(&ina,0x399F);
  INA_Status_t flag =  INA219_Begin();
  float I,V,P;
  INA219_Calibrate_No_Overflow(&I,&V,&P);
  if(flag == INA_DEVICE_ONLINE)
  {

	  HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
  }
  /* USER CODE END 2 */
  int16_t shunt_v, bus_v,current,power;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //read power
	  INA219_Get_Shunt_Voltage(&shunt_v);
	  INA219_Get_Bus_Voltage(&bus_v);
	  INA219_Get_Current(&current);
	  INA219_Get_Power(&power);
	  char buff[1000];
	  sprintf(buff,"V_shunt: %d.%d mV\tV_bus: %d.%d V\tI: %d.%d mA\tP: %d mW\r\n",shunt_v/100,shunt_v %100,bus_v/1000,bus_v%1000,current/10,current%10,power);
	  HAL_UART_Transmit(&huart2,(uint8_t*)buff,strlen(buff),100);
	  HAL_Delay(500);
	  __NOP();

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
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
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00200C28;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
#include "HAL_MPU6050.h"
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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t I2C_TX_CPLT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
mpu_status_t MPU6050_Get_ID(I2C_HandleTypeDef *hi2c,uint8_t* ID);
mpu_status_t MPU6050_Get_MST_Status(I2C_HandleTypeDef *hi2c, uint8_t* status_byte);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 mpu_status_t MPU6050_Get_ID(I2C_HandleTypeDef *hi2c,uint8_t* ID)
 {
	  uint8_t whoami = 0;
	  //check if device is I2C ready
	  if(HAL_I2C_IsDeviceReady(hi2c,MPU_Device_Address,10,100)== HAL_OK)
	  {
		  	  //read 1 byte of WHO_AM_I register into variable
			  HAL_StatusTypeDef flag = HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,WHO_AM_I,1,&whoami,1,100);
			  //verify read was successful
			  if(flag != HAL_OK)
			  {
				 if(flag == HAL_BUSY)
				 {
					 return MPU_I2C_DEVICE_BUSY;
				 }

				 return MPU_I2C_ERROR;
			  }
			  *ID = whoami;
	  }
	  else
	  {
		  //unable to connect to device
		  return MPU_I2C_DEVICE_OFFLINE;
	  }
	  return MPU_OK;
 }

 mpu_status_t MPU6050_Get_MST_Status(I2C_HandleTypeDef *hi2c, uint8_t* status_byte)
 {
	 if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,I2C_MST_STATUS,1,status_byte,1,100) != HAL_OK)
	 {
		 return MPU_I2C_ERROR;
	 }
	 return MPU_OK;
 }

mpu_status_t MPU6050_Get_FactoryTrim_Values(I2C_HandleTypeDef *hi2c,MPU_SelfTest_t *mpu)
 {
	 uint8_t temp[4]={0};
	 if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,SELF_TEST_X,1,temp,4,100)!= HAL_OK)
	 {
		 return MPU_I2C_ERROR;
	 }
	 mpu->A_x = (temp[0]&0b1110000)>>2 | ((temp[3]&0b00110000)>>4);
	 mpu->G_x =  temp[0]&0b0001111;
	 mpu->A_y = (temp[1]&0b1110000)>>2 | ((temp[3]&0b00001100)>>2);
	 mpu->G_y =  temp[1]&0b0001111;
	 mpu->A_z = (temp[2]&0b1110000) |	(temp[3]&0b11);
	 mpu->G_z =  temp[2]&0b0001111;
	 return MPU_OK;
 }

mpu_status_t MPU6050_Set_Cycle_Power_Mode(I2C_HandleTypeDef *hi2c,uint8_t Cycles)
{
	uint8_t pwr1byte = 0;
	pwr1byte |= PWR_MGMT_1_CYCLE_EN;
	//pwr1byte &= ~PWR_MGMT_1_SLEEP_EN;
	uint8_t pwr2byte = Cycles;

	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&pwr1byte,1,100)!= HAL_OK)
	 {
			return MPU_I2C_ERROR;
	 }
	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_2,1,&pwr2byte,1,100)!= HAL_OK)
	{
			return MPU_I2C_ERROR;
	}
	return MPU_OK;
}

mpu_status_t MPU6050_Set_Low_Power_Mode_Acc(I2C_HandleTypeDef *hi2c,uint8_t Cycles)
{
	uint8_t byte[2] = {0};
	byte[0]  = (PWR_MGMT_1_CYCLE_EN | PWR_MGMT_1_TEMP_DIS);
	byte[1]  = (Cycles | PWR_MGMT_2_STBY_XG|PWR_MGMT_2_STBY_YG|PWR_MGMT_2_STBY_ZG);
	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&byte[0],1,100)!= HAL_OK)
	 {
			return MPU_I2C_ERROR;
	 }
	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_2,1,&byte[1],1,100)!= HAL_OK)
	{
			return MPU_I2C_ERROR;
	}
	 return MPU_OK;

}

mpu_status_t MPU6050_Set_Wake(I2C_HandleTypeDef *hi2c)
 {
	 uint8_t byte[2] = {0x00,00};

	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&byte[0],1,100)!= HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&byte[1],1,100)!= HAL_OK)
	{
	 	return MPU_I2C_ERROR;
	}
	 return MPU_OK;
 }

mpu_status_t MPU6050_Set_Sleep_Power_Mode(I2C_HandleTypeDef *hi2c)
 {
	 uint8_t byte = PWR_MGMT_1_SLEEP_EN;

	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&byte,1,100)!= HAL_OK)
	{
		return MPU_I2C_ERROR;
	}

	 return MPU_OK;
 }

mpu_status_t MPU6050_Set_Gyro_FSR(I2C_HandleTypeDef *hi2c,uint8_t FSR)
{
	uint8_t byte = FSR;
	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,GYRO_CONFIG,1,&byte,1,100)!= HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	 return MPU_OK;
}

mpu_status_t MPU6050_Set_Acc_FSR(I2C_HandleTypeDef *hi2c,uint8_t FSR)
{
	uint8_t byte = FSR &0xFF;
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,ACCELEROMETER_CONFIG,1,&byte,1,100)!= HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	 return MPU_OK;
}

mpu_status_t MPU6050_Set_Sample_Rate(I2C_HandleTypeDef *hi2c)
{
	//check if DLPF is enabled
	uint8_t configbyte = 0;
	uint8_t smplrtdiv= 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,CONFIG,1,&configbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	//digital low pass value is last 3 bits of CONFIG register
	if((configbyte &0b111) > 0)
	{
		smplrtdiv= (uint8_t)(GYRO_OUTPUT_RATE_DLPF_EN/SAMPLE_RATE - 1);
	}else
	{
		smplrtdiv = (uint8_t)(GYRO_OUTPUT_RATE_DLPF_DIS/SAMPLE_RATE -1);
	}

	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,SMPLRT_DIV,1,&smplrtdiv,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return HAL_OK;
}

mpu_status_t MPU6050_Set_FSync(I2C_HandleTypeDef *hi2c, uint8_t Fsync)
{
	//get CONFIG Register
	uint8_t configbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,CONFIG,1,&configbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}

	configbyte|= (Fsync&0b111000);
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,CONFIG,1,&configbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;
}

mpu_status_t MPU6050_Set_DLPF(I2C_HandleTypeDef *hi2c, uint8_t DLPF)
{
	//mask unwanted bits
	uint8_t configbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,CONFIG,1,&configbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	configbyte |= (DLPF &0b111);
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,CONFIG,1,&configbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;
}

mpu_status_t MPU6050_Enable_Interrupt(I2C_HandleTypeDef *hi2c, uint8_t interrupts)
{
	uint8_t iconfigbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,INT_ENABLE,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	//set enable bit
	iconfigbyte |= interrupts;
	//write to register
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,INT_ENABLE,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;
}

mpu_status_t MPU6050_Get_Interrupt_Status(I2C_HandleTypeDef *hi2c, Interrupt_source_t interrupt_src,uint8_t* res)
{
	uint8_t istatus = 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,INT_STATUS,1,&istatus,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}

	switch (interrupt_src)
	{
		case FIFO_OVERFLOW:
			*res = (istatus&INT_STATUS_FIFO_OVERFLOW)>>4;
			break;
		case I2C_MST_INT:
			*res = (istatus&INT_STATUS_I2C_MST)>>3;
		case DATA_READY:
			*res = (istatus&INT_STATUS_DATA_RDY);
		default:
			break;

	}

	return HAL_OK;
}

mpu_status_t MPU6050_Get_IMU_Data(I2C_HandleTypeDef *hi2c,uint8_t* imu)
{

	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,ACCEL_XOUT_H,1,imu,14,10) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}

	return MPU_OK;
}
/*
 * @brief: Enables the IMU FIFO buffer and configures it to recieve
 * 		   Data from peripherals determined by the fifo_Mask
 * @param: hi2c - pointer to I2C handle typedef
 * 		   enable - can either be (ENABLE - enables FIFO Buffer), DISABLE - disables BUFFER, RESET (3)- resets buffer
 */
mpu_status_t MPU6050_FIFO_Init(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
	//get USER_CTRL register Data
	uint8_t uc_byte;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,USER_CTRL,1,&uc_byte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	//set relevent bits
	if(enable == ENABLE)
	{
		uc_byte |= USER_CTRL_FIFO_EN;
	}else if(enable == DISABLE)
	{
		uc_byte &= ~USER_CTRL_FIFO_EN;
	}else if(enable == 3)
	{
		uc_byte &= ~USER_CTRL_FIFO_EN;
		uc_byte |= USER_CTRL_FIFO_RESET;
	}
	//write values to data register
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,USER_CTRL,1,&uc_byte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;

}

mpu_status_t MPU6050_Signal_conditioned_Reset(I2C_HandleTypeDef *hi2c)
{
	//write reset condition to USER_CTRL
	uint8_t byte = USER_CTRL_SIG_COND_RESET;
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,USER_CTRL,1,&byte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	(void)byte;
	//wait for sig cond bit to reset
	uint8_t sig_reset_complete = 1;
	while(sig_reset_complete)
	{
		if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,USER_CTRL,1,&sig_reset_complete,1,100) != HAL_OK)
		{
			return MPU_I2C_ERROR;
		}
		sig_reset_complete &= 0b1;
	}
	return MPU_OK;
}

mpu_status_t  MPU6050_FIFO_Config(I2C_HandleTypeDef *hi2c, uint8_t fifo_mask )
{
	uint8_t fifobyte;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,FIFO_EN,1,&fifobyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	fifobyte |= fifo_mask;
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,FIFO_EN,1,&fifobyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;
}

mpu_status_t MPU6050_Get_FIFO_Count(I2C_HandleTypeDef *hi2c,uint16_t* count)
{
	uint8_t fifo_reg[2] = {0};
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,FIFO_COUNTH,2,fifo_reg,2,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}

	*count = ((fifo_reg[0]&0xFF)<<8) | (fifo_reg[1] &0xFF);
	return MPU_OK;

}

/*
 * @brief: Function to enable/disable the temperature sensor
 *
 * @param: hi2c - pointer to I2C handle
 * 		   cmd 	- set to ENABLE to enable the reading, DISABLE to disable
 */
mpu_status_t MPU6050_init_TempSensor(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
	//get register data
	uint8_t pwrmgmtbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}

	if(cmd == ENABLE)
	{
		pwrmgmtbyte &= ~(PWR_MGMT_1_TEMP_DIS);
	}else if(cmd == DISABLE)
	{
		pwrmgmtbyte |= PWR_MGMT_1_TEMP_DIS;
	}else
	{
		return MPU_INIT_CMD_ERROR;
	}
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;
}
//----------------------------- TEST MODULES ----------------------------------//
void Test_PowerMode(void)
{
	  MPU6050_Set_Cycle_Power_Mode(&hi2c1,PWR_MGMT_2_LP_WAKE_CTRL_40HZ);
	  uint8_t byte[2];
	  HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,PWR_MGMT_1,1,byte,2,100);
	  MPU6050_Set_Low_Power_Mode_Acc(&hi2c1,PWR_MGMT_2_LP_WAKE_CTRL_1_25HZ);
	  HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,PWR_MGMT_1,1,byte,2,100);
	  MPU6050_Set_Sleep_Power_Mode(&hi2c1);
	  HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,PWR_MGMT_1,1,byte,2,100);
	  MPU6050_Set_Wake(&hi2c1);
	  HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,PWR_MGMT_1,1,byte,2,100);
}

void MPU6050_DMA_PeriphIRQHandler(void)
{
	I2C_TX_CPLT = 1;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t whoami = 0;
  uint8_t status = 0;
  MPU6050_Get_ID(&hi2c1,&whoami);
  if(whoami == WHO_AM_I_VALUE)
  {
	  MPU_SelfTest_t mpu;
	  MPU6050_Get_FactoryTrim_Values(&hi2c1,&mpu);
	 //wake up device
	 MPU6050_Set_Wake(&hi2c1);
	 //configure gyroscope settings
	 //set accel only mode
	 MPU6050_Set_Low_Power_Mode_Acc(&hi2c1,PWR_MGMT_2_LP_WAKE_CTRL_20HZ);
	 MPU6050_Set_Gyro_FSR(&hi2c1,GYRO_CONFIG_FSSEL_250DPS);
	 //configure accelerometer settings
	 MPU6050_Set_Acc_FSR(&hi2c1,ACC_CONFIG_AFSSEL_4G);
	 //configure digital lowpass filter
	// MPU6050_Set_DLPF(&hi2c1,CONFIG_DLFP_3);
	 //disable temp sensor
	 MPU6050_init_TempSensor(&hi2c1,DISABLE);
	 //configure outputrate
	 MPU6050_Set_Sample_Rate(&hi2c1);
	 //enable interrupt when data ready
	 MPU6050_Enable_Interrupt(&hi2c1,INT_ENABLE_DATA_RDY_EN);



  }
  /* USER CODE END 2 */
  	  uint8_t res[14];
  	  uint8_t interrupt_status = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  //wait untill data ready becomes set
	  while(!interrupt_status)
	  {
		  MPU6050_Get_Interrupt_Status(&hi2c1,DATA_READY,&interrupt_status);
	  }
	  interrupt_status = 0;
	 MPU6050_Get_IMU_Data(&hi2c1,res);
	 HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00200C28;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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

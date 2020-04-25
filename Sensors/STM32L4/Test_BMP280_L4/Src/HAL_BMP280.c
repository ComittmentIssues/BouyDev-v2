/*
 * HAL_BMP280.c
 *
 *  Created on: Apr 25, 2020
 *      Author: jamie
 */

#include "HAL_BMP280.h"
#include  "stm32l4xx_hal.h"

static HAL_StatusTypeDef MX_SPI1_Init(void);
static void Init_Control_Pins(void);

static HAL_StatusTypeDef MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
  return HAL_OK;
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void Init_Control_Pins(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SPI_CS_PIN_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

/*
 * @brief: Function to Read data from an 8bit register in the BMP280. Setting the size variable to
 * 		   A number > 1 configures the device for burst read.
 * @param: hspi - SPI handler for communication
 * @param: reg: uint8_t address of the register
 * @param: size: number of registers to be read from
 * @param: data: a buffer to hold the data
 */
BMPStatus_t BMP280_Reset(void)
{
	uint8_t data = BMP280_Soft_Reset&0xFF;
	if(BMP280_Write_Register(Sreset,1,&data) != BMP_OK)
	{
		return BMP_RESET_ERROR;
	}
	HAL_Delay(4);

	return BMP_OK;
}

BMPStatus_t BMP280_Set_PowerMode(uint8_t mode)
{
	uint8_t reg =0;
	if(BMP280_Read_Register(ctrl_meas,1,&reg) != BMP_OK)
	{
		return BMP_SPI_READ_ERROR;
	}
	//clear the old settings
	reg &= 0b11111100;
	//configure the new mode
	reg |= mode;
	if(BMP280_Write_Register(ctrl_meas,1,&reg) != BMP_OK)
	{
		return BMP_Config_Error;
	}
	return BMP_OK;
}

BMPStatus_t BMP280_Set_Oversample(uint8_t osrs_t, uint8_t osrs_p)
{
	uint8_t reg = 0;
	if(BMP280_Read_Register(ctrl_meas,1,&reg) != BMP_OK)
	{
		return BMP_SPI_READ_ERROR;
	}
	//clear the old settings
	reg &= 0b11;
	//configure the new mode
	reg |= (osrs_t |osrs_p);
	if(BMP280_Write_Register(ctrl_meas,1,&reg) != BMP_OK)
	{
		return BMP_Config_Error;
	}
	return BMP_OK;
}

BMPStatus_t BMP280_Set_IIR_Filter_Coeff(uint8_t iircoef)
{
	uint8_t reg,mask = 0b11100;
	if(BMP280_Read_Register(config,1,&reg) != BMP_OK)
	{
		return BMP_SPI_READ_ERROR;
	}
	//clear the old settings
	reg &= ~mask;
	//configure the new mode
	reg |= iircoef;
	if(BMP280_Write_Register(config,1,&reg) != BMP_OK)
	{
		return BMP_Config_Error;
	}
	return BMP_OK;
}

BMPStatus_t BMP280_Set_Standby_Time (uint8_t stdby)
{
	uint8_t reg,mask = 0b11100000;
	if(BMP280_Read_Register(config,1,&reg) != BMP_OK)
	{
		return BMP_SPI_READ_ERROR;
	}
	//clear the old settings
	reg &= ~mask;
	//configure the new mode
	reg |= stdby;
	if(BMP280_Write_Register(config,1,&reg) != BMP_OK)
	{
		return BMP_Config_Error;
	}
	return BMP_OK;
}

BMPStatus_t BMP280_Get_Status(BMP_Handle_Typedef* bmp)
{
	//read contents of register
	uint8_t statusbyte = 0;
	if(BMP280_Read_Register(status,1,&statusbyte) != BMP_OK)
	{
		return BMP_SPI_READ_ERROR;
	}
	bmp->IM_Status = statusbyte&0b1;
	bmp->M_Status = statusbyte&0b1<<3;
	return BMP_OK;
}

BMPStatus_t BMP280_Write_Register(uint8_t reg,int32_t size, uint8_t* data)
{
	uint8_t temp = reg & BMP280_SPI_WRITE;
	__SPI_CS_ENABLE();
	 if(HAL_SPI_Transmit(bmp.bmp_spi,&temp,1,100) != HAL_OK)
	{
		 __SPI_CS_DISABLE();
		 return BMP_SPI_WRITE_ERROR;
	}
	 if(HAL_SPI_Transmit(bmp.bmp_spi,data,size,100) != HAL_OK)
	 {	 __SPI_CS_DISABLE();
		 return BMP_SPI_WRITE_ERROR;
	 }
	 __SPI_CS_DISABLE();
	 return BMP_OK;
}

BMPStatus_t BMP280_Read_Register(uint8_t reg,int32_t size, uint8_t* data)
{
		uint8_t temp = reg | BMP280_SPI_READ;
  	  __SPI_CS_ENABLE();
	  if(HAL_SPI_Transmit(bmp.bmp_spi,&temp,1,100) == HAL_OK)
	  {
		  if(HAL_SPI_Receive(bmp.bmp_spi,data,size,100) != HAL_OK)
		  {
			  __SPI_CS_DISABLE();
			  return BMP_SPI_READ_ERROR;
		  }

	  }else
		  {
		  __SPI_CS_DISABLE();
		  return BMP_SPI_WRITE_ERROR;
		  }

	  __SPI_CS_DISABLE();
	  return BMP_OK;
}

BMPStatus_t BMP280_Get_ID(SPI_HandleTypeDef *hspi, uint8_t* dev_id)
{
	 return BMP280_Read_Register(id,1,dev_id);

}

BMPStatus_t BMP280_Get_FactoryTrim( BMP280_trim_t *bmpt)
{
	uint8_t trim[24]= {0};
	BMP280_Read_Register(calib_0,24,trim);
	bmpt->dig_T1 = 			((uint8_t)trim[1]<<8) |((uint8_t)trim[0]);
	bmpt->dig_T2 = (int16_t)((uint8_t)trim[3]<<8) |((uint8_t)trim[2]);
	bmpt->dig_T3 = (int16_t)((int8_t)trim[5]<<8) |((int8_t)trim[4]);
	/*Preaseure Measurement Readouts */
	bmpt->dig_P1 = 			((uint8_t)trim[7]<<8) |((uint8_t)trim[6]);
	bmpt->dig_P2 = (int16_t)(((uint8_t)trim[9]<<8) |((uint8_t)trim[8]));
	bmpt->dig_P3 = (int16_t)(((uint8_t)trim[11]<<8)|((uint8_t)trim[10]));
	bmpt->dig_P4 = (int16_t)(((uint8_t)trim[13]<<8)|((uint8_t)trim[12]));
	bmpt->dig_P5 = (((uint8_t)trim[15]<<8)|((uint8_t)trim[14]));
	bmpt->dig_P6 = (int16_t)(((uint8_t)trim[17]<<8)|((uint8_t)trim[16]));
	bmpt->dig_P7 = (int16_t)(((uint8_t)trim[19]<<8)|((uint8_t)trim[18]));
	bmpt->dig_P8 = (int16_t)(((uint8_t)trim[21]<<8)|((uint8_t)trim[20]));
	bmpt->dig_P9 = (int16_t)(((uint8_t)trim[23]<<8)|((uint8_t)trim[22]));
	return BMP_OK;
}

BMPStatus_t BMP280_Force_Measure(uint32_t* temp,uint32_t* pressure)
{
	//write forced mode to register
	BMP280_Set_PowerMode(BMP280_CTRLMEAS_MODE_FORCED);
	//wait for device to finish converting
	uint8_t status;
	BMP280_Read_Register(ctrl_meas,1,&status);
	while((status&0b11) != 0)
	{
		BMP280_Read_Register(ctrl_meas,1,&status);
	}
	//wait for control register to finish
	BMP280_Get_Measurements(temp,pressure);
	return BMP_OK;
}

BMPStatus_t BMP280_Get_Measurements(uint32_t* adc_Temp,uint32_t* adc_Press)
{
	uint8_t data[6] = {0};
	BMP280_Read_Register(press_msb,6,data);
	*adc_Temp = ((data[3]&0xFF)<<16)|((data[4]&0xFF)<<8)|((data[5]&0xF0));
	*adc_Temp= *adc_Temp>> 4;
	uint32_t temp= ((data[0]&0xFF)<<16)|((data[1]&0xFF)<<8)|((data[2]&0xFF))/16;
	*adc_Press = temp/16;
	//convert to 20 bit unsigned integers
	return BMP_OK;
}
/*
 * @brief: function to read the 3 temperature registers and combine the data into a 20bit long signed integer.
 * 		   Temp is stored as 3 unsigned integers 8 bits long in registers 0xFA - 0xFB on the BMP280 Sensor. Data
 * 		   is Read MSB first. These 3 bytes represent the raw temperature reading and must be compensated using the
 * 		   algorithm outlined in the data sheet
 *
 * @param: adc_temp - pointer to an signed32_t integer to hold the converted value of temperature
 *
 * @return: BMPStatus_t status of the conversion for error handling
 */

BMPStatus_t BMP280_Get_Temp(uint32_t* adc_Temp)
{
	uint8_t Ttemp[3] = {0};
	//read the registers MSB first
	if(BMP280_Read_Register(temp_msb,3,Ttemp) != BMP_OK)
	{
		return BMP_TEMP_READ_ERROR;
	}
	*adc_Temp = ((Ttemp[0]&0xFF)<<16)|((Ttemp[1]&0xFF)<<8)|((Ttemp[2]&0xF0));
	*adc_Temp= *adc_Temp>> 4;

	return BMP_OK;
}

int32_t BMP280_Compensate_Temp(int32_t T_val,int32_t* t_fine, BMP280_trim_t bmp_trim)
{

		//compensate Temperature from datasheet
		int32_t var1 = (((T_val>>3)- ((int32_t)bmp_trim.dig_T1<<1))*((int32_t)bmp_trim.dig_T2))>>11;
	    int32_t var2 =  (((((T_val>>4) - ((int32_t)bmp_trim.dig_T1)) * ((T_val>>4) - ((int32_t)bmp_trim.dig_T1))) >> 12)*((int32_t)bmp_trim.dig_T3)) >> 14;
		int32_t temp = var1+var2; //for storage in global variable
		*t_fine = temp;
		return (temp*5 +128)/256;
}
/*
 * @brief: function to read the 3 pressure registers and combine the data into a 20bit long signed integer.
 * 		   Pressure is also is stored as 3 unsigned integers 8 bits long in registers 0xF7 - 0xF9 on the BMP280 Sensor. Data
 * 		   is Read MSB first. These 3 bytes represent the raw pressurereading and must be compensated using the
 * 		   algorithm outlined in the data sheet
 *
 * @param: adc_Press - pointer to an signed32_t integer to hold the converted value of pressure
 *
 * @return: BMPStatus_t status of the conversion for error handling
 */

BMPStatus_t BMP280_Get_Pressure(uint32_t* adc_Press)
{
	uint8_t Ptemp[3] = {0};
	//read the registers MSB first
	if(BMP280_Read_Register(press_msb,3,Ptemp) != BMP_OK)
	{
		return BMP_PRESS_READ_ERROR;
	}
	uint32_t temp= ((Ptemp[0]&0xFF)<<16)|((Ptemp[1]&0xFF)<<8)|((Ptemp[2]&0xFF))/16;
	*adc_Press = temp/16;
	return BMP_OK;
}

uint32_t BMP280_Compensate_Pressure(uint32_t P_val,int32_t t_fine,BMP280_trim_t bmp_trim)
{
	//Compensation formula
	int32_t var1 = (int64_t)t_fine - 128000;
	int64_t var2 = var1*var1*((int64_t)(bmp_trim.dig_P6));
	var2 = var2 + (((int64_t)bmp_trim.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bmp_trim.dig_P3)>>8) + ((var1 * (int64_t)bmp_trim.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp_trim.dig_P1)>>33;
	//check for divide by 0 error
	if(var1 == 0)return 0;
	int64_t P = 1048576 - (int32_t)P_val;
	P = (((P<<31)-var2)*3125)/var1;
	var1 = (((int64_t)(bmp_trim.dig_P9)) * (P>>13) * (P>>13)) >> 25;
	var2 = (((int64_t)(bmp_trim.dig_P8)) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)(bmp_trim.dig_P7))<<4);
	return (uint32_t)P;
}
/*
 * @brief: Initialises the sensor handler and performs power on/ begining procedure.
 * 		   After this, the device is configured to user defined settings which are
 * 		   then written to the registers. Note: A BMP_INit_Typedef struct must be initialised
 * 		   and set before this function is called. THis can be done individually or by using the functions
 * 		   BMP280_Init_Preset_Mode(), BMP280_Init_Custom()
 * @param: BMP_InitStruct - struct containing the configuration values for initialising the sensor
 * @return BMPStatus_t - return value showing the status of the function for error handling
 */

BMPStatus_t BMP280_Init(BMP_Init_Typedef * BMP_InitStruct)
{
	//initialise SPI peripheral and control pins
	  MX_SPI1_Init();
	  Init_Control_Pins();
	//first attach an spi handler to the bmp sensor
#ifdef USE_SPI_1
		bmp.bmp_spi = &hspi1;
#elif USE_SPI_2
		bmp.bmp_spi = &hspi2;
#elif USE_SPI_3
		bmp.bmp_spi = &hspi3;
#endif
	//read device id to ensure device is online and functioning
		uint8_t dev_id = 0;
		uint8_t flag = BMP280_Get_ID(bmp.bmp_spi,&dev_id);
		if( flag!= BMP_OK)
		{
			return flag;
		}
		//check device ID matches
		if(dev_id != BMP280_ID)
		{
			return BMP_DEVICE_CHECK_ERROR;
		}
		//soft reset
		BMP280_Reset();
		//read factory trim values
		BMP280_Get_FactoryTrim(&bmp.Factory_Trim);
		//get init data
		bmp.Init = *BMP_InitStruct;
		//write data to ctrl_meas register
		uint8_t ctrlmeasbyte = bmp.Init.BMP_Pressure_OverSample | bmp.Init.BMP_Temperature_OverSample;
		uint8_t configbyte = bmp.Init.BMP_t_Standby | bmp.Init.BMP_IIR_FILTER_COEFFICIENTS;
		flag = BMP280_Write_Register(ctrl_meas,1,&ctrlmeasbyte);
		if(flag != BMP_OK)
		{
			return flag;
		}
		flag = BMP280_Write_Register(config,1,&configbyte);
		if(flag != BMP_OK)
		{
			return flag;
		}

		//in a seperate write, configure the power mode
		if(bmp.Init.BMP_Power_Mode != BMP280_CTRLMEAS_MODE_SLEEP)
		{
			flag = BMP280_Set_PowerMode(bmp.Init.BMP_Power_Mode);
			if(flag != BMP_OK)
			{
				return BMP_POWER_CONFIG_ERROR;
			}
		}
		return BMP_OK;
}

void BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct)
{
	switch(BMP_MODE)
	{
		case HandHeld_LP:
			BMP_InitStruct->BMP_Power_Mode = BMP280_CTRLMEAS_MODE_NORMAL;
			BMP_InitStruct->BMP_Pressure_OverSample = BMP280_CTRLMEAS_OSRSP_OS_16;
			BMP_InitStruct->BMP_Temperature_OverSample = BMP280_CTRLMEAS_OSRST_OS_2;
			BMP_InitStruct->BMP_IIR_FILTER_COEFFICIENTS = BMP280_CONFIG_FILTER_COEFF_4;
			BMP_InitStruct->BMP_t_Standby = BMP280_CONFIG_tsb_62_5;
			break;

		case HandHeld_Dynamic:
			BMP_InitStruct->BMP_Power_Mode = BMP280_CTRLMEAS_MODE_NORMAL;
			BMP_InitStruct->BMP_Pressure_OverSample = BMP280_CTRLMEAS_OSRSP_OS_4;
			BMP_InitStruct->BMP_Temperature_OverSample = BMP280_CTRLMEAS_OSRST_OS_1;
			BMP_InitStruct->BMP_IIR_FILTER_COEFFICIENTS = BMP280_CONFIG_FILTER_COEFF_16;
			BMP_InitStruct->BMP_t_Standby = BMP280_CONFIG_tsb_0_5;
			break;
		case Weather_Monitoring:
			BMP_InitStruct->BMP_Power_Mode = BMP280_CTRLMEAS_MODE_FORCED;
			BMP_InitStruct->BMP_Pressure_OverSample = BMP280_CTRLMEAS_OSRSP_OS_1;
			BMP_InitStruct->BMP_Temperature_OverSample = BMP280_CTRLMEAS_OSRST_OS_1;
			BMP_InitStruct->BMP_IIR_FILTER_COEFFICIENTS = BMP280_CONFIG_FILTER_COEFF_OFF;
			BMP_InitStruct->BMP_t_Standby = 0;
			break;
		default:
			break;
	}
}

void BMP280_Init_Custom(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct, uint8_t mode, uint8_t osrsp, uint8_t osrst,uint8_t ifcoef, uint8_t t_stdby)
{
	BMP_InitStruct->BMP_Power_Mode = mode;
	BMP_InitStruct->BMP_Pressure_OverSample = osrsp;
	BMP_InitStruct->BMP_Temperature_OverSample = osrst;
	BMP_InitStruct->BMP_IIR_FILTER_COEFFICIENTS = ifcoef;
	BMP_InitStruct->BMP_t_Standby = t_stdby;
}


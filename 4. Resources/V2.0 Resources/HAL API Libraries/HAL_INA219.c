/*
 * HAL_INA219.c
 *
 *  Created on: May 3, 2020
 *      Author: Jamie Jacobson
 */
//============================= 1. Includes ==============================================
#include "HAL_INA219.h"

//========================= 2. Private Variables =========================================

I2C_HandleTypeDef hi2c2;	//I2C Handle Instance for communications

//========================= 3. Private Function Prototypes ===============================

/*
 * Function Name static HAL_StatusTypeDef MX_I2C2_Init(void)
 *
 * @brief Initialises I2C communications on I2C2 (Pins PB10 and PB11)
 *
 * @param: none
 *
 * @retval: HAL_StatusTypeDef - Hal Function return status
 */
static HAL_StatusTypeDef MX_I2C2_Init(void);

//======================== 4. Private Function Definitions ==============================

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static HAL_StatusTypeDef MX_I2C2_Init(void)
{
  hi2c2.Instance = INA_I2C_Perpheral;
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
    return HAL_ERROR;
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;


}

//=================== 5. MSP Function Definitions =======================================

/* MSP FUNCTIONS:
 *
 *  Functions designed to replace the _weak MSP peripheral initialization
 *  function definitions in the HAL Library files.
 *
 *  NB!!!! before running the code do the following:
 *
 *  1. Uncomment the desired MSP Function
 *
 *  2. Cut the function and paste it in the stm32l4xx_hal_msp.c file
 *
 *  3. In the stm32l4xx_hal_msp.c file, include the header "HAL_INA219.h"
 */

///**
//* @brief I2C MSP Initialization
//* This function configures the hardware resources used in this example
//* @param hi2c: I2C handle pointer
//* @retval None
//*/
//void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(hi2c->Instance==I2C2)
//  {
//
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    /**I2C2 GPIO Configuration
//    PB10     ------> I2C2_SCL
//    PB11     ------> I2C2_SDA
//    */
//    GPIO_InitStruct.Pin = INA_I2C_SCL_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
//    HAL_GPIO_Init(INA_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = INA_I2C_SDA_PIN;
//    HAL_GPIO_Init(INA_I2C_SDA_GPIO_PORT,&GPIO_InitStruct);
//    /* Peripheral clock enable */
//    __HAL_RCC_I2C2_CLK_ENABLE();
//
//  }
//
//}
//
///**
//* @brief I2C MSP De-Initialization
//* This function freeze the hardware resources used in this example
//* @param hi2c: I2C handle pointer
//* @retval None
//*/
//void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
//{
//  if(hi2c->Instance==I2C2)
//  {
//
//    /* Peripheral clock disable */
//    __HAL_RCC_I2C2_CLK_DISABLE();
//
//    /**I2C2 GPIO Configuration
//    PB10     ------> I2C2_SCL
//    PB11     ------> I2C2_SDA
//    */
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Pin = INA_I2C_SCL_PIN;
//    HAL_GPIO_Init(INA_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);
//
//    GPIO_InitStruct.Pin = INA_I2C_SDA_PIN;
//    HAL_GPIO_Init(INA_I2C_SDA_GPIO_PORT,&GPIO_InitStruct);
//
//  }
//
//}

//================== 6. Init Function Definitions =======================================

INA_Status_t INA219_Init_Sensor(void)
{
	  // initialise I2C peripheral
	  MX_I2C2_Init();
	  if(INA219_Begin() == INA_DEVICE_READY)
	  {
		  float I,V,P;
		  // perform soft reset of device
		  INA219_Reset();
		  //configure the register
		  INA219_Set_Reg_Config(&ina);
		  //calibrate current and power reading
		  INA219_Calibrate_16V_1_2A(&I,&V,&P);
		  //enable power mode
		  INA219_Set_Power_Mode(INA219_CONFIG_MODE_ADC_DIS);
		  return INA_OK;
	  }
	  HAL_I2C_DeInit(&ina.ina_i2c);
	  return INA_INIT_ERROR;
}

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

	uint16_t configbyte = (temp[0]<<8) | temp[1];
	if(configbyte == INA219_DEFAULT_CONFIG) //this will occure if a reset has occured or if the device has not been configured
	{
		ina.Use_Config = Default;
	}else
	{
		ina.Use_Config = Configured;
	}

	return INA_DEVICE_READY;
}

//================== 7. Calib Function Definitions =======================================

/*
 * Function Name INA_Status_t INA219_Calibrate_32V_2A(float *I_MBO, float *V_MBO, float *P_Max)
 *
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
 *
 *@return INA_Status_t integer to show the status of the function
 */

INA_Status_t INA219_Calibrate_32V_2A(float *I_MBO, float *V_MBO, float *P_Max)
{
	//calculate maximum current
	ina.INA219_I_LSB = 100.0/1000000.0;
	uint16_t I_cal_val = (uint16_t)(0.04096/(ina.INA219_I_LSB*INA219_R_SHUNT));
	ina.INA219_P_LSB = 20*ina.INA219_I_LSB;
	//calculate max current and shunt voltage values
	float I_max = ina.INA219_I_LSB*32767;
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

/*
 * Function Name INA_Status_t INA219_Calibrate_16V_1_2A(float *I_MBO, float *V_MBO, float *P_Max)
 * @brief: The following calibration formula is similar to the above one however,
 * 			the values are calculated for 16V range with a 2A expected current and
 * 			a 160mV shunt range
 *
 * 	Step 1: V_Bus_Max = 16V
 * 			V_Shunt_Max = 160mV
 * 			R_Shunt = 0.1 Ohm
 *
 * 	Step 2: Max Possible I = 1.6A
 *
 * 	Step 3: Let I Max Expected = 1.2A
 *
 * 	Step 4: Min LSB = 36.6 uA/LSB
 * 			Max LSB = 292.97 uA
 *
 * 			Choose LSB = 100 uA
 * 	Step 5: Set Calibration value = 4096
 */

INA_Status_t INA219_Calibrate_16V_1_2A(float *I_MBO, float *V_MBO, float *P_Max)
{
	//set Current Step Size
	ina.INA219_I_LSB = 100.0/1000000.0;
	uint16_t I_cal_val = (uint16_t)(0.04096/(ina.INA219_I_LSB*INA219_R_SHUNT));
	ina.INA219_P_LSB = 20*ina.INA219_I_LSB;
	float I_max = ina.INA219_I_LSB*32767;
	if(I_max  > 1.6) //max possible current
	{
		*I_MBO = 1.6;
	}else
	{
		*I_MBO = I_max;
	}
	float Vshunt_max = *I_MBO*INA219_R_SHUNT;
	if(Vshunt_max > 0.16)
	{
		*V_MBO = 0.16;
	}
	else
	{
		*V_MBO = Vshunt_max;
	}
	*P_Max = *I_MBO*16;

	//write I_Cal_val to register
	uint8_t temp[2] = {(I_cal_val&0xFF00)>>8,(I_cal_val&0x00FF)};
	if(HAL_I2C_Mem_Write(&ina.ina_i2c,INA219_I2C_Address,CALIBRATION_REG,1,temp,2,100) != HAL_OK)
	{
		return INA_I2C_WRITE_ERROR;
	}

	return INA_OK;
}

//================= 8. Register Function Definitions ======================================

INA_Status_t INA219_Get_Reg_Config(INA219_Handle_Typedef* hina)
{
	uint8_t temp [2] = {0};
	if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,CONFIG_REG,1,temp,2,100) != HAL_OK)
	{
		return INA_I2C_READ_ERROR;
	}
	uint16_t config_reg = temp[0]<<8 |temp[1];
	hina->Init.INA_BUS_VOLTAGE_RANGE = config_reg & (0b1<<13);
	hina->Init.INA_SHUNT_PGA_RANGE = config_reg & (0b11<<11);
	hina->Init.INA_BUS_ADC_RESOLUTION = config_reg & (0b1111<<7);
	hina->Init.INA_SHUNT_RESOLUTION = config_reg & (0b1111<<3);
	return HAL_OK;
}

INA_Status_t INA219_Reset(void)
{
	//write reset bit to register
	uint8_t temp[2] = {INA219_CONFIG_RESET>>8,0};
	if(HAL_I2C_Mem_Write(&ina.ina_i2c,INA219_I2C_Address,CONFIG_REG,1,temp,2,100) != HAL_OK)
	{
		return INA_I2C_WRITE_ERROR;
	}
	//wait for bit to clear
	while((temp[0]&(INA219_CONFIG_RESET>>8)) != 0 )
	{
		if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,CONFIG_REG,1,temp,2,100) != HAL_OK)
		{
				return INA_I2C_READ_ERROR;
		}
	}

	//check if control reg is default values
	uint16_t reg = temp[0]<<8 | temp[1];
	if(reg != INA219_DEFAULT_CONFIG)
	{
		return INA_RESET_ERROR;
	}
	return HAL_OK;
}

INA_Status_t INA219_Set_Power_Mode(uint16_t PWR_MODE)
{
	//get power mode
	uint8_t temp [2] = {0};
	if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,CONFIG_REG,1,temp,2,100) != HAL_OK)
	{
		return INA_I2C_READ_ERROR;
	}
	uint16_t config_reg = temp[0]<<8 |temp[1];

	//clear power mode
	config_reg &= ~(0b11);
	//set to new value
	config_reg |= PWR_MODE;
	//write to reg
	temp[0] = (config_reg &0xFF00)>>8;
	temp[1] = config_reg &0xFF;
	if(HAL_I2C_Mem_Write(&ina.ina_i2c,INA219_I2C_Address,CONFIG_REG,1,temp,2,100) != HAL_OK)
	{
		return INA_I2C_WRITE_ERROR;
	}
	return INA_OK;
}

INA_Status_t INA219_Set_Reg_Config(INA219_Handle_Typedef *hina)
{
	hina->Init.INA_BUS_VOLTAGE_RANGE = INA219_CONFIG_BRNG_16V;
	hina->Init.INA_SHUNT_PGA_RANGE = INA219_CONFIG_PG_4;
	hina->Init.INA_BUS_ADC_RESOLUTION = INA219_CONFIG_BADC_MODE_SAMPLE_12_BIT;
	hina->Init.INA_SHUNT_RESOLUTION = INA219_CONFIG_SADC_MODE_SAMPLE_12_BIT;
	hina->Config_val = (hina->Init.INA_BUS_VOLTAGE_RANGE | hina->Init.INA_SHUNT_PGA_RANGE | hina->Init.INA_BUS_ADC_RESOLUTION | hina->Init.INA_SHUNT_RESOLUTION);
	uint8_t byte[2] = {(hina->Config_val&0xFF00)>>8,hina->Config_val&0x00FF};
	//write value to register
	if(HAL_I2C_Mem_Write(&ina.ina_i2c,INA219_I2C_Address,CONFIG_REG,1,byte,2,100) != HAL_OK)
	{
		return INA_I2C_WRITE_ERROR;
	}
	return INA_OK;
}

//================ 9. Measurement Function Prototypes ====================================

/*
 * @brief: Triggers a single shot ADC conversion of voltage values. The conversions
 * 		   performed depend on the ones specified by the user
 *
 * @param: val - a number between 0 and 2 which enables conversions for the device.
 * 				 0 - Shunt Voltage conversion only
 * 				 1 - Bus Voltage conversion only
 * 				 2 - both shunt and bus conversion
 *
 * @return: INA_Status_t - return value showing the status of the function
 */

INA_Status_t INA219_Trigger_Conversion(uint8_t val)
{
	//set Power mode
	uint16_t mode = 0;
	switch(val){
	case 0:
		mode = INA219_CONFIG_MODE_SHUNT_TRIGGERERD;
		break;
	case 1:
		mode = INA219_CONFIG_MODE_BUS_TRIGGERED;
		break;
	case 2:
		mode = INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED;
		break;
	}
	if(INA219_Set_Power_Mode(mode) != INA_OK)
	{
		return INA_I2C_WRITE_ERROR;
	}
	//wait for conversion ready flag to be set in register
	uint8_t temp[2];
	uint16_t u_temp = 0;
	while(((u_temp&INA219_FLAG_CNVR)>>1) != SET)
	{
		if(HAL_I2C_Mem_Read(&ina.ina_i2c,INA219_I2C_Address,V_BUS_REG,1,temp,2,100)!= HAL_OK)
		{
			return INA_I2C_READ_ERROR;
		}
		u_temp = (temp[0]<<8) | temp[1];
	}
	return INA_OK;
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
	//mulitply by PG gain
	switch(ina.Init.INA_SHUNT_PGA_RANGE)
	{
		case INA219_CONFIG_PG_8:
			*Shunt_Voltage = *Shunt_Voltage*1;
			break;
		case INA219_CONFIG_PG_4:
			*Shunt_Voltage = *Shunt_Voltage*4;
			break;
		case INA219_CONFIG_PG_2:
			*Shunt_Voltage = *Shunt_Voltage*8;
		case INA219_CONFIG_PG_1:
			*Shunt_Voltage = *Shunt_Voltage*16;
			break;
	}
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


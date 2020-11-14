/*
 * HAL_MPU6050.c
 *
 *  Created on: Nov 14, 2020
 *      Author: jamie
 */

#include "HAL_MPU6050.h"


uint8_t I2C_TX_CPLT;

//------------------ Register Configuration functions -------------------------//

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

mpu_status_t MPU6050_Get_SelfTestResponse_Values(I2C_HandleTypeDef *hi2c,MPU_SelfTest_t *mpu)
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
	  uint8_t byte =0;
	  byte |= FSR;
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,GYRO_CONFIG,1,&byte,1,100)!= HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	 return MPU_OK;
}

mpu_status_t MPU6050_Set_Acc_FSR(I2C_HandleTypeDef *hi2c,uint8_t FSR)
{
	uint8_t byte = 0;
	byte = FSR ;
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

mpu_status_t MPU6050_Configure_Interrupt_Pin( uint8_t level,uint8_t open,  uint8_t latch)
{
	//get configbyte
	uint8_t byte = 0;
	if(HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,INT_PIN_CFG,1,&byte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	//clear config params
	byte &= 0x1F;
	byte |= (open | level | latch);

	if(HAL_I2C_Mem_Write(&hi2c1,MPU_Device_Address,INT_PIN_CFG,1,&byte,1,100)!= HAL_OK)
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

mpu_status_t MPU6050_Disable_Interrupt(I2C_HandleTypeDef *hi2c, uint8_t interrupts)
{
	uint8_t iconfigbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,INT_ENABLE,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	//set enable bit
	iconfigbyte &= ~interrupts;
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

mpu_status_t MPU6050_Get_IMU_RawData(I2C_HandleTypeDef *hi2c,uint8_t* imu)
{
	  __HAL_DMA_ENABLE_IT(&hdma_i2c1_rx, DMA_IT_TC);
	if( HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,ACCEL_XOUT_H,1,imu,12,100) != HAL_OK)
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
	}else if(enable == MPU_RESET)
	{
		uc_byte &= ~USER_CTRL_FIFO_EN;
		uc_byte |= USER_CTRL_FIFO_RESET;
	}
	//write values to data register
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,USER_CTRL,1,&uc_byte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	if(enable == MPU_RESET)
	{
		//poll User control FIFO reset bit untill it resets
		while((uc_byte& USER_CTRL_FIFO_RESET) != 0 )
		{
			if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,USER_CTRL,1,&uc_byte,1,100) != HAL_OK)
			{
				return MPU_RESET_FAIL;
			}
		}
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

mpu_status_t  MPU6050_FIFO_Config(I2C_HandleTypeDef *hi2c, uint8_t fifo_mask, uint8_t cmd )
{
	uint8_t fifobyte;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,FIFO_EN,1,&fifobyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	if(cmd == ENABLE)
	{
		fifobyte |= fifo_mask;
	}
	else if(cmd == DISABLE)
	{
		fifobyte &= ~fifo_mask;
	}
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,FIFO_EN,1,&fifobyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;
}

mpu_status_t MPU6050_Get_FIFO_Count(I2C_HandleTypeDef *hi2c,uint16_t* count)
{
	uint8_t fifo_reg[2] = {0};
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,FIFO_COUNTH,1,fifo_reg,2,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}

	*count = ((fifo_reg[0]&0xFF)<<8) | (fifo_reg[1] &0xFF);
	return MPU_OK;

}

mpu_status_t MPU6050_FIFO_CMD(I2C_HandleTypeDef *hi2c,uint8_t cmd)
{
	uint8_t fifobyte;
		if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,USER_CTRL,1,&fifobyte,1,100) != HAL_OK)
		{
			return MPU_I2C_ERROR;
		}

		if(cmd == ENABLE)
		{
			fifobyte |= USER_CTRL_FIFO_EN;
		}else if(cmd == DISABLE)
		{
			fifobyte &= ~(USER_CTRL_FIFO_EN);
		}else
		{
			//invalid cmd sent
			return MPU_INIT_CMD_ERROR;
		}
		//write byte to user control
		if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,USER_CTRL,1,&fifobyte,1,100) != HAL_OK)
		{
			return MPU_I2C_ERROR;
		}
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

mpu_status_t MPU6050_Set_PLLSrc(I2C_HandleTypeDef *hi2c, uint8_t PLL)
{
	//get PWR_MGMT_1 reg data
	uint8_t pwrmgmtbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	//configure PLL source
	pwrmgmtbyte = (pwrmgmtbyte &0b11111000)| PLL;

	//write byte to register
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	return MPU_OK;
}

mpu_status_t MPU6050_reset(I2C_HandleTypeDef *hi2c)
{
	//set bit in register
	uint8_t pwrmgmtbyte = (uint8_t)PWR_MGMT_1_DEVICE_RESET;
	if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	//wait for bit to clear
	uint8_t retries = 0;
	while((pwrmgmtbyte &PWR_MGMT_1_DEVICE_RESET)!=0)
	{
		//poll data register untill bit is cleared
		if(HAL_I2C_Mem_Read(&hi2c1,MPU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100)!= HAL_OK)
		{
			//failiure to read register
				retries++;
		}
		if(retries >= 100)
		{
			return MPU_RESET_FAIL;
		}
	}

	return HAL_OK;
}
//----------------------- Data Processing Functions----------------------------//

/*
 * @brief:
 *
 * This function is used to perform a self test on the device. This function
 * should be run during a power on/ first time start.
 *
 * @description:
 * Self Test is applied to all 6 axes of imu. Individual axis can be enabled for self test
 * by setting the nA_ST bit in the Acceleration Config Register (n is the x,y,z axis)
 *
 * Once, activated, the sensors are electronically actuated over a set distance simulating an
 * external force. Once complete, a corresponding output signal is produced which is then
 * subtracted from the values in the self test register resulting in a self test response.
 *
 *@set up:
 *@set GYRO FSR to 250 dps
 *@set ACCEL range to 8+-g
 *@set PLL clock source to x-axis gyro ref
 *
 *@param: hi2c - i2c handler
 *@param: test_res - an array to hold the results of the self test for each axis. Note: must be an array with a length >= 6
 *
 *@return mpu_status_t - value to show the status of the function
 *
 */
mpu_status_t MPU6050_SelfTest(I2C_HandleTypeDef *hi2c,float* test_res)
{
	// set clock source to PLL gyro x axis
	MPU6050_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_1);
	//set Gyro FSR to 250 dps
	MPU6050_Set_Gyro_FSR(&hi2c1,GYRO_CONFIG_FSSEL_250DPS);
	//set Acc FSR to +-8g
	MPU6050_Set_Acc_FSR(&hi2c1,ACC_CONFIG_AFSSEL_8G);
	//enable selftest
	uint8_t stbyte[2];
	 if(HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,GYRO_CONFIG,1,stbyte,2,100)!= HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	stbyte[0] |= GYRO_CONFIG_ST_EN_X | GYRO_CONFIG_ST_EN_Y | GYRO_CONFIG_ST_EN_Z;
	stbyte[1] |= ACC_CONFIG_ST_EN_X | ACC_CONFIG_ST_EN_Y | ACC_CONFIG_ST_EN_Z;
	 if(HAL_I2C_Mem_Write(hi2c,MPU_Device_Address,GYRO_CONFIG,1,stbyte,2,100)!= HAL_OK)
	{
		return MPU_I2C_ERROR;
	}
	 HAL_I2C_Mem_Read(hi2c,MPU_Device_Address,GYRO_CONFIG,1,stbyte,2,100);
	HAL_Delay(250); //TODO: Replace with a wait till self test updated function. Value taken from https://github.com/kriswiner/MPU6050/blob/master/MPU6050BasicExample.ino line 662

	//read Self Test register
	MPU_SelfTest_t mpu_str;
	MPU_FT_t mpu_ft;
	if(MPU6050_Get_SelfTestResponse_Values(&hi2c1,&mpu_str) != MPU_OK)
	{
		return MPU_STR_READ_ERROR;
	}
	//calculate Factory Trim for gyro
	mpu_ft.G_x = 25*131*pow(1.046,(float)mpu_str.G_x -1);
	mpu_ft.G_y =-25*131*pow(1.046,(float)mpu_str.G_y -1);
	mpu_ft.G_z = 25*131*pow(1.046,(float)mpu_str.G_z -1);

	//calculate Factory Trim for Acc
	mpu_ft.A_x = 4096*0.34*pow(0.92/0.34,(((float)mpu_str.A_x-1)/30.0));
	mpu_ft.A_y = 4096*0.34*pow(0.92/0.34,(((float)mpu_str.A_y-1)/30.0));
	mpu_ft.A_z = 4096*0.34*pow(0.92/0.34,(((float)mpu_str.A_z-1)/30.0));

	//calculate factory trim chage
	test_res[0] = 100+ 100*((float)mpu_str.A_x - mpu_ft.A_x)/mpu_ft.A_x;
	test_res[1] = 100+ 100*((float)mpu_str.A_y - mpu_ft.A_y)/mpu_ft.A_y;
	test_res[2] = 100+ 100*((float)mpu_str.A_z - mpu_ft.A_z)/mpu_ft.A_z;
	test_res[3] = 100+ 100*((float)mpu_str.G_x - mpu_ft.G_x)/mpu_ft.G_x;
	test_res[4] = 100+ 100*((float)mpu_str.G_y - mpu_ft.G_y)/mpu_ft.G_y;
	test_res[5] = 100+ 100*((float)mpu_str.G_z - mpu_ft.G_z)/mpu_ft.G_z;
	return MPU_OK;


}

mpu_status_t MPU6050_Calibrate_Acc(I2C_HandleTypeDef *hi2c,float* accel_bias)
{
	//Initialise registers
	MPU6050_reset(&hi2c1);
	MPU6050_Set_Wake(&hi2c1); //wake up device
	MPU6050_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_1); // set clock source to PLL gyro x axis
	HAL_Delay(400);
	MPU6050_Disable_Interrupt(&hi2c1,(INT_ENABLE_DATA_RDY_EN|INT_ENABLE_FIFO_OFLOW_EN | INT_ENABLE_I2C_MST_INT_EN)); //Disable all interrupts
	MPU6050_FIFO_Config(&hi2c1,0xFF,DISABLE); 	//reset FIFO
	MPU6050_FIFO_Init(&hi2c1,DISABLE);
	MPU6050_FIFO_Init(&hi2c1,MPU_RESET);

	//configure Gyro and Accelerometer
	MPU6050_Set_Gyro_FSR(&hi2c1,GYRO_CONFIG_FSSEL_500DPS); //set Gyro FSR to 500 dps
	MPU6050_Set_Acc_FSR(&hi2c1,ACC_CONFIG_AFSSEL_8G);		//set Acc FSR to +-8g
	int32_t acc_res =ACC_8G_WORD_LENGTH;
	MPU6050_Set_DLPF(&hi2c1,CONFIG_DLFP_1);					//set dlpf to 188 Hz
	MPU6050_Set_Sample_Rate(&hi2c1);						// set sample rate to user defined value in HAL_MPU6050.h
	MPU6050_FIFO_Init(&hi2c1,ENABLE);						//enable FIFO
	//enable acc and gyro in FIFO buffer
	MPU6050_Enable_Interrupt(&hi2c1, INT_ENABLE_FIFO_OFLOW_EN);
	MPU6050_FIFO_Config(&hi2c1,FIFO_EN_ACC,ENABLE);
	//wait until fifo is completed
	uint8_t interrupt_status = 0;
	while(!interrupt_status)
	{
		 MPU6050_Get_Interrupt_Status(&hi2c1,FIFO_OVERFLOW,&interrupt_status);
	}
	MPU6050_FIFO_Config(&hi2c1,FIFO_EN_ACC,DISABLE);
	uint16_t count = 0;
	MPU6050_Get_FIFO_Count(&hi2c1,&count);
	uint8_t buffer[1024];
	uint8_t addr = FIFO_R_W;

	if(HAL_I2C_Master_Transmit(&hi2c1,MPU_Device_Address,&addr,1,100)== HAL_OK)
	{
		if(HAL_I2C_Master_Seq_Receive_DMA(&hi2c1,MPU_Device_Address,buffer,count,I2C_FIRST_FRAME) != HAL_OK)
		{
			return MPU_FIFO_READ_ERROR;
		}
		//wait for read to finish
		while(I2C_TX_CPLT != 1);

	}
	//Processing algorythm:
	int32_t n_s = count/6; //3 axes, 2 bytes per axis
	int32_t imu_bias[3] = {0};	//data array for storing the
	int16_t acc_temp[3] ={0};// gyro_temp[3] = {0};
	for (int i = 0; i < n_s; ++i)
	{
		//get set of imu data
		acc_temp[0]= ((int16_t)buffer[6*i])<<8 | ((int16_t)buffer[6*i+1]&0xFF);
		acc_temp[1]= ((int16_t)buffer[6*i+2])<<8 | ((int16_t)buffer[6*i+3]&0xFF);
		acc_temp[2]= ((int16_t)buffer[6*4])<<8 | ((int16_t)buffer[6*i+5]&0xFF);
		int j = 0;
		for (j = 0; j < 3; ++j)
		{
			imu_bias[j] += (int32_t)acc_temp[j];
		}
	}
	//divide total by number of samples to get offset
	imu_bias[0] /= n_s;
	imu_bias[1] /=n_s;
	imu_bias[2] = (imu_bias[2] - acc_res)/n_s;
	accel_bias[0] = (float)imu_bias[0]/(float)acc_res;
	accel_bias[1] = (float)imu_bias[1]/(float)acc_res;
	accel_bias[2] = (float)imu_bias[2]/(float)acc_res;
	return MPU_CAL_SUCCESS;
}




//----------------------------- Peripheral Handlers ----------------------------------//

//Initialisation Functions
void MPU6050_DMA_PeriphIRQHandler(void)
{
	I2C_TX_CPLT = 1;
}

/*
 * Function Name mpu_status_t MPU6050_Init(uint8_t g_fsr,uint8_t a_fsr, uint8_t dlpf_coeff)
 *
 * @brief: Initialise I2C and DMA Microcontroller Peripherals. Wake Up sensor and configure
 *
 */
mpu_status_t MPU6050_Init(uint8_t g_fsr,uint8_t a_fsr, uint8_t dlpf_coeff)
{

	//step 2: Request acknowledgement
	//initialise buffers
	sample_count = 0;
	memset(IMU_Buffer,0,IMU_BUFFER_SIZE);
	uint8_t whoami = 0;
	if(MPU6050_Get_ID(&hi2c1,&whoami) != MPU_OK)
	{
		return MPU_I2C_DEVICE_OFFLINE;
	}

	if(whoami != WHO_AM_I_VALUE)
	{
		return MPU_I2C_ID_ERROR;
	}
	//Initialise registers
	uint8_t byte[2];
	MPU6050_reset(&hi2c1);
	HAL_Delay(100);
	MPU6050_Set_Wake(&hi2c1); //wake up device
	MPU6050_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_3); // set clock source to PLL gyro x axis
	//Configure sensor settings
	MPU6050_Set_Acc_FSR(&hi2c1,a_fsr);
	MPU6050_Set_Gyro_FSR(&hi2c1,g_fsr);
	MPU6050_Set_DLPF(&hi2c1,dlpf_coeff);
	MPU6050_Set_Sample_Rate(&hi2c1);
	//configure interrupt pin
	MPU6050_Configure_Interrupt_Pin(INT_PIN_CFG_LEVEL_HIGH, INT_PIN_CFG_PIN_PUSH_PULL,INT_PIN_CFG_LATCH_INT_EN);
	MPU6050_Enable_Interrupt(&hi2c1,INT_ENABLE_DATA_RDY_EN);
	MPU6050_Get_Interrupt_Status(&hi2c1,DATA_READY,(uint8_t*)0xFF);


	return MPU_OK;
}

mpu_status_t MPU6050_Deinit(void)
{
	 MPU6050_Disable_Interrupt(&hi2c1,DATA_READY);
	 MPU6050_Get_Interrupt_Status(&hi2c1,DATA_READY,(uint8_t*)0xFF);
	//reset Device and place in sleep mode
	MPU6050_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_0);
	MPU6050_reset(&hi2c1);
	HAL_Delay(100);
	//Deinit I2C peripheral
	if(HAL_I2C_DeInit(&hi2c1) != HAL_OK)
	{
	  return MPU_CONFIG_ERROR;
	}
	if(HAL_DMA_DeInit(&hdma_i2c1_rx) != HAL_OK)
	{
		MPU_CONFIG_ERROR;
	}
	sample_count = 0;
	memset(IMU_Buffer,0,IMU_BUFFER_SIZE);
	return MPU_OK;
}

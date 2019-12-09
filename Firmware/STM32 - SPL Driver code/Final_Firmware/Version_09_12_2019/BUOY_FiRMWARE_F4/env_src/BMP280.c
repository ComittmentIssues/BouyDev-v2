/*
 * BMP280.c
 *
 *  Created on: Oct 10, 2019
 *      Author: jamie
 */

#include "BMP280.h"
#include "math.h"
#include "SPI.h"
/* Private variables ---------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

uint8_t BMP280_GetID(void)
{		int8_t data;
		BMP280_Read_Register(id,&data,1,SPI3);
		return data;
}

BMPStatus_t  BMP280_Begin(void)
{
	init_SPI();
	if (BMP280_GetID()== 0x58)
	{
		DelayInit();
		BMP280_Write_Register(Sreset,BMP280_Soft_Reset,SPI3);
		//TODO: 4ms delay
		DelayMs(4);
		return BMP_OK;
	}

	return BMP_ERROR;
}

BMPStatus_t BMP280_Read_Register(uint8_t reg,int8_t* regdata,uint8_t len, SPI_TypeDef* SPIx)
{
	//signal device read by setting bit 7
	uint8_t RR = reg | 0x80;
	//SPI Transfer
	SS_ENABLE;
	SPI_Transfer(RR,SPIx);
	for (int i = 0; i < len; ++i)
	{
		regdata[i] = SPI_Transfer(0xFF,SPIx);
	}
	SS_DISABLE;
	return BMP_OK;
}
BMPStatus_t BMP280_Configure_CTRLMEAS(uint8_t osrs_t,uint8_t osrs_p,uint8_t mode)
{
	//combine settings into byte for register
	int8_t mcbyte,flag;
	BMP280_Read_Register(ctrl_meas,&mcbyte,1,SPI3);
	mcbyte |= (osrs_p | osrs_t |mode);
	BMP280_Write_Register(ctrl_meas,mcbyte,SPI3);
	//verify register settings successful
	BMP280_Read_Register(ctrl_meas,&flag,1,SPI3);
	if(flag != mcbyte)
	{
		return BMP_WRITE_ERROR;
	}
	return BMP_OK;
}

BMPStatus_t BMP280_Configure_Config(uint8_t tsb, uint8_t filter, uint8_t spi1en)
{
	int8_t flag,config_byte = 0;
	config_byte |= tsb | filter |spi1en;
	BMP280_Write_Register(config,config_byte,SPI3);
	BMP280_Read_Register(config,&flag,1,SPI3);
	if(flag != config_byte)
		{
			return BMP_WRITE_ERROR;
		}
	return BMP_OK;
}

BMPStatus_t BMP280_GetCoeff(BMP280_trim_t* bmpt)
{
	//start at base address

	int8_t trim[24]= {0};
	BMP280_Read_Register(calib_0,trim,BMP280_CALIB_DATA_LEN,SPI3);
	/* Temperature trim parameter read out*/
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
BMPStatus_t BMP280_Write_Register(uint8_t reg,uint8_t regdata,SPI_TypeDef* SPIx)
{
	//signal device write by setting bit 7 high
	uint8_t RR = reg & 0x7F;
	SS_ENABLE;
	SPI_Transfer(RR,SPIx);
	SPI_Transfer(regdata&0xFF,SPIx);
	SS_DISABLE;
	return BMP_OK;
}

BMPStatus_t BMP280_GetConversionStatus(void)
{
	int8_t flag;
	BMP280_Read_Register(status,&flag,1,SPI3);
	uint8_t status = (flag&0b100)>>2;
	if(status == 0)
	{
		return BMP_Conversion_Complete;
	}
	if(status == 1)
	{
		return BMP_Converting;
	}
	return BMP_ERROR;
}
/*
 * @brief Returns the compensated Temperature in degrees celsius to
 * @param None
 * @retval double temp
 */
int32_t BMP280_GetTemp(void)
{
	//perform a test read
	int8_t Ttemp[3];
	BMP280_Read_Register(temp_msb,Ttemp,3,SPI3);
	int32_t T_val = ((Ttemp[0]&0xFF)<<16)|((Ttemp[1]&0xFF)<<8)|((Ttemp[2]&0xF0));
	T_val= T_val>> 4;
	//compensate Temperature from datasheet
	int32_t var1 = (((T_val>>3)- ((int32_t)bmp.dig_T1<<1))*((int32_t)bmp.dig_T2))>>11;
    int32_t var2 =  (((((T_val>>4) - ((int32_t)bmp.dig_T1)) * ((T_val>>4) - ((int32_t)bmp.dig_T1))) >> 12)*((int32_t)bmp.dig_T3)) >> 14;
	t_fine = var1+var2; //for storage in global variable
	return (t_fine)/5120;

}

/*
 * @brief Returns the compensated Pressure in Pascals. Sensor stores data in 3 bytes little endian
 * 		  To calculate pressure, the bytes need to be combined into a 20 bit word then compensated
 * 		  using the formula in the data sheet.
 * 		  Note: xlsb bit is 4 bits long not 8
 * @param None
 * @retval float pressure
 */
uint32_t BMP280_GetPressure(void)
{
	//Calculate t_fine without returning temp val
	int8_t press[3];
	BMP280_Read_Register(press_msb,press,3,SPI3);
	//combine bytes into 20 bit word
	int32_t P_val = ((press[0]&0xFF)<<16|(press[1]&0xFF)<<8 |(press[2]&0xFF))>>4;
	//Compensation formula
	int64_t var1 = (int64_t)t_fine - 128000;
	int64_t var2 = var1*var1*((int64_t)bmp.dig_P6);
	var2 = var2 + (((int64_t)bmp.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)bmp.dig_P3)>>8) + ((var1 * (int64_t)bmp.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmp.dig_P1)>>33;
	//check for divide by 0 error
	if(var1 == 0)return 0;

	int64_t P = 1048576 - P_val;
	P = (((P<<31)-var2)*3125)/var1;
	var1 = (((int64_t)bmp.dig_P9) * (P>>13) * (P>>13)) >> 25;
	var2 = (((int64_t)bmp.dig_P8) * P) >> 19;
	P = ((P + var1 + var2) >> 8) + (((int64_t)bmp.dig_P7)<<4);
	return (uint32_t)P;
}
// SysTick_Handler function will be called every 1 us
void SysTick_Handler()
{
    if (usTicks != 0)
    {
        usTicks--;
    }
}

BMPStatus_t BMP280_Force_Measure(int32_t* temp,uint32_t* pressure)
{
	int8_t mcbyte,flag;
	BMP280_Read_Register(ctrl_meas,&mcbyte,1,SPI3);
	//check if in sleep
	if((mcbyte&0b11) != BMP280_CTRLMEAS_MODE_SLEEP)
	{
		mcbyte &= 0b11111100; //clear mode
	}
	mcbyte |= BMP280_CTRLMEAS_MODE_FORCED;
	BMP280_Write_Register(ctrl_meas,mcbyte,SPI3);
	//verify register settings successful
	BMP280_Read_Register(ctrl_meas,&flag,1,SPI3);
	if(flag != mcbyte)
	{
		return BMP_WRITE_ERROR;
	}
	//wait for mode to return to sleep

	while((flag&0b11) != BMP280_CTRLMEAS_MODE_SLEEP)
	{
		BMP280_Read_Register(ctrl_meas,&flag,1,SPI3);
	}
	//read temp
	*temp = BMP280_GetTemp();
	*pressure = BMP280_GetPressure();
	return BMP_OK;
}
void DelayInit()
{
    // Update SystemCoreClock value
    SystemCoreClockUpdate();
    // Configure the SysTick timer to overflow every 1 us
    SysTick_Config(SystemCoreClock / 1000000);
}

void DelayUs(uint32_t us)
{
    // Reload us value
    usTicks = us;
    // Wait until usTick reach zero
    while (usTicks);
}

void DelayMs(uint32_t ms)
{
    // Wait until ms reach zero
    while (ms--)
    {
        // Delay 1ms
        DelayUs(1000);
    }
}

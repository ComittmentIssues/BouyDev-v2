/*
 * BMP280.h
 *
 *  Created on: Sep 11, 2019
 *      Author: jamie
 */

#ifndef BMP280_H_
#define BMP280_H_

#include "stm32f4xx.h"

typedef struct{
	uint16_t 	dig_T1;
	int16_t 	dig_T2;
	int16_t 	dig_T3;
	uint16_t 	dig_P1;
	int16_t 	dig_P2;
	int16_t 	dig_P3;
	int16_t 	dig_P4;
	int16_t 	dig_P5;
	int16_t 	dig_P6;
	int16_t 	dig_P7;
	int16_t 	dig_P8;
	int16_t 	dig_P9;

}BMP280_trim_t;
typedef enum
{
	temp_xlsb = 0xFC,
	temp_lsb = 0xFB,
	temp_msb = 0xFA,
	press_xlsb = 0xF9,
	press_lsb =0xF8,
	press_msb = 0xF7,
	config = 0xF5,
	ctrl_meas = 0xF4,
	status = 0xF3,
	Sreset = 0xE0,
	id = 0xD0,
	calib_0 = 0x88

} BMPRegisterMap_t;

typedef enum
{
	BMP_TIMEOUT,
	BMP_OK,
	BMP_ERROR,
	BMP_WRITE_ERROR,
	BMP_Converting,
	BMP_Conversion_Complete,

}BMPStatus_t;

/* Private define ------------------------------------------------------------*/
#define BMP280_I2C_ADDRESS1 0x76
#define BMP280_I2C_ADDRESS2 0x77

#define BMP280_CTRLMEAS_OSRSP_SKIP  0 << 5
#define BMP280_CTRLMEAS_OSRSP_OS_1  1 << 5
#define BMP280_CTRLMEAS_OSRSP_OS_2  2 << 5
#define BMP280_CTRLMEAS_OSRSP_OS_4  3 << 5
#define BMP280_CTRLMEAS_OSRSP_OS_8  4 << 5
#define BMP280_CTRLMEAS_OSRSP_OS_16 5 << 5

#define BMP280_CTRLMEAS_OSRST_SKIP  0 << 2
#define BMP280_CTRLMEAS_OSRST_OS_1  1 << 2
#define BMP280_CTRLMEAS_OSRST_OS_2  2 << 2
#define BMP280_CTRLMEAS_OSRST_OS_4  3 << 2
#define BMP280_CTRLMEAS_OSRST_OS_8  4 << 2
#define BMP280_CTRLMEAS_OSRST_OS_16 5 << 2

#define BMP280_CTRLMEAS_MODE_SLEEP  0
#define BMP280_CTRLMEAS_MODE_FORCED 1
#define BMP280_CTRLMEAS_MODE_NORMAL 2

#define BMP280_CONFIG_tsb_0_5  0 << 5
#define BMP280_CONFIG_tsb_62_5 1 << 5
#define BMP280_CONFIG_tsb_125  2 << 5
#define BMP280_CONFIG_tsb_250  3 << 5
#define BMP280_CONFIG_tsb_500  4 << 5
#define BMP280_CONFIG_tsb_1000 5 << 5
#define BMP280_CONFIG_tsb_2000 6 << 5
#define BMP280_CONFIG_tsb_4000 7 << 5

#define BMP280_CONFIG_FILTER_COEFF_OFF 0 << 2
#define BMP280_CONFIG_FILTER_COEFF_2   1 << 2
#define BMP280_CONFIG_FILTER_COEFF_4   2 << 2
#define BMP280_CONFIG_FILTER_COEFF_8   3 << 2
#define BMP280_CONFIG_FILTER_COEFF_16  4 << 2

#define BMP280_CONFIG_SPI3_EN  1
#define BMP280_CONFIG_SPI3_DIS 0


#define BMP280_CALIB_DATA_LEN 24
#define BMP280_Soft_Reset 0xB6

BMP280_trim_t bmp;
uint32_t usTicks;
double t_fine;

/* Private function prototypes -----------------------------------------------*/
void init_LED(void);
void DelayInit(void);
void DelayMs(uint32_t ms);
void Delayus(uint32_t us);
uint8_t BMP280_GetID(void);
BMPStatus_t  BMP280_Begin(void);
BMPStatus_t BMP280_Write_Register(uint8_t reg,uint8_t regdata,SPI_TypeDef* SPIx);
BMPStatus_t BMP280_Read_Register(uint8_t reg,int8_t* regdata,uint8_t len, SPI_TypeDef* SPIx);
BMPStatus_t BMP280_Configure_CTRLMEAS(uint8_t osrs_t,uint8_t osrs_p,uint8_t mode);
BMPStatus_t BMP280_Configure_Config(uint8_t tsb, uint8_t filter, uint8_t spi3en);
BMPStatus_t BMP280_GetConversionStatus(void);
BMPStatus_t BMP280_GetCoeff(BMP280_trim_t* bmpt);
BMPStatus_t BMP280_Force_Measure(double* temp,double* pressure);
double BMP280_GetTemp(void);
float BMP280_GetPressure(void);

#endif /* BMP280_H_ */

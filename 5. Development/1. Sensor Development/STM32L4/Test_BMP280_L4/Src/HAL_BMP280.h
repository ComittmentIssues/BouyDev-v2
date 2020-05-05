/*
 * HAL_BMP280.h
 *
 *  Created on: Apr 25, 2020
 *      Author: jamie
 */

#ifndef HAL_BMP280_H_
#define HAL_BMP280_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
#include "stm32l4xx_hal_gpio.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
	BMP_SPI_WRITE_ERROR,
	BMP_SPI_READ_ERROR,
	BMP_RESET_ERROR,
	BMP_DEVICE_CHECK_ERROR,
	BMP_Config_Error,
	BMP_TEMP_READ_ERROR,
	BMP_PRESS_READ_ERROR,
	BMP_POWER_CONFIG_ERROR,
	BMP_TIMEOUT,
	BMP_OK,
	BMP_ERROR,
	BMP_WRITE_ERROR,
	BMP_Converting,
	BMP_Conversion_Complete,

}BMPStatus_t;

typedef enum
{
	HandHeld_LP,
	HandHeld_Dynamic,
	Weather_Monitoring
}BMP_Opp_Mode_t;

typedef enum
{
	BMP_Measurement_Busy = 0b1000,
	BMP_Measurement_Complete = 0

}BMP_Measurement_Status_t;
typedef enum
{
	BMP_Data_Shadowing_Busy = 0b1,
	BMP_Data_Shadowing_Complete = 0

}BMP_IM_Status_t;
/*
 * A struct representing the sensor on the microcontroller side.
 * This will contain the settings for configuring the sensor
 */

typedef struct
{
	uint8_t BMP_Power_Mode;
	uint8_t BMP_Pressure_OverSample;
	uint8_t BMP_Temperature_OverSample;
	uint8_t BMP_t_Standby;
	uint8_t BMP_IIR_FILTER_COEFFICIENTS;
} BMP_Init_Typedef;

typedef struct
{
	BMP_Init_Typedef Init;
	BMP_IM_Status_t IM_Status;
	BMP_Measurement_Status_t M_Status;
	BMP280_trim_t Factory_Trim;
	SPI_HandleTypeDef *bmp_spi;
}BMP_Handle_Typedef;

/* USER CODE END PTD */


/* Private defines -----------------------------------------------------------*/
#define SPI_CS_Pin GPIO_PIN_10
#define SPI_CS_GPIO_Port GPIOA

/*
 * USE_SPI_x maps the spi handler to the correct SPI POrt. On the stm32l476RG, there are 3 spi peripherals.
 * Note: set x to a value from 1 - 3 otherwise the code will not work
 *
 */
#define USE_SPI_1
#define BMP280_I2C_ADDRESS1 0x76
#define BMP280_I2C_ADDRESS2 0x77

#define BMP280_CTRLMEAS_OSRSP_SKIP  0 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_1  1 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_2  2 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_4  3 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_8  4 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_16 5 << 2

#define BMP280_CTRLMEAS_OSRST_SKIP  0 << 5
#define BMP280_CTRLMEAS_OSRST_OS_1  1 << 5
#define BMP280_CTRLMEAS_OSRST_OS_2  2 << 5
#define BMP280_CTRLMEAS_OSRST_OS_4  3 << 5
#define BMP280_CTRLMEAS_OSRST_OS_8  4 << 5
#define BMP280_CTRLMEAS_OSRST_OS_16 5 << 5

#define BMP280_CTRLMEAS_MODE_SLEEP  0
#define BMP280_CTRLMEAS_MODE_FORCED 1
#define BMP280_CTRLMEAS_MODE_NORMAL 3

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
#define BMP280_SPI_READ 0x80
#define BMP280_SPI_WRITE 0x7F
#define BMP280_ID 0x58
#define __SPI_CS_ENABLE()	(SPI_CS_GPIO_Port->ODR) &= ~ SPI_CS_Pin
#define __SPI_CS_DISABLE()	(SPI_CS_GPIO_Port->ODR) |=  SPI_CS_Pin

/* Private variables ---------------------------------------------------------*/
BMP_Handle_Typedef bmp;
extern SPI_HandleTypeDef hspi1;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

//Sensor Config Functions
BMPStatus_t BMP280_Reset(void);
BMPStatus_t BMP280_Init(BMP_Init_Typedef * BMP_InitStruct);
void BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct);
void BMP280_Init_Custom(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct, uint8_t mode, uint8_t osrsp, uint8_t osrst,uint8_t ifcoef, uint8_t t_stdby);
BMPStatus_t BMP280_Set_PowerMode(uint8_t mode);
BMPStatus_t BMP280_Set_Oversample(uint8_t osrs_t, uint8_t osrs_p);
BMPStatus_t BMP280_Set_IIR_Filter_Coeff(uint8_t iircoef);
BMPStatus_t BMP280_Set_Standby_Time (uint8_t stdby);
//Register Functions
BMPStatus_t BMP280_Write_Register(uint8_t reg,int32_t size, uint8_t* data);
BMPStatus_t BMP280_Read_Register(uint8_t reg,int32_t size, uint8_t* data);

//Data Read functions
BMPStatus_t BMP280_Get_Measurements(uint32_t* adc_Temp,uint32_t* adc_Press);
BMPStatus_t BMP280_Get_Status(BMP_Handle_Typedef* bmp);
BMPStatus_t BMP280_Get_ID(SPI_HandleTypeDef *hspi, uint8_t* dev_id);
BMPStatus_t BMP280_Get_FactoryTrim( BMP280_trim_t *bmpt);
BMPStatus_t BMP280_Force_Measure(uint32_t* temp,uint32_t* pressure);
BMPStatus_t BMP280_Get_Measurements(uint32_t* adc_Temp,uint32_t* adc_Press);
BMPStatus_t BMP280_Get_Temp(uint32_t* adc_Temp);
int32_t BMP280_Compensate_Temp(int32_t T_val,int32_t* t_fine, BMP280_trim_t bmp_trim);
BMPStatus_t BMP280_Get_Pressure(uint32_t* adc_Press);
uint32_t BMP280_Compensate_Pressure(uint32_t P_val,int32_t t_fine,BMP280_trim_t bmp_trim);
//Measurement functions

#endif /* HAL_BMP280_H_ */

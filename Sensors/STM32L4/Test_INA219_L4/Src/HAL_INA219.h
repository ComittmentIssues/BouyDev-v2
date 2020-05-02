/*
 * HAL_INA219.h
 *
 *  Created on: May 2, 2020
 *      Author: jamie
 */

#ifndef HAL_INA219_H_
#define HAL_INA219_H_

/* Private Includes -----------------------------------------------------------*/

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_gpio.h"

/* Private typedef -----------------------------------------------------------*/
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
	INA_INIT_ERROR,
	INA_I2C_READ_ERROR,
	INA_I2C_WRITE_ERROR,
	INA_RESET_ERROR,
	INA_DEVICE_ONLINE,
	INA_DEVICE_OFFLINE,
	INA_DEVICE_READY
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
}INA219_Init_Typedef;

typedef struct
{
	INA219_Init_Typedef Init;
	uint16_t Config_val;
	I2C_HandleTypeDef ina_i2c;
	INA_Config_Status Use_Config;

	float INA219_I_LSB;
	float INA219_Vshunt_LSB;
	float INA219_P_LSB;
} INA219_Handle_Typedef;

/* Private define ------------------------------------------------------------*/

#define USE_I2C_2 //change this to the corresponding I2C handler
//config register definitions
#define INA219_CONFIG_RESET 0b1<<15	//setting this bit causes device to reset
#define INA219_CONFIG_BRNG_32V 0b1<<13
#define INA219_CONFIG_BRNG_16V 0b0<<13

#define INA219_CONFIG_PG_1 0b00<<11
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

#define INA219_FLAG_CNVR 0b10
#define INA219_FLAG_MOF  0b1

#define INA219_I2C_Address 0x80
#define INA219_DEFAULT_CONFIG 0x399F
#define INA219_R_SHUNT 0.1
/* Private variables ---------------------------------------------------------*/
INA219_Handle_Typedef ina;

/* Private function prototypes -----------------------------------------------*/
INA_Status_t INA219_Begin(void);
INA_Status_t INA219_Get_Reg_Config(INA219_Handle_Typedef* hina);
INA_Status_t INA219_Reset(void);
INA_Status_t INA219_Set_Power_Mode(uint16_t PWR_MODE);
INA_Status_t INA219_Set_Reg_Config(INA219_Handle_Typedef *hina);
INA_Status_t INA219_Get_Shunt_Voltage(int16_t *Shunt_Voltage);
INA_Status_t INA219_Get_Bus_Voltage(int16_t *Bus_Voltage);
INA_Status_t INA219_Get_Current(int16_t *current);
INA_Status_t INA219_Get_Power(int16_t *power);
INA_Status_t INA219_Calibrate_32V_2A(float *I_MBO, float *V_MBO, float *P_Max);
INA_Status_t INA219_Calibrate_16V_1_2A(float *I_MBO, float *V_MBO, float *P_Max);
INA_Status_t INA219_Trigger_Conversion(uint8_t val);
#endif /* HAL_INA219_H_ */

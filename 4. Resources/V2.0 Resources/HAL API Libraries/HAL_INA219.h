/*
 * HAL_INA219.h
 *
 *  Created on: May 2, 2020
 *      Author: Jamie Nicholas Jacobson
 *      Student Number: JCBJAM007
 *      For: The University of Cape Town
 *========================================================================================================
 *
 * All functions and definitions in this library have been programmed in accordance with the
 * INA219 datasheet here: https://www.ti.com/lit/ds/symlink/ina219.pdf?ts=1595920418905&ref_url=https%253A%252F%252Fwww.google.com%252F
 *
 * This Library is designed to interface with the Texas Instruments INA219 I2C Digital Power Monitor
 * using the STM HAL Libraries. This library was originally written using HAL version 1.14
 * and has been modified for use with HAL version 1.15.1 as of 24/07/2020.
 *
 * An important part of the SHARC Buoy deployment scheme is battery monitoring. It is important that
 * the device be able to run off of a limited power supply for a long period of time. In order to gauge
 * the performance of the buoy, the battery voltage and input current variation over time must be recorded
 *
 * The INA 219 Is an extremely low power, High precision power monitor that can withstand temperatures up to
 * -40 degrees C. The input voltage to the sensor can vary from 0V - 16V with an extremely small supply current.
 * The device uses I2C to communicate with a host device All Communication is facilitated through
 * an INA219_HandleTypedef. This is a custom handler that abstracts most of the microcontroller functions
 * Making this library simple and easy to use.
 *
 * Power measurements occur through a shunt voltage channel
 * and a bus voltage channel. These signals pass through an ADC and are stored in a 16-bit wide register. A full
 * register map is given in the INA_Register_t enum. The shunt resistor must be of a known value and is provided
 * by the user.
 *
 * Before the chip can be used, a calibration procedure must be performed in accordance with section
 * 8.5 of the reference manual (pages 12 - 13). This procedure determines the correct current resolution
 * allowing for Current and Power measurements to be calculated by the device.
 *
 * HOW TO USE:
 *
 *  1. Call the Function INA219_Begin() to initialise peripheral communication
 *  2. Reset the device
 *  3. set INA settings using INA219_Set_Reg_Config() *Note: customise this function to your desire
 *  4. Calibrate the sensor
 *  5. Set the power mode
 *
 * Measuring Power in Triggered mode:
 *
 * 1. A conversion is triggered by writting the trigger mode to the config register
 * 2. A flag is reset in the Bus voltage register indicating that a conversion is taking place
 * 3. When the flag is set (by hardware) a value can be read
 * 4. Read the values using a register read
 *
 * Measuring Power in Continuous mode:
 *
 * Values can be read from the register at any time during continuos mode
 *
 * Registers are updated to most recent values with out any prompt
 *
 * Therefore it is important to keep track of the Conversion status flag
 *
 *
 * This Library contains the following:
 *
 * 1. Complete Register Map of the Device
 * 2. Register Macro Definitions
 * 3. I2C Register Read/Write Functions
 * 4. 2 Calibration routines for 16V Max and 32V Max
 * 5. INA219 Configuration Functions
 * 6. Power Mode Configuration Functions
 * 7. ADC Val to Voltage/Current/Power Conversion
 * 8. Init/ Deint Functions
 *
 *========================================================================================================
 */

#ifndef HAL_INA219_H_
#define HAL_INA219_H_

//============================= 1. Includes ==============================================

#include "stm32l4xx_hal.h" //HAL Library includes

//============================= 2. Typedefs ==============================================

/*
 * INA_Register_t
 *
 * @brief: register map of the IN219 device. registers are 16 bit wide and a full description
 * 		   of the registers can be found on pages 18 - 24 of the reference manual
 */
typedef enum
{
	CONFIG_REG = 0x00,
	V_SHUNT_REG = 0x01,
	V_BUS_REG = 0x02,
	POWER_REG = 0x03,
	CURRENT_REG = 0x04,
	CALIBRATION_REG = 0x05
}INA_Register_t;

/*
 * INA_Status_t
 *
 * @brief: returns the status of an INA function. Used to represent
 * 		   numeric statuses returned as a result of running Sensor-register level
 * 		   functions e.g. using i2c to write a value for Power mode. Successful return
 * 		   statuses: INA_OK, INA_DEVICE_ONLINE, INA_DEVICE, READY
 * 		   unsuccessful return statuses: INA_xxx_ERROR, INA_DEVICE_OFFLINE
 */
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

/*
 * INA_Config_Status
 *
 * @brief: used to determine the state of the sensor during the initialization routine
 * 		   shows whether the value in the config register is the reset value (Default)
 * 		   or the user customised value (Configured)
 */
typedef enum
{
 Default,
 Configured,
}INA_Config_Status;

/*
 * INA219_Init_Typedef
 *
 * Struct used to configre settings for the INA219 Device.
 *
 * This struct is configured with predefined values
 * when the function INA_Status_t INA219_Set_Reg_Config(INA219_Handle_Typedef *hina) is called.
 *
 * The user can also create their own initialization function and use their settings however,
 * the user must use a value defined in the macros section below.
 *
 * uint16_t INA_BUS_VOLTAGE_RANGE - choose voltage range +-16V or +132V
 *
 * uint16_t INA_SHUNT_PGA_RANGE - Programmable Gain Amplifier (/8, /4, /2, /1 options)
 *
 * uint16_t INA_BUS_ADC_RESOLUTION - set resolution or number of samples per average for Bus Voltage ADC
 *
 * uint16_t INA_SHUNT_RESOLUTION = set resolution or number of samples per average for the Shunt Voltage ADC
 */
typedef struct
{
	uint16_t INA_BUS_VOLTAGE_RANGE;
	uint16_t INA_SHUNT_PGA_RANGE;
	uint16_t INA_BUS_ADC_RESOLUTION;
	uint16_t INA_SHUNT_RESOLUTION;
}INA219_Init_Typedef;

/*
 * INA219_Handle_Typedef
 *
 * Custom Sensor handler designed to facilitate communications with the sensor.
 * This is a virtual representation of the sensor and contains details about the
 * - configuration settings
 * - I2C Peripheral Port
 * - Configuration Status
 * - Calibrated Current, Shunt Voltage and Power Step size
 *
 * This library uses an intrinsic INA219 handler for functiosn which is defined in
 * the variable sections. All functions call this handler. The user has the option
 * to define an external handler and use that.
 */
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

//======================== 3. Macro Definitions =========================================

//I2C Definitions
#define INA_I2C_Perpheral I2C2		//I2Cx Port
#define INA_I2C_SCL_GPIO_PORT GPIOB
#define INA_I2C_SCL_PIN GPIO_PIN_10

#define INA_I2C_SDA_GPIO_PORT GPIOB
#define INA_I2C_SDA_PIN GPIO_PIN_11
#define USE_I2C_2 //change this to the corresponding I2C handler

//Config Register settings
#define INA219_CONFIG_RESET 0b1<<15		//setting this bit causes device to reset
#define INA219_CONFIG_BRNG_32V 0b1<<13	//Set Bus voltage range to +-32V
#define INA219_CONFIG_BRNG_16V 0b0<<13  //Set Bus Voltage Range to +-16V
#define INA219_CONFIG_PG_1 0b00<<11		//Set PGA to /1
#define INA219_CONFIG_PG_2 0b01<<11		//Set PGA to /2
#define INA219_CONFIG_PG_4 0b10<<11		//Set PGA to /4
#define INA219_CONFIG_PG_8 0b11<<11		//Set PGA to /8

//BUS ADC
#define INA219_CONFIG_BADC_MODE_SAMPLE_9_BIT    0b0000<<7	//Set Bus ADC to 9 Bit mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_10_BIT   0b0001<<7	//Set Bus ADC to 10 Bit mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_11_BIT   0b0010<<7	//Set Bus ADC to 11 Bit mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_12_BIT   0b0011<<7	//Set Bus ADC to 12 Bit mode 1
#define INA219_CONFIG_BADC_MODE_SAMPLE_12_BIT_2 0b1000<<7	//Set Bus ADC to 12 Bit mode 2

#define INA219_CONFIG_BADC_MODE_SAMPLE_2_samples   0b1001<<7	//Set Bus ADC to 2 Sample mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_4_samples   0b1010<<7	//Set Bus ADC to 4 Sample mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_8_samples   0b1011<<7	//Set Bus ADC to 8 Sample mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_16_samples  0b1100<<7	//Set Bus ADC to 16 Sample mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_32_samples  0b1101<<7	//Set Bus ADC to 32 Sample mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_64_samples  0b1110<<7	//Set Bus ADC to 64 Sample mode
#define INA219_CONFIG_BADC_MODE_SAMPLE_128_samples 0b1111<<7	//Set Bus ADC to 128 Sample mode

//SHUNT ADC
#define INA219_CONFIG_SADC_MODE_SAMPLE_9_BIT    0b0000<<3		//Set Shunt ADC to 9 Bit mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_10_BIT   0b0001<<3		//Set Shunt ADC to 10 Bit mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_11_BIT   0b0010<<3		//Set Shunt ADC to 11 Bit mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_12_BIT   0b0011<<3		//Set Shunt ADC to 12 Bit mode 1
#define INA219_CONFIG_SADC_MODE_SAMPLE_12_BIT_2 0b1000<<3		//Set Shunt ADC to 12 Bit mode 2

#define INA219_CONFIG_SADC_MODE_SAMPLE_2_samples   0b1001<<3	//Set Bus ADC to 2 Sample mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_4_samples   0b1010<<3	//Set Bus ADC to 4 Sample mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_8_samples   0b1011<<3	//Set Bus ADC to 8 Sample mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_16_samples  0b1100<<3	//Set Bus ADC to 16 Sample mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_32_samples  0b1101<<3	//Set Bus ADC to 32 Sample mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_64_samples  0b1110<<3	//Set Bus ADC to 64 Sample mode
#define INA219_CONFIG_SADC_MODE_SAMPLE_128_samples 0b1111<<3	//Set Bus ADC to 128 Sample mode

//Power Mode Config
#define INA219_CONFIG_MODE_POWER_DOWN 			    0b000		//Power Down Mode
#define INA219_CONFIG_MODE_SHUNT_TRIGGERERD 	    0b001		//Shunt Only Triggered Conversion
#define INA219_CONFIG_MODE_BUS_TRIGGERED 			0b010		//Bus Only Triggered Conversion
#define INA219_CONFIG_MODE_SHUNT_BUS_TRIGGERED 		0b011		//Shunt and Bus triggered conversion
#define INA219_CONFIG_MODE_ADC_DIS 	 	 			0b100		//Device on, No Conversions
#define INA219_CONFIG_MODE_SHUNT_CTS 	 			0b101		//Shunt Only Continuous Conversion
#define INA219_CONFIG_MODE_BUS_CTS 	 	 			0b110		//Bus Only Continuous Conversion
#define INA219_CONFIG_MODE_SHUNT_BUS_CTS 			0b111		//Shunt and Bus Continuous Conversion

//Signal Flags
#define INA219_FLAG_CNVR 0b10	//Conversion Status bit
#define INA219_FLAG_MOF  0b1	//Math Overflow Status bit

//Device Reg info
#define INA219_I2C_Address 0x80			//I2C Adress (see ref manual)
#define INA219_DEFAULT_CONFIG 0x399F	//Register Reset Value
#define INA219_R_SHUNT 0.1				//Value of Shunt Resistor on SHARC BUOY

//======================== 4. Private Variables =========================================

INA219_Handle_Typedef ina;		//instance of INA219_Handle Typedef

//=================== 5. Init Function Prototypes =======================================

/*
 * Function Name INA_Status_t INA219_Init_Sensor(void);
 *
 * @brief: Preset Initialization function - initialises I2C communications on
 * 		   I2C2 and configures the device with the following parameters:
 *
 * 		   Bus Voltage Range: 16V
 * 		   PGA /4
 * 		   ADC Resolution 12-bit
 * 		   Power Mode: Bus and Shunt Trigger Mode
 *
 * @param none
 *
 * @return INA_Status_t - return status of function
 */
INA_Status_t INA219_Init_Sensor(void);

/*
 * Function Name INA_Status_t INA219_Begin(void);
 *
 * @brief: Function to initialize i2c Communications and begin interfacing with the sensor object
 *
 * 		   Function will assign an i2c handler to the ina handler based on the macro USE_I2C_1
 * 		   It will then check if the sensor is online and read the configuration register.
 * 		   The INA config register defaults to 0x399F on power up or reset
 *		   If the device has been configured, the register will return a different value
 * 		   The USE_Config member of the struct INA will be updated based on whether the
 * 		   Received config value matches that of the default config value (note: this should)
 * 		   only happen if the device has been unconfigured or a power reset has occurred
 * @param none
 *
 * @return INA_Status_t value showing the status of the function
 */
INA_Status_t INA219_Begin(void);

//=================== 6. Calib Function Prototypes =======================================

/*
 * Function Name INA_Status_t INA219_Calibrate_32V_2A(float *I_MBO, float *V_MBO, float *P_Max);
 *
 * @brief: Calibrates device for 32V 2A max operation
 *
 * @param I_MBO pointer to float to hold the result of the I_Max before overflow calculation
 * @param V_MBO pointer to float to hold the result of the V_Max before overflow calculation
 * @param P_max pointer to float to hold the result of the max power calculation
 *
 * @return INA_Status_t value showing the status of the function
 */
INA_Status_t INA219_Calibrate_32V_2A(float *I_MBO, float *V_MBO, float *P_Max);

/*
 * Function Name INA_Status_t INA219_Calibrate_32V_2A(float *I_MBO, float *V_MBO, float *P_Max);
 *
 * @brief: Calibrates device for 16V 1 - 2A max operation
 *
 * @param I_MBO pointer to float to hold the result of the I_Max before overflow calculation
 * @param V_MBO pointer to float to hold the result of the V_Max before overflow calculation
 * @param P_max pointer to float to hold the result of the max power calculation
 *
 * @return INA_Status_t value showing the status of the function
 */
INA_Status_t INA219_Calibrate_16V_1_2A(float *I_MBO, float *V_MBO, float *P_Max);

//================= 7. Register Function Prototypes ======================================

/*
 * Function Name INA_Status_t INA219_Reset(void);
 *
 * @brief: Performs a software reset on the sensor
 *
 * @param none
 *
 * @return INA_Status_t value showing the status of the function
 */
INA_Status_t INA219_Reset(void);

/*
 * Function Name INA_Status_t INA219_Get_Reg_Config(INA219_Handle_Typedef* hina);
 *
 * @brief: Reads the value in the 16-bit config register
 *
 * @param hina - pointer to INA219_Handle_Typedef instance
 *
 * @return INA_Status_t value showing the status of the function
 */
INA_Status_t INA219_Get_Reg_Config(INA219_Handle_Typedef* hina);

/*
 * Function Name INA_Status_t INA219_Set_Power_Mode(uint16_t PWR_MODE);
 *
 * @brief: Sets the power mode in the Config Register
 *
 * @param PWR_MODE - desired opperational mode (as defined in the macros above)
 *
 * @return INA_Status_t value showing the status of the function
 */
INA_Status_t INA219_Set_Power_Mode(uint16_t PWR_MODE);

/*
 * Function Name INA_Status_t INA219_Set_Reg_Config(INA219_Handle_Typedef *hina);
 *
 * @brief: Writes a 16 bit integer representing the desired configuration settings
 * 		   to the INA config register
 *
 * @param: hina - pointer to INA219_Handle_Typedef instance
 *
 * @return INA_Status_t value showing the status of the function
 */
INA_Status_t INA219_Set_Reg_Config(INA219_Handle_Typedef *hina);

//================ 8. Measurement Function Prototypes ====================================

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
INA_Status_t INA219_Trigger_Conversion(uint8_t val);

/*
 * Function Name INA_Status_t INA219_Get_Shunt_Voltage(int16_t *Shunt_Voltage);
 *
 * @brief: Reads the ADC value from the Shunt Register and converts it into a
 * 		   shunt voltage based on the PGA setting
 *
 * @Param: Shunt_Voltage - pointer to int16_t variable to hold the value
 *
 * @return: INA_Status_t - return value showing the status of the function
 *
 */
INA_Status_t INA219_Get_Shunt_Voltage(int16_t *Shunt_Voltage);

/*
 * Function Name INA_Status_t INA219_Get_Bus_Voltage(int16_t *Bus_Voltage);
 *
 * @brief: Reads the ADC value from the Bus Register and converts it into a
 * 		   bus voltage.
 *
 * @Param: Bus_Voltage - pointer to int16_t variable to hold the value
 *
 * @return: INA_Status_t - return value showing the status of the function
 *
 */
INA_Status_t INA219_Get_Bus_Voltage(int16_t *Bus_Voltage);

/*
 * Function Name INA_Status_t INA219_Get_Current(int16_t *current);
 *
 * @brief: Reads the ADC value from the Current Register and converts it into a
 * 		   current value.
 *
 * @Param: current - pointer to int16_t variable to hold the value
 *
 * @return: INA_Status_t - return value showing the status of the function
 *
 */
INA_Status_t INA219_Get_Current(int16_t *current);

/*
 * Function Name INA_Status_t INA219_Get_Power(int16_t *power);
 *
 * @brief: Reads the ADC value from the Power Register and converts it into a
 * 		   Power value in Watts
 *
 * @Param: power - pointer to int16_t variable to hold the value
 *
 * @return: INA_Status_t - return value showing the status of the function
 *
 */
INA_Status_t INA219_Get_Power(int16_t *power);


#endif /* HAL_INA219_H_ */

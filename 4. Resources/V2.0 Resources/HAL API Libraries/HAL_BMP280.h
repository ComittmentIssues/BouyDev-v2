/*
 * HAL_BMP280.h
 *
 *  Created on: Apr 25, 2020
 *  Author: Jamie Jacobson
 *  Student No: JCBJAM007
 *  For: University of Cape Town
 *
 *========================================================================================================
 *
 * This Library is designed to interface with the Bosch BMP280 Pressure and Temp sensor
 * using the STM HAL Libraries. This library was originally written using HAL version 1.14
 * and has been modified for use with HAL version 1.15.1 as of 24/07/2020.
 *
 * This device is a digital sensor with both temperature and pressure sensing capabilities.
 * The device can be configured in a variety of modes to optimize power consumption and
 * measurement accuracy with speed. The device also has oversample settings for temperature
 * and pressure as well as a configurable IIR filter. Details of how to set the filter can
 * be found in the reference manual
 * here: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf
 *
 * This library also contains a register map that is located in the BMPRegisterMap_t enum.
 *
 * Temperature and Pressure values are represented by 20 bit long integers. These values are stored in
 * Separate 8-bit registers denoted as temp_XLSB,temp_LSB, temp_MSB (or press...) these values must be read
 * using an SPI multi byte read otherwise the values become desynchronised and improper readings will be
 * given. The ADC values are converted into temperature and pressure using the BMP compensation algorythm
 * (more information on pages: 20 - 23). Each sensor has a set of unique compensation value: 3 for Temp
 * and 9 for pressure. Each value is a 16 bit integer that is either signed or unsigned (see 23 for more info)
 * and is stored in pairs of registers. The compensation formula uses these values to compensate for irregularities
 * from each sensor.
 *
 *  This library interfaces with the hardware on the STM32L4 through a BMP280 Handler object. This is a custom
 *  handler that is used to represent the sensor on the microcontroller. Communication is performed through the
 *  SPI_HandleTypedef* bmp_SPI which points to an spi handler instance. You can specify the specific spi port
 *  used by modifying the macro #define USE_SPI_x (where 1 <= x <= 3)
 *
 * HOW TO USE:
 *
 * 1. Declare a BMP_Handle_Typedef instance in the global area of the main program.
 *
 * 2. Set the parameters of the BMP sensor. This can be done in 3 ways:
 * |----a. Using one of the pre-made functions with preset modes:
 * |	   void BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct);
 * |
 * |----b. Using  one of the premade functions with custom parameters
 * |	   void BMP280_Init_Custom(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct, uint8_t mode, uint8_t osrsp, uint8_t osrst,uint8_t ifcoef, uint8_t t_stdby);
 * |
 * |----c. Modifying the  values of the BMP_Init_Typedef  struct in the BMP_Handle_Typedef
 *
 * 3. Initialise the sensor using the BMP280_Init() Function
 *
 * This library contains the following
 *
 * 1. Initialization/De Initialization functions
 * 2. Register map, bit definitions, enumerations of modes
 * 3. Compensation functions
 * 4. Sample functions
 *=========================================================================================================
 */

#ifndef HAL_BMP280_H_
#define HAL_BMP280_H_

//============================= 1. Includes ==============================================

#include "stm32l4xx_hal.h" //stm32_L4 HAL Header

//============================= 2. Includes ==============================================

/*
 * BMP280_trim_t
 *
 * Structure used to hold the values stored in the compensation registers: calib0 - calib25
 *
 * NOTE: This structure must be used with the function: BMPStatus_t BMP280_Get_FactoryTrim( BMP280_trim_t *bmpt)
 */
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

/*
 * BMPRegisterMap_t
 *
 * This enumerations maps the entire memory layout of the BMP280 sensor.
 * Each register is 8 bits long and each enumeration represents the base address
 * of that register. A full description of the memory map is
 * privided in the reference manual pages 24 - 27. please note that there are
 * 26 registers for the trim parameters named calib_0 - calib_25. The base address (calib0)
 * is provided and the nth register (calib_n) can be access by adding n to the base address.
 */

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

/*
 * BMP_Opp_Mode_t
 *
 * Enumeration of preset modes described in Table 15. Page 19 of the reference manual. By using one of these
 * values in the function BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct).
 * Settting the value of BMP_MODE automatically configures BMP_InitStruct with the settings that allow the device
 * to opperate in that mode
 *
 * 	MODE NAME:...................MODE:............Standby Time:......OSRSP:......OSRST:.......IIRC:
 * 	HandHeld_LP..................NORMAL...........62.5ms.............16..........2............4
 * 	HandHeld_Dynamic.............NORMAL...........0.5ms..............4...........1............16
 * 	WEATHER_Monitoring...........FORCED...........X..................1...........1............OFF
 *
 */
typedef enum
{
	HandHeld_LP,
	HandHeld_Dynamic,
	Weather_Monitoring
}BMP_Opp_Mode_t;
/*
 * BMP_Measuremment_Status_t
 *
 * Values that represent the conversion status of the sensor
 */
typedef enum
{
	BMP_Measurement_Busy = 0b1000,
	BMP_Measurement_Complete = 0

}BMP_Measurement_Status_t;

/*
 * BMP_IM_Status_t
 *
 * Indicates the status of data being copied to the shadow registers
 */

typedef enum
{
	BMP_Data_Shadowing_Busy = 0b1,
	BMP_Data_Shadowing_Complete = 0

}BMP_IM_Status_t;

/*
 * BMP_Init_Typedef
 *
 * A struct representing the sensor on the microcontroller side.
 * This will contain the settings for configuring the sensor.
 *
 * This struct must be initialized and configured before the
 * sensor is initialized using the Macros shown in section 3.
 *
 * The following needs to be configured:
 *
 * BMP_Power_Mode:	The power mode of the device (SLEEP, FORCED, NORMAL)
 * BMP_Pressure/Temperature_OverSample: Oversample settings for temp and pressure (OSRSP/OSRST)
 * BMP_t_Standby: Set the output rate by setting the standby time between samples (BMP280_CONFIG_tsb)
 * BMP_IIR_FILTER_COEFFICIENTS: set the filter coefficients for the IIR Filter (BMP280_CONFIG_FILTER_COEFF)
 */

typedef struct
{
	uint8_t BMP_Power_Mode;
	uint8_t BMP_Pressure_OverSample;
	uint8_t BMP_Temperature_OverSample;
	uint8_t BMP_t_Standby;
	uint8_t BMP_IIR_FILTER_COEFFICIENTS;
} BMP_Init_Typedef;

/*
 * BMP_Handle_Typedef
 *
 * This structure virtualises the BMP instance on the microcontroller side
 * all communications, configuration settings and compensation values are
 * faciliated through this handler.
 *
 * Init:				Instance of BMP_Init_Typedef
 * IM_Status:			Status of Shadow register copying process
 * M_Status:			Status of ADC conversion
 * Factory_Trim:		Structure to hold compensation values
 * bmp_spi:				Pointer to spi handle typedef
 */
typedef struct
{
	BMP_Init_Typedef Init;
	BMP_IM_Status_t IM_Status;
	BMP_Measurement_Status_t M_Status;
	BMP280_trim_t Factory_Trim;
	SPI_HandleTypeDef *bmp_spi;
}BMP_Handle_Typedef;



//======================== 3. Macro Definitions =========================================

//SPI Definitions
#define SPI_CS_Pin GPIO_PIN_10
#define SPI_CS_GPIO_Port GPIOA

/*
 * USE_SPI_x maps the spi handler to the correct SPI POrt. On the stm32l476RG, there are 3 spi peripherals.
 * Note: set x to a value from 1 - 3 otherwise the code will not work
 */
#define USE_SPI_1					//SPI Instance

#ifdef USE_SPI_1
#define BMP_SPI_PORT SPI1
#endif

#ifdef USE_SPI_2
#define BMP_SPI_PORT SPI2
#endif

#ifdef USE_SPI_3
#define BMP_SPI_PORT SPI3
#endif

//SPI Pin Definitions

#define BMP_SPI_SCLK_PIN GPIO_PIN_3
#define BMP_SPI_MISO_PIN GPIO_PIN_4
#define BMP_SPI_MOSI_PIN GPIO_PIN_5

#define BMP_SPI_GPIO_PORT GPIOB

#define BMP280_I2C_ADDRESS1 0x76	//FOR I2C use
#define BMP280_I2C_ADDRESS2 0x77

//BMP CTRLMEAS Register Definitions

//Pressure Oversample values
#define BMP280_CTRLMEAS_OSRSP_SKIP  0 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_1  1 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_2  2 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_4  3 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_8  4 << 2
#define BMP280_CTRLMEAS_OSRSP_OS_16 5 << 2

//Temperature Oversample values
#define BMP280_CTRLMEAS_OSRST_SKIP  0 << 5
#define BMP280_CTRLMEAS_OSRST_OS_1  1 << 5
#define BMP280_CTRLMEAS_OSRST_OS_2  2 << 5
#define BMP280_CTRLMEAS_OSRST_OS_4  3 << 5
#define BMP280_CTRLMEAS_OSRST_OS_8  4 << 5
#define BMP280_CTRLMEAS_OSRST_OS_16 5 << 5

//Power Mode Settings
#define BMP280_CTRLMEAS_MODE_SLEEP  0
#define BMP280_CTRLMEAS_MODE_FORCED 1
#define BMP280_CTRLMEAS_MODE_NORMAL 3

//CONFIG Register Definitions

//Standby Time configuration
#define BMP280_CONFIG_tsb_0_5  0 << 5
#define BMP280_CONFIG_tsb_62_5 1 << 5
#define BMP280_CONFIG_tsb_125  2 << 5
#define BMP280_CONFIG_tsb_250  3 << 5
#define BMP280_CONFIG_tsb_500  4 << 5
#define BMP280_CONFIG_tsb_1000 5 << 5
#define BMP280_CONFIG_tsb_2000 6 << 5
#define BMP280_CONFIG_tsb_4000 7 << 5

//IIR Filter Coeeficients
#define BMP280_CONFIG_FILTER_COEFF_OFF 0 << 2
#define BMP280_CONFIG_FILTER_COEFF_2   1 << 2
#define BMP280_CONFIG_FILTER_COEFF_4   2 << 2
#define BMP280_CONFIG_FILTER_COEFF_8   3 << 2
#define BMP280_CONFIG_FILTER_COEFF_16  4 << 2

//SPI Settings
#define BMP280_CONFIG_SPI3_EN  1
#define BMP280_CONFIG_SPI3_DIS 0

//Constants and Litterals

#define BMP280_CALIB_DATA_LEN 24	//Number of registers to be read from
#define BMP280_Soft_Reset 0xB6		//Causes device to reset when written to the reset register
#define BMP280_SPI_READ 0x80		//SPI READ COMMAND
#define BMP280_SPI_WRITE 0x7F		//SPI WRITE COMMAND
#define BMP280_ID 0x58				//Device ID
#define __SPI_CS_ENABLE()	(SPI_CS_GPIO_Port->ODR) &= ~ SPI_CS_Pin	//SETS CHIP SELECT ENABLED
#define __SPI_CS_DISABLE()	(SPI_CS_GPIO_Port->ODR) |=  SPI_CS_Pin	//SETS CHIP SELECT DISABLED

//======================== 4. Private Variables =========================================

BMP_Handle_Typedef bmp;			//instance of BMP_Handle_Typedef

SPI_HandleTypeDef hspi1;		//instance of SPI handle_Typedef

//======================== 5. Init Function Prototypes =========================================

/*
 * Function Name void BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct);
 *
 * @brief: Configures the paramaters of the BMP_InitStruct Mode to configure the BMP Sensor. Custom preset
 * 		   modes are defined in table 15. Page 19 of the reference manual
 *
 * @param: BMP_MODE	- the preset mode desired. This enumeration is defined in BMP_Opp_Mode_t
 *
 * @param BMP_IniStruct - empty Configuration struct
 *
 * @return: None
 */
void BMP280_Init_Preset_Mode(BMP_Opp_Mode_t BMP_MODE, BMP_Init_Typedef* BMP_InitStruct);

/*
 * Function Name void BMP280_Init_Custom(BMP_Init_Typedef* BMP_InitStruct, uint8_t mode, uint8_t osrsp, uint8_t osrst,uint8_t ifcoef, uint8_t t_stdby)
 *
 * @brief: configures paramaters of the BMP InitStruct using user specified values
 *
 * @param BMP_IniStruct - empty Configuration struct
 * @param mode - Device mode (Forced, Sleep, Normal)
 * @param osrsp - Pressure Oversample Value
 * @param osrst - Temperature Oversaple Value
 * @param ifcoef - IIR Filter Coefficient value
 * @param stdby - Standby Time value
 *
 * @return: None
 */
void BMP280_Init_Custom(BMP_Init_Typedef* BMP_InitStruct, uint8_t mode, uint8_t osrsp, uint8_t osrst,uint8_t ifcoef, uint8_t t_stdby);

/*
 * @Param: Initialises the BMP Sensor and Configures the registers to the settings in the BMP_InitStruct
 *
 * @param: BMP_Init_Typedef* - pointer to BMP_InitStruct
 *
 * @return: BMPStatus_t - status of initialisation function
 *
 */
BMPStatus_t BMP280_Init(BMP_Init_Typedef* BMP_InitStruct);

/*
 * Function Name BMPStatus_t BMP280_DeInit(void)
 *
 * @brief: De Intialise SPI Peripherals and set device to sleep mode
 *
 * @param: none
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_DeInit(void);

//======================== 6. Reg Config Function Prototypes =========================================

/*
 * Function Name BMPStatus_t BMP280_Reset(void);
 *
 * @brief: Performs a Device Software reset
 *
 * @param: None
 *
 * @return: BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Reset(void);

/*
 * Function Name BMPStatus_t BMP280_Set_PowerMode(uint8_t mode);
 *
 * @brief: Configures the sensor Power mode
 *
 * @param mode: desired power mode setting
 *
 * @return: BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Set_PowerMode(uint8_t mode);

/*
 * Function Name BMPStatus_t BMP280_Set_Oversample(uint8_t osrs_t, uint8_t osrs_p);
 *
 * @brief: set the over-sampling parameter in the CTRL MEAS register for temperature and pressure
 *
 * @param: osrs_t - the desired over-sample parameter for pressure.
 * @param: osrs_p - the desired over-sample parameter for temperature.
 *
 * @return BMPStatus_t - return status of the function
 */
BMPStatus_t BMP280_Set_Oversample(uint8_t osrs_t, uint8_t osrs_p);

/*
 * Function Name BMPStatus_t BMP280_Set_IIR_Filter_Coeff(uint8_t iircoef);
 *
 * @brief: sets the IIR filter coefficients in the config register
 *
 * @param: iircoef - desired oversampling parameter
 *
 * @return BMPStatus_t - return status of the function
 */
BMPStatus_t BMP280_Set_IIR_Filter_Coeff(uint8_t iircoef);

/*
 * Function Name BMPStatus_t BMP280_Set_Standby_Time (uint8_t stdby);
 *
 * @brief: set the standby time between measurements for normal mode in the
 * 		   config register
 *
 * @param: stdby- bit representation of desired standby time
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Set_Standby_Time (uint8_t stdby);

/*
 * Function Name BMPStatus_t BMP280_Write_Register(uint8_t reg,int32_t size, uint8_t* data);
 *
 * @brief: Function to set values in a specified register on the BMP280. Register definitions
 * 		   are given in the BMPRegisterMap_t enumeration
 *
 * @param: reg - base address of the register to be written to
 *
 * @param: size - number of bytes to be writte
 *
 * @param data - pointer to array/memory address of data to be sent
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Write_Register(uint8_t reg,int32_t size, uint8_t* data);

/*
 * Function Name BMPStatus_t BMP280_Read_Register(uint8_t reg,int32_t size, uint8_t* data);
 *
 * @brief: reads a specified number of bytes from the device from the base register specified by
 * 		   the user.
 *
 * @param: reg - starting address of register to be read from
 *
 * @param: size - number of bytes to be read
 *
 * @param: data - pointer to a uin8_t buffer to hold data
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Read_Register(uint8_t reg,int32_t size, uint8_t* data);

//======================== 7. Data Read Function Prototypes=========================================

/*
 * Function Name: BMPStatus_t BMP280_Get_Measurements(uint32_t* adc_Temp,uint32_t* adc_Press);
 *
 * @brief: reads the temperature and pressure measurement registers from the device and combines
 * 		   them into 32 bit integer ADC values. Note: this function reads the values when the
 * 		   device is set to normal mode
 *
 * @param: adc_Temp - pointer to uint32_t variable to store the recorded Temperature value
 *
 * @param: adc_Press - pointer to uint32_t variable to store the recorded pressure value
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Get_Measurements(uint32_t* adc_Temp,uint32_t* adc_Press);

/*
 * Function Name BMPStatus_t BMP280_Get_Status(BMP_Handle_Typedef* bmp);
 *
 * @brief: reads the status register on the device and updates the status variables
 * 		   of the bmp_handle_Typedef
 *
 * @param: bmp - pointer to BMP_HandleTypedef
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Get_Status(BMP_Handle_Typedef* bmp);

/*
 * Function Name BMPStatus_t BMP280_Get_ID(uint8_t* dev_id);
 *
 * @brief: reads the id register of the device and returns the value
 *
 * @param: dev_id - pointer to variable to store the value read from the register
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Get_ID(uint8_t* dev_id);

/*
 * Function Name BMPStatus_t BMP280_Get_FacTrim( BMP280_trim_t *bmpt);
 *
 * @brief: Fetches the compensation parameters from calibration registers
 * 		   0 - 25. Converts values to uint16_t/int16_t and stores them
 * 		   in a BMP280_trim_t struct.
 * @param bmpt - pointer to a struct to hold the compensation parameters
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Get_FactoryTrim( BMP280_trim_t *bmpt);

//======================== 8. Measurement Function Prototypes=========================================
/*
 * Function Name BMPStatus_t BMP280_Force_Measure(uint32_t* temp,uint32_t* pressure);
 *
 * @brief: Triggers a temperature/ pressure measurement while the device is operating in forced mode.
 * 		   A Forced measurement occurs when the power mode is changed to force mode. This causes a
 * 		   conversion to occur. when the conversion has finnished, the device returns to sleep mode
 * 		   untill another measurement is forced
 * @param temp - pointer to 32 bit integer to store the temperature ADC value
 *
 * @param pressure - pointer to 32 bit integer to store the pressure ADC value.
 *
 * @return BMPStatus_t - return status of function
 */
BMPStatus_t BMP280_Force_Measure(uint32_t* temp,uint32_t* pressure);

/*
 * Function Name BMPStatus_t BMP280_Get_Temp(uint32_t adc_TEMP);
 *
 * @brief: reads the temperature XLSB, LSB, MSB registers and combines the
 * 		   values to create a 20 bit integer.
 *
 * @param pressure - pointer to 32 bit integer to store the temperature ADC value.
 *
 * @return BMPStatus_t - return status of function
 *
 */
BMPStatus_t BMP280_Get_Temp(uint32_t* adc_Temp);

/*
 * Function Name BMPStatus_t BMP280_Get_Pressure(uint32_t adc_Press);
 *
 * @brief: reads the pressure XLSB, LSB, MSB registers and combines the
 * 		   values to create a 20 bit integer.
 *
 * @param pressure - pointer to 32 bit integer to store the pressure ADC value.
 *
 * @return BMPStatus_t - return status of function
 *
 */
BMPStatus_t BMP280_Get_Pressure(uint32_t* adc_Press);

/*
 * Function Name: int32_t BMP280_Compensate_Temp(int32_t T_val,int32_t* t_fine, BMP280_trim_t bmp_trim);
 *
 * @brief: Calculates the true temperature measurement using the Bosch BMP temperature compensation formula
 * 		   (page 21 - 23 of the datasheet). Also used to calculate the temperature calibration parameter t_fine
 * 		   for the pressure compensation algorithm.
 *
 * @param: T_val - ADC temperature
 *
 * @param: t_fine - pointer to variable to hold the paramter t_fine
 *
 * @param: bmp_trim - struct containing the calibration parameters
 *
 * @return int32_t representation of the compensated temperature. in °C
 * 		   (last 2 digits of the value represent the first 2 signifcant places)
 */
int32_t BMP280_Compensate_Temp(int32_t T_val,int32_t* t_fine, BMP280_trim_t bmp_trim);

/*
 * Function Name: uint32_t BMP280_Compensate_Pressure(uint32_t P_val,int32_t t_fine,BMP280_trim_t bmp_trim);
 *
 * @brief: Calculates the true Atmospheric pressure measurement using the Bosch compensation formula
 *
 * @param P_val - ADC Pressure value
 *
 * @param t_fine - t_fine calibration parameter calculated from the Temperature Compensation formula
 *
 * @param bmp_trim - struct containing BMP calibration parameters
 *
 * @return uin32_t representation of compensated atmoshperic pressure in Pa
 */
uint32_t BMP280_Compensate_Pressure(uint32_t P_val,int32_t t_fine,BMP280_trim_t bmp_trim);


#endif /* HAL_BMP280_H_ */

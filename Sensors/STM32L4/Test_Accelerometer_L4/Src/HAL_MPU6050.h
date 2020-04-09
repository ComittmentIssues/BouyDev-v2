/*
 * HAL_MPU6050.h
 *
 *  Created on: Apr 9, 2020
 *      Author: jamie
 */

#ifndef HAL_MPU6050_H_
#define HAL_MPU6050_H_

/* Private typedef -----------------------------------------------------------*/
typedef enum{
	SELF_TEST_X = 0XD,
	SELF_TEST_Y = 0xE,
	SELF_TEST_Z = 0xF,
	SELF_TEST_A = 0x10,
	SMPLRT_DIV = 0x19,
	CONFIG = 0x1A,
	GYRO_CONFIG = 0x1B,
	ACCELEROMETER_CONFIG = 0x1C,
	FIFO_EN = 0x23,
	I2C_MST_CTRL = 0x24,
	I2C_SLV0_ADDR = 0x25,
	I2C_SLV0_REG = 0x26,
	I2C_SLV0_CTRL = 0x27,
	I2C_SLV1_ADDR = 0x28,
	I2C_SLV1_REG = 0x29,
	I2C_SLV1_CTRL = 0x2A,
	I2C_SLV2_ADDR = 0x2B,
	I2C_SLV2_REG = 0x2C,
	I2C_SLV2_CTRL = 0x2D,
	I2C_SLV3_ADDR = 0x2E,
	I2C_SLV3_REG = 0x2F,
	I2C_SLV3_CTRL = 0x30,
	I2C_SLV4_ADDR = 0x31,
	I2C_SLV4_REG = 0x32,
	I2C_SLV4_D0 = 0x33,
	I2C_SLV4_CTRL = 0x34,
	I2C_SLV4_DI = 0x35,
	I2C_MST_STATUS = 0x36,
	INT_PIN_CFG = 0x37,
	INT_ENABLE = 0x38,
	INT_STATUS = 0x3A,
	ACCEL_XOUT_H = 0x3B,
	ACCEL_XOUT_L = 0x3C,
	ACCEL_YOUT_H = 0x3D,
	ACCEL_YOUT_L = 0x3E,
	ACCEL_ZOUT_H = 0x3F,
	ACCEL_ZOUT_L = 0x40,
	TEMP_OUT_H = 0x41,
	TEMP_OUT_L = 0x42,
	GYRO_XOUT_H = 0x43,
	GYRO_XOUT_L = 0x44,
	GYRO_YOUT_H = 0x45,
	GYRO_YOUT_L = 0x46,
	GYRO_ZOUT_H = 0x47,
	GYRO_ZOUT_L = 0x48,
	EXT_SENS_DATA_BASE = 0x4A, //address of EXT_SENSE_DATA_00. to access registers 1 - 23, add the corressponding number
	I2C_SLV0_DO = 0x63,
	I2C_SLV1_DO = 0x64,
	I2C_SLV2_DO = 0x65,
	I2C_SLV3_DO = 0x66,
	I2C_MST_DELAY_CTRL = 0x67,
	SIGNAL_PATH_RESET = 0x68,
	USER_CTRL = 0x6A,
	PWR_MGMT_1 = 0x6B,
	PWR_MGMT_2 = 0x6C,
	FIFO_COUNTH = 0x72,
	FIFO_COUNTL = 0x73,
	FIFO_R_W  = 0x74,
	WHO_AM_I = 0x75
}MPU_6050_Register_t;

typedef enum
{
	MPU_I2C_ERROR,
	MPU_I2C_DEVICE_BUSY,
	MPU_I2C_ACK_NACK,
	MPU_I2C_DEVICE_OFFLINE,
	MPU_I2C_DEVICE_ONLINE,
	MPU_I2C_ID_ERROR,
	MPU_OK
}mpu_status_t;

//@brief: PASS_THROUGH This bit reflects the status of the FSYNC interrupt from an external device
//into the MPU-60X0. This is used as a way to pass an external interrupt
//through the MPU-60X0 to the host application processor. When set to 1, this
//bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
//(Register 55).
//I2C_SLV4_DONE Automatically sets to 1 when a Slave 4 transaction has completed. This
//triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
//(Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
//I2C_SLV4_CTRL register (Register 52).
//I2C_LOST_ARB This bit automatically sets to 1 when the I2C Master has lost arbitration of the
//auxiliary I2C bus (an error condition). This triggers an interrupt if the
//I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
//I2C_SLV4_NACK This bit automatically sets to 1 when the I2C Master receives a NACK in a
//transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
//bit in the INT_ENABLE register (Register 56) is asserted.
//I2C_SLV3_NACK This bit automatically sets to 1 when the I2C Master receives a NACK in a
//transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
//bit in the INT_ENABLE register (Register 56) is asserted.
//I2C_SLV2_NACK This bit automatically sets to 1 when the I2C Master receives a NACK in a
//transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
//bit in the INT_ENABLE register (Register 56) is asserted.
//I2C_SLV1_NACK This bit automatically sets to 1 when the I2C Master receives a NACK in a
//transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
//bit in the INT_ENABLE register (Register 56) is asserted.
//I2C_SLV0_NACK This bit automatically sets to 1 when the I2C Master receives a NACK in a
//transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
//bit in the INT_ENABLE register (Register 56) is asserted.
typedef enum
{
	PASS_THROUGH_INT_SET = 0b1<<7,
	SLAVE_4_COMPLETE	= 0b1<<6,
	I2C_LOST_ARB		= 0b1<<5,
	I2C_SLV4_NACK		= 0b1<<4,
	I2C_SLV3_NACK		= 0b1<<3,
	I2C_SLV2_NACK		= 0b1<<2,
	I2C_SLV1_NACK		= 0b1<<1,
	I2C_SLV0_NACK		= 0b1<<0,
}MPU_MST_Status_t;

/* Private define ------------------------------------------------------------*/


//The DLPF is configured by DLPF_CFG. The accelerometer and gyroscope are filtered according to
//the value of DLPF_CFG as shown in the table below.
//DLPF_CFG Accelerometer
//(Fs = 1kHz)
//Gyroscope
//Bandwidth
//(Hz)
//Delay
//(ms)
//Bandwidth
//(Hz)
//Delay
//(ms)
//Fs (kHz)
//0 260 0 256 0.98 8
//1 184 2.0 188 1.9 1
//2 94 3.0 98 2.8 1
//3 44 4.9 42 4.8 1
//4 21 8.5 20 8.3 1
//5 10 13.8 10 13.4 1
//6 5 19.0 5 18.6 1
//7 RESERVED RESERVED 8

#define CONFIG_DLFP_0 0
#define CONFIG_DLFP_1 1
#define CONFIG_DLFP_2 2
#define CONFIG_DLFP_3 3
#define CONFIG_DLFP_4 4
#define CONFIG_DLFP_5 5
#define CONFIG_DLFP_6 6
#define CONFIG_DLFP_7 7

#define CONFIG_EXT_SYNC_DISABLE 0
#define CONFIG_EXT_SYNC_TEMP 	1<<3
#define CONFIG_EXT_SYNC_GYRO_X 	2<<3
#define CONFIG_EXT_SYNC_GYRO_Y 	3<<3
#define CONFIG_EXT_SYNC_GYRO_Z 	4<<3
#define CONFIG_EXT_SYNC_ACC_X 	5<<3
#define CONFIG_EXT_SYNC_ACC_Y 	6<<3
#define CONFIG_EXT_SYNC_ACC_Z 	7<<3

//GYRO_CONFIG Macros
#define GYRO_CONFIG_FSSEL_250DPS	0<<3
#define GYRO_CONFIG_FSSEL_500DPS	1<<3
#define GYRO_CONFIG_FSSEL_1000DPS	2<<3
#define GYRO_CONFIG_FSSEL_2000DPS	3<<3

#define GYRO_CONFIG_ST_EN_X	0b1<<7
#define GYRO_CONFIG_ST_EN_Y	0b1<<6
#define GYRO_CONFIG_ST_EN_Z	0b1<<5

//ACC_CONFIG Macros
#define ACC_CONFIG_AFSSEL_2G	0<<3
#define ACC_CONFIG_AFSSEL_4G	1<<3
#define ACC_CONFIG_AFSSEL_8G	2<<3
#define ACC_CONFIG_AFSSEL_16G	3<<3

#define ACC_CONFIG_ST_EN_X	0b1<<7
#define ACC_CONFIG_ST_EN_Y	0b1<<6
#define ACC_CONFIG_ST_EN_Z	0b1<<5

//FIFO EN
//Data stored inside the sensor data registers (Registers 59 to 96) will be loaded into the FIFO buffer if
//a sensor’s respective FIFO_EN bit is set to 1 in this register.
#define FIFO_EN_TEMP
#define FIFO_EN_XG
#define FIFO_EN_YG
#define FIFO_EN_ZG
#define FIFO_EN_ACC
#define FIFO_EN_SLV2
#define FIFO_EN_SLV1
#define FIFO_EN_SLV0

//I2C_MST_CTRL Macros
/*	Note On I2C MST_CTRL from MPU6050 Register Map:

	This register configures the auxiliary I2C bus for single-master or multi-master control. In addition, the
	register is used to delay the Data Ready interrupt, and also enables the writing of Slave 3 data into
	the FIFO buffer. The register also configures the auxiliary I
	2C Master’s transition from one slave read
	to the next, as well as the MPU-60X0’s 8MHz internal clock.
	Multi-master capability allows multiple I2C masters to operate on the same bus. In circuits where
	multi-master capability is required, set MULT_MST_EN to 1. This will increase current drawn by
	approximately 30µA.
	In circuits where multi-master capability is required, the state of the I2C bus must always be
	monitored by each separate I2C Master. Before an I2C Master can assume arbitration of the bus, it
	must first confirm that no other I2C Master has arbitration of the bus. When MULT_MST_EN is set to
	1, the MPU-60X0’s bus arbitration detection logic is turned on, enabling it to detect when the bus is
	available.
	When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be delayed until External
	Sensor data from the Slave Devices are loaded into the EXT_SENS_DATA registers. This is used to
	ensure that both the internal sensor data (i.e. from gyro and accel) and external sensor data have
	been loaded to their respective data registers (i.e. the data is synced) when the Data Ready interrupt
	is triggered.
*/


#define I2C_MST_CTRL_MULT_MST_EN 0b1<<7
#define I2C_MST_CTRL_MULT_MST_DIS  ~I2C_MST_CTRL_MULT_MST_EN
#define I2C_MST_CTRL_WAIT_FOR_ES_EN 0b1<<6
#define I2C_MST_CTRL_WAIT_FOR_ES_DIS ~I2C_MST_CTRL_WAIT_FOR_ES_EN
#define I2C_MST_CTRL_SLV3_FIFO_EN 0b1<<5
#define I2C_MST_CTRL_SLV3_FIFO_DIS ~I2C_MST_CTRL_SLV3_FIFO_EN
#define I2C_MST_CTRL_I2C_MST_P_NSR_RESTART 0b0<<4
#define I2C_MST_CTRL_I2C_MST_P_NSR_STOP 0b1<<4

//I2C_MST_CLK : Change the value in this macro to a number between 0 and 15: description below
//2C Master Clock
//Speed
//8MHz Clock
//Divider
//0 348 kHz 23
//1 333 kHz 24
//2 320 kHz 25
//3 308 kHz 26
//4 296 kHz 27
//5 286 kHz 28
//6 276 kHz 29
//7 267 kHz 30
//8 258 kHz 31
//9 500 kHz 16
//10 471 kHz 17
//11 444 kHz 18
//12 421 kHz 19
//13 400 kHz 20
//14 381 kHz 21
//15 364 kHz 22
#define I2C_MST_CTRLCLK 0

//I2C_SLAVE_X macros
#define I2C_SLVx_READ 0b1<<7
#define I2C_SLVx_WRITE ~I2C_SLAVE_X_READ
#define I2C_SLVx_EN 0b1<<7
#define I2C_SLVx_BYTE_SW_EN 0b1<<6
#define I2C_SLVx_REG_DIS 0b1<<5 //enabling will only process data in register and leave the address
#define I2C_SLVx_GRP 0<<4
//I2C_SLAVE4 MAcros
/*
 * Slave 4, is similar to slaves 1-3 however, it contains 2 extra registers
 * DI: Data read from the device is placed into this register
 * DO: Data to be written to the slave is placed here
 * The SLAVE_CTRL register also contains a bit for interrupts
 * I2C_MST_DLY: configures the reduced access rate of I2C slaves relative to the Sample Rate. When
 * a slave’s access rate is decreased relative to the Sample Rate, the slave is accessed every
 * 1 / (1 + I2C_MST_DLY) samples
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and DLPF_CFG
 * (register 26). Whether a slave’s access rate is reduced relative to the Sample Rate is determined by
 * I2C_MST_DELAY_CTRL (register 103)
 */
#define I2C_SLV4_INT_EN 0b1<<6

//INT_PIN_CFG

#define INT_PIN_CFG_LEVEL_LOW 0b1<<7 //active logic level for pin
#define INT_PIN_CFG_LEVEL_HIGH ~INT_PIN_CFG_LEVEL_LOW
#define INT_PIN_CFG_PIN_OPEN_DRAIN 0b1<<6
#define INT_PIN_CFG_PIN_PUSH_PULL ~INT_PIN_CFG_PIN_OPEN_DRAIN
#define INT_PIN_LATCH_INT_EN 0b1<<5 //pin held high untill interrupt is cleared (if set to 0, pin emits a 50 us pulse)
#define INT_PIN_INT_RD_CLEAR 0b1<<4
#define INT_PIN_FSYNC_INT_LEVEL_LOW 0b1<<3
#define INT_PIN_FSYNC_INT_LEVEL_HIGH ~INT_PIN_FSYNC_INT_LEVEL_LOW
#define INT_PIN_FSYNC_INT_EN 0b1<<2
#define INT_PIN_I2C_BYPASS_EN 0b1<<1

//INT_ENABLE
#define INT_ENABLE_FIFO_OFLOW_EN 0b1<<4
#define INT_ENABLE_I2C_MST_INT_EN 0b1<<3
#define INT_ENABLE_DATA_RDY_EN 0b1

//I2C_MST_DELAY_CTRL
#define I2C_MST_DELAY_CTRL_DELAY_ES_SHADOW 0b1<<7
#define I2C_MST_DELAY_CTRL_SLV4_DLY_EN 0b1<<4
#define I2C_MST_DELAY_CTRL_SLV3_DLY_EN 0b1<<3
#define I2C_MST_DELAY_CTRL_SLV2_DLY_EN 0b1<<2
#define I2C_MST_DELAY_CTRL_SLV1_DLY_EN 0b1<<1
#define I2C_MST_DELAY_CTRL_SLV0_DLY_EN 0b1

//SIGNAL_PATH_RESET
#define SIGNAL_PATH_GYRO_RESET 0b1<<2
#define SIGNAL_PATH_ACC_RESET  0b1<<1
#define SIGNAL_PATH_TEMP_RESET 0b1

//USER_CTRL
#define USER_CTRL_FIFO_EN 0b1<<6
#define USER_CTRL_I2C_MST_EN 0b1<<5
#define USER_CTRL_I2C_IF_DIS 0b1<<4//disabling interface allows for SPI mode
#define USER_CTRL_FIFO_RESET 0b1<<2
#define USER_CTRL_I2C_MST_RESET 0b1<<1
#define USER_CTRL_SIG_COND_RESET 0b1

//PWR_MGMT
/*
	An internal 8MHz oscillator, gyroscope based clock, or external sources can be selected as the
	MPU-60X0 clock source. When the internal 8 MHz oscillator or an external source is chosen as the
	clock source, the MPU-60X0 can operate in low power modes with the gyroscopes disabled.
	Upon power up, the MPU-60X0 clock source defaults to the internal oscillator. However, it is highly
	recommended that the device be configured to use one of the gyroscopes (or an external clock
	source) as the clock reference for improved stability. The clock source can be selected according to
	the following table.
	CLKSEL Clock Source
	0 Internal 8MHz oscillator
	1 PLL with X axis gyroscope reference
	2 PLL with Y axis gyroscope reference
	3 PLL with Z axis gyroscope reference
	4 PLL with external 32.768kHz reference
	5 PLL with external 19.2MHz reference
	6 Reserved
	7 Stops the clock and keeps the timing generator in reset

 */

#define PWR_MGMT_1_CLK_SEL_0 0
#define PWR_MGMT_1_CLK_SEL_1 1
#define PWR_MGMT_1_CLK_SEL_2 2
#define PWR_MGMT_1_CLK_SEL_3 3
#define PWR_MGMT_1_CLK_SEL_4 4
#define PWR_MGMT_1_CLK_SEL_5 5
#define PWR_MGMT_1_CLK_SEL_6 6
#define PWR_MGMT_1_CLK_SEL_7 7

/*
	By setting SLEEP to 1, the MPU-60X0 can be put into low power sleep mode. When CYCLE is set to
	1 while SLEEP is disabled, the MPU-60X0 will be put into Cycle Mode. In Cycle Mode, the device
	cycles between sleep mode and waking up to take a single sample of data from accelerometer at a
	rate determined by LP_WAKE_CTRL (register 108). To configure the wake frequency, use
	LP_WAKE_CTRL within the Power Management 2 register (Register 108).
 */

#define PWR_MGMT_DEVICE_RESET 0b1<<7
#define PWR_MGMT_1_SLEEP_EN 0b1<<6
#define PWR_MGMT_1_CYCLE_EN 0b1<<5
#define PWR_MGMT_1_TEMP_DIS 0b1<<4

//PWR_MGMT_2

//Description:
//This register allows the user to configure the frequency of wake-ups in Accelerometer Only Low
//Power Mode. This register also allows the user to put individual axes of the accelerometer and
//gyroscope into standby mode.
//The MPU-60X0 can be put into Accelerometer Only Low Power Mode using the following steps:
//(i) Set CYCLE bit to 1
//(ii) Set SLEEP bit to 0
//(iii) Set TEMP_DIS bit to 1
//(iv) Set STBY_XG, STBY_YG, STBY_ZG bits to 1
//All of the above bits can be found in Power Management 1 register (Register 107).
//In this mode, the device will power off all devices except for the primary I2C interface, waking only
//the accelerometer at fixed intervals to take a single measurement. The frequency of wake-ups can
//be configured with LP_WAKE_CTRL as shown below.

#define PWR_MGMT_2_LP_WAKE_CTRL_1_25HZ	0b00111111
#define PWR_MGMT_2_LP_WAKE_CTRL_5HZ		0b01 <<6
#define PWR_MGMT_2_LP_WAKE_CTRL_20HZ	0b10 <<6
#define PWR_MGMT_2_LP_WAKE_CTRL_40HZ	0b11 <<6
#define PWR_MGMT_2_STBY_XA				0b1<<5
#define PWR_MGMT_2_STBY_YA				0b1<<4
#define PWR_MGMT_2_STBY_ZA				0b1<<3
#define PWR_MGMT_2_STBY_XG				0b1<<2
#define PWR_MGMT_2_STBY_YG				0b1<<1
#define PWR_MGMT_2_STBY_ZG				0b1


/*
 * S Start Condition: SDA goes from high to low while SCL is high
* AD Slave I2C address
* W Write bit (0)
* R Read bit (1)
* ACK Acknowledge: SDA line is low while the SCL line is high at the
* 9th clock cycle
* NACK Not-Acknowledge: SDA line stays high at the 9th clock cycle
* RA MPU-60X0 internal register address
* DATA Transmit or received data
* P Stop condition: SDA going from low to high while SCL is high
 */
#define I2C_SLAVE_ADDRESS_LOW 0xD0
#define I2C_SLAVE_ADDRESS_HIGH 0b1101001
#define I2C_WRITE_BIT 0b1
#define I2C_READ_BIT  0b0
#define BASE_REGISTER_RESET_VALUE 0x00
#define PWR_MGMT_RESET_VALUE 0x40
#define WHO_AM_I_VALUE  0x68

/* USER CODE END PD */



#endif /* HAL_MPU6050_H_ */

/*
 * Sharc_Frame.h
 *
 *  Created on: Jun 6, 2020
 *      Author: jamie
 */

#ifndef SHARC_FRAME_H_
#define SHARC_FRAME_H_

/* Private includes ----------------------------------------------------------*/

#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "HAL_GPS.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "HAL_Iridium.h"

/* Private typedef -----------------------------------------------------------*/

/*
 * 							PWR_MODE_t
 *
 * @brief: enumeration of possibled deep sleep modes. To be used as input for function
 * 		   static void Go_To_Sleep(PWR_MODE_t mode, uint32_t seconds)
 * @param SHUTDOWN - enter shutdown mode (0kb Ram etention, Power circuitry disabled)
 * @param STDBY	   - enter Standby mode (32KB RAM retention, power circuitry enabled)
 */

typedef enum
{
	SHUTDOWN,
	STDBY
}PWR_MODE_t;

/*
 * 							Buoy_State_typedef
 *
 * @brief: enumeration of states. Here, each abstract state is enumerated with a unique
 * 		   value. States are set in and read from byte 0 of RTC Back Up register 0. When
 * 		   the device enter shutdown/ standby mode, the back up domain is still supplied
 * 		   allowing for 1Kb of data retention in powerdown mode. More information is given
 * 		   in the document  State Machine Design.pdf
 *
 *@param  STATE_INIT		- Initialisation Mode:		Calibration and initialisation routines for first time start up
 * @param STATE_RESET		- Reset Mode:				system initailisad
 * @param STATE_SAMPLE  	- Data acquisition Mode:	Sensors initialised and sampled
 * @param STATE_SLEEP		- SLEEP MODE:				Buoy is in power saving mode with wake up pins active
 * @param STATE_TRANSMIT	- Transmit Mode:			Iridium Modem  activated and transmitted/recieved
 * @param STATE_ASYNCINT	- Asynchronous Interrupt:	Device recieved rising edge on one of 2 interrupt pins resulting in the activation of asynchronous routines
 */
typedef enum
{
	STATE_INIT			= 0b00,
	STATE_RESET 		= 0b01,
	STATE_SAMPLE 		= 0b10,
	STATE_SLEEP 		= 0b11,
	STATE_TRANSMIT  	= 0b110,
	STATE_ASYNCINT		= 0b111
}Buoy_State_typedef;

/*
 * @brief: enum structure contains values to be used as input to the function  void set_WUP_Pin(uint32_t Pin,PinMode_typedef mode),
 * 	 	   Used to set the type interrupt mode for the desired pin. Note: Functionality changes depending on the power mdoe
 * 	 	   of the buoy. Make sure the wake up pins are reconfigured after each power mode transition
 *
 * @param:  MODE_EXTI - Pin will be configured as an external interrupt pin on a rising trigger.
 *
 * @param:  MODE_WUP  - Pin will be configured as an external wake up pin
 */

typedef enum
{
   MODE_WUP,
   MODE_EXTI
} PinMode_typedef;

/*
 * @brief: Structure to store data from GPS in an organised format. Note: custom data types from HAL_GPS.h
 */
typedef struct
{
	uint32_t Etime;			// UTC Epoch representation of time
	Coord_t  coordinates;	//GPS coordinates
	Diagnostic_t diag;		//Diagnostic information
	uint32_t env_Temp;		//environmental temperature
	int32_t  atm_Press;		//atmospheric pressure
}GPS_Data_t;


//============
/* Private Macro Functions ------------------------------------------------------------*/

//The following lines of code allow for printf statements to output to serial via USART2. Remove code if not neccessary
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

// Custom flag set detction for Event Power on Reset (PWR_BORST, PWR_SWRST, PWR_NRST Set )
#define __HAL_RCC_GET_PORRST_FLAG() ((READ_REG(RCC->CSR)&(RCC_FLAG_PORRST))>>26)&&0b111

// Time Conversion Functions to seconds for use in function:
//static void Go_To_Sleep(PWR_MODE_t mode, uint32_t seconds)

#define __MINS_TO_SECS(x) (x)*60	//for Twake > 60 seconds

#define __HRS_TO_SECS(x) (x)*3600   //for Twake > 60 mins

#define __DAYS_TO_SECS(x) (x)*__HRS_TO_SECS(24) //for Twake > 1 day

#define __WEEKS_TO_SECS(x) x*__DAYS_TO_SECS(7)  //for Twake > 6 days

/*
 * @brief: state machine Macros
 */
//Returns the state the buoy was in prior to state check
#define __GET_PREV_STATE() (RTC->BKP0R)&0xFF

#define __SET_CURRENT_STATE(state) WRITE_REG(RTC->BKP0R,(RTC->BKP0R &0xFF00)|(state))

#define __GET_SAMPLE_COUNT()  ((RTC->BKP0R)&0xFF00)>>8

#define __SET_SAMPLE_COUNT(count)  WRITE_REG(RTC->BKP0R,(RTC->BKP0R &0xFF)|((count)<<8))

/* Private define ------------------------------------------------------------*/


/*
 * Wake up pin definitions: change these to the pins you are using
 */

// Iridium Ring Pin Interrupt line on Pin PC13

//PWR Definitions
#define IRIDIUM_RING_WAKE_FLAG PWR_FLAG_WUF2	//flag definition

#define IRIDIUM_RING_WAKE_PIN PWR_WAKEUP_PIN2	//Pin Definition

//EXTI Definitions
#define EXTI_IRIDIUM_RING_WAKE_PIN GPIO_PIN_13	//pin definition

#define EXTI_IRIDIUM_RING_WAKE_PORT GPIOC		//Port Definitions
//IMU Interrupt Line on Pin PC 5

//PWR Definitions
#define IMU_EVENT_WAKE_FLAG PWR_FLAG_WUF5	//flag definition

#define IMU_EVENT_WAKE_PIN PWR_WAKEUP_PIN5  //Pin Definition

//EXTI Definitions
#define EXTI_IMU_EVENT_WAKE_PIN		GPIO_PIN_5			//Pin Definition

#define EXTI_IMU_EVENT_WAKE_PORT	GPIOC				//Port Definition
/*
 * Debug Defines: These macros enable/ disable debuggin functions for the buoy such as testing the clock input
 *
 */

/*
 * @brief: enables function to map the Master Clock output to Pin PA8
 */
#define DEBUG_HSE_OUTPUT_ENABLE
/*
 * @brief: enables initialisation of USART2 to allow buoy to print data to  computer
 */
#define DEBUG_USART_ENABLE
/*
 * @brief: enables initialisation of LED1: on the NUCLEO-L4 development board
 */
#define DEBUG_LED1_ENABLE
/*
 * @brief: enables Debug for low power mode allowing the debugger to still work when mcu is in stop, sleep or stdby mode
 */
#define DEBUG_LP_ENABLE

#define RCC_FLAG_PORRST (0b101<<26)

//Optional: define deep sleep wake up period

#define RTC_WUCK_Period 1799 //s

//define GPIO Port for LED
#define LD2_GPIO_Port GPIOA

//define GPIO Pin for LED
#define LD2_Pin GPIO_PIN_5

//define Number of onboard FLASH Chips
#define FLASH_CHIPS 4

//define Size of Drift data payload
#define DRIFTBUFFER_SIZE 29

/* Private variables ---------------------------------------------------------*/

extern RTC_HandleTypeDef hrtc;
RTC_DateTypeDef hdate;
RTC_TimeTypeDef htime;
//__IO uint8_t in_Shutdown = 0;
#ifdef DEBUG_USART_ENABLE
UART_HandleTypeDef huart2;
#endif

//uint32_t time;
Buoy_State_typedef Current_State;
uint8_t Sample_On,sample_count;

/* Private function prototypes -----------------------------------------------*/

/*
 * Function Name: static void Init_Debug(void);
 *
 * @brief: Function to initialise debugging capabilities based on macro definitions
 *
 * @param: void
 *
 * @return: HAL_StatusTypeDef - return status of the function based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef Init_Debug(void);

/*
 * Function name HAL_StatusTypeDef SystemClock_Config(void);
 *
 * @brief: Clock configuration function: Configures the sysclk
 * 		   to 26MHz using the MSI and LSE in a Phase Lock Loop
 * 		   Secondary clock source HSI for wake up from stop mode
 *
 * @param: void
 *
 * @return: HAL_StatusTypeDef - return status of the function based on HAL_StatusTypeDef
 */
HAL_StatusTypeDef SystemClock_Config(void);
/*
 * Function Name: static void GPIO_Set_Pin_LP(void);
 * @brief: Sets all GPIO Pins to Analog Mode No Pull Up for lowest possible current consumption
 *
 * @param:	void
 *
 * @return: void
 */

void GPIO_Set_Pin_LP(void);

/*
 * Function Name: static void USART_Enter_Standby_for_data(UART_HandleTypeDef *huart);
 * @brief: Sets device power mode to stop mode and sets the UART to wake the device on
 * 		   reception of a start bit on the Rx Line. This allows the device to conserve
 * 		   additional power during GPS/Iridium data reception allowing the device to
 * 		   conserve additional power (see Power Mode and Clock Selction / STM32L476 Wake Up from Stop Mode)
 *
 * @param:	UART_HandleTypeDef *huart:	Pointer to UART instance to be configured
 *
 * @return: HAL_StatusTypeDef
 */

HAL_StatusTypeDef USART_Enter_Standby_for_data(UART_HandleTypeDef *huart);

/*
 * Function Name: void set_WUP_Pin(uint32_t Pin,PinMode_typedef mode);
 * @brief: This function initializes a pin to be used as an interrupt source to wake up the device from sleep mode
 * 			There are 5 wake up pins on the device. The following table shows the wake up pin mapping to GPIO pin port and
 * 			the input argument to the function to enable it
 * 			+------------------------------------------+
 * 			| Wake Up Pin | GPIO Pin | Argument        |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 1   | PA0      | PWR_WAKEUP_PIN1 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 2   | PC13     | PWR_WAKEUP_PIN2 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 3   | PE6      | PWR_WAKEUP_PIN3 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 4   | PA2      | PWR_WAKEUP_PIN4 |
 * 			|-------------|----------|-----------------|
 * 			| WUP PIN 5   | PC5      | PWR_WAKEUP_PIN5 |
 * 			+------------------------------------------+
 *
 * @param: uint32_t Pin: Pin to be configured (Must be 1 value from the arguement column in the table above)
 *
 * @return: void
 */

void set_WUP_Pin(uint32_t Pin,PinMode_typedef mode);

/*
 * Function Name: HAL_StatusTypeDef Go_To_Sleep(PWR_MODE_t mode, uint32_t seconds)
 *
 * @brief: This function is the routine that places the device into sleep mode
 * 		   The user specifies how deep the sleep is and for how long. 2 wake up sources
 * 		   are set: An internal wake up timer routed through EXTI line 20 and 2 external
 * 		   wake up pins. The wake up source is set as flag in the PWR_SR register
 *
 * @param: PWR_MODE_t: The sleep mode to be set as defined in the PWR_MODE_t enum (STDBY/SHUTDOWN)
 *
 * @param: uint32_t seconds: Sets the sleep period in seconds.
 *
 * @return: HAL_StatusTypeDef
 */
HAL_StatusTypeDef Go_To_Sleep(PWR_MODE_t mode, uint32_t seconds);

/*
 * Function Name: void Get_Current_Address_Pointer(uint8_t chip,uint8_t* address_Array);
 *
 * @brief: Returns the address of the latest available memory block in a Flash chip. The
 * 		   Address for each chip is a 3 byte long array stored in Big Endian Format in
 * 		   the RTC Back up registers. Each reigster is 32 bits long and the register memory
 * 		   is allocated as follows:
 * 		   RTC->BKUP_2	FLASH CHIP 1
 * 		   RTC->BKUP_3	FLASH CHIP 2
 * 		   RTC->BKUP_4	FLASH CHIP 3
 *
 * 		   Note: In order to access the back up registers, RCC must be enabled to the PWR before
 * 		   any opperation takes place
 *
 * @param: chip: Number of the flash chip to get the address for (must be in range 1 - 4)
 *
 * @param: address_Array: Pointer to a 3 byte long uint8_t array to hold the address
 *
 * @return: uint8_t: status of read function, 1 is success, 0 is fail
 */
uint8_t Get_Current_Address_Pointer(uint8_t chip,uint8_t* address_Array);

/*
 * Function Name: uint8_t Set_Current_Address_Pointer(uint8_t chip,uint8_t* address_Array);
 *
 * @brief: Stores the latest Available memory address of a specified flash chip in the RTC Back-Up Register.
 * 		   The Address for each chip is a 3 byte long array stored in Big Endian Format in
 * 		   the RTC Back up registers. Each reigster is 32 bits long and the register memory
 * 		   is allocated as follows:
 * 		   RTC->BKUP_2	FLASH CHIP 1
 * 		   RTC->BKUP_3	FLASH CHIP 2
 * 		   RTC->BKUP_4	FLASH CHIP 3
 *
 * @param: chip: Number of the flash chip to get the address for (must be in range 1 - 4)
 *
 * @param: address_Array: Pointer to a 3 byte long uint8_t array to hold the address
 *
 * @return: uint8_t: status of read function, 1 is success, 0 is fail
 */
uint8_t Set_Current_Address_Pointer(uint8_t chip,uint8_t* address_Array);
/*
 *  POWER ON RESET HANDLER:
 *
 *  Routine that runs when device encounters a Power on Reset Event
 *
 */

/*
 * Function Name: uint8_t Get_Active_Chip(void);
 *
 * @brief: Returns the active chip number from RTC->BKUP1 Register
 *
 * @param: void
 *
 * @return: uint8_t chip number 1 < 4, Error (0) if number outside this range
 */
uint8_t Get_Active_Chip(void);

/*
 * Function Name: void Set_Active_Chip(uint8_t chipnumber);
 *
 * @brief: Stores the number of the active chip in the RTC->BKUP1 Register
 *
 * @param: uint8_t chipnumber: integer number of the chip to be used
 *
 * @return: void
 */
void Set_Active_Chip(uint8_t chipnumber);

uint8_t Get_Next_Active_Chip(void);

void Set_Next_Active_Chip(uint8_t chipnumber);
/*
 * Function Name: uint8_t* to_binary_format(GPS_Data_t gps_data ,uint8_t ID);
 *
 * @brief: Converts the data in the GPS_Data_t and
 */

uint8_t* to_binary_format(GPS_Data_t gps_data ,uint8_t ID);

/*
 * Function Name: uint8_t  get_driftBuffer_Size(void);
 */
uint8_t  get_driftBuffer_Size(void);

/**
  * @brief Place code for Iridium Message Receive event here
  * @param None
  * @retval None
  */

void Routine_ASYNC_IRIDIUM_RX(void);

void Routine_Async_IMUevent_Sample(void);

void POR_Handler(void);

/*
 * BROWN OUT RESET HANDLER:
 *
 * NB: Device must have Vbrownout threshold set in option bytes
 *
 *  Routines that runs when device recovers from a brown out Vbat < Vbrountout
 */
void BOR_Handler(void);

#endif /* SHARC_FRAME_H_ */

/*
 * HAL_GPS.h
 *
 *  Created on: Mar 11, 2020
 *      Author: Jamie Jacobson
 *      Student No: JCBJAM007
 *      For: University of Cape Town
 *
 *  This library is designed to be used with the STM Hal libraries. Written for
 *  version 1.15.1 Note: This version is compatible with version 1.14.x
 *
 *  The library is designed to communicate with a U-Blox Neo GPS through USART
 *  Communication at 115200 bit/s 8 data bits 1 stop bit. Data is transmitted and
 *  Received via DMA with signaling flags to show the status of such transactions.
 *
 *  Ublox Neo transmits data in the form of NMEA message strings. For the purposes
 *  of this project, all NMEA messages are turned off except for GLL (positioning data),
 *  GSA (diagnostic information), ZDA( time information)
 *
 *  Configurations are performed using UBX commands (refer to the U-Blox Neo 7 receiver description guide)
 *  A Timer has configured in slave reset mode to signal the following has occurred:
 *
 *  1. No data has been received after a predefined period of time (# of Data bytes < DMA Buffer Size )
 *  2. The Device has timed out on communications (# of Data bytes == 0)
 *
 *  The library contains the following:
 *
 *  1. Init/ De init functions
 *  2. GPS Module configuration functions
 *  3. NMEA Message Parsers
 *  4. UBX configuration functions
 *
 */

#ifndef HAL_GPS_H_
#define HAL_GPS_H_

/* The following macros allow for the time.h library to be ported to embedded systems*/

#define _XOPEN_SOURCE // Definition for time.h to work
#define ARM_MATH_CM4  // Also defined for the time.h embedded implementation

//============================= 1. Includes ==============================================

#include <stm32l4xx_hal.h> //StandardPeriph Driver Header
#include "string.h"	   // for string handlings
#include "stdio.h"
#include "stdlib.h"
#include <math.h>  //Header for additional Math Functions
#include <time.h>		// contains function to convert UTC to Epoch Time

//========================== 2. Structs & Enums ===========================================

typedef enum{
	UBX_OK = 2,
	UBX_ACK_ACK = 1,
	UBX_ACK_NACK = 0,
	UBX_ERROR = -1,
	UBX_TIMEOUT_Tx = -2,
	UBX_TIMEOUT_Rx = -3,

} UBX_MSG_t;

typedef enum{
	GPS_Init_OK = 1,
	GPS_Init_Ack_Error = 2,
	GPS_Init_Baud_Config_Error = 3,
	GPS_Init_MSG_Config_Error = 4,
	GPS_Init_Ack_Tx_Error = 5,
	GPS_Init_Periph_Config_Error = 6,
	GPS_Init_Offline_Error = 7

}GPS_Init_msg_t;


/*
 * Coordinate Object
 *
 * Stores the Cordinates of GPS in the form DDMM.mmmm
 * where DD - Degrees
 * 	     MM - Minutes
 * 	   mmmm - Fractional minutes
 * Variables:	Name.............Type.................................Description
 * 				lat..............float32_t............................GPS Lattitude
 * 				longi............float32_t............................GPS Longitude
 */
typedef struct
{
	float_t lat;
	float_t longi;
}Coord_t;
/*
 * DOP Object
 *
 * Dilation of Precision is a metric of how the elevation, satelite number and satelite spread
 * affects the accuracy of the signal. This object allows for the conversion and storage of a
 * 32 bit float value representing a DOP into an 8 bit digit and 8 bit precision the range of
 * values outputted from the GPS is between 0,0 and 99.9. This will allow for accuracy up to 2
 * decimal places.
 *
 * Variables:	Name.............Type.................................Description
 * 				digit............uint8_t..............................8 bit representation of the whole number
 * 				precision........uint8_t..............................2 significant places representation of the fraction
 *
 */
typedef struct
{
	uint8_t digit;
	uint8_t precision;
}DOP_t;

/*
 * Diagnostic Object
 *
 * Structure to Hold the GPS data signal diagnostics
 *
 * Variables:	Name.............Type.................................Description
 * 				PDOP.............DOP_t................................Positional Dilation of Precision (3D)
 * 				HDOP.............DOP_t................................Horizontal Dilation of Precision
 * 				VDOP.............DOP_t................................Vertical   Dilation of Precision
 * 				num_sats.........uint8_t..............................Number of Satelites used to obtain positional Fix
 * 				fix_type.........uint8_t..............................number between 1-3 describing the type of fix obtained
 *
 * Fix types
 * 1 - No Fix
 * 2 - 2D Fix (No altitude)
 * 3 - 3D Fix
 */

typedef struct
{
	DOP_t PDOP;
	DOP_t HDOP;
	DOP_t VDOP;
	uint8_t num_sats;
	uint8_t fix_type;
}Diagnostic_t;

/*
 * GPS Sensor Handler
 *
 * This is the default struct that interfaces with the hall library. The struct contains the following:
 *  a pointer to a communication handler (UART), a pointer to a dma memory handler (DMA), a pointer to
 *  a timing (handler) for timing. This will allows the library to effectively handle circular buffer
 *  communication in an effective manner. In addition, the handler contains pointers to memory buffers
 *  to allow data to be read from/ written to the sensor.
 *
 *  When using this library, declare a GPS_Handle_Typedef object as a global varaible. then pass it as
 *
 *  a parameter to the functions in this library
 *
 *  Variables
 */
typedef struct
{
	UART_HandleTypeDef* gps_huart;		/* Pointer to UART Instance */

	DMA_HandleTypeDef*  gps_hdmamem;    /* Pointer to DMA MEM 2 MEM Handler */

	TIM_HandleTypeDef*  gps_htim;		/* Pointer to TIM Handler (Note: Must be a general purpose timer with atleast 2 channels)*/

	uint8_t*	GPS_Rx_Buffer;			/* Pointer to buffer to hold incoming data from GPS peripheral */

	uint8_t*	GPS_Tx_Buffer;			/* Pointer to buffer to hold outgoing data to GPS periphera */

	uint8_t*    GPS_Mem_Buffer;			/* Pointer from data saved from GPS_Rx_Buffer */

	GPS_Init_msg_t status;				/* Variable to hold the status of peripheral functions */

}GPS_Handle_Typedef;

//======================== 3. Macro Definitions =========================================

/* TIM Defines */
#define GPS_TIM_PORT TIM2					//General Purpose Timer

#define GPS_TIM_OC_CHANNEL TIM_CHANNEL_1	//channel to hold output compare value

#define GPS_TIM_IC_CHANNEL TIM_CHANNEL_2	//channel for input capture

#define CCR1_VAL 8000						//Counter value

/* UART Defines */
#define GPS_UART_PORT UART4					//USART GPS Port

#define GPS_TX_Pin GPIO_PIN_10				//USART Tx GPIO Pin

#define GPS_TX_GPIO_Port GPIOC				//USART Tx GPIO Port

#define GPS_RX_Pin GPIO_PIN_11				//USART Rx GPIO Pin

#define GPS_RX_GPIO_Port GPIOC				//USART Rx GPIO Port

/* Note: Some USART peripherals are mapped to pins on different io ports. These are available in the AF table in the reference manual */

/* Data Buffers*/
#define DMA_TX_BUFFER_SIZE 2048		//size in bytes of Transmission buffer

#define DMA_RX_BUFFER_SIZE 2048		//size in bytes of Receive Buffer

#define GNSS_BUFFER_SIZE 2048		//size in bytes of DMA Memory Buffer

/* Other Defines*/

#define GPS_INIT_RETRIES 100 		//maximum number of attempts to receive an acknowledgment


//========================== 4. Global Variables ==========================================

/* Private Variables */
GPS_Handle_Typedef hgps; 			// GPS Handler Instance

Coord_t GPS_coord;					//storage variable for coordinates from GPS

uint32_t eTime;						//storage variable for Epoch Time

Diagnostic_t diag;					//storage variable for Diagnostic information

uint8_t M2M_Txfer_Cplt;				//Signaling flag to show status of DMA Memory to Memory transfer

uint8_t TIM_IDLE_Timeout; 			//Flag used to indicate whether a timeout has occurred

uint8_t TX_Cplt;					//Flag used to indicate whether USART DMA Transmission has been completed

int gnss_length;					//length of data in Iridium Rx Buffer when timeout occurs or idle line has been detected

uint8_t packet_full;				//shows whether the full set of gps data has been captured

uint8_t log_gps;					//flag set by user. Starts logging gps data to memory

//============================= 5. Handlers =============================================


DMA_HandleTypeDef hdma_uart4_rx;				//Handle DMA Channel for USART Rx

DMA_HandleTypeDef hdma_uart4_tx;				// Handle DMA Channel for USART Tx

TIM_HandleTypeDef htim2;						//GPS Timeout Handler

UART_HandleTypeDef huart4;						//GPS UART Communication Handler

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;  //Handle DMA GPS Memory to Memory Handler

//============================ 6. Data Buffers ==========================================

/* Private Buffers */
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];		// Buffer for Data from GPS

uint8_t GNSS_Buffer[GNSS_BUFFER_SIZE];			// Buffer for Data from Memory

uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];		// Buffer for data to GPS

//======================== 7. Peripheral Functions Prototypes =======================================
//Peripheral functions

/*
 * Function Name: HAL_StatusTypeDef USART_Set_Baudrate(GPS_Handle_Typedef *hgps,uint32_t baud);
 *
 * @brief: reconfigure the baud rate settings for USART
 *
 * @param *hgps - Pointer to GPS handle Object
 * @param baud - new baud rate
 *
 * @retval HAL_StatusTypeDef: status of function
 */

HAL_StatusTypeDef USART_Set_Baudrate(GPS_Handle_Typedef *hgps,uint32_t baud);

/*
 * Function Name: GPS_Init_msg_t init_GPS(GPS_Handle_Typedef *hgps);
 *
 * @brief: 	Initialize UART peripheral for communication with GPS
 *
 * 		   	UART SETTIINGS:
 * 							baud rate:	9600 bit/s
 * 							data bits:	8	 bits
 * 							stop bits:	1	 bit
 * 							parity:		None
 *
 *			Upon successful configuration, The device requests an acknowledgment from the GPS.
 *			This check acts to verify that a sensor is connected and acting properly.
 *			Should a device return an acknowledgment, the GPS module must be configured
 *			as such:
 *
 *			GPS SETTINGS:
 *
 *							baud rate:	115200 bit/s
 *							data bits:	8	   bit
 *							stop bits:	1	   bit
 *							parity:		None
 *							MESSAGES:	ZDA, GLL, GSA Active
 *
 *			If the buoy is already configured to the new UART settings then the previous step is not
 *			necessary however, The micro-controller UART peripheral will be reinitialized with the new settings
 *			If the buoy fails to transmit an acknowledgment, the init_function returns a failure code
 *			(as defined in GPS_Init_MSG_t) and the peripheral will be deinitialized. The user can add in code to
 *			retry the function when the exit failure occurs
 *
 * @param:  *hgps - Pointer to GPS Handler Object
 *
 * @return: GPS_Init_msg_t - return status of the function based on GPS_Init_msg_t
 */

GPS_Init_msg_t init_GPS(GPS_Handle_Typedef *hgps);

/*
 * Function Name:  GPS_Init_msg_t deinit_GPS(GPS_Handle_Typedef* hgps);
 *
 * @brief: De-initializes the GPS communication peripherals
 *
 * @param:  *hgps - Pointer to GPS Handler object
 *
 *
 * @return: GPS_Init_msg_t - return status of the function based on GPS_Init_msg_t
 */
GPS_Init_msg_t deinit_GPS(GPS_Handle_Typedef* hgps);

/*
 * Function Name: UBX_Send_Ack(GPS_Handle_Typedef *hgps)
 *
 * @brief: Function used to check if device is working. Sends an acknowledgement
 * 		   string in UBX protocol format. Waits for GPS to return an acknowledgement
 *
 * @param: hgps: GPS handler object
 *
 * @return: UBX_MSG_t - status of function
 */

//======================== 8. UBX Tx/Rx Functions Prototypes =======================================

UBX_MSG_t UBX_Send_Ack(GPS_Handle_Typedef *hgps);
/*
 * Function Name: UBX_MSG_t UBX_Configure_Baudrate(GPS_Handle_Typedef *hgps);
 *
 * @brief: reconfigures UART communications for a speed of 115200 bit/s
 * 		   transmits a configuration string and reconfigures the uart peripheral
 * 		   to the new baud rate. returns an error if unsuccessful
 * @param: *hgps: Pointer to GPS Handler Object
 *
 * @return: UBX MSG_t status of ubx transmission function
 */
UBX_MSG_t UBX_Configure_Baudrate(GPS_Handle_Typedef *hgps);

/*
 * Function Name: UBX_MSG_t UBX_Configure_Messages(UART_HandleTypeDef *huart);
 *
 * @brief: transmits a UBX configuration string to disable all NMEA output messages a
 * 		   except for GLL, GSA, ZDA messages.
 * 		   Note: You can substitute the message string with a custom one provided it
 * 		   is formatted according to the protocols in the UBlox Neo 7m - Receiver Manual
 *
 * @param: *hgps: Pointer to GPS Handler Object
 *
 * @return: UBX MSG_t status of ubx transmission function
 */
UBX_MSG_t UBX_Configure_Messages(GPS_Handle_Typedef *hgps);

/*
 * Function Name: void GPS_Log_Begin(void);
 *
 * @brief: Prepares the device to recieve data
 *
 * @param: void
 *
 * @return: void
 */

//======================== 9. Utility Function Prototypes =======================================

/*
 * Function Name: void GPS_Log_Begin(void);
 *
 * @brief: Enables data logging
 *
 * @param: void
 *
 * @return: void
 */
void GPS_Log_Begin(GPS_Handle_Typedef* hgps);

/*
 * Function Name: void GPS_Log_Stop(void);
 *
 * @brief: Disables data logging
 *
 * @param: void
 *
 * @return: void
 */
void GPS_Log_Stop(GPS_Handle_Typedef* hgps);

/*
 * Function Name: void  Clear_Buffer(uint8_t *buffer,uint32_t size);
 *
 * @brief: sets all data points in an array to NULL
 *
 * @param: buffer - Pointer to uint8_t array
 * @param: size	  - size of the array
 *
 * @return: void
 */

void  Clear_Buffer(uint8_t *buffer,uint32_t size);

/*
 * Function Name: uint8_t char_to_hex(char c);
 *
 * @brief: takes in a char hex digit and converts it to an integer
 *
 * @param: c - ASCII charachter
 *
 * @return uint8_t - Integer representation of the digit
 */
uint8_t char_to_hex(char c);

/*
 * Function Name: uint8_t is_valid(char* nmeamsg);
 *
 * @brief: Checks NMEA message string to determine if the message was transmitted
 * 			with no data corruption. Returns a value that identifies what the message
 * 			is based off the 5 letter identifier tag GxYYY (x is the satelite constellation,
 * 			YYY is a 3 letter combination that tells what the message contains)
 *
 * @param: nmeamsg - NMEA message string
 *
 * @return: uint8_t - flag showing result of function
 *
 * 			values: -1		- 		invalid message string
 * 					 1		-		GLL Message
 * 					 2		-		GSA Message
 * 					 3		-		ZDA Messages
 *
 */
uint8_t is_valid(char* nmeamsg);

/*
 * Function Name: uint8_t parse_ZDA(char* ZDAstring);
 *
 * @brief: Extracts the time and date information from a ZDA message string and converts it to Unix Epoch Time
 * 		   Data is stored in a global uint32_t variable eTime
 *
 * @param ZDA string - NMEA time message
 *
 * @return uint8_t - function status
 */
uint8_t parse_ZDA(char* ZDAstring);

/*
 * Function Name: uint8_t parse_GLL(char* GLLstring);
 *
 * @brief: Extracts Global Positioning data from the message and stores it in a Global Coord_T variable:
 * 			Note, the string does not contain any altitude data. This is acceptable as user requirements
 * 			indicate that altitude measurements are unreliable for this scope of work
 *
 * @param GLL string - Global Latutude Longitude message
 *
 * @return uint8_t - function status
 */
uint8_t Parse_GLL(char* GLLstring);
/*
 * Function Name: uint8_t parse_GSA(char* GSA_string);
 *
 * @brief: Extracts Diagnostic data from the message and stores it in a Global Diagnostic_t variable:
 *
 * @param GSA string - Diagnostic information featuring nubmer of satelites, DOP and fix type
 *
 * @return uint8_t - function status
 */
uint8_t parse_GSA(char* GSA_string);

//======================== 10. IRQ Handlers Function Prototypes =======================================

/*
 * @brief: Peripheral Handler for GPS USART
 * 		   Handlers for:
 * 		   IDLE Line Detection		-		detects when the device has finished receiving data and turns off DMA Peripheral Stream
 * 		   Error Detection			-		Overruns, Noise, Parity and Frame error handling
 * 		   Transmission Complete	-		Turns off Tx DMA stream when TXE/ TC interrupt occurs
 *
 * @param: hgps - Pointer to GPS Handler Object
 *
 * @return: void
 */

void USART_GPS_IRQHandler(GPS_Handle_Typedef *hgps);

/*
 * @brief: Mem 2 Mem stream DMA Handler for Memory Stream from GPS_Rx_Buffer to GPS_Mem_Buffer
 *
 * @param: hgps - Pointer to GPS Handler Object
 *
 * @return: void
 */

void DMA_GNSS_MEM_IRQHandler(GPS_Handle_Typedef *hgps);

/*
 * @brief: Periph 2 Mem stream DMA Handler for Memory Stream from USART Rx register to GPS_Rx Data Buffer
 *
 * @param: hgps - Pointer to GPS Handler Object
 *
 * @return: void
 */

void DMA_GNSS_Periph_IRQHandler(GPS_Handle_Typedef *hgps);
/*
 * @brief: TIM2 Interrupt Handler. Set to handle Update Interrupts and CApture Compare Interrupts on Channel 1
 *
 * @param: htim - Pointer to General Purpose Timer Handler Object
 *
 * @return: void
 */

void USART_TIM_RTO_Handler(TIM_HandleTypeDef *htim);

//========================================= END ======================================================

#endif /* HAL_GPS_H_ */

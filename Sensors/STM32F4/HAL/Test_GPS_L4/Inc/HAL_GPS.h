/*
 * HAL_GPS.h
 *
 *  Created on: Dec 9, 2019
 *      Author: jamie
 */



/*
 * HAL_GPS.h
 *
 *
 *  Created on: Dec 09, 2019
 *      Author: Jamie Jacobson
 *      Std No: JCBJAM007
 *
 *  Description: This File contains all the macros, function definitions and typedefs used
 *  in GPS.c It is based off the SPL architecture.
 *
 *  The USART is configured on a User defined USART Port. Make sure to update the Macros
 *  to match the corresponding Ports and Peripherals
 *
 *  This library is designed to communicate with The GPS By polling data for Transmission
 *  and Receiving data using The Direct Memory Access Controller (DMA) in a Circular Buffer
 *  This allows for efficient reception of variable length data.
 *
 *  Each Peripheral is mapped to The DMA through a Channel and A stream (page 207 STM32F446RE reference manual)
 *  Each stream can be configured as either Peripheral to Memory, Memory to Peripheral or Memory to Memory
 *  The DMA is a dual configuration with Memory to Memory access only allowed in DMA2
 *
 *  For this library, 3 streams have been set up for: USART Transmission, USART Reception, Memory - Memory Transfer
 *
 *
 */

/* Additional Macros*/
#define _XOPEN_SOURCE // Definition for time.h to work
#ifndef HAL_GPS_H_
#define HAL_GPS_H_
#define ARM_MATH_CM4  // Also defined for the time.h embedded implementation

//======================================================================================
/* Includes */
#include <stm32l4xx_hal.h>  //StandardPeriph Driver Header
#include "string.h"	   		// for string handlings
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include <math.h>  			//Header for additional Math Functions
#include <time.h>			// contains function to convert UTC to Epoch Time
							//Import time functions for timeout
//=======================================================================================
/* private structs */

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
	float lat;
	float longi;
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
 *  UBX_MSG_t: enumaration representing the returned message from the GPS
 *  Description: When any UBX message is sent to the receiver, The receiver
 *  			responds with an acknowledgment. IF a message is successfully received
 *  			an acknowledgment is sent otherwise a NACK is sent,
 *  			UBX messages contain a checksum at the end.
 *  			The program also contains validity Checks. IF the Headers are wrong
 *  			or the check sum is invalid, the return type UBX_ERROR is returned
 *  			otherwise a UBX_OK is returned
 *
 * Name.................Description
 * UBX_OK				USED When Functions execute successfully
 * UBX_ACK_ACK..........GPS Has recieved and understood the message
 * UBX_ACK_NACK.........GPS Has received but not understood the message
 */
typedef enum{
	UBX_OK = 2,
	UBX_ACK_ACK = 1,
	UBX_ACK_NACK = 0,
	UBX_ERROR = -1,
	UBX_TIMEOUT_Tx = -2,
	UBX_TIMEOUT_Rx = -3,

} UBX_MSG_t;

//==========================================================================


/*
 * Note: system converts time to Unix Epochs which start from Jan 1 1970
 * 		 GPS Epoch start from Jan 5 1980. To calculate the correct epoch
 * 		 We need to include an offset. NB: LEAP SECOND MUST BE UPDATED EVERY
 * 		 YEAR
 */

//===========================================================================

/* USART Private Macros*/

#define USART_GPS UART4	//defines the nucleo peripheral for UART communication

#define DMA_Rx_Flag_TCF DMA_FLAG_TC5 //Transfer complete flag for USART Rx DMA stream
#define DMA_Rx_ISR_TCF DMA_ISR_TCIF5 // ISR Transfer flag for Channel x
#define DMA_Rx_ISR_HTF DMA_ISR_HTIF5 // ISR Half Transfer flag for Channel x
#define DMA_Rx_ISR_TE DMA_ISR_TEIF5 // ISR Transfer flag for Channel x

#define DMA_Rx_IT_TCF DMA_IT_TC    //Transfer complete Interrupt for USART Rx DMA stream
//===========================================================================


//USART BUFFER lengths
#define DMA_RX_BUFFER_SIZE          500
#define DMA_TX_BUFFER_SIZE			500
#define GNSS_LOG_BUFFER_SIZE		500


uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];		//large buffer to hold data from GPS
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];		//large buffer for transmitting to device
uint8_t GNSS_LOG_Buffer[GNSS_LOG_BUFFER_SIZE];  //large buffer for logging to SD Card

//UBX Message Buffers

//==========================================================================================

/* State Variables */

volatile int RX_COMPLETE_FLAG;				//Indicates That GPS has completed data transfer to Device
volatile int TX_COMPLETE_FLAG;
volatile int RX_TIMEOUT_FLAG;				//indicates a timeout has occured
volatile int gnss_length;					//counter to keep track of data transfer in DMA

//===========================================================================================

/* Private variables */
Coord_t GPS_coord;		//storage variable for coordinates from GPS
uint32_t eTime;			//storage variable for Epoch Time
Diagnostic_t diag;		//storage variable for Diagnostic information

//===========================================================================================

/* private flag*/

uint8_t packet_full; //0bx x x x	x T D C		//T = successfully recorded Time
												//D = successfully recorded diagnostic
												//C = successfully recorded coordinate
/*
 * Flag Description:
 *
 * The following flags are used to interpret the data from the GPS.
 *
 * The library expects two types of data: Either a UBX message or an NMEA Message
 *
 * The UBX message contains important configuration information. It is also used
 * to check if the device has acknowledged the data we have transmitted.By setting the
 * Ack_message flag to 1, the data will be interpreted as a UBX message
 *
 * The NMEA message contains GPS Data which can come as a coordinate string (GLL),
 * Diagnostics string (GSA) or Time string (ZDA). There are other messages that exists
 * however you will need to create your own message parsers. By setting the Recieve_GPS_Data
 * flag, the data will be interpreted as an NMEA string
 *
 * this flags are set by software functions and cleared in the DMA IRQ Handler
 *
 * Flags will have 2 states:
 *
 *  1 - Set
 *  0 - Not Set
 *
 * Name..................Description
 * Ack_message...........Tells system it is about to read a UBX message
 * Recieve_GPS_Data .....Tells system it is about to read an NMEA message
 * GPS_tx_Complete...........Tells system that message has been transfered successfully
 */
uint8_t Ack_message, Recieve_GPS_Data,GPS_tx_Complete,USART_TX_Ready; //flags to show Micro how to deal with incoming messages

//=============================================================================================
/* private peripheral handlers*/

/*
 * These instances are used to handle data coming to and from peripherals
 */

//=============================================================================================
uint8_t init_GPS(void);
extern void USART_GPS_IRQHandler( UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma );
extern void DMA_Rx_IRQHandler( DMA_HandleTypeDef* hdma, UART_HandleTypeDef* huart );
extern void DMA_Tx_IRQHandler( DMA_HandleTypeDef* hdma, UART_HandleTypeDef* huart );
UBX_MSG_t UBX_Send_Ack(void);
UBX_MSG_t UBX_Configure_BaudRate(void);
void HAL_USART_Error_Handle(UART_HandleTypeDef *huart);
void USART_GPS_Timout_Handler(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart);
void USART_Begin_Timeout(TIM_HandleTypeDef *htim,uint32_t ms);
void Clear_Buffer(uint8_t* buffer,int size);
#endif /* HAL_GPS_H_ */

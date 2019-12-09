/*
 * GPS.h
 *
 *
 *  Created on: Jun 20, 2019
 *      Author: Jamie
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

#ifndef GPS_H_
#define GPS_H_
#define ARM_MATH_CM4  // Also defined for the time.h embedded implementation
//======================================================================================

/* Includes */
#include <stm32f4xx.h> //StandardPeriph Driver Header
#include "string.h"	   // for string handlings
#include "stdio.h"
#include "stdlib.h"
#include <arm_math.h>  //Header for additional Math Functions
#include <time.h>		// contains function to convert UTC to Epoch Time
#include "Delay.h"		//Import time functions for timeout
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
	float32_t lat;
	float32_t longi;
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


/* uncomment this line to enable Access to USART 1 or 6 */
//#define USE_APB2

/* Uncomment this line to enable access to USART 2, 3 and UART 4, 5 */
#define USE_APB1

//DO NOT LEAVE BOTH LINES UNCOMMENTED

/*
 * USART DEFINITIONS:
 * Note change ALL OF these definitions
 * to correspond with the USART/UART port that
 * the GPS is connected to
 */

#define USART_GPS UART4	//defines the nucleo peripheral for UART communication
#define GPIO_GPS GPIOC
//change definitation to corresponding GPIO ports
#define RCC_GPIOPeriph RCC_AHB1Periph_GPIOC // to enable power to the correct GPIO port

//change definition to corresponding USART/UART peripheral
/*
 * NOTE: Some of the USART peripherals are on different Peripheral Busses
 *
 * FOR USART 1, 6 uncomment the line USE_APB2
 * for USART 2, 3 and UART 4, 5  ignore
 */

//#define USE_APB2

#ifdef USE_APB2
#define RCC_USARTPeriph RCC_APB2Periph_USART1
#endif

#ifdef USE_APB1
#define RCC_USARTPeriph RCC_APB1Periph_UART4 // for power to the correct peripheral
#endif
// GPIO Pin Definitions
#define GPIO_USART_RX  GPIO_Pin_10
#define GPIO_USART_TX  GPIO_Pin_11
//GPIO Pin Source Definitions
#define GPIO_USART_RX_SRC GPIO_PinSource10
#define GPIO_USART_TX_SRC GPIO_PinSource11
#define USART_AF	GPIO_AF_UART4		// For correct Alternate Function Mappings
#define USART_GPS_IRQn UART4_IRQn		// Define the IRQn for Interrupts in the NVIC
#define USART_GPS_IRQHandler UART4_IRQHandler //define the IRQ_Handler for Recieving

//=====================================================================================
/* DMA Private Macros */

/*
 * Note: Before updating these values, consult the reference manuals to ensure that you have the correct
 * DMA Number
 * Channel
 * Stream
 *
 * before changing the definitions below
 */
#define USART_DMA DMA1								//DMA for Peripheral to Memory Data transfer
#define RCC_USART_DMAPeriph RCC_AHB1Periph_DMA1 	//RCC Clock line on AHB1
#define USART_DMA_RxChannel DMA_Channel_4			//Rx DMA Channel for UART4
#define USART_DMA_TxChannel DMA_Channel_4			//Tx DMA Channel for UART4
#define USART_DMA_RxStream DMA1_Stream2				//Rx DMA Stream
#define USART_DMA_TxStream DMA1_Stream4				//Tx DMA Stream
#define DMA_Rx_Flag_TCF DMA_FLAG_TCIF2				//Transfer Complete flag, corresponds to the stream number, tells system DMA transfer is complet
#define DMA_Rx_IT_TCF DMA_IT_TCIF2					// Transfer Complete Interrupt
#define DMA_Rx_IRQn DMA1_Stream2_IRQn				// Rx DMA transfer Interrupt for NVIC triggers when data transfer is half complete
#define DMA_Rx_IRQHandler DMA1_Stream2_IRQHandler	// UART4 DMA transfer complete IRQ Handler
//==========================================================================================


#define STM32_GNSS_USE_DMA  //enables initialization and use of UART-MEM DMA Transfer
#define STM32_GMEM_USE_DMA	//enables initialization and use of MEM-MEM DMA Transfer

//USART BUFFER lengths
#define DMA_RX_BUFFER_SIZE          2000
#define GNSS_LOG_BUFFER_SIZE		2000


uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];		//large buffer to hold data from GPS
uint8_t GNSS_LOG_Buffer[GNSS_LOG_BUFFER_SIZE];  //large buffer for logging to SD Card

//==========================================================================================

/* State Variables */

volatile int RX_COMPLETE_FLAG;				//Indicates That GPS has completed data transfer to Device
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
 * The following flags are used to interrpret the data from the GPS.
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
 */
uint8_t Ack_message, Recieve_GPS_Data; //flags to show Micro how to deal with incoming messages

//=============================================================================================

/* Private function prototypes */
/*
 * Description: Initializes USART DMA for communication with GPS
 * 				Peripheral Settings are defined in GPS.h
 * Parameters: baudrate - set the communication speed. Note: cannot be a fraction or negative number
 *
 * return: void
 */
void init_USART_GPS(uint32_t baudrate);

/*
 * Description: disables DMA, Interrupts and Power to Peripherals.
 * 				Sets GPIO Pins to default values and completely shuts
 * 				down the USART Peripheral. This function saves power by
 * 				turning off the peripheral when not in use and prevents
 * 				unwanted interrupts from occuring. The function will attempt
 * 				to communicate on the factory set baud rate however if the device
 * 				has been configured already, it will attempt a connection on the
 * 				set baudrate.
 *
 * 				Factory Baudrate: 9600 bit/s
 * 				Set Baudrate: 115200 bit/s
 *
 * parameters: void
 *
 * return: void
 */
void deinit_USART_GPS(void);

/*
 * Description: Initializes the GPS and requests
 * 				an Acknowledgment to determine
 * 				if device is working properly.
 * parameters: void
 *
 * return: uin8_t status - 1 if initialization was successful
 * 						 - 0 if an error occurred
 */
uint8_t  init_GPS(void);

/*
 * Description: Polls an unsigned byte for Transmission
 *
 * parameters: byte - data to be sent
 *
 * return type: void
 *
 */

void USART_transmit_byte(uint8_t byte);

/*
 *  Description: Polls an array of data for transmission
 *
 *  parameters: data - a pointer to the array to be sent, len - the number of butes to be transmitted
 *
 *  return: void
 */
void USART_transmit_data(uint8_t* data,int len);

/*
 * Description: Polls the reception of a single unsigned byte of data.
 * 				This is a legacy function to test the functionality of
 * 				the Device's usart peripheral
 *
 * parameters: void
 *
 * return type: void
 */

unsigned char USART_receive_byte(void);

/*
 * Description: Function to set all values in
 * 				the DMA_RX_BUFFER to 0
 *
 * 	parameters: void
 *
 * 	return: void
 */

void zero_dma_gnss_memory(void);

/*
 * Description: Function to set all values in
 * 				the GNSS_LOG_BUFFER to 0
 *
 * 	parameters: void
 *
 * 	return: void
 */

void zero_gnss_memory(void);


// GPS parse functions
/*
 * Add your own for the message you want to decode
 * Note: These messages need to be activated first otherwise
 *		 the data will not appear. For More information consult
 *		 u-blox 7 Receiver Description Including Protocol Specification V14
 *		 - https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescriptionProtocolSpec_%28GPS.G7-SW-12001%29_Public.pdf
 */

/*
 *  Description: Function to Decode Positional Data.
 *  			 Takes in an NMEA GxGLL String, extracts
 *  			 the position data and stores it in Coord_t object.
 *
 *  Parameter: GSAstring - unsigned char* array containing the NMEA message from the GPS
 *
 *  return: void
 */

uint8_t Parse_GLL(char* GLLstring);

/*
 *  Description: Function to Decode Time Information.
 *  			 Takes in an NMEA GxZDA String, extracts
 *  			 the date time and converts to Unix Epoch Time in seconds. (1st January 1970)
 *
 *  Parameter: GLLstring - unsigned char* array containing the NMEA message from the GPS
 *
 *  return: void
 */

uint8_t parse_ZDA(char* ZDAstring);

/*
 *  Description: Function to Decode Signal Diagnostic Information.
 *  			 Takes in an NMEA GxGSA String and extracts vital Diagnostic information
 *  			 Data is stored in the global diag_t object and contains the following information
 *  - PDOP
 *  - VDOP
 *  - HDOP
 *  - Number of Satelites
 *  - Signal Type
 *
 *  Parameter: GSAstring - unsigned char* array containing the NMEA message from the GPS
 *
 *  return: void
 */

uint8_t parse_GSA(char* GSA_string);

/*
 * Description: Validity check function for NMEA Messages.
 * 				This function analyses the NMEA string before it
 * 				is parsed. The function takes in a string and checks
 * 				that it conforms to NMEA protocols. Then it checks that
 * 				the checksum at the end of the message is valid. If the message is
 * 				valid, function will return the type of message it is
 *
 *
 * 	parameters: nmeamsg - unsigned char pointer to a null terminated array containg
 * 						  the data to be analyzed
 *  return: NMEA message - uint8_t flag that tells whether the data was valid and what type it was
 *
 *  -1 invalid
 *   1 GLL msg
 *   2 GGA msg
 *   3 ZDA msg
 */
uint8_t is_valid(char* nmeamsg);

/*
 * Description: Function to Convert a char value to hexadecimal number
 * 				Function is not case sensitive
 *
 * parameters: c - char to be converted (0 - 9 or a - f)
 *
 * return: uint8_t number
 */
uint8_t char_to_hex(char c);

#endif /* GPS_H_ */

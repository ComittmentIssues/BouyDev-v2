/*
 * GPS.h
 *
 *  Created on: Jun 20, 2019
 *      Author: Jamie
 */
#define _XOPEN_SOURCE
#ifndef GPS_H_
#define GPS_H_
#define ARM_MATH_CM4
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "arm_math.h"
#include <time.h>
/* private structs */
typedef struct
{
	float32_t lat;
	float32_t longi;
}Coord_t;
typedef struct
{
	uint8_t digit;
	uint8_t precision;
}DOP_t;
typedef struct
{
	DOP_t PDOP;
	DOP_t HDOP;
	DOP_t VDOP;
	uint8_t num_sats;
	uint8_t fix_type;
}Diagnostic_t;
/* private enums*/
/*
 * Note: system converts time to Unix Epochs which start from Jan 1 1970
 * 		 GPS Epoch start from Jan 5 1980. To calculate the correct epoch
 * 		 We need to include an offset. NB: LEAP SECOND MUST BE UPDATED EVERY
 * 		 YEAR
 */

/* private defines*/
#define GPIO_USART1_RX  GPIO_Pin_6
#define GPIO_USART1_TX  GPIO_Pin_7
#define GPIO_USART1_RX_SRC GPIO_PinSource6
#define GPIO_USART1_TX_SRC GPIO_PinSource7
#define offset 315964800
#define GPS_BAUDRATE  115200
#define STM32_GNSS_USE_DMA
#define STM32_GMEM_USE_DMA

//USART BUFFERS
#define DMA_RX_BUFFER_SIZE          2000
#define GNSS_LOG_BUFFER_SIZE		2000


uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
uint8_t GNSS_LOG_Buffer[GNSS_LOG_BUFFER_SIZE];
volatile int RX_COMPLETE_FLAG;
volatile int GNSS_Data_NOT_EMPTY;
volatile int gnss_length;


/* Private macro */
/* Private variables */
Coord_t GPS_coord;
uint32_t eTime;
Diagnostic_t diag;
/* private flag*/
uint8_t packet_full; //0bx x x x	x T D C		//T = successfully recorded coordinate
											//D = successfully recorded diagnostic
											//C = successfully recorded coordinate

/* Private function prototypes */
void init_USART_GPS(void);
void deinit_USART_GPS(void);
void USART_transmit_byte(uint8_t byte);
void USART_transmit_string(unsigned char*);
unsigned char USART_receive_byte(void);
void zero_dma_gnss_memory(void);
void zero_gnss_memory(void);
uint8_t transmit_message(uint8_t* message);
// GPS parse functions
uint8_t Parse_GLL(char* GSAstring);
uint8_t parse_ZDA(char* ZDAstring);
uint8_t is_valid(char* nmeamsg);
uint8_t char_to_hex(char c);
uint8_t parse_GSA(char* GGA_string);
#endif /* GPS_H_ */

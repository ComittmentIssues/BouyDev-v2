/*
 * HAL_GPS.h
 *
 *  Created on: Mar 11, 2020
 *      Author: jamie
 */

#ifndef HAL_GPS_H_
#define HAL_GPS_H_

#define _XOPEN_SOURCE // Definition for time.h to work
#define ARM_MATH_CM4  // Also defined for the time.h embedded implementation
//======================================================================================

/* Includes */
#include <stm32l4xx.h> //StandardPeriph Driver Header
#include "string.h"	   // for string handlings
#include "stdio.h"
#include "stdlib.h"
#include <math.h>  //Header for additional Math Functions
#include <time.h>		// contains function to convert UTC to Epoch Time
//=======================================================================================

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
	GPS_Init_Periph_Config_Error = 6

}GPS_Init_msg_t;

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

//=======================================================================================
#define CCR1_VAL 8000
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOC
#define GPS_RX_Pin GPIO_PIN_11
#define GPS_RX_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

#define DMA_TX_BUFFER_SIZE 2048
#define DMA_RX_BUFFER_SIZE 2048
#define GNSS_BUFFER_SIZE 2048

//=======================================================================================

/* Private Variables */
Coord_t GPS_coord;		//storage variable for coordinates from GPS
uint32_t eTime;			//storage variable for Epoch Time
Diagnostic_t diag;		//storage variable for Diagnostic information
uint8_t M2M_Txfer_Cplt, TIM_IDLE_Timeout,TX_Cplt;
int gnss_length;
uint8_t packet_full, log_gps;
//=======================================================================================

/* Private Variables */

extern DMA_HandleTypeDef hdma_uart4_rx;

extern DMA_HandleTypeDef hdma_uart4_tx;

extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart4;

extern DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
//=======================================================================================
/* Private Buffers */
uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];
uint8_t GNSS_Buffer[GNSS_BUFFER_SIZE];
uint8_t DMA_TX_Buffer[DMA_TX_BUFFER_SIZE];
//=======================================================================================
//Peripheral functions

HAL_StatusTypeDef USART_Set_Baudrate(UART_HandleTypeDef* huart, TIM_HandleTypeDef *htim, uint32_t baud);

//CUSTOM IRQ Handle Functions
void USART_GPS_IRQHandler(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_mem);
void DMA_GNSS_MEM_IRQHandler(DMA_HandleTypeDef *hdma, TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart);
void DMA_GNSS_Periph_IRQHandler(DMA_HandleTypeDef *hdma_periph, DMA_HandleTypeDef *hdma_mem, TIM_HandleTypeDef *htim);
void USART_TIM_RTO_Handler(TIM_HandleTypeDef *htim);
//GPS FUNCTIONS
GPS_Init_msg_t init_GPS(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
UBX_MSG_t UBX_Send_Ack(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim);
UBX_MSG_t UBX_Configure_Baudrate(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
UBX_MSG_t UBX_Configure_Messages(UART_HandleTypeDef *huart);
//utility Functions
void  Clear_Buffer(uint8_t *buffer,uint32_t size);
uint8_t char_to_hex(char c);
uint8_t is_valid(char* nmeamsg);
uint8_t parse_ZDA(char* ZDAstring);
uint8_t Parse_GLL(char* GLLstring);
uint8_t parse_GSA(char* GSA_string);
#endif /* HAL_GPS_H_ */

/*
 * HAL_Iridium.h
 *
 *  Created on: Mar 28, 2020
 *      Author: Jamie Jacobson
 *      Student No: JCBJAM007
 *      For: University of Cape Town
 *========================================================================================================================
 *  This library is designed to be used with the STM Hal libraries. Written for
 *  version 1.15.1 Note: This version is compatible with version 1.14.x
 *
 *  This Library is designed to Interface with the Iridium 9603 Modem.
 *  This is a 5V UART module that can be controlled by sending AT Commands to the device. all commands start with
 *  the prefix AT and must be followed by a terminator "\r". The function send_AT_Command() allows the user to
 *  create functions using AT commands.
 *
 *  The 10 pin interface consists of 1 Control Pin, 2 Indicator Pins, 3.3V and 5V power inputs, and
 * 	USART Tx/Rx pins aswell as flow control CTS/RTS pins.
 *
 * 	This library is designed to send both binary and ascii messages via Iridium network using the functions:
 * 	IR_Status_t send_Bin_String(uint8_t* bin_string,uint32_t len);
 *	IR_Status_t send_String(char* string);
 *
 *	The library also contains functions to recieve satelite messages.
 *
 *	IR_Status_t recieve_String(uint8_t* MSG_Buff,uint32_t MSG_BUFF_SIZE, uint16_t *num_messages);
 *
 * 	Messages are uploaded / downloaded to the modem's internal message buffers.
 *
 * 	The buffer MO (Mobile Originated) is 340 bytes long and stores data to be sent
 * 	The buffer MT (Mobile Terminated) is 270 bytes long and stores downloaded messages
 *
 * 	The Modem uses SBD (short burst data) protocols to send and recieve data. The Network information exchange
 * 	is performed by initiating an SBD Session using the function command AT+SBDIX the library contains the function
 *
 * 	IR_Status_t start_SBD_Session(SBDX_Status_t* sbd);
 *
 * 	Which contains the algorithm neccessary to perform this function.
 *
 * 	Control Pins:
 * 	On/Off - This is a digital output pin that is used to put the device into sleep mode (lowest current draw)
 * 			 and wake the device up again. Once in sleep mode, the device cannot recieve satelite messages, ring
 * 			 alerts or AT Commands. The device is put to sleep by writing a digital Low. Leaving the pin floating
 * 			 or setting the pin to high causes the device to wake up and become active
 *
 * 	Indicator Pins:
 *
 * 	Ring Alert - The ring Indicator signals to the device that the Iridium Modem has a message queued for reception
 * 				 This is achieved by transmitting a 5 second long pulse every 20 seconds. t
 *
 * 	Network Availability - This is a digital input indicator that determine whether their is sufficient satelite
 * 						  coverage for successful data reception/transmission. A digital High indicates sufficient
 * 						  reception. A digital Low indicates unsuccessful reception
 *
 * 	Setting up The Modem:
 *
 * The Device must be connected to a 5V power supply with a maximum in rush current of at least
 * 200mA (expect a maximum of 600mA)
 *
 * Once Connected, a red LED will come on. This indicates that the device is connected to a power source.
 * During this time, the module charges the supercapictor by sinking a maximum of 650mA.
 * When the device has finished Charging a green LED will Appear. This means the modem is on and ready to recieve data.
 * ========================================================================================================================
 */

#ifndef HAL_IRIDIUM_H_
#define HAL_IRIDIUM_H_

//============================= 1. Includes ==============================================

#include "stm32l4xx_hal.h"
#include "stm32l476xx.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

typedef enum
{
	IR_OK,
	IR_Pin_CFG_Error,
	IR_Rx_Error,
	IR_Tx_Error,
	IR_Rx_Incplt,
	IR_Rx_Timeout,
	IR_Data_Error,
	IR_Ack_Error,
	IR_CFG_Error,
	IR_MSG_UPLOAD_ERROR,
	IR_MSG_UPLOAD_OK,
	IR_SBDWB_STATUS_ERROR,
	IR_SBDWB_TIMEOUT,
	IR_SBDWB_MSGOVERRUN_ERROR,
	IR_SBDWB_CHECKSUM_ERROR,
	IR_SBDIX_SESSION_ERROR,
	IR_SBDIX_NO_NEW_MESSAGE,
	IR_SBDIX_MAIL_CHECK_ERROR,
	IR_SBDRT_Rx_Error,
	IR_CSQ_Ack_Error
} IR_Status_t;
//========================== 2. Structs & Enums ===========================================
typedef enum
{
	NONE,
	SBDWB,
	SBDIX,
	SBDRT,
	CSQ

}Session_t;


typedef struct
{
	uint8_t MO_Status;
	uint32_t MO_MSN;
	uint8_t MT_Status;
	uint32_t MT_MSN,MT_length,MT_Queued;

} SBDX_Status_t;

//======================== 3. Macro Definitions =========================================

#define IR_OnOff_Pin GPIO_PIN_10
#define IR_OnOff_GPIO_Port GPIOA
#define IR_OnOff_PWR_GPIO_Port PWR_GPIO_A
#define IR_RIng_Pin GPIO_PIN_11
#define IR_RIng_GPIO_Port GPIOA
#define IR_RIng_EXTI_IRQn EXTI15_10_IRQn
#define IR_NetAv_Pin GPIO_PIN_12
#define IR_NetAv_GPIO_Port GPIOA
#define IR_NetAv_EXTI_IRQn EXTI15_10_IRQn
#define IR_TX_Pin GPIO_PIN_12
#define IR_TX_GPIO_Port GPIOC
#define IR_RX_Pin GPIO_PIN_2
#define IR_RX_GPIO_Port GPIOD

#define IR_USART_PORT UART5

#define IR_TIM_PORT TIM3

#define TX_BUFFER_SIZE 2046
#define RX_BUFFER_SIZE 2046
#define RM_BUFFER_SIZE 500
#define ASCII_MSG_BYTE_LEN 9

//========================== 4. Global Variables ==========================================
int8_t RX_Flag,TIM_IDLE_Timeout;
Session_t Session_Flag;
uint32_t gnss_length;
uint32_t msg_len;

//============================= 5. Handlers =============================================
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel2;
uint8_t RX_Buffer[RX_BUFFER_SIZE];
uint8_t TX_Buffer[TX_BUFFER_SIZE];
uint8_t RM_Buffer[RM_BUFFER_SIZE];



IR_Status_t send_AT_CMD(char* cmd);
IR_Status_t IR_Init_Module(void);
IR_Status_t IR_DeInit_Module(void);
IR_Status_t get_Signal_Strength(uint8_t* signal_Strength);
IR_Status_t start_SBD_Session(SBDX_Status_t* sbd);
IR_Status_t send_Bin_String(uint8_t* bin_string,uint32_t len);
IR_Status_t send_String(char* string);
IR_Status_t recieve_String(uint8_t* MSG_Buff,uint32_t MSG_BUFF_SIZE, uint16_t *num_messages);

void DMA_Iridium_Periph_IRQHandler(UART_HandleTypeDef *huart);
void DMA_Iridium_MEM_IRQHandler(DMA_HandleTypeDef *hdma_mem);
void USART_RTO_IRQHandler(TIM_HandleTypeDef *htim);
void USART_Iridium_IRQHandler(UART_HandleTypeDef *huart);
void Iridium_ControlPin_IRQHandler(void);

#endif /* HAL_IRIDIUM_H_ */

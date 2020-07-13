/*
 * HAL_Iridium.h
 *
 *  Created on: Mar 28, 2020
 *      Author: jamie
 */

#ifndef HAL_IRIDIUM_H_
#define HAL_IRIDIUM_H_

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

typedef enum
{
	NONE,
	SBDWB,
	SBDIX,
	SBDRT

}Session_t;
typedef struct
{
	uint8_t MO_Status;
	uint32_t MO_MSN;
	uint8_t MT_Status;
	uint32_t MT_MSN,MT_length,MT_Queued;

} SBDX_Status_t;

#define IR_OnOff_Pin GPIO_PIN_10
#define IR_OnOff_GPIO_Port GPIOA
#define IR_RIng_Pin GPIO_PIN_11
#define IR_RIng_GPIO_Port GPIOA
#define IR_RIng_EXTI_IRQn EXTI15_10_IRQn
#define IR_NetAv_Pin GPIO_PIN_12
#define IR_NetAv_GPIO_Port GPIOA
#define IR_NetAv_EXTI_IRQn EXTI15_10_IRQn
#define Iridium_TX_Pin GPIO_PIN_12
#define Iridium_TX_GPIO_Port GPIOC
#define Iridium_RX_Pin GPIO_PIN_2
#define Iridium_RX_GPIO_Port GPIOD

#define TX_BUFFER_SIZE 2046
#define RX_BUFFER_SIZE 2046
#define RM_BUFFER_SIZE 500
#define ASCII_MSG_BYTE_LEN 9
int8_t RX_Flag,TIM_IDLE_Timeout;
Session_t Session_Flag;

uint8_t RX_Buffer[RX_BUFFER_SIZE];
uint8_t TX_Buffer[TX_BUFFER_SIZE];
uint8_t RM_Buffer[RM_BUFFER_SIZE];
uint32_t gnss_length;
uint32_t msg_len;

extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart5;

extern DMA_HandleTypeDef hdma_uart5_rx;

extern DMA_HandleTypeDef hdma_memtomem_dma1_channel2;

void  Clear_Buffer(uint8_t *buffer,uint32_t size);
uint16_t calculate_checkSum(uint8_t* messagebuff, uint8_t size);

IR_Status_t send_AT_CMD(char* cmd);
IR_Status_t IR_Init_Module(void);
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

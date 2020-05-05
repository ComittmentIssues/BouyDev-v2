/*
 * Iridium.h
 *
 * Iridium communication uses USART3 for serial comms. This maps to GPIOB.
 * This can be changed in the defines
 *  Created on: Jun 11, 2019
 *      Author: Jamie
 */

#ifndef IRIDIUM_H_
#define IRIDIUM_H_

#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "Delay.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4_discovery.h"

/*Private Macros*/
#define IRIDIUM_Periph_Use_DMA
#define IRIDIUM_MEM_Use_DMA

#define Iridium_GPIO_RCCPeriph RCC_AHB1Periph_GPIOB
#define Iridium_USART_RCCPeriph RCC_APB1Periph_USART3

#define Iridium_GPIO_PeriphClockCommand RCC_AHB1PeriphClockCmd
#define Iridium_USART_PeriphClockCommand RCC_APB1PeriphClockCmd
#define DMA_AHB1PeriphClockCmd RCC_AHB1PeriphClockCmd

#define Iridium_USART USART3
#define Iridium_GPIO GPIOB

#define Iridium_Periph_DMA DMA1;
#define Iridium_MEM_DMA DMA2
#define Iridium_DMA_RX_Stream DMA1_Stream1
#define Iridium_DMA_RX_Channel DMA_Channel_4
#define Iridium_DMA_MEM_Stream DMA2_Stream1
#define Iridium_DMA_MEM_Channel DMA_Channel_1

#define DMA_AHB1Periph RCC_AHB1Periph_DMA1
#define DMA2_AHB1Periph RCC_AHB1Periph_DMA2
#define DMA_USART_RX_IRQn	DMA1_Stream1_IRQn
#define DMA_USART_MEM_IRQn	DMA2_Stream1_IRQn
#define DMA_USART_RX_IRQHandler DMA1_Stream1_IRQHandler
#define DMA_USART_MEM_IRQHandler DMA2_Stream1_IRQHandler

#define Iridium_NetAv_Pin GPIO_Pin_0
#define Iridium_Wakeup_Pin GPIO_Pin_9
#define Iridium_USART_TX GPIO_Pin_10
#define Iridium_USART_RX GPIO_Pin_11

#define Iridium_USART_TXsrc GPIO_PinSource10
#define Iridium_USART_RXsrc GPIO_PinSource11
#define Iridium_Wakeup_Pinsrc GPIO_PinSource9
#define Iridium_NetAv_Pinsrc GPIO_PinSource0

#define NetAv_EXTI_Line EXTI_Line0
#define NetAv_EXTI_IRQn EXTI0_IRQn
#define NetAv_EXTIPortsource EXTI_PortSourceGPIOB
#define NetAv_EXTIPinSource  EXTI_PinSource0
#define NetAV_EXTI_IRQHandler EXTI0_IRQHandler


#define Iridium_GPIO_AF GPIO_AF_USART3
#define Iridium_USART_IRQHandler USART3_IRQHandler
#define Iridium_USART_IRQn USART3_IRQn
#define Iridium_Baudrate 19200
#define Iridium_Verbose_Token (0xD <<8)|(0xA) //'\r\n'

#define length(x) sizeof(x)/sizeof(*x)
#define Iridium_RX_Buffsize 1000 //increase if necessary
#define Iridium_message_Buffsize 1000
/* Private Variables */
uint8_t Iridium_Rx_Buff[Iridium_RX_Buffsize];
size_t Iridium_data_length;
char message_buff[Iridium_message_Buffsize];
char temp_buff[Iridium_message_Buffsize];
int16_t SBDIX_status[6];

/* Status Flags*/
uint8_t IR_Rx_done;
uint8_t status_Received;
uint8_t bin_message_received;
uint8_t network_available
;
uint8_t session_flag;

/*Private functions*/
// init Functions
void init_Iridium_USART(void);
void init_Rx_Buff(void);
void init_message_buff(void);
void init_Control_Pins(void);
int8_t init_Iridium_Module(void);
void deinit_Iridium_Module(void);
//AT functions
void transmit_Data(char* tx_buff,size_t len);
void Iridium_Rx_status(void);
char* get_data_from_buff(void);
int8_t send_ATcmd(char* cmd, uint32_t delay);
char* get_AT_response(void);
//Message Functions
uint8_t send_ASCII_Message(char* msg);
int8_t create_SBD_Session(void);
void clear_Status(void);
void get_status(char* cmd);
uint8_t send_Binary_Message(uint8_t *msg, uint16_t size);
uint16_t calculate_checkSum(uint8_t* messagebuff, uint8_t size);
#endif /* IRIDIUM_H_ */

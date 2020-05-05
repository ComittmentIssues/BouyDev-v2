/*
 * Iridium.h
 *
 * Author: Jamie Jacobson
 *
 * Header file containing definitions, function prototypes
 * and Macros for use with the Iridium.c File
 *
 * The Rockblock 9603 is a development module that allows
 * for the wireless transmission of data over the Iridium
 * Satellite constellation. The module requires 5V to be powered
 * and uses a 3,3V logic level. The module uses USART to communicate
 * with pins available for Flow control. For the purposes of this project
 * the code will allow the micro controller to interface with the modem
 * with no FLOW control
 *
 * The micro controller communicates with the device using USART where the device
 * polls data for transmission and recieves data through the DMA. The Micro Controller
 * transmits AT commands (a Full reference can be found here https://www.remotesatellite.com/supportdocs/support/iridium/Beam-Iridium-AT-Command-Guide.pdf)
 * These commands are transmitted via USART to the module. All Messages must be terminated with '\r'.
 *  The module then process the commands and returns a message string. This string can vary depending on the
 *  command given. The Functions in this library use a selection of commands to achieve their function.
 *
 *  The Iridium has the following functional pins:
 *
 * Iridium Pins:	Pin No.............. Name.................Nucleo Pin..........Function
 * 					   1.................RXD..................PD2..................Data Output from RockBlock
 * 					   2.................CTS..................N/C..................Flow Control Clear to send (output from Modem)
 * 					   3.................RTS..................N/C..................Flow Control Request to send (input to modem)
 * 					   4.................NetAv................PC13.................Network Available (Indicates when signal is strong enough to transmit a message) (1 - Network available, 0 - No Network)
 * 					   5.................RingIndicator........PC14.................Ring Indicator for incoming messages
 * 					   6.................TXD..................PC12.................Data Input to RockBlock
 * 					   7.................OnOff................PC15.................Digital Control Pin to put modem to sleep/ wake up
 * 					   8.................5V...................N/C..................5V power (note, Iridium pins dont have enough current to power the device)
 * 					   9.................LiOn.................N/C..................3.7V Lithium Ion battery power
 * 					   10................GND..................GND..................Ground
 *
 */

#ifndef IRIDIUM_H_
#define IRIDIUM_H_

/* System Includes */

#include <stm32f4xx.h>			//Std Periph Library Header
#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx_gpio.h"		//Std Periph GPIO functions
#include "stm32f4xx_usart.h"	//Std Peripg USART functions
#include "string.h"				//For String handling functions
#include "stdlib.h"				//sprintf and c functions
#include "Delay.h"				//Custom Delay library for timeouts
#include "stm32f4xx_exti.h"		// configure IO port for event-based interrupts
#include "stm32f4xx_syscfg.h"

//==============================================================================

/*Private Macros*/


#define IRIDIUM_Periph_Use_DMA	//Enable Peripheral - Memory DMA initialization code and Functions
#define IRIDIUM_MEM_Use_DMA		// Enable Memory - Memory DMA initialization code and Functions

/*
 *  These Macros are used to configure a user Specified USART PORT
 * 	If you would like to use a different Port, then the following
 * 	variables need to be updated
 */
// * * * * * * * * START * * * * * * * * * * * * * * * * * * * * *

#define Iridium_GPIO_RCCPeriph RCC_AHB1Periph_GPIOC		//Port Power on AHB1

//change definition to corresponding USART/UART peripheral
/*
 * NOTE: Some of the USART peripherals are on different Peripheral Busses
 *
 * FOR USART 1, 6 use RCC_APB2PeriphClockCmd and RCC_APB2Periph_XXXX
 * for USART 2, 3 and UART 4, 5  use RCC_APB1PeriphClockCmd and RCC_APB1Periph_XXXX
 */
#define Iridium_USART_RCCPeriph RCC_APB1Periph_UART5

#define Iridium_GPIO_PeriphClockCommand RCC_AHB1PeriphClockCmd		//function pointer to Enable RCC Clock on AHB1

#define Iridium_USART_PeriphClockCommand RCC_APB1PeriphClockCmd		//function pointer to Enable RCC CLock on APB1

#define DMA_AHB1PeriphClockCmd RCC_AHB1PeriphClockCmd				//simialr to line 82

#define Iridium_USART UART5											//UART Peripheral

#define Iridium_GPIO GPIOC											// Port Corresponding to UART Peripheral
/*
 * Note: Some UARTS have pin mappings on different
 * GPIO ports. To enable these pins makes sure USE_2_GPIO_Ports is defined
 * and set the second GPIO as desired. Make sure that line is commented out
 *
 */
#define USE_2_GPIO_Ports

#ifdef USE_2_GPIO_Ports
#define Iridium_GPIO_2 GPIOD										//Second GPIO Port for UART 5
#define Iridium_GPIO_RCCPeriph_2 RCC_AHB1Periph_GPIOD
#endif

//* * * * * END * * * * * * * * * * * * * * * * * * * * *
//==================================================================================================

/* DMA Private Macros */

/*
 * Note: Before updating these values, consult the reference manuals to ensure that you have the correct
 * DMA Number
 * Channel
 * Stream
 *
 * before changing the definitions below
 */

#define Iridium_Periph_DMA DMA1;				//DMA Controller for Peripheral To Memory Data Transfer
#define Iridium_MEM_DMA DMA2					//DMA Controller for Memory To Memory Data Transfer
#define Iridium_DMA_RX_Stream DMA1_Stream0		//DMA Stream for USART Data Reception
#define Iridium_DMA_RX_Channel DMA_Channel_4	//DMA Channel for USART Data Reception
#define Iridium_DMA_MEM_Stream DMA2_Stream1		//DMA Stream for Memory - Memory Transfer
#define Iridium_DMA_MEM_Channel DMA_Channel_1	//DMA Channel for Memory - Memory Transfer

#define DMA_AHB1Periph RCC_AHB1Periph_DMA1		// Enable DMA PEripheral on AHB1
#define DMA2_AHB1Periph RCC_AHB1Periph_DMA2		// Enable DMA Peripheral 2 on AHB1
#define DMA_USART_RX_IRQn	DMA1_Stream0_IRQn	// NVIC DMA Rx Stream IRQ line
#define DMA_USART_MEM_IRQn	DMA2_Stream1_IRQn	// NVIC DMA Memory Stream IRQ Line
#define DMA_USART_RX_IRQHandler DMA1_Stream0_IRQHandler	//DMA Rx Stream IRQ Handler
#define DMA_USART_MEM_IRQHandler DMA2_Stream1_IRQHandler	//DMA Memory Stream IRQ Handler

//=========================================================================================================

/* Iridum Pin Macro */

/*
 * MAcro Definitions for Nucleo Pin mapping to
 * USART Ports:
 *
 * Note: Make sure you consult the reference manual for Pin and AF Mapping
 *
 * Do Not forget to update these macros if you change the USART Peripheral
 */
#define Iridium_NetAv_Pin GPIO_Pin_13		// Network Available pin
#define Iridium_Wakeup_Pin GPIO_Pin_15		// ON Off Pin
#define Iridium_USART_TX GPIO_Pin_12		// Transmit Pin
#define Iridium_USART_RX GPIO_Pin_2 		// Recieve pin (on Port D for UART5)

// Macros for Alternate Function Mapping
#define Iridium_USART_TXsrc GPIO_PinSource12 	//UART5 TX Pin source for AF Map
#define Iridium_USART_RXsrc GPIO_PinSource2		//UART5 Rx Pin source
#define Iridium_Wakeup_Pinsrc GPIO_PinSource15  // OnOFf Pin source
#define Iridium_NetAv_Pinsrc GPIO_PinSource13	//Network Availabilty Pin Source

/*
 * Network Available Pin is mapped to an external interrupt
 * This will be used to passively scan in the background for
 * available network. The following Macros are used to configure
 * the line
 */
#define NetAv_EXTI_Line EXTI_Line13					//EXTI Line
#define NetAv_EXTI_IRQn EXTI1_IRQn					// NVIC Interrupt Request Lineline
#define NetAv_EXTIPortsource EXTI_PortSourceGPIOC	// Map EXTI Line to Port C
#define NetAv_EXTIPinSource  EXTI_PinSource13		// Map EXTI Line to Pin 13
#define NetAV_EXTI_IRQHandler EXTI0_IRQHandler		// EXTI IRQ Handler

#define Iridium_GPIO_AF GPIO_AF_UART5				//Macro to map GPIO Pin to UART AF
#define Iridium_USART_IRQHandler UART5_IRQHandler	// UART IRQ Handler
#define Iridium_USART_IRQn UART5_IRQn				// NVIC IRQ Line
#define Iridium_Baudrate 19200						// Iridium baudrate as defined in the user manual
#define Iridium_Verbose_Token (0xD <<8)|(0xA) 		// Iridium EOL charachter sequence'\r\n'

#define length(x) sizeof(x)/sizeof(*x)				// Function to determine the size of an array
#define Iridium_RX_Buffsize 1000 					// Buffer length of RX buffer to receive data from Iridium via USART DMA increase if necessary
#define Iridium_message_Buffsize 1000				// Size of Additional buffer to extract AT messages

//==================================================================================================

/* Private Variables */

uint8_t Iridium_Rx_Buff[Iridium_RX_Buffsize];	//receive data from Iridium via USART DMA increase if necessary
size_t Iridium_data_length;						// Length of recieved message
char message_buff[Iridium_message_Buffsize];	//Additional buffer to extract AT messages
char temp_buff[Iridium_message_Buffsize];		//additional buffer to save data
int16_t SBDIX_status[6];						// Stores the returned values from the SBDIX session command

//============================================================================================================

/* Status Flags*/

/*
 * These flags show the status of Iridum processes.
 *
 * Flag Name:.............Function.................................................On State.......................Off State
 * IR_Rx_done.............Show status of data being received.......................Transfer Finished..............Transfer In progress
 * status_Received........System recieved valid message from device................Waiting for status.............Status Recieved
 * bin_message_recieved...Shows weather modem successfully recieved binary message.Message Recieved...............Idle
 * network_Available......Shows signal Strength....................................Sufficientsignal to transmit...No Signal
 */
uint8_t IR_Rx_done;
uint8_t status_Received;
uint8_t bin_message_received;
uint8_t network_available;
uint8_t session_flag;
//===========================================================================================================================

/*Private functions*/

/*
 * Description: Initializes USART DMA for communication with  Iridium
 * 				Peripheral Settings are defined in Iridium.h
 * parameters:  void
 *
 * return: 		void
 */

void init_Iridium_USART(void);

/*
 * Description: deletes data in the Rx_Buff Array, initializes all values to zero
 *
 * parameters: void
 *
 * return:	   void
 */

void init_Rx_Buff(void);

/*
 * Description: deletes data in the message_Buff Array, initializes all values to zero
 *
 * parameters: void
 *
 * return:	   void
 */

void init_message_buff(void);

/*
 * Description: Initialization function for Extra Function Pins
 * 				Configures The Network available pin for digital
 * 				input through an external interrupt line and configures
 * 				OnOff pin for Digital Output
 * 	parameters: void
 *
 * 	return:		void
 */
void init_Control_Pins(void);

/*
 * Descriptions: Function to determine the state of the modem
 * 			     Function initializes communication and sends
 * 			     an AT command. The function waits for an acknowledgement
 * 			     message from the modem. If no message or an invalid message
 * 			     is recieved, a 0 is returned. IF successful, a 1 is returned
 * 	parameters: void
 *
 * 	return: status - uint8_T
 * 			1........ Iridium online
 * 			0........ Error Initializing Iridium
 */

int8_t init_Iridium_Module(void);

/*
 * Description: Function to de-initialize and turn off the Iridium Module
 * 				This function is used to save power by disabling unused
 * 				peripherals and placing the pins in a low power state.
 * 				The function first determines the state of the modem
 * 				by sending an acknowledge message. THe the Data buffer
 * 				is cleared and the UART peripheral initialised to its
 * 				default values with the interrupts disabled.
 * 				The Module is disabled and the GPIO pins are returned to
 * 				default state.
 *
 * 	parameters: void
 *
 * 	return: void
 *
 */

void deinit_Iridium_Module(void);
//===========================================================================

//AT functions

/*
 * Description: Transmits a Character array of User Defined length to the
 * 				Iridium Buffer. The function takes in a string of data,
 * 				and polls each byte for transmission
 * parameters: tx_bff - string of data to be sent, len - number of bytes to be sent
 *
 * return: void
 */
void transmit_Data(char* tx_buff,size_t len);

/*
 *
 */

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

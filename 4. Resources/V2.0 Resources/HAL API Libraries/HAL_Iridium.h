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
 *	Information in the library is based off the API docs located here: https://docs.rockblock.rock7.com/docs
 *
 *  This Library is designed to Interface with the Iridium 9603 Modem.
 *  This is a 5V UART module that can be controlled by sending AT Commands to the device. all commands start with
 *  the prefix AT and must be followed by a terminator "\r". The function send_AT_Command() allows the user to
 *  create functions using AT commands.
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

#include "stm32l4xx_hal.h"		//HAL header

#include "stm32l476xx.h"		//CMSIS Includes

#include "string.h"				//String Handling Functions

#include "stdio.h"				//Standard C Functions

#include "stdlib.h"				//additional memset

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
/*
 * SESSION ENUMERATION
 *
 * Represents the type of AT Query sent to the Modem
 *
 * These querries have specific return types or timing requirements and signal
 * the IRQHandlers to process them in a unique way
 */
typedef enum
{
	NONE,
	SBDWB,
	SBDIX,
	SBDRT,
	CSQ

}Session_t;

/*
 * SBD Object
 *
 * Stores the status flags sent to the device after an SBD Session has been initiated
 *
 * VARIABLES: Name.............Type.................................Description
 * 			  MO_STATUS........uint8_t..............................Message Transmission Status
 * 			  MO_MSN...........uint32_t.............................Message Number (16 bit number)
 * 			  MT_STATUS........uint32_t.............................Message Reception Status
 * 			  MT_LENGTH........uint32_t.............................Recieved Message byte Length
 * 			  MT_QUEUED........utin32_t.............................Number of Messages left in Queue
 */

typedef struct
{
	uint8_t MO_Status;
	uint32_t MO_MSN;
	uint8_t MT_Status;
	uint32_t MT_MSN,MT_length,MT_Queued;

} SBDX_Status_t;

//======================== 3. Macro Definitions =========================================

//GPIO Pins
#define IR_OnOff_Pin GPIO_PIN_10				//ONOFF Control pin
#define IR_Ring_Pin GPIO_PIN_11					//Ring Indicator Pin
#define IR_NetAv_Pin GPIO_PIN_12				//Network Available Pin

//GPIO Ports
#define IR_OnOff_GPIO_Port GPIOA				//ONOFF GPIO PORT
#define IR_RIng_GPIO_Port GPIOA					//RING INDICATOR GPIO PORT
#define IR_NetAv_GPIO_Port GPIOA				//NETWORK AVAILABILITY GPIO PORT


#define IR_OnOff_PWR_GPIO_Port PWR_GPIO_A		//ONOFF PWR Port

//IRQn Definitions
#define IR_Ring_EXTI_IRQn EXTI15_10_IRQn		//RING INDICATOR IRQn Line
#define IR_NetAv_EXTI_IRQn EXTI15_10_IRQn		//NETWORK AVAILABILTY IRQN LINE

//USART Definitions
#define IR_USART_PORT UART5						//IRIDIUM USART PERIPHERAL
#define IR_TX_Pin GPIO_PIN_12					//IRIDIUM USART TX PIN
#define IR_RX_Pin GPIO_PIN_2					//IRIDIUM USART RX PIN
#define IR_TX_GPIO_Port GPIOC					//IRIDIUM USART TX GPIO PORT
#define IR_RX_GPIO_Port GPIOD					//IRIDIUM USART RX GPIO PORT

//TIM Definitions
#define IR_TIM_PORT TIM3						//IRIDIUM TIMER PERIPHERAL

//Constant Definitions
#define TX_BUFFER_SIZE 2046						//SIZE OF USART TX BUFFER
#define RX_BUFFER_SIZE 2046						//SIZE OF USART RX BUFFER
#define RM_BUFFER_SIZE 500						//SIZE OF MESSAGE RECIEVE BUFFER
#define ASCII_MSG_BYTE_LEN 9					//Number of Bytes in  the string "AT+SBDWT="

//========================== 4. Global Variables ==========================================
int8_t IR_RX_Flag;				//Flag showing status of USART DMA Recieve
int8_t IR_TIM_IDLE_Timeout;		//Flag showing status of Reciever Timout
Session_t Session_Flag;			//Flag showing if the AT command is a special command
uint32_t IR_length;			//Recieved USART data length
uint32_t msg_len;				//AT return message length

//============================= 5. Handlers =============================================
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;

DMA_HandleTypeDef hdma_uart5_rx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel2;

//============================ 6. Data Buffers ==========================================

uint8_t IR_RX_Buffer[RX_BUFFER_SIZE];
uint8_t IR_TX_Buffer[TX_BUFFER_SIZE];
uint8_t RM_Buffer[RM_BUFFER_SIZE];

//======================== 7. Functions Prototypes =======================================



/*
 * Function Name IR_Status_t IR_Init_Module(void);
 *
 * @brief: Initialises the peripherals on the Device for UART DMA Circular buffer with Slave Reset.
 *
 *		   UART SETTIINGS:
 * 							baud rate:	19200 bit/s
 * 							data bits:	8	 bits
 * 							stop bits:	1	 bit
 * 							parity:		None
 *
 * 		  The Function will initialise the corresponding peripherals and transmit the AT command "AT\r".
 * 		  This serves as an acknowledgement from the device. If it returns the string "OK", the acknowledgemnet
 * 		  was successful and the device is online. Any other value will result in an error return status
 *
 * @param:  void
 *
 * @return: IR_Status t return status of the function
 */
IR_Status_t IR_Init_Module(void);

/*
 * Function Name IR_Status_t IR_DeInit_Module(void);
 *
 * @brief: Deinitialises Iridium Peripherals and sets registers to reset state, then sets device to sleep mode.
 *
 * @param: void
 *
 * @return: IR_Status_t return status of the function
 */
IR_Status_t IR_DeInit_Module(void);

/*
 * Function Name IR_Status_t send_AT_CMD(char* cmd);
 *
 * @brief: Sends an AT Command String to the Iridium Modem over USART. The device then recieves the return message
 * 		   based on the command. If the return message is valid, the message will be stored in the RM_Buffer. This
 * 		   function uses the USART DMA in a circular buffer with Slave reset to allow for variable data to be sent
 * 		   through the DMA
 *
 * @param: cmd - AT Command string. Must contain a "\r" terminator
 *
 * @return: IR_Status_t return status of the function
 */
IR_Status_t IR_send_AT_CMD(char* cmd);


/*
 * Function Name IR_Status_t get_Signal_Strength(uint8_t* signal_Strength);
 *
 * @brief: Gets the Network Signal strength from the modem by transmitting the string "AT+CSQ\r"
 *		   The return message contains a number from 0 - 5 indicating the network strength with
 *		   5 being the strongest, 1 being the weakest and 0 being no signal.
 *
 * @param: signal_Strength - pointer to uint8_t variable to hold the signal strength value
 *
 * @return: IR_Status_t return status of the function
 */
IR_Status_t IR_get_Signal_Strength(uint8_t* signal_Strength);

/*
 * Function Name: IR_Status_t start_SBD_Session(SBDX_Status_t* sbd);
 *
 * @brief: This function is used to create an SBD session. This allows for Data in the MO buffer to be transmitted
 * 		   via the satelite network to the ROCKBLOCK portal. It is then directed to a specified email address as a
 * 		   payload. This function returns an sbd status string.
 * 		   NOTE:
 *	  	Data must be less than 340 bytes in length
 *		Network availability does not result in success
 *  	Successful query will return: +SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MT queued>
 *
 *		MO status:
 *		 	 * 0 - 2 successful transmit
 *		 	 * 32 - no network available
 *		 	 * anything else is a failure
 *		 MOMSN:
 *		 	 *Mobile Originated Message Sequence Number
 *		 	 *incremented each time successful
 *		 MT Status:
 *		 	 * 0 - no message to receive
 *		 	 * 1 - message successfully received
 *		 	 * 2 - error checking mailbox
 *		 MTMSN:
 *		 	 *The Mobile Terminated Message Sequence Number
 *		 MT length
 *		 	 *Number of bytes received
 *		 	 *Count of mobile terminated SBD messages waiting at the GSS to be transferred to the ISU.
 *
 * @param: sbd - SBDX_Status_t structure that holds the return status from the SBD session
 *
 * @return:	IR_Status_t - status of the function
 */
IR_Status_t IR_start_SBD_Session(SBDX_Status_t* sbd);

/*
 * Function Name IR_Status_t send_Bin_String(uint8_t* bin_string,uint32_t len);
 *
 * @brief: Uploads binary data to the Iridium Message buffer. Binary messages must be uploaded with a 2 byte
 * 		   checksum. This is caluclated by summing the bytes in the entire message and taking the 2 least
 * 		   significant bytes. The modem returns a status to show whether the message upload was successful.
 * 		   The status messages are as follows:
 * 		   0	SBD message successfully written to the ISU.
 *		   1	SBD message write timeout. An insufficient number of bytes were transferred to ISU during
 *				the transfer period of 60 seconds.
 *		   2	SBD message checksum sent from DTE does not match the checksum calculated at the ISU.
 *		   3	SBD message size is not correct. The maximum mobile originated SBD message length is 340 bytes. The minimum mobile originated SBD message length is 1 byte.
 *
 * @param: bin_string - Pointer an 8 bit data buffer cotaining the message to be uploaded.
 *
 * @param: len - size of message in bytes
 *
 * @return: IR_Status_t  - status of the function
 */
IR_Status_t IR_send_Bin_String(uint8_t* bin_string,uint32_t len);

/*
 * Function Name IR_Status_t send_String(char* string);
 *
 * @brief: Uploads a Character array in ASCII format to the Message buffer. ASCII messages are
 * 		   Quite Data heavy as each character in the string occupies 1 byte. It is advisable to
 * 		   use the send_Bin_String() function if sending numbers or measurements.
 *
 * @param: string - buffer containing null-terminated ASCII message
 *
 * @return: IR_Status_t - status of the function
 */
IR_Status_t IR_send_String(char* string);

/*
 * Function Name
 *
 * @brief: Downloads data from the MT buffer on the Iridium Modem.
 * 		   Function creates an SBD session and evaluates the sbd status message returns.
 * 		   If there is data available in the buffer, The data will be downloaded using the
 * 		   command AT+SBDRT. Satelite messages are queued and sent to the modem one at a time
 * 		   a counter variable is sent that indicates the number of messages left in the queue
 * 		   everytime a message is downloaded. Reading the message clears the buffer and queues the
 * 		   next message automatically
 *
 * @param: MSG_Buff - pointer to buffer to store data
 * @param: MSG_BUFF_SIZE - size of the data to be downloaded
 * @param: num_messages - pointer to location to store the number of messages left in the queue
 *
 * @return: IR_Status_t - status of the function
 */
IR_Status_t IR_recieve_String(uint8_t* MSG_Buff,uint32_t MSG_BUFF_SIZE, uint16_t *num_messages);

//================ 8. IRQ Handler Functions Prototypes ==================================

void DMA_Iridium_Periph_IRQHandler(UART_HandleTypeDef *huart);
void DMA_Iridium_MEM_IRQHandler(DMA_HandleTypeDef *hdma_mem);
void USART_RTO_IRQHandler(TIM_HandleTypeDef *htim);
void USART_Iridium_IRQHandler(UART_HandleTypeDef *huart);

void Iridium_ControlPin_IRQHandler(void);

#endif /* HAL_IRIDIUM_H_ */

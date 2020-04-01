/*
 * DATAFLASH.h
 *
 *  Created on: Nov 21, 2019
 *      Author: Administrator
 */

#ifndef INC_DATAFLASH_H_
#define INC_DATAFLASH_H_

#endif /* INC_DATAFLASH_H_ */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_uart.h"
#include <stdint.h>

/** Notes on the use of the library (with Example Code):
 * 	In all the functions that are used to communicate with the Dataflash AT45DB641E Chip the control
 * 	of the chip select is done for you but does need to be edited if the pinouts are changed from the
 * 	original purpose of this library (UCT SHARC). The data inputs to these functions must be arrays of
 * 	type uint8_t. Even if you are writing only one Byte at a time, you must input a one element array.
 * 	These functions also have a built in delay that checks if the chips are busy. This usually does not
 * 	noticeably affect performace provided you do not do a sector or chip erase. Sector erases can take up
 * 	to 6.5 seconds and Chip Erases can take up to 208s.
 *
 * 	It is desgined to work with the stm32L476 Nucleo-64 Board but besides have to reconfigure pinouts
 * 	and some different drivers it should work with other boards and uC series from stm32.
 * 	Also it is advised to set the NSS to software and configure the NSS pins as GPIO outputs instead of SPI NSS.
 * 	Also the drivers from this project must be in the same directory as file (can be different folder but must be
 * 	part of project)
 *
 * 	The library should be used in the following way:
 * 	0. Each of the Chips available should have their page size configured using FLASH_CONFIG_PageSize()
 * 	1. The address should be set to 0x00 00 00 using FLASH_SetAddress()
 * 	2. Then using the WRITE set of FUNCTIONS the chips may be written to.
 * 	2.1 FLASH_WRITE_Page() and FLASH_WRITE_ReadModifyWrite() are the two most useful WRITE functions
 * 	2.2 All the WRITE and ERASE functions return 0 if the WRITE was successful and 1 if unsuccessful
 * 	2.3 This should be checked before using FLASH_IncAddress(size of data written)
 * 	2.4 Also the MAX_ADDRESS_FLAG must be Checked to ensure data is not overwritten.
 * 		If the Flag is = 1. The next available chip should be written to and the Should be reset.
 *
 * 	3. FLASH_READ_Page() can be used to read a page of data from the flash chip
 * 	3.1 FLASH_IncAddress() should also be called after reading a page of data and should
 * 		have the size of the data read passed into the function.
 *
 * 	4. Checking for correct WRITE operation can be done using FLASH_ADDITIONAL_BufferPageCompare()
 * 		which will return 1 if the buffer used to the most recent page in memory doesn't matches that page in memory.
 * 		This should be called immediately after FLASH_WRITE_Page() or any function that WRITEs through
 * 		a buffer or transfers a Buffer to Page or visa versa.
 * 		You can also use the EPE bit in the Status Register (bit 5 of byte 2) or FLASH_ADDITIONAL_GetEPEBit().
 * 		This bit will be 1 if any ERASE/WRITE Operation was unsuccessful. This function is called inside all
 * 		ERASE/WRITE functions and the value is return through those functions.
 *
 * 		There is more detailed documentation of each function in DATAFLASH.c
 *
 * 		//Example Code
 * 		//Configure Page Size
 * 		FLASH_CONFIG_PageSize(256);
 *
 * 		//Set Address
 * 		FLASH_SetAddress(0x00,0x00,0x00);
 *
 * 		//Populate Array
 * 		uint8_t Data_t256[256];
 * 		for(int i = 0 ; i<256; i++)
 * 		{
 * 			Data_t256[i] = 0x01;
 * 		}
 *
 * 		//WRITE a page of data
 * 		FLASH_WRITE_Page(2, BUFFER2, Data_t256);
 *
 * 		//Modify page of Data
 * 		uint8_t Data_t5[5] = {0xFF,0x0F,0x03,0x08,0x20};
 * 		FLASH_WRITE_ReadModifyWrite(2, BUFFER1, Data_t5, 5);
 *
 * 		//WRITE 5 pages of data
 * 		for(int i = 0 ; i<5;i++)
 * 		{
 * 			FLASH_IncAddress(256);
 * 			FLASH_WRITE_Page(2,BUFFER2,Data_t256);
 * 		}
 *
 * 		//READ 6 Pages of newly Written data
 * 		FLASH_SetAddress(0x00,0x00,0x00);
 * 		uint8_t* Data_p;
 * 		uint8_t	Data_r256[256];
 * 		for(int i = 0; i<6;i++)
 * 		{
 * 			Data_p = FLASH_READ_Page(2);
 * 			for(int j = 0; j<256 ; j++)
 * 			{
 * 				Data_r256[j] = *(Data_p+i);
 * 			}
 * 			//Do something with Data_r256 because it will be repopulated next iteration
 * 			FLASH_IncAddress(256);
 * 		}
 *
 * 		//ERASE 6 Pages of newly Written Data
 * 		FLASH_SetAddress(0x00,0x00,0x00);
 * 		for(int i = 0; i<6;i++)
 * 		{
 * 			FLASH_ERASE_Page(2);
 * 			FLASH_IncAddress(256);
 * 		}
 */

typedef enum
{
	CS_OPEN = 0,
	CS_CLOSED = 1
} CS_State_t;

//CONFIGURATION DEFINES
//RCC CLOCK TO PORT ENABLE LINE
#define RCC_GPIOA_CLK_ENABLE 1	 	//1 = enabled, 0 = disabled
#define RCC_GPIOB_CLK_ENABLE 1	 	//1 = enabled, 0 = disabled
#define RCC_GPIOC_CLK_ENABLE 1	 	//1 = enabled, 0 = disabled
#define RCC_GPIOD_CLK_ENABLE 0	 	//1 = enabled, 0 = disabled
#define RCC_GPIOE_CLK_ENABLE 0	 	//1 = enabled, 0 = disabled
#define RCC_GPIOF_CLK_ENABLE 0	 	//1 = enabled, 0 = disabled
#define RCC_GPIOG_CLK_ENABLE 0	 	//1 = enabled, 0 = disabled
#define RCC_GPIOH_CLK_ENABLE 1	 	//1 = enabled, 0 = disabled

//Control Pin Definitions
#define GPIO_CHIP_1_CS	GPIO_PIN_0
#define GPIO_CHIP_2_CS	GPIO_PIN_1
#define GPIO_CHIP_3_CS	GPIO_PIN_0
#define GPIO_CHIP_4_CS  GPIO_PIN_4

#define GPIO_CHIP_WP 	GPIO_PIN_3
//control Pin GPIO port Definitions
#define GPIO_CHIP_1_CS_PORT GPIOC
#define GPIO_CHIP_2_CS_PORT GPIOC
#define GPIO_CHIP_3_CS_PORT GPIOB
#define GPIO_CHIP_4_CS_PORT GPIOA
#define GPIO_WP_PORT GPIOC


//GPIO PIN STATE INIT
#define GPIOA_PINS 					((uint16_t)0x0010)	 	//1 = High, 0 = Low
#define GPIOB_PINS 					((uint16_t)0x0001)	 	//1 = High, 0 = Low
#define GPIOC_PINS 					((uint16_t)0x000B)	 	//1 = High, 0 = Low
#define GPIOD_PINS 					((uint16_t)0x0000)	 	//1 = High, 0 = Low
#define GPIOE_PINS 					((uint16_t)0x0000)	 	//1 = High, 0 = Low
#define GPIOF_PINS 					((uint16_t)0x0000)	 	//1 = High, 0 = Low
#define GPIOG_PINS 					((uint16_t)0x0000)	 	//1 = High, 0 = Low
#define GPIOH_PINS 					((uint16_t)0x0000)	 	//1 = High, 0 = Low

/* CONFIRGURING OF PIN MODER AND PUPDR SHOULD BE DONE IN SOURCE FILE UNDER
 * INDIVIDUAL PIN CONFIGURATIONS IN THE MX_GPIO_Init(). USE DEFAULT SETTINGS AS
 * GUIDE FOR SYNTAX.
*/

//SPI CONFIGURATION
//CHOOSE SPI PERIPHERALS
#define SPI1_ENABLED 0 	//1 = enabled, 0 = disabled
#define SPI2_ENABLED 1 	//1 = enabled, 0 = disabled
#define SPI3_ENABLED 0 	//1 = enabled, 0 = disabled

//CONFIGURE SPI1
#define SPI_MODE 							SPI_MODE_MASTER
#define SPI_DIRECTION 						SPI_DIRECTION_2LINES
#define SPI_DATASIZE 						SPI_DATASIZE_8BIT
#define SPI_CLKPolarity 					SPI_POLARITY_LOW
#define SPI_CLKPhase 						SPI_PHASE_1EDGE
#define SPI_NSS 							SPI_NSS_SOFT
#define SPI_BAUDRATEPRESCALER 				SPI_BAUDRATEPRESCALER_8
#define SPI_FIRSTBIT 						SPI_FIRSTBIT_MSB
#define SPI_TIMODE 							SPI_TIMODE_DISABLE
#define SPI_CRCCALCULATION 					SPI_CRCCALCULATION_DISABLE
#define SPI_CRCPOLYNOMIAL	 				7
#define SPI_CRCLENGTH	 					SPI_CRC_LENGTH_DATASIZE
#define SPI_NSSPMODE	 					SPI_NSS_PULSE_ENABLE


/*SPI pins are configured when HAL_SPI_Init() is called in DATAFLASH.c.
 * HAL_SPI_Init() (defined in stm32l4xx_hal_spi.c) calls HAL_SPI_MspInit()
 * which is defined in stm32l4xx_hal_msp.c. The pins used for the SPI interface
 * are set to PB13 -> SCK, PB14 -> MISO, PB15 -> MOSI by default.
 * */

//UART/USART CONFIGURATION
//CHOOSE UART/USART PERIPHERALS
#define USART1_ENABLED 	0 	//1 = enabled, 0 = disabled
#define USART2_ENABLED	1 	//1 = enabled, 0 = disabled
#define USART3_ENABLED 	0 	//1 = enabled, 0 = disabled
#define UART4_ENABLED 	1 	//1 = enabled, 0 = disabled
#define UART5_ENABLED 	0 	//1 = enabled, 0 = disabled

//CONFIGURE USART1
#define USART_BaudRate  		115200
#define USART_WordLength  		UART_WORDLENGTH_8B;
#define USART_StopBits  		UART_STOPBITS_1;
#define USART_Parity  			UART_PARITY_NONE;
#define USART_Mode  			UART_MODE_TX_RX;
#define USART_HwFlowCtl  		UART_HWCONTROL_NONE;
#define USART_OverSampling  	UART_OVERSAMPLING_16;
#define USART_OneBitSampling  	UART_ONE_BIT_SAMPLE_DISABLE;
#define USART_AdvFeatureInit  	UART_ADVFEATURE_NO_INIT;

/**CONFIGURE UART/USART PINS
 * To change the pinouts find the stm32L4xx_hal_msp.c file in src and go to HAL_UART_MspInit()
 * and Change the pins and port based on the alternate function map provided.
 * */


//PRIVATE DEFINES
#define DUMMYBYTE ((uint8_t)0xFF) //Used in command structures not for user use
#define BUFFER1 1	//Used to quickly choose buffer 1
#define BUFFER2 2	//Used to quickly choose buffer 2

//Can help with address management if the MS, middle or LS Bytes need to be isolated
#define BYTE_Mask_MSB 	0xFF0000
#define BYTE_Mask_MID 	0x00FF00
#define BYTE_Mask_LSB 	0x0000FF

//Last address number on each chip
#define FLASH_MAX_ADDRESS 0x7FFFFF

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* Private function prototypes -----------------------------------------------*/
void MX_GPIO_Init(void);			//Configure GPIO ports and Pins
HAL_StatusTypeDef MX_SPI_Init(SPI_TypeDef *SPIx, SPI_HandleTypeDef *hspi);		//Configure SPI Peripherals (Pins are configured in stm32l4xx_hal_msp.c under HAL_SPI_MspInit()
HAL_StatusTypeDef MX_UART_Init(USART_TypeDef *USARTx, UART_HandleTypeDef *huart);
void GPIO_PIN_STATE_Init(void);
void GPIO_RCC_Init(void);

//UART FUNCTIONS
void UART_SendData(uint8_t* Data_tx, int size);
uint8_t* UART_ReceiveDataPage();
uint8_t* UART_ReceiveDataByte();

//MISCELLANEOUS
void FLASH_ChipSelect_setState(int ChipNumber, CS_State_t state); 	//sets the CS pin for any chip high/low
uint8_t* FLASH_GetStatusRegister(int ChipNumber);					//Sets and Returns chip Status Register
void FLASH_Delay(int ChipNumber);									//Causes a delay when called that ends when the chip is no longer busy
int FLASH_GetPageSize();											//Returns the Current PageSize

//ADDRESS MANAGEMENT
void FLASH_SetAddress(uint8_t MSB,uint8_t MID,uint8_t LSB);	//Sets the Current Address
void FLASH_IncAddress(int size);							//Increments the Current address by size (number of bytes of data entered)
int FLASH_GetAddress();										//returns the Current Address
int FLASH_GetMaxAddressFlag();								//returns the MAX_ADDRESS FLAG if Current Address has gone over the limit
void FLASH_ResetMaxAddressFlag();							//Resets MAX_ADDRESS FLAG

//READ
uint8_t* FLASH_READ_Page(int ChipNumber);					//Reads one page of data from current address
uint8_t* FLASH_READ_BufferHF(int ChipNumber, int BUFFERx);	//Reads one page of data from buffer (buffers are the size of one page) at a High Frequency
uint8_t* FLASH_READ_BufferLF(int ChipNumber, int BUFFERx);	//Reads one page of data from buffer (buffers are the size of one page) at a Low Frequency

//WRITE
int FLASH_WRITE_Page(int ChipNumber, int BUFFERx, uint8_t* data);						//Writes one page of data to current address
int FLASH_WRITE_ReadModifyWrite(int ChipNumber, int BUFFERx, uint8_t* Data, int size);	//Writes a user chosen number of bytes to current address
int FLASH_WRITE_PageOrByte_NoErase(int ChipNumber,  uint8_t* Data, int size);			//Writes a user chosen number of bytes to current address, current page needs to be erased first
int FLASH_WRITE_Buffer(int ChipNumber, int BUFFERx, uint8_t* Data, int size);			//Writes a user chosen number of bytes to buffer of users choice
int FLASH_WRITE_BufferToPage(int ChipNumber, int BUFFERx);								//Transfer the current contents of a buffer of users choice to a page at current address
int FLASH_WRITE_BufferToPage_NoErase(int ChipNumber, int BUFFERx);						//Transfer the current contents of a buffer of users choice to a page at current address, page needs to be erased first

//ERASE
int FLASH_ERASE_Page(int ChipNumber);		//Erases one Page at current address, takes 7-25ms
int FLASH_ERASE_Block(int ChipNumber);		//Erasses one Block at current address, takes 25-50ms
int FLASH_ERASE_Sector(int ChipNumber);		//Erases one Sector at current address,	takes 2.5-6.5s
int FLASH_ERASE_Chip(int ChipNumber);		//Erases entire Chip, takes 80-208s

//SUSPEND AND RESUME
void FLASH_SUSPEND(int ChipNumber);	//Suspends the current ERASE/WRITE operation
void FLASH_RESUME(int Chipnumber);	//Resumes first the suspended WRITE operation and if called again resumes the suspended erase opertion

//ADDITIONAL COMMANDS
void FLASH_ADDITIONAL_PageToBuffer(int ChipNumber, int BUFFERx);		//Tranfers page at current address to buffer of users choice
int FLASH_ADDITIONAL_PageBufferCompare(int ChipNumber, int BUFFERx);	//Compares the contents of one buffer to page at current address, returns 1 if no match
void FLASH_ADDITIONAL_AutoPageRewrite(int ChipNumber, int BUFFERx);		//Rewrites page at current address, should be done every 50000 ERASE/WRITE operations to a page
int FLASH_ADDITIONAL_GetEPEbit(int ChipNumber);							//Gets the EPE (ERASE_PROGRAM_ERROR) bit from Status Register

//CONFIGURE DATAFLASH
void FLASH_CONFIG_PageSize(int ChipNumber, int StdOr256);	//Configures page size to standard 264 Bytes per page or 256 Bytes per page




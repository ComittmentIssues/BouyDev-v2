/*
 * DATAFLASH.c
 *
 *  Created on: Nov 21, 2019
 *      Author: Administrator
 */

#include "stm32l4xx_hal.h"
#include "DATAFLASH.h"
#include <stdint.h>


/* Private variables ---------------------------------------------------------*/
//SPI INSTANCES
static SPI_HandleTypeDef hspi1;

static SPI_HandleTypeDef hspi3;
static UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
static UART_HandleTypeDef huart5;

//ADDRESS
static uint8_t Current_Address[3] = {0x00,0x00,0x00};

//Starting addresses of each sector
const uint8_t FLASH_Sector0a[3] = {0x00,0x00,0x00};
const uint8_t FLASH_Sector0b[3] = {0x00,0x08,0x00};
const uint8_t FLASH_Sector1[3]  = {0x04,0x00,0x00};
const uint8_t FLASH_Sector2[3] 	= {0x08,0x00,0x00};
const uint8_t FLASH_Sector3[3]  = {0x0C,0x00,0x00};
const uint8_t FLASH_Sector4[3]	= {0x10,0x00,0x00};
const uint8_t FLASH_Sector5[3]	= {0x14,0x00,0x00};
const uint8_t FLASH_Sector6[3]	= {0x18,0x00,0x00};
const uint8_t FLASH_Sector7[3]	= {0x1C,0x00,0x00};
const uint8_t FLASH_Sector8[3]	= {0x20,0x00,0x00};
const uint8_t FLASH_Sector9[3]	= {0x24,0x00,0x00};
const uint8_t FLASH_Sector10[3] = {0x28,0x00,0x00};
const uint8_t FLASH_Sector11[3] = {0x2C,0x00,0x00};
const uint8_t FLASH_Sector12[3] = {0x30,0x00,0x00};
const uint8_t FLASH_Sector13[3] = {0x34,0x00,0x00};
const uint8_t FLASH_Sector14[3]	= {0x38,0x00,0x00};
const uint8_t FLASH_Sector15[3]	= {0x3C,0x00,0x00};
const uint8_t FLASH_Sector16[3]	= {0x40,0x00,0x00};
const uint8_t FLASH_Sector17[3]	= {0x44,0x00,0x00};
const uint8_t FLASH_Sector18[3]	= {0x48,0x00,0x00};
const uint8_t FLASH_Sector19[3]	= {0x4C,0x00,0x00};
const uint8_t FLASH_Sector20[3] = {0x50,0x08,0x00};
const uint8_t FLASH_Sector21[3] = {0x54,0x00,0x00};
const uint8_t FLASH_Sector22[3] = {0x58,0x00,0x00};
const uint8_t FLASH_Sector23[3] = {0x5C,0x00,0x00};
const uint8_t FLASH_Sector24[3]	= {0x60,0x00,0x00};
const uint8_t FLASH_Sector25[3]	= {0x64,0x00,0x00};
const uint8_t FLASH_Sector26[3]	= {0x68,0x00,0x00};
const uint8_t FLASH_Sector27[3]	= {0x6C,0x00,0x00};
const uint8_t FLASH_Sector28[3]	= {0x70,0x00,0x00};
const uint8_t FLASH_Sector29[3]	= {0x74,0x00,0x00};
const uint8_t FLASH_Sector30[3] = {0x78,0x08,0x00};
const uint8_t FLASH_Sector31[3] = {0x7C,0x00,0x00};

//REGISTERS
static uint8_t FLASH_STATUS_REGISTER[2] = {0x00,0x00};

//BUFFERS
static uint8_t Data_r256[256] = {0x00}; //256 Byte Buffer
static uint8_t Data_r264[264] = {0x00};	//264 Byte Buffer
static uint8_t Data_rByte[1] = {0x00}; 	//Single Byte Buffer
static int _264or256 = 256;				//Keeps track of page size

//FLAGS
static int FLASH_MAX_ADDRESS_FLAG = 0;	//Keeps track of if max address has been passed

//------------------------------------------FUNCTION DEFINITIONS------------------------------------------------------------------------------------

//---------------------------------------------MISCELLEANOUS-----------------------------------------------------------------

/**
  * @DESCRIPTION Opens communications with one of DATAFLASH slave devices.
  * If more chips are added this function needs to be expanded to include
  * those chips. Just Copy one of cases and change the GPIO settings to
  * match the GPIO pin and port used for the new chip.
  */
void FLASH_ChipSelect_setState(int ChipNumber, CS_State_t state)
{
	/*If more Dataflash chips are added
	 * this needs to be expanded to include their respective NSS pins*/
	switch (ChipNumber)
	{
		case 1:
	    	HAL_GPIO_WritePin(GPIO_CHIP_1_CS_PORT, GPIO_CHIP_1_CS, state);
	      break;
	    case 2:
	    	HAL_GPIO_WritePin(GPIO_CHIP_2_CS_PORT, GPIO_CHIP_2_CS, state);
	      break;
	    case 3:
	    	HAL_GPIO_WritePin(GPIO_CHIP_3_CS_PORT, GPIO_CHIP_3_CS, state);
	      break;
	    case 4:
	    	HAL_GPIO_WritePin(GPIO_CHIP_4_CS_PORT, GPIO_CHIP_4_CS, state);
	      break;
	}
}

/**
  * @DESCRIPTION Closes communications with one of DATAFLASH slave devices.
  * If more chips are added this function needs to be expanded to include
  * those chips. Just Copy one of cases and change the GPIO settings to
  * match the GPIO pin and port used for the new chip.
  */


/**
  * @DESCRIPTION Sets FLASH_STATUS_REGISTER to the Status Register of the selected Chip.
  * FLASH_STATUS_REGISTER is contained in an array containing two bytes formatted to uint8_t.
  * Notable bits include: 	Bit 7 of both Bytes 	-> RDY bit (tells if Chip is busy or not)
  * 					  	Bit 6 of Byte 1 		-> COMP bit (displays result of PageBufferCompare)
  * 					  	Bit 0 of Byte 1			-> Page Size bit (1 is 256, 0 is 264 Standard size)
  * 					  	Bit 5 of Byte 2			-> EPE bit (1 if Erase/Program Error occured in latest ERASE/WRITE operation)
  * 					  	See DataSheet for more information (Page 28-29)
  * 					  	Returns a pointer to FLASH_STATUS_REGISTER
  */
uint8_t* FLASH_GetStatusRegister(int ChipNumber)
{
	//Define Command
	uint8_t command[1] = {0xD7};

	//Open line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit command
	HAL_SPI_Transmit(&hspi2, command, 1, 100);

	//Update Register
	HAL_SPI_Receive(&hspi2, FLASH_STATUS_REGISTER, 2, 100);

	//Close line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Return pointer to FLASH_STATUS_REGISTER
	return FLASH_STATUS_REGISTER;
}

/**
  * @DESCRIPTION Constantly calls 1ms delay if the selected chip is bust with an operation
  * This is called in most functions in this library already to ensure no data corruption occurs.
  */
void FLASH_Delay(int ChipNumber)
{

	FLASH_GetStatusRegister(ChipNumber); 					//Sets FLASH_STATUS_REGISTER
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);	//Turns off OnBoard LED for visual display
	while((FLASH_STATUS_REGISTER[0])<128){					//Checks if RDY bit is off
		FLASH_GetStatusRegister(ChipNumber);				//Sets FLASH_STATUS_REGISTER for next iteration
		HAL_Delay(1);										//1ms Delay while not RDY
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);		//Turns on OnBoard LED to show Delay is finished
}

/**
  * @DESCRIPTION Returns a pointer to the 264 or 256 Byte buffer depending on current page size.
  * Called in READ functions. The Page size int is changed automatically by
  * FLASH_CONFIG_PageSize(). Static function and so is "hidden" from outside use.
  */
static uint8_t* Data_rx()
{
	if (_264or256 == 256){ 	//Checks if Page size is 256
		return Data_r256;	//Returns the 256 Byte buffer
	} else
	{
		return Data_r264;	//Returns the 264 Byte buffer
	}
}

/**
  * @DESCRIPTION Returns the current page size.
  */
int FLASH_GetPageSize()
{
	return _264or256; //Return Page size
}

//---------------------------------------------UART FUNCTIONS------------------------------------------------------------------------------------

void UART_SendData(uint8_t* Data_tx, int size)
{
	//HAL_UART_Transmit(&huart4, Data_tx, size, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, Data_tx, size, HAL_MAX_DELAY);
}

uint8_t* UART_ReceiveDataPage()
{
	HAL_UART_Receive(&huart2, Data_rx(), _264or256, HAL_MAX_DELAY);
	return Data_rx();
}

uint8_t* UART_ReceiveDataByte()
{
	HAL_UART_Receive(&huart2, Data_rByte, 1, HAL_MAX_DELAY);
	return Data_rByte;
}


//---------------------------------------ADDRESS MANAGEMENT-------------------------------------------------

/**
  * @DESCRIPTION Sets the current address to the entered value.
  * The AT45DB641E DATAFLASH chip works with 3 address bytes at a time so this library has a
  * uint8_t 3 element array which contains the current address. The first element Current Address[0]
  * is Most Significant Byte of the address and the last element is the Least Significant byte of the
  * address. Recommended to set the current address to {0x00, 0x00, 0x00} and then use FLASH_IncAddress
  * function to handle addressing of the chip.
  */
void FLASH_SetAddress(uint8_t MSB,uint8_t MID,uint8_t LSB)
{
	Current_Address[0] = MSB;	//Sets the Most Significant Byte of Address
	Current_Address[1] = MID;	//Sets the Middle Byte of Address
	Current_Address[2] = LSB;	//Sets the Least Significant Byte of Address
}

/**
  * @DESCRIPTION Increments the address by the number of Bytes entered.
  * The AT45DB641E DATAFLASH chip works with 3 address bytes at a time so this library has a
  * uint8_t 3 element array which contains the current address. The first element Current Address[0]
  * is Most Significant Byte of the address and the last element is the Least Significant byte of the
  * address. Recommended to be used after a FLASH READ, WRITE or ERASE operation.
  * E.g. if 256 Bytes is written to the chip then call FLASH_IncAddres(256) to move to the next empty
  * address.
  */
void FLASH_IncAddress(int size)
{
	int address = (Current_Address[0]<<16)+(Current_Address[1]<<8)+(Current_Address[2]); //Turns array into 24 bit int
	int new_address = address + size; //Increments address by number of bytes entered
	if (new_address >= FLASH_MAX_ADDRESS)	//Checks if  Address has gone over MAX_ADDRESS
	{
		new_address = new_address - FLASH_MAX_ADDRESS;	//Sets the new address to however much it went over by
		FLASH_MAX_ADDRESS_FLAG = 1;						//Sets the address flag to tell user that the MAX_ADDRESS has been passed and data overwrite is possible
	}
	Current_Address[0] = (new_address&BYTE_Mask_MSB)>>16; //Reconstructs Address Array
	Current_Address[1] = (new_address&BYTE_Mask_MID)>>8;
	Current_Address[2] = (new_address&BYTE_Mask_LSB);
}

/**
  * @DESCRIPTION Returns a pointer to Current_Address array
  */
int FLASH_GetAddress()
{
	return (Current_Address[0]<<16)+(Current_Address[1]<<8)+(Current_Address[2]); //Return current Address as 24bit int
}

/**
  * @DESCRIPTION Returns 1 if the Maximum address on the chip has been passed by FLASH_IncAddress.
  * If this occurs the DATAFLASH chip will start writing from the beginning of main memory again.
  * This can potentially result in Data corruption or loss. It is recommended to switch to another chip
  * if this FLAG has been raised.
  */
int FLASH_GetMaxAddressFlag()
{
	return FLASH_MAX_ADDRESS_FLAG; //Returns the MAX_ADDRESS_FLAG
}

/**
  * @DESCRIPTION Sets the FLASH_MAX_ADDRESS_FLAG to zero.
  * DATAFLASH chip will start writing from the beginning of main memory again if the end of memory
  * is reached. This can potentially result in Data corruption or loss. It is recommended to switch
  * to another chip if this happens. When switching to another chip use FLASH_ResetMaxAddressFlag to
  * set the FLAG to zero again.
  */
void FLASH_ResetMaxAddressFlag()
{
	FLASH_MAX_ADDRESS_FLAG = 0; //Set MAX_ADDRESS_FLAG to 0
}



//----------------------------------------------READ--------------------------------------------------------

/**
  * @DESCRIPTION READS one page of data from the Current_Address and stores it in uint8_t Data_rx[256].
  * If the next pages needs to READ then call FLASH_IncAddres(256 or 264) and then FLASH_READ_Page() again.
  * The data in Data_rx will be replaced with the new page data. Returns pointer to correct buffer.
  */
uint8_t* FLASH_READ_Page(int ChipNumber)
{
	//Define Command
	uint8_t command[8] = {0xD2, Current_Address[0], Current_Address[1], Current_Address[2], DUMMYBYTE, DUMMYBYTE, DUMMYBYTE, DUMMYBYTE};

	//Delay function if chip is busy
	FLASH_Delay(ChipNumber);

	//Open line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit command
	HAL_SPI_Transmit(&hspi2, command, 8, 800);

	//Update Read Buffer
	HAL_SPI_Receive(&hspi2, Data_rx(), _264or256, HAL_MAX_DELAY);

	//Close line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Return pointer to array using Data_rx() to return correctly sized buffer
	return Data_rx();
}

/**
  * @DESCRIPTION READS one page of data from a buffer of the users choice.
  * HF in function name signifies High Frequency read and supports a SCK of 50 - 85 MHz
  */
uint8_t* FLASH_READ_BufferHF(int ChipNumber, int BUFFERx)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0xD4;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0xD6;
	}
	uint8_t AddressByte = 0x00;
	uint8_t command[5] = {OpCode, DUMMYBYTE, DUMMYBYTE, AddressByte, DUMMYBYTE};

	//Delays if Flash chip is busy
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 5, 500);

	//Receive Data
	HAL_SPI_Receive(&hspi2, Data_rx(), _264or256, HAL_MAX_DELAY);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber, CS_CLOSED);

	//Return Correctly sized buffer using Data_rx()
	return Data_rx();
}

/**
  * @DESCRIPTION READS one page of data from a buffer of the users choice.
  * LF in function name signifies Low Frequency read and supports a SCK of 40 - 50 MHz
  */
uint8_t* FLASH_READ_BufferLF(int ChipNumber, int BUFFERx)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0xD1;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0xD3;
	}
	uint8_t AddressByte = 0x00;
	uint8_t command[5] = {OpCode, DUMMYBYTE, DUMMYBYTE, AddressByte, DUMMYBYTE};

	//Delays if Flash chip is busy
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 5, 500);

	//Receive Data
	HAL_SPI_Receive(&hspi2, Data_rx(), _264or256, HAL_MAX_DELAY);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);
	//Return Correctly sized buffer using Data_rx()
	return Data_rx();
}



//----------------------------------------------WRITE-------------------------------------------------------

/**
  * @DESCRIPTION WRITES one page of data through a buffer of the users choice.
  * This function needs to take in 256/264 Bytes of data in the form of 256/264 element uint8_t array.
  * Return value describes whether WRITE was successful or not. 1 = Error, 0 = Success.
  */
int FLASH_WRITE_Page(int ChipNumber, int BUFFERx, uint8_t* data)
{
	//Writes to page through buffer, if it fills buffer it starts at the beginning
	//therefore can only take 256 bytes at a time
	//Has built in erase
	//Define Command
	uint8_t OpCode;
		if (BUFFERx == BUFFER1){
			OpCode = 0x82;
		} else if (BUFFERx == BUFFER2){
			OpCode = 0x85;
		}
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1], 0x00};

	//Delay function if chip is busy
	FLASH_Delay(ChipNumber);

	//Open line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit command
	HAL_SPI_Transmit(&hspi2, command, 4, 800);

	//Transmit Data
	HAL_SPI_Transmit(&hspi2, data, _264or256, HAL_MAX_DELAY);

	//Close line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if Write was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION WRITES a variable number of bytes through a buffer of the users choice.
  * This function takes in data in the form of a uint8_t array. The size of array needs to be specified
  * and passed into the function.
  */
int FLASH_WRITE_ReadModifyWrite(int ChipNumber, int BUFFERx, uint8_t* Data, int size)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0x58;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0x59;
	}
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1], Current_Address[3]};

	//Delays if Flash chip is busy
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);
	HAL_SPI_Transmit(&hspi2, Data, size, HAL_MAX_DELAY);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if Write was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION WRITES a variable number of bytes to a buffer of the users choice.
  * This function takes in data in the form of a uint8_t array. The size of array needs to be specified
  * and passed into the function.
  */
int FLASH_WRITE_Buffer(int ChipNumber, int BUFFERx, uint8_t* Data, int size)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0x84;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0x87;
	}
	uint8_t command[4] = {OpCode, DUMMYBYTE, DUMMYBYTE, 0x00};

	//Delays if Flash chip is busy
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);
	HAL_SPI_Transmit(&hspi2, Data, size, HAL_MAX_DELAY);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if Write was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION Transfers the contents of a buffer of the users choice to a page in memory.
  * The address of the page is the Current_Address Variable.
  */
int FLASH_WRITE_BufferToPage(int ChipNumber, int BUFFERx)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0x83;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0x86;
	}
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1], DUMMYBYTE};

	//Delays if Flash chip is busy
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if Write was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION WRITES to a page in main memory through Buffer 1 and can write up to 256 Bytes.
  * The size of the uint8_t data array needs to specified and passed in the function.
  */
int FLASH_WRITE_PageOrByte_NoErase(int ChipNumber,  uint8_t* Data, int size)
{
	//Only writes through buffer 1
	//Define Command to be transmitted
	uint8_t OpCode = 0x02;
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1], 0x00};

	//Delays if Flash chip is busy
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);
	HAL_SPI_Transmit(&hspi2, Data, size, HAL_MAX_DELAY);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if Write was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION WRITES Transfers the contents of a buffer of the users choice to a page in memory.
  * The address of the page is the Current_Address Variable. This function has no built in erase and
  * an ERASE function needs to be called before this function is called.
  */
int FLASH_WRITE_BufferToPage_NoErase(int ChipNumber, int BUFFERx)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0x88;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0x89;
	}
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1], DUMMYBYTE};

	//Delays if Flash chip is busy
	FLASH_Delay(ChipNumber);
	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);
	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if Write was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}



//----------------------------------------------ERASE-------------------------------------------------------

/**
  * @DESCRIPTION ERASES a page from the currently selected chip at the Current_Address.
  * Default erase value is 1 for each bit. This operation takes approx. 7 - 35ms.
  */
int FLASH_ERASE_Page(int ChipNumber)
{
	//Define Command
	uint8_t command[4] = {0x81, Current_Address[0], Current_Address[1], DUMMYBYTE};

	//Delay function if chip is busy
	FLASH_Delay(ChipNumber);

	//Open line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit command
	HAL_SPI_Transmit(&hspi2, command, 4, 100);

	//Close line to slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if ERASE was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION ERASES a Block of 8 pages from the currently selected chip at the Current_Address.
  * Default erase value is 1 for each bit. This operation takes approx. 25 - 50ms.
  */
int FLASH_ERASE_Block(int ChipNumber)
{
	//Define Command to be transmitted
	uint8_t OpCode = 0x50;
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1], DUMMYBYTE};

	//Checks if device is busy
	//If busy it delays the function
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if ERASE was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION ERASES a Sector of 128 Blocks from the currently selected chip at the Current_Address.
  * Default erase value is 1 for each bit. This operation takes approx. 2.5 - 6.5s.
  * Note Secto0a only has 1 block and Sector0b has 127 blocks.
  */
int FLASH_ERASE_Sector(int ChipNumber)
{
	//Define Command to be transmitted
	uint8_t OpCode = 0x7C;
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1], DUMMYBYTE};

	//Checks if device is busy
	//If busy it delays the function
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if ERASE was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}

/**
  * @DESCRIPTION ERASES Entire Chip.
  * Default erase value is 1 for each bit. This operation takes approx. 80 - 208s.
  */
int FLASH_ERASE_Chip(int ChipNumber)
{
	//Define Command to be transmitted
	uint8_t command[4] = {0xC7,0x94,0x80,0x9A};

	//Checks if device is busy
	//If busy it delays the function
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	//Returns 1 if ERASE was unsuccessful and 0 if successful. This is determined from EPE bit in the Status Register
	return FLASH_ADDITIONAL_GetEPEbit(ChipNumber);
}



//---------------------------------------SUSPEND AND RESUME-------------------------------------------------

/**
  * @DESCRIPTION SUSPENDS the current WRITE/ERASE operation in a sector so other operations can take place.
  * Will set either PS1 or PS2 bit in the Status Register to 1  (bit 1 or 2).
  */
void FLASH_SUSPEND(int ChipNumber)
{
	//Define Command to be transmitted
	uint8_t command[1] = {0xB0};

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 1, 100);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);
}

/**
  * @DESCRIPTION RESUMES the current WRITE/ERASE operation in a sector so other operations can take place.
  * Will set either PS1 or PS2 bit in the Status Register to 1  (bit 1 or 2). If an ERASE and WRITE are
  * suspended it will resume the WRITE first. This command must be issued again to resume the ERASE.
  */
void FLASH_RESUME(int ChipNumber)
{
	//Define Command to be transmitted
	uint8_t command[1] = {0xD0};

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 1, 100);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

}



//--------------------------------------ADDITIONAL COMMANDS-------------------------------------------------

/**
  * @DESCRIPTION Transfers a page in main memory at Current_Address to a buffer of the users choice.
  * */
void FLASH_ADDITIONAL_PageToBuffer(int ChipNumber, int BUFFERx)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0x53;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0x55;
	}
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1],DUMMYBYTE};

	//If Chip is Busy cause a Delay
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);
}

/**
  * @DESCRIPTION Compares a page in main memory at Current_Address to a buffer of the users choice.
  * Sets the COMP bit in the Status Register to 1 if the page and buffer match.
  * Can be used to check if a successful BufferToPage operation has been performed.
  * */
int FLASH_ADDITIONAL_PageBufferCompare(int ChipNumber, int BUFFERx)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0x60;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0x61;
	}
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1],DUMMYBYTE};

	//Checks if device is busy
	//If busy it delays the function
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);

	///Gets the COMP bit from the STATUS REGISTER and returns it
	FLASH_GetStatusRegister(2);
	int COMP = (FLASH_STATUS_REGISTER[0]&(0x40))>>6;
	return COMP;
}

/**
  * @DESCRIPTION Rewrites the page at Current_Address to ensure data integrity.
  * Needs to be called ever 50 000 WRITE/ERASE operation done to a page.
  * */
void FLASH_ADDITIONAL_AutoPageRewrite(int ChipNumber, int BUFFERx)
{
	//Define Command to be transmitted
	uint8_t OpCode;
	if (BUFFERx == BUFFER1){
		OpCode = 0x58;
	} else if (BUFFERx == BUFFER2){
		OpCode = 0x59;
	}
	uint8_t command[4] = {OpCode, Current_Address[0], Current_Address[1],DUMMYBYTE};

	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);
}

int FLASH_ADDITIONAL_GetEPEbit(int ChipNumber)
{
	FLASH_GetStatusRegister(ChipNumber); 			//Update STATUS REGISTER
	int EPE = (FLASH_STATUS_REGISTER[1]&0x20)>>5;	//Extract EPE bit
	return EPE;										//Return EPE bit

}


//--------------------------------------CONFIGURE DATAFLASH-------------------------------------------------

/**
  * @DESCRIPTION CONFIGURES the page size to 256 Byte or 264 Byte Standard size.
  * */
void FLASH_CONFIG_PageSize(int ChipNumber, int StdOr256)
{
	//Define Command to be transmitted
	uint8_t command[4] = {0};
	if (StdOr256 == 256)
	{
		command[0] = 0x3D;
		command[1] = 0x2A;
		command[2] = 0x80;
		command[3] = 0xA6;
		_264or256 = 256; 	//Sets Page size int to 256
	} else
	{
		command[0] = 0x3D;
		command[1] = 0x2A;
		command[2] = 0x80;
		command[3] = 0xA7;
		_264or256 = 264;	//Sets Page size int to 264
	}

	//Checks if device is busy
	//If busy it delays the function
	FLASH_Delay(ChipNumber);

	//Select Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_OPEN);

	//Transmit Command
	HAL_SPI_Transmit(&hspi2, command, 4, 400);

	//Deselect Slave
	FLASH_ChipSelect_setState(ChipNumber,CS_CLOSED);
}


/**
  * @brief System Clock Configuration done by CubeMX
  * @retval None
  */


/**GPIO AND SPI TYPEDEFs FROM HAL LIBRARY FOR DOCUMENTATION PURPOSES
//SPI Structure
 typedef struct
{
  uint32_t Mode;                !< Specifies the SPI operating mode.
                                     This parameter can be a value of @ref SPI_Mode

  uint32_t Direction;           !< Specifies the SPI bidirectional mode state.
                                     This parameter can be a value of @ref SPI_Direction

  uint32_t DataSize;            !< Specifies the SPI data size.
                                     This parameter can be a value of @ref SPI_Data_Size

  uint32_t CLKPolarity;         !< Specifies the serial clock steady state.
                                     This parameter can be a value of @ref SPI_Clock_Polarity

  uint32_t CLKPhase;            !< Specifies the clock active edge for the bit capture.
                                     This parameter can be a value of @ref SPI_Clock_Phase

  uint32_t NSS;                 !< Specifies whether the NSS signal is managed by
                                     hardware (NSS pin) or by software using the SSI bit.
                                     This parameter can be a value of @ref SPI_Slave_Select_management

  uint32_t BaudRatePrescaler;   !< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock.
                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                     @note The communication clock is derived from the master
                                     clock. The slave clock does not need to be set.

  uint32_t FirstBit;            !< Specifies whether data transfers start from MSB or LSB bit.
                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission

  uint32_t TIMode;              !< Specifies if the TI mode is enabled or not.
                                     This parameter can be a value of @ref SPI_TI_mode

  uint32_t CRCCalculation;      !< Specifies if the CRC calculation is enabled or not.
                                     This parameter can be a value of @ref SPI_CRC_Calculation

  uint32_t CRCPolynomial;       !< Specifies the polynomial used for the CRC calculation.
                                     This parameter must be an odd number between Min_Data = 1 and Max_Data = 65535

  uint32_t CRCLength;           !< Specifies the CRC Length used for the CRC calculation.
                                     CRC Length is only used with Data8 and Data16, not other data size
                                     This parameter can be a value of @ref SPI_CRC_length

  uint32_t NSSPMode;            !< Specifies whether the NSSP signal is enabled or not .
                                     This parameter can be a value of @ref SPI_NSSP_Mode
                                     This mode is activated by the NSSP bit in the SPIx_CR2 register and
                                     it takes effect only if the SPI interface is configured as Motorola SPI
                                     master (FRF=0) with capture on the first edge (SPIx_CR1 CPHA = 0,
                                     CPOL setting is ignored)..
} SPI_InitTypeDef;


//GPIO Structure
typedef struct
{
  uint32_t Pin;        !< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins

  uint32_t Mode;       !< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode

  uint32_t Pull;       !< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull

  uint32_t Speed;      !< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed

  uint32_t Alternate;  !< Peripheral to be connected to the selected pins
                            This parameter can be a value of @ref GPIOEx_Alternate_function_selection
}GPIO_InitTypeDef;
*/
/*SPI pins are configured when HAL_SPI_Init() is called in DATAFLASH.c.
 * HAL_SPI_Init() is called by MX_SPI2_Init (defined in stm32l4xx_hal_spi.c) and calls HAL_SPI_MspInit()
 * which is defined in stm32l4xx_hal_msp.c. The pins used for the SPI interface
 * are set to PB13 -> SCK, PB14 -> MISO, PB15 -> MOSI by default in HAL_SPI_MspInit().
 * */

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
HAL_StatusTypeDef MX_UART_Init(USART_TypeDef *USARTx, UART_HandleTypeDef *huart)
{


		huart->Instance = USARTx;
		huart->Init.BaudRate = USART_BaudRate;
		huart->Init.WordLength = USART_WordLength;
		huart->Init.StopBits = USART_StopBits;
		huart->Init.Parity = USART_Parity;
		huart->Init.Mode = USART_Mode;
		huart->Init.HwFlowCtl = USART_HwFlowCtl;
		huart->Init.OverSampling = USART_OverSampling;
		huart->Init.OneBitSampling = USART_OneBitSampling;
		huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(huart) != HAL_OK)
		{
			return HAL_ERROR;
		}
		return HAL_OK;
}


/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
HAL_StatusTypeDef MX_SPI_Init(SPI_TypeDef *SPIx, SPI_HandleTypeDef *hspi)
{

	//For stm32L4 Nucleo 64 board there are 3 SPI peripherals
		/* SPI1 parameter configuration*/
		hspi->Instance = SPIx;										//Determines Which SPI is being used
		hspi->Init.Mode = SPI_MODE;									//Determines if in Master or Slave Mode
		hspi->Init.Direction = SPI_DIRECTION;						//Determines if Full or Half Duplex is used
		hspi->Init.DataSize = SPI_DATASIZE;							//Determines Datasize to be sent across
		hspi->Init.CLKPolarity = SPI_CLKPolarity ;					//Determines if Clock idles High or Low
		hspi->Init.CLKPhase = SPI_CLKPhase;							//Determines if signal sampled on Rising or falling edge of clock
		hspi->Init.NSS = SPI_NSS;									//Determines if Slave select is Hardware or Software enabled
		hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER;		//Determines the SPI and therefore Slave clock speed
		hspi->Init.FirstBit = SPI_FIRSTBIT;							//Determines if LSB or MSB first
		hspi->Init.TIMode = SPI_TIMODE;								//Determines if TI mode is used
		hspi->Init.CRCCalculation = SPI_CRCCALCULATION;				//Enables CRC Calculation
		hspi->Init.CRCPolynomial = SPI_CRCPOLYNOMIAL;				//Sets the CRC Polynomial
		hspi->Init.CRCLength = SPI_CRCLENGTH;						//Sets the CRC length
		hspi->Init.NSSPMode = SPI_NSSPMODE;							//Sets the NSSP mode
		if (HAL_SPI_Init(hspi) != HAL_OK)
		{
			return HAL_ERROR;
		}

		return HAL_OK;
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //GPIO Ports Clock Enable
  GPIO_RCC_Init();

  //SET PIN STATE
  GPIO_PIN_STATE_Init();

  /*Configure J6 Input pin
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
   */

  /*Configure GPIO pins : PC0 PC1 PC3 */
  /*
   * Chips 1,2 have CS lines running to PC0 and PC1
   * All chips have wp on PC3
   */
  GPIO_InitStruct.Pin =  GPIO_CHIP_1_CS|GPIO_CHIP_2_CS|GPIO_CHIP_WP;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_CHIP_1_CS_PORT, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 */
  //for Chip 4 CS
  GPIO_InitStruct.Pin = GPIO_CHIP_2_CS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_CHIP_4_CS_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  //for Chip 3 CS
  GPIO_InitStruct.Pin = GPIO_CHIP_3_CS;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_CHIP_3_CS_PORT, &GPIO_InitStruct);

}

void GPIO_RCC_Init(void)
{
	//GPIOA
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOA_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOA_CLK_DISABLE();
	  }

	  //GPIOB
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOB_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOB_CLK_DISABLE();
	  }

	  //GPIOC
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOC_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOC_CLK_DISABLE();
	  }

	  //GPIOD
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOD_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOD_CLK_DISABLE();
	  }

	  //GPIOE
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOE_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOE_CLK_DISABLE();
	  }

	  //GPIOF
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOF_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOF_CLK_DISABLE();
	  }

	  //GPIOG
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOG_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOG_CLK_DISABLE();
	  }

	  //GPIOH
	  if(RCC_GPIOA_CLK_ENABLE){
		  __HAL_RCC_GPIOH_CLK_ENABLE();
	  } else {
		  __HAL_RCC_GPIOH_CLK_DISABLE();
	  }

}

void GPIO_PIN_STATE_Init()
{
	/*Configure GPIOS pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, GPIOA_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, ~GPIOA_PINS, GPIO_PIN_RESET);

	  /*Configure GPIOB pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, GPIOB_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB, ~GPIOB_PINS, GPIO_PIN_RESET);

	  /*Configure GPIOC pin Output Level */
	  HAL_GPIO_WritePin(GPIOC, GPIOC_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOC, ~GPIOC_PINS, GPIO_PIN_RESET);

	  /*Configure GPIOD pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, GPIOD_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, ~GPIOD_PINS, GPIO_PIN_RESET);

	  /*Configure GPIOE pin Output Level */
	  HAL_GPIO_WritePin(GPIOE, GPIOE_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, ~GPIOE_PINS, GPIO_PIN_RESET);

	  /*Configure GPIOF pin Output Level */
	  HAL_GPIO_WritePin(GPIOF, GPIOF_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOF, ~GPIOF_PINS, GPIO_PIN_RESET);

	  /*Configure GPIOG pin Output Level */
	  HAL_GPIO_WritePin(GPIOE, GPIOG_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, ~GPIOG_PINS, GPIO_PIN_RESET);

	  /*Configure GPIOH pin Output Level */
	  HAL_GPIO_WritePin(GPIOF, GPIOH_PINS, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOF, ~GPIOH_PINS, GPIO_PIN_RESET);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


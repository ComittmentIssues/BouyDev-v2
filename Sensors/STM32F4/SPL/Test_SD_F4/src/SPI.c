/*
 * SPI.c
 *
 *  Created on: Sep 20, 2019
 *      Author: jamie
 */

#include "stm32f4xx.h"
#include "SPI.h"


void init_SPI(void)
{
	//ENABLE SPI from clock source
#ifdef USE_SPI1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
#else
#ifdef USE_SPI2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
#endif
#ifdef USE_SPI3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
#endif
#endif

	//enable GPIO port x for AF functions
	RCC_AHB1PeriphClockCmd(RCC_GPIO_SPIPeriph,ENABLE);
	//optional: enable custom CS pin (only if GPIO port of desired pin is different to the SPI GPIO port
#ifdef USE_CUSTOM_CS
	RCC_AHB1PeriphClockCmd(RCC_GPIO_CSPeriph, ENABLE);
#endif
	//SPI Init
	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_Init(SPI_x,&SPI_InitStruct);
	SPI_Cmd(SPI_x, ENABLE);

	//GPIO pin config
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = (MISO_Pin|MOSI_Pin | SCLK_Pin);
	GPIO_Init(SPI_GPIO,&GPIO_InitStruct);

	//GPIO AF
	GPIO_PinAFConfig(SPI_GPIO,SCLK_Pinsrc,GPIO_AF_SPIx);
	GPIO_PinAFConfig(SPI_GPIO,MOSI_Pinsrc,GPIO_AF_SPIx);
	GPIO_PinAFConfig(SPI_GPIO,MISO_Pinsrc,GPIO_AF_SPIx);

	//GPIO Control Pin config
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = (CS_Pin);
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	//GPIO CS init
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = CS_Pin;
	GPIO_Init(SPI_GPIO,&GPIO_InitStruct);

	//disable Slave
	SS_DISABLE;
}

uint8_t SPI_Transfer(uint8_t byte, SPI_TypeDef* SPIx)
{
	//write Data to register
	SPIx->DR = byte;
	//TODO: Time out Condition
	while(!(SPIx->SR & SPI_SR_TXE));
	//wait until data recieved
	while(!(SPIx->SR & (SPI_SR_RXNE)));
	//wait for transfer to finish
	while(SPIx->SR & SPI_SR_BSY);
	return (SPIx->DR&0xFF);
}

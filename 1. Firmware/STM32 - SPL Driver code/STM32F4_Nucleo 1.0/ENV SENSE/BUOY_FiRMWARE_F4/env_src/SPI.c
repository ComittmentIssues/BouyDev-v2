/*
 * SPI.c
 *
 *  Created on: Sep 20, 2019
 *      Author: jamie
 */

#include "stm32f4xx.h"
#include "SPI.h"
void init_SPI()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	//SPI Init
	SPI_InitTypeDef SPI_InitStruct;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;
	SPI_Init(SPI3,&SPI_InitStruct);
	SPI_Cmd(SPI3, ENABLE);

	//GPIO AF Init
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = (MISO_Pin |MOSI_Pin | SCLK_Pin);
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	//GPIO AF
	GPIO_PinAFConfig(GPIOB,SCLK_Pinsrc,GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB,MOSI_Pinsrc,GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB,MISO_Pinsrc,GPIO_AF_SPI3);

	init_Control_pin();

	//disable Slave
	SS_DISABLE;
}
void init_Control_pin(void)
{
	//change to match CS pin
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin = (CS_Pin);
	GPIO_Init(GPIOA,&GPIO_InitStruct);
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

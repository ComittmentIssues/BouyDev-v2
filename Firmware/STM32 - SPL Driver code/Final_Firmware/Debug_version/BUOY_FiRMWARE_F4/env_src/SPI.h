/*
 * SPI.h
 *
 *  Created on: Sep 20, 2019
 *      Author: jamie
 *
 *      Very simple library for interfacing with components via SPI
 */

#ifndef SPI_H_
#define SPI_H_

#define CS_Pin GPIO_Pin_10
#define SCLK_Pin GPIO_Pin_3
#define MISO_Pin GPIO_Pin_4
#define MOSI_Pin GPIO_Pin_5

#define CS_Pinsrc GPIO_PinSource10
#define SCLK_Pinsrc GPIO_PinSource3
#define MISO_Pinsrc GPIO_PinSource4
#define MOSI_Pinsrc GPIO_PinSource5
/* Private macro -------------------------------------------------------------*/
#define SS_DISABLE (GPIOA->ODR = 0b1<<10)
#define SS_ENABLE (GPIOA->ODR &= ~(0b1<<10))
/* Private Function Prototypes _----------------------------------------------*/
void init_SPI(void);
void init_Control_pin(void);
uint8_t SPI_Transfer(uint8_t byte, SPI_TypeDef* SPIx);
#endif /* SPI_H_ */

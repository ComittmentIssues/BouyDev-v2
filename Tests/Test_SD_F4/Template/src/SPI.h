/*
 * SPI.h
 *
 *  Created on: Sep 20, 2019
 *      Author: jamie
 *
 *      Very simple library for interfacing with components via SPI
 *
 *      How To Use
 *
 *      Change Macros to match specified SPI peripheral
 *      1: select pin source and change The Pin lables
 *      2: change the pin source lables
 *      3: change the SS_DISABLE and SS_ENABLE to match the number of the CS pin (if the CS pin is on PB12,
 *      	Then the macro must be chnaged to shift the enable bit up by 12
 *      4: under GPIO_AF_SPIx change the value to match the corresponding SPI peripheral
 *		5: Under RCC peripherals change RCC_GPIOxPeriph to RCC_AHB1Periph_GPIOy where y is [A,B,C,D,E,F,G,H,I]
*/
#ifndef SPI_H_
#define SPI_H_
/*
 * Note SPI1 is on RCC_APB2 whereas SPI2 and 3 are on APB1
 * Select the clock source you want by declaring in the define area USE_SPIx where x is [1,2,3]
 *
 * If you want to use a custom CS pin then in the defines area define USE_CUSTOM_CS
 */

/******************************************************************************
 *  DEFINES AREA
 *******************************************************************************/
#define USE_SPI2
#define USE_CUSTOM_CS
/******************************************************************************
 *  END
 *******************************************************************************/
//pin definitions
#define CS_Pin GPIO_Pin_11
#define SCLK_Pin GPIO_Pin_5
#define MISO_Pin GPIO_Pin_6
#define MOSI_Pin GPIO_Pin_7

//peripheral Definitions and AF maps
#define SPI_x SPI1
#define SPI_GPIO GPIOA
#define GPIO_AF_SPIx GPIO_AF_SPI1
//pin sources
#define CS_Pinsrc GPIO_PinSource11
#define SCLK_Pinsrc GPIO_PinSource5
#define MISO_Pinsrc GPIO_PinSource6
#define MOSI_Pinsrc GPIO_PinSource7

//RCC peripherals
#define RCC_GPIO_SPIPeriph RCC_AHB1Periph_GPIOA

#ifdef USE_CUSTOM_CS
#define RCC_GPIO_CSPeriph RCC_AHB1Periph_GPIOC
#endif

/* Private macro -------------------------------------------------------------*/
#define SS_DISABLE (GPIOB->ODR = 0b1<<11)
#define SS_ENABLE (GPIOB->ODR &= ~(0b1<<11))

/* Private Function Prototypes _----------------------------------------------*/
void init_SPI(void);
void init_Control_pin(void);
uint8_t SPI_Transfer(uint8_t byte, SPI_TypeDef* SPIx);

#endif  /*SPI_H_ */

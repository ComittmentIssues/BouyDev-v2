/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp_template.c
  * @author  MCD Application Team
  * @brief   HAL MSP module.
  *          This file template is located in the HAL folder and should be copied 
  *          to the user folder.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "HAL_GPS.h"

#include "HAL_Iridium.h"

#include "DATAFLASH.h"

#include "HAL_BMP280.h"

#include "HAL_INA219.h"
/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP HAL MSP module driver
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_uart4_rx;

extern DMA_HandleTypeDef hdma_uart4_tx;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */ 
	  __HAL_RCC_SYSCFG_CLK_ENABLE();
	  __HAL_RCC_PWR_CLK_ENABLE();
}

/**
  * @brief  DeInitialize the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */
}

/**
  * @brief  Initialize the PPP MSP.
  * @param  None
  * @retval None
  */
void HAL_PPP_MspInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitialize the PPP MSP.
  * @param  None  
  * @retval None
  */
void HAL_PPP_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */
}

/**
* @brief RTC MSP Initialization
* This function configures the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();
    /* RTC interrupt Init */
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

  }

}

/**
* @brief RTC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hrtc: RTC handle pointer
* @retval None
*/
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /* RTC interrupt DeInit */
    HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  // Debug USART Peripheral
  if(huart->Instance==USART2)
  {

    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }

  //GPS UART Init
  if(huart->Instance==UART4)
  {

    /* Peripheral clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPS_TX_Pin|GPS_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA2_Channel5;
    hdma_uart4_rx.Init.Request = DMA_REQUEST_2;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_NORMAL;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      __NOP();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_uart4_rx);

    /* UART4_TX Init */
    hdma_uart4_tx.Instance = DMA2_Channel3;
    hdma_uart4_tx.Init.Request = DMA_REQUEST_2;
    hdma_uart4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_tx.Init.Mode = DMA_NORMAL;
    hdma_uart4_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_uart4_tx) != HAL_OK)
    {
      __NOP();
    }

    __HAL_LINKDMA(huart,hdmatx,hdma_uart4_tx);

    /* UART4 interrupt Init */

	CLEAR_REG(huart->Instance->CR1);
	CLEAR_REG(huart->Instance->CR2);
	CLEAR_REG(huart->Instance->CR3);

    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
  //Iridium Peripheral
  if(huart->Instance==UART5)
   {


     /* Peripheral clock enable */
     __HAL_RCC_UART5_CLK_ENABLE();

     __HAL_RCC_GPIOC_CLK_ENABLE();
     __HAL_RCC_GPIOD_CLK_ENABLE();
     /**UART5 GPIO Configuration
     PC12     ------> UART5_TX
     PD2     ------> UART5_RX
     */
     GPIO_InitStruct.Pin = IR_TX_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
     HAL_GPIO_Init(IR_TX_GPIO_Port, &GPIO_InitStruct);

     GPIO_InitStruct.Pin = IR_RX_Pin;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
     HAL_GPIO_Init(IR_RX_GPIO_Port, &GPIO_InitStruct);

     /* UART5 DMA Init */
     /* UART5_RX Init */
     hdma_uart5_rx.Instance = DMA2_Channel2;
     hdma_uart5_rx.Init.Request = DMA_REQUEST_2;
     hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
     hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
     hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
     hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
     hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
     hdma_uart5_rx.Init.Mode = DMA_NORMAL;
     hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
     if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
     {

     }

     __HAL_LINKDMA(huart,hdmarx,hdma_uart5_rx);

     /* UART5 interrupt Init */
     HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(UART5_IRQn);

   }
}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  //Serial Output UART Deinit
  if(huart->Instance==USART2)
  {

    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);

  }
  //GPS UART Deinit
  if(huart->Instance==UART4)
  {

    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPS_TX_Pin|GPS_RX_Pin);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* UART4 interrupt DeInit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);

  }

  //Iridium UART DeInit
  if(huart->Instance==UART5)
   {

     /* Peripheral clock disable */
     __HAL_RCC_UART5_CLK_DISABLE();

     /**UART5 GPIO Configuration
     PC12     ------> UART5_TX
     PD2     ------> UART5_RX
     */
     HAL_GPIO_DeInit(IR_TX_GPIO_Port, IR_TX_Pin);

     HAL_GPIO_DeInit(IR_RX_GPIO_Port, IR_RX_Pin);

     /* UART5 DMA DeInit */
     HAL_DMA_DeInit(huart->hdmarx);

     /* UART5 interrupt DeInit */
     HAL_NVIC_DisableIRQ(UART5_IRQn);

   }


}


/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM2)
  {

    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

  }
  if(htim_base->Instance==TIM3)
  {

    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {

	    /* TIM2 interrupt DeInit */
	    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
	    HAL_NVIC_DisableIRQ(TIM2_IRQn);

    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA1     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

  }
  if(htim_base->Instance==TIM3)
   {

     /* Peripheral clock disable */
     __HAL_RCC_TIM3_CLK_DISABLE();

     /* TIM3 interrupt DeInit */
     HAL_NVIC_DisableIRQ(TIM3_IRQn);

   }

}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  //BMP280 SPI Init
  if(hspi->Instance==SPI1)
   {
     /* Peripheral clock enable */
     __HAL_RCC_SPI1_CLK_ENABLE();
     __HAL_RCC_GPIOB_CLK_ENABLE();
     /**SPI1 GPIO Configuration
     PB3 (JTDO-TRACESWO)     ------> SPI1_SCK
     PB4 (NJTRST)     ------> SPI1_MISO
     PB5     ------> SPI1_MOSI
     */
     GPIO_InitStruct.Pin = BMP_SPI_SCLK_PIN|BMP_SPI_MISO_PIN| BMP_SPI_MOSI_PIN;
     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
     HAL_GPIO_Init(BMP_SPI_GPIO_PORT, &GPIO_InitStruct);
   }
  //FLASH CHIPS SPI Init
  if(hspi->Instance==SPI2)
  {
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
     PB13     ------> SPI2_SCK
     PB14     ------> SPI2_MISO
     PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  //BMP280 SPI DE-INIT
  if(hspi->Instance==SPI1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PB3 (JTDO-TRACESWO)     ------> SPI1_SCK
    PB4 (NJTRST)     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(BMP_SPI_GPIO_PORT,BMP_SPI_SCLK_PIN|BMP_SPI_MISO_PIN| BMP_SPI_MOSI_PIN);
  }
  //FLASH CHIPS SPI DE-INIT
  if(hspi->Instance==SPI2)
  {
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    PB13     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
  }

}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C2)
  {

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = INA_I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(INA_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = INA_I2C_SDA_PIN;
    HAL_GPIO_Init(INA_I2C_SDA_GPIO_PORT,&GPIO_InitStruct);
    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C2)
  {

    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = INA_I2C_SCL_PIN;
    HAL_GPIO_Init(INA_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = INA_I2C_SDA_PIN;
    HAL_GPIO_Init(INA_I2C_SDA_GPIO_PORT,&GPIO_InitStruct);

  }

}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

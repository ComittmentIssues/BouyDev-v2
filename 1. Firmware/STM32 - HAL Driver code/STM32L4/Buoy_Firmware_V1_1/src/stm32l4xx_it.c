/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
#include "HAL_GPS.h"
#include "HAL_Iridium.h"
#include "Sharc_Frame.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern GPS_Handle_Typedef hgps;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE NonMaskableInt_IRQn 0 */


}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE HardFault_IRQn 0 */


  while (1)
  {

  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE MemoryManagement_IRQn 0 */


  while (1)
  {

  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */


  while (1)
  {

  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */


  while (1)
  {

  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE SVCall_IRQn 0 */


}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE DebugMonitor_IRQn 0 */


}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE PendSV_IRQn 0 */


}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  HAL_IncTick();

}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC wake-up interrupt through EXTI line 20.
  */
void RTC_WKUP_IRQHandler(void)
{
  /* USER CODE RTC_WKUP_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);

}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE TIM2_IRQn 0 */
  USART_TIM_RTO_Handler(hgps.gps_htim); //custom call back function

  HAL_TIM_IRQHandler(hgps.gps_htim);	 //HAL default handler

}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	USART_RTO_IRQHandler(&htim3);
	HAL_TIM_IRQHandler(&htim3);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);

}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE UART4_IRQn 0 */
	USART_GPS_IRQHandler(&hgps); //custom user Call Back function

}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
  USART_Iridium_IRQHandler(&huart5);
}

/**
  * @brief This function handles DMA2 channel2 global interrupt.
  */
void DMA2_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  DMA_Iridium_Periph_IRQHandler(&huart5);
}

/**
  * @brief This function handles DMA2 channel3 global interrupt.
  */
void DMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);

}


/**
  * @brief This function handles DMA2 channel5 global interrupt.
  */
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE DMA2_Channel5_IRQn 0 */
  DMA_GNSS_Periph_IRQHandler(&hgps);
}


/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_memtomem_dma1_channel1);
  DMA_GNSS_MEM_IRQHandler(&hgps);

}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_memtomem_dma1_channel2);
  DMA_Iridium_MEM_IRQHandler(&hdma_memtomem_dma1_channel2);

}

/*
 * @brief: EXTI IRQ Handler for Wake Up Pin 2 (PC13)
 */
void EXTI15_10_IRQHandler(void)
{
	/* Wake Up Pin Handler */
	if(__HAL_GPIO_EXTI_GET_FLAG(EXTI_IRIDIUM_RING_WAKE_PIN))
	{
		__HAL_GPIO_EXTI_CLEAR_FLAG(EXTI_IRIDIUM_RING_WAKE_PIN);
		//interrupt source from PWR WAKE PIN 2 == IRIDIUM Recieve Event
		Routine_ASYNC_IRIDIUM_RX();
	}

	/* Iridium Control Pin Handlers */

	Iridium_ControlPin_IRQHandler();
}

/*
 * @brief: EXTI IRQ Handler for Wake Up Pin 5 (PC5)
 */
void EXTI9_5_IRQHandler(void)
{
	/* Wake Up Pin Handler */
	if(__HAL_GPIO_EXTI_GET_FLAG(EXTI_IMU_EVENT_WAKE_PIN))
	{
		__HAL_GPIO_EXTI_CLEAR_FLAG(EXTI_IMU_EVENT_WAKE_PIN);

		//ROUTINE START
		Routine_Async_IMUevent_Sample();
	}

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

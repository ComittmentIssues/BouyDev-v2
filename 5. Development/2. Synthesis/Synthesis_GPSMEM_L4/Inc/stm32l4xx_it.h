/*
 * stm32l4xx_it.h
 *
 *  Created on: Jun 16, 2020
 *      Author: jamie
 */

#ifndef INC_STM32L4XX_IT_H_
#define INC_STM32L4XX_IT_H_


#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void RTC_WKUP_IRQHandler(void);
void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif



#endif /* INC_STM32L4XX_IT_H_ */

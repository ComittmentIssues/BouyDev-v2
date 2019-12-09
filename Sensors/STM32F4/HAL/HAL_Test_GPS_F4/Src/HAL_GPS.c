/*
 * HAL_GPS.c
 *
 *  Created on: Dec 9, 2019
 *      Author: jamie
 */


#include "HAL_GPS.h"



/* IRQ Handlers */

/*
 * These Functions handle the Transfer of ALL messages from the
 * GPS to the micro. Data is transmitted constantly from the GPS.
 * When a block of data is being recieved, the end of transfer is
 * marked by the Rx line going idle. When this happens, the USART_IRQ
 * will Clear the data register and Disable The Peripheral To Memory Stream
 *
 * The DMA Peripheral- Memory IRQ will fire when the system has detected that
 * the Peripheral - Memory Transfer is completed. IT will then determine how much data has been transfered into the buffer
 * Finally, it will enable the Memory - Memory Data Stream
 *
 * The DMA Memory - Memory IRQ will fire when data transfer to the memory address has been completed.
 * Here the handler will process the data depending on what the status of the flags are.
 *
 */

void USART_GPS_IRQHandler( UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma )
{

	if(huart->Instance->SR & UART_FLAG_IDLE)
	{

			RX_COMPLETE_FLAG = 0;		//signal start of DMA transfer
			//clear Status and Data from instance
			volatile uint32_t tmp;
			tmp = huart->Instance->DR;
			tmp = huart->Instance->SR;
			(void)tmp;
			//Disable DMA Peripheral to Memory Stream
			hdma->Instance->CR &= ~DMA_SxCR_EN;
	}
}

void DMA_Rx_IRQHandler( DMA_HandleTypeDef* hdma, UART_HandleTypeDef* huart )
{
	if(__HAL_DMA_GET_IT_SOURCE(hdma,DMA_IT_TC))
	{
		RX_COMPLETE_FLAG = 0;
		//get data that still needs to be transferred
		gnss_length = DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(hdma,DMA_FLAG_TCIF2_6);

		//enable stream
		//hdma->Instance->CR |= DMA_SxCR_EN;
	}
}

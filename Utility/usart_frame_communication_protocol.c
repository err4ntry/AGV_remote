/**
  ******************************************************************************
  * @file    usart_frame_communication_protocol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the Frame Communication Protocol for USART component of the Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


    /* ****************************************** */
    /* *** BEWARE! THIS CODE IS FOR STM32F3xx *** */
    /* ****************************************** */

/* Includes ------------------------------------------------------------------*/
#include "usart_frame_communication_protocol.h"
//#include "ui_irq_handler.h"

/** @addtogroup MCSDK
  * @{
  */

/**
 * @addtogroup MCUI
 * @{
 */

/** @defgroup UFCP USART Frame Communication Protocol
  * @brief UFCP UFCP component of the Motor Control SDK
  *
  * Detailed documentation for the component.
  * @{
  */

/* Private macros ------------------------------------------------------------*/
#define UFCP_IRQ_FLAG_RX      0
#define UFCP_IRQ_FLAG_TX      1
#define UFCP_IRQ_FLAG_OVERRUN 2
#define UFCP_IRQ_FLAG_TIMEOUT 3
#define UFCP_IRQ_FLAG_ATR     4

/* Private function prototypes -----------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static const uint16_t UFCP_Usart_Timeout_none = 0;
static const uint16_t UFCP_Usart_Timeout_start = 1;
static const uint16_t UFCP_Usart_Timeout_stop = 2;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

uint8_t rx_buffer[BUFFER_SIZE]={0};  //
uint8_t tx_buffer[BUFFER_SIZE]={0};  //

/* Functions ---------------------------------------------------------*/

__weak void UFCP_Init( UFCP_Handle_t * pHandle )
{

  /* Initialize generic component part */
  FCP_Init( & pHandle->_Super );
}

/*
 *
 */
__weak void * UFCP_RX_IRQ_Handler( UFCP_Handle_t * pHandle)
{
  void * ret_val = (void *) & UFCP_Usart_Timeout_none;
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;
  uint8_t rx_len;
	
  //HAL_UART_DMAStop(&huart1);

  if ( FCP_TRANSFER_IDLE != pBaseHandle->RxFrameState )
  {
	  uint8_t rx_len;
	  uint8_t temp;
	  uint8_t idx;
      
	  temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//   
	  rx_len =  BUFFER_SIZE - temp; 
	  
	  pBaseHandle->RxFrame.Head = rx_buffer[0]|rx_buffer[1]<<8;
		pBaseHandle->RxFrame.Size = rx_buffer[3];
		pBaseHandle->RxFrame.ID = rx_buffer[2];
		if (pBaseHandle->RxFrame.Head != 0xffff || 
				pBaseHandle->RxFrame.ID == 4 ||
				pBaseHandle->RxFrame.ID == 0xfa)
		{
//			printf("ignore\r\n");
//			for(int i = 0; i <rx_len; i++)
//			printf("%02X ",rx_buffer[i]);
//			printf("\r\n");
			UFCP_AbortReceive(pBaseHandle);
      UFCP_Receive(pBaseHandle);
			return 0 ;
		}
		pBaseHandle->RxFrameState = FCP_TRANSFER_IDLE;     
      pBaseHandle->RxFrame.Code = rx_buffer[4];
      for (idx = 0; idx < pBaseHandle->RxFrame.Size-2; idx++)
	  {
		pBaseHandle->RxFrame.Buffer[idx] =  rx_buffer[idx + 5];
	  }	  
	  pBaseHandle->RxFrame.FrameCRC = rx_buffer[ pBaseHandle->RxFrame.Size + 3];
		
		for(int i = 0; i <rx_len; i++)
			printf("%02X ",rx_buffer[i]);
			printf("\r\n");
		uint8_t rest = rx_len;
		uint8_t pos = 0;
		
		while(rest)   //do multi frame 
		{
			uint8_t size = pBaseHandle->RxFrame.Size + 4;
			rest -= size;
			printf("rest=%d\r\n",rest);
			
			
			
			for(int i = 0; i <size; i++)
			printf("%02X ",rx_buffer[pos+i]);
			printf("****\r\n");
			
			
			/**************************************************/
			
			 /* Check the Control Sum */
          if ( FCP_CalcCRC( & pBaseHandle->RxFrame ) == pBaseHandle->RxFrame.FrameCRC )
          {
            /* OK. the frame is considered correct. Let's forward to client. */
            pBaseHandle->ClientFrameReceivedCallback( pBaseHandle->ClientEntity);
          }
          else
          {
            error_code = FCP_MSG_RX_BAD_CRC;
						pBaseHandle->TxFrame.ID = pBaseHandle->RxFrame.ID;
						pBaseHandle->TxFrame.Size = 2;		
						pBaseHandle->TxFrame.Code = 0x10;
						pBaseHandle->TxFrame.FrameCRC = FCP_CalcCRC( & pBaseHandle->TxFrame );
            UFCP_Send( pBaseHandle);
						printf("err_crc\r\n");
          }
			
			
			/**************************************************/
					
					
					pos += size;

			pBaseHandle->RxFrame.Size = rx_buffer[pos+3];
			pBaseHandle->RxFrame.Head = rx_buffer[pos+0]|rx_buffer[pos+1]<<8;
		  pBaseHandle->RxFrame.ID = rx_buffer[pos+2];
			if (pBaseHandle->RxFrame.Head != 0xffff || 
				pBaseHandle->RxFrame.ID == 4 ||
				pBaseHandle->RxFrame.ID == 0xfa)
		{
//			printf("ignore\r\n");
//			for(int i = 0; i <rx_len; i++)
//			printf("%02X ",rx_buffer[i]);
//			printf("\r\n");
			UFCP_AbortReceive(pBaseHandle);
      UFCP_Receive(pBaseHandle);
			return 0 ;
		}
		
		pBaseHandle->RxFrame.Code = rx_buffer[pos+4];
		for (idx = 0; idx < pBaseHandle->RxFrame.Size-2; idx++)
	  {
		pBaseHandle->RxFrame.Buffer[idx] =  rx_buffer[pos + idx + 5];
	  }	  
	  pBaseHandle->RxFrame.FrameCRC = rx_buffer[ pos + pBaseHandle->RxFrame.Size + 3];
      			
		}  //end of while(1)
	  

			


         
  } /* end of if ( FCP_TRANSFER_IDLE != pBaseHandle->RxFrameState ) */

  return ret_val;
}

/*
 *
 */
__weak void UFCP_TX_IRQ_Handler( UFCP_Handle_t * pHandle )
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;

  if ( FCP_TRANSFER_IDLE != pBaseHandle->TxFrameState )
  {
   
      //LL_USART_DisableIT_TXE(pHandle->USARTx);
      pBaseHandle->TxFrameState = FCP_TRANSFER_IDLE;

      pBaseHandle->ClientFrameSentCallback( pBaseHandle->ClientEntity );

  } /* end of if ( FCP_TRANSFER_IDLE != pBaseHandle->TxFrameState ) */
}

/*
 *
 */
__weak void UFCP_OVR_IRQ_Handler( UFCP_Handle_t * pHandle )
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  error_code = UFCP_MSG_OVERRUN;
 // (void) UFCP_Send( pBaseHandle, FCP_CODE_NACK, & error_code, 1 );

}

/*
 *
 */
__weak void UFCP_TIMEOUT_IRQ_Handler( UFCP_Handle_t * pHandle )
{
  FCP_Handle_t * pBaseHandle = & pHandle->_Super;
  uint8_t error_code;

  error_code = FCP_MSG_RX_TIMEOUT;
  //(void) UFCP_Send( pBaseHandle, FCP_CODE_NACK, & error_code, 1 );
	//FCP_SendFct_t

}

__weak uint8_t UFCP_Receive( FCP_Handle_t * pHandle )
{
  uint8_t ret_val;

  if ( FCP_TRANSFER_IDLE == pHandle->RxFrameState )
  {
    UFCP_Handle_t * pActualHandle = (UFCP_Handle_t *) pHandle;

    pHandle->RxFrameLevel = 0;
    pHandle->RxFrameState = FCP_TRANSFER_ONGOING;
	  
	HAL_GPIO_WritePin(GPIOA, RS485_RW_Pin, 0);   //低电平收
	HAL_UART_Receive_DMA(pActualHandle->huart,rx_buffer,BUFFER_SIZE);

    //LL_USART_EnableIT_RXNE(pActualHandle->USARTx);
    ret_val = FCP_STATUS_WAITING_TRANSFER;
  }
  else
  {
    ret_val = FCP_STATUS_TRANSFER_ONGOING;
  }

  return ret_val;
}

__weak uint8_t UFCP_Send( FCP_Handle_t * pHandle)
{
  uint8_t ret_val;

  if ( FCP_TRANSFER_IDLE == pHandle->TxFrameState )
  {
    UFCP_Handle_t * pActualHandle = (UFCP_Handle_t *) pHandle;
	uint8_t idx;
    tx_buffer[0] = 0xff;
	tx_buffer[1] = 0xff;
	tx_buffer[2] = pHandle->TxFrame.ID;
	tx_buffer[3] = pHandle->TxFrame.Size;
	tx_buffer[4] = pHandle->TxFrame.Code;
	for (idx = 0; idx < pHandle->TxFrame.Size-2; idx++)
	{
	  tx_buffer[idx + 5] = pHandle->TxFrame.Buffer[idx];
	}
    pHandle->TxFrame.FrameCRC = FCP_CalcCRC( & pHandle->TxFrame );	  	  
	tx_buffer[pHandle->TxFrame.Size+3] = pHandle->TxFrame.FrameCRC;
	  
	
    pHandle->TxFrameState = FCP_TRANSFER_ONGOING;
		
    HAL_GPIO_WritePin(GPIOA, RS485_RW_Pin, 1);   //高电平发**************bug?????***************
	HAL_UART_Transmit_DMA(pActualHandle->huart, tx_buffer, pHandle->TxFrame.Size+4);
	//HAL_UART_Transmit_DMA(pActualHandle->huart, tx_buffer, 6);

    //LL_USART_EnableIT_TXE(pActualHandle->USARTx);
    ret_val = FCP_STATUS_WAITING_TRANSFER;
  }
  else
  {
    ret_val = FCP_STATUS_TRANSFER_ONGOING;
  }

  return ret_val;
}

__weak void UFCP_AbortReceive( FCP_Handle_t * pHandle )
{
  pHandle->RxFrameState = FCP_TRANSFER_IDLE;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __usart_H
#define __usart_H

#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

	 
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);
	 
	 
extern uint8_t USART_RX_BUF[100]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USART_RX_STA;         		//接收状态标记	
extern UART_HandleTypeDef UART1_Handler; //UART句柄
extern uint8_t aRxBuffer[1];//HAL库USART接收Buffer

void uart_init(uint32_t bound);
uint8_t update_sbus(uint8_t *buf);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */


typedef struct
{
	uint16_t CH1;
	uint16_t CH2;
	uint16_t CH3;
	uint16_t CH4;
	uint16_t CH5;
	uint16_t CH6;
  uint16_t CH7;
  uint16_t CH8;
  uint16_t CH9;
  uint16_t CH10;
  uint16_t CH11;
  uint16_t CH12;
  uint16_t CH13;
  uint16_t CH14;
  uint16_t CH15;
  uint16_t CH16;
	uint8_t ConnectState;
}SBUS_CH_Struct;


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

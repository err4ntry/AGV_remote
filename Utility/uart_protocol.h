/**
  ******************************************************************************
  * @file     uart_protocol.h  
  * @author  
	* @brief
	* @date
	******************************************************************************
  */
	
#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H
#include "stdint.h"
#define FRAME_DATA_SIZE 16


#define Uart_Motor_Enable   					(uint8_t)0x1
#define Uart_Motor_Disable     				(uint8_t)0x2
#define Uart_Motor_Break_open    			(uint8_t)0x3
#define Uart_Motor_Break_close    		(uint8_t)0x4
#define Uart_Motor_TorqueMode 				(uint8_t)0x5
#define Uart_Motor_SpeedMode 					(uint8_t)0x6
#define Uart_Motor_PositionMode  			(uint8_t)0x7
#define Uart_Motor_Encoder_Align 			(uint8_t)0x8
#define Motor_Write_speed  		 			 	(uint8_t)0x9
#define Motor_Write_torque  					(uint8_t)0xB
#define Motor_Write_position  				(uint8_t)0xC
#define Uart_Motor_Read_Enable				(uint8_t)0x10//
#define Uart_Motor_Read_Disable				(uint8_t)0x11
#define Uart_Motor_Error_Handle 			(uint8_t)0xF1

typedef struct Uart_Frame_s {
	uint16_t head;
	uint8_t id;
	uint8_t len;
	uint8_t code;
	uint8_t buffer[FRAME_DATA_SIZE];
	uint8_t frameCRC;
} Uart_Frame_t;


void UP_WaitNextFrame(void);
void UP_ReceiveHandle(void);
void UP_PackTransmit(void);
void UP_SendHandle(void);


#endif /* __UART_PROTOCOL_H */

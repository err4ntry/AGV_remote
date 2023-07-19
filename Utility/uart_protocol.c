/**
  ******************************************************************************
  * @file     uart_protocol.c  
  * @author  
	* @brief
	* @date
	******************************************************************************
  */
	
#include "uart_protocol.h"
#include "stm32f4xx_hal.h"
#include "debug_printf.h"
#include "canFestival.h"
#include "master_node.h"
#include "master_node_aux.h"
#include "user_interface.h"
#include "stdbool.h"
#define HUART_HANDLE huart2
extern UART_HandleTypeDef HUART_HANDLE;


Uart_Frame_t uartRxFrame;
Uart_Frame_t uartTxFrame;
uint8_t DMA_Rxbuffer[FRAME_DATA_SIZE + 12];
uint8_t DMA_Txbuffer[FRAME_DATA_SIZE + 12];
uint8_t DMA_MltiTxbuf[24];

void FrameAnalysis(Uart_Frame_t *pFrame);
void Motor_Disable(void);
void Motor_Enable(void);
void Break_Enable(void);
void Break_Disable(void);
void Motor_TorqueMode(void);
void Motor_SpeedMode(void);
void Motor_PositionMode(void);
void Motor_Encoder_Align(void);
void Motor_WriteSpeed(void);
uint8_t UP_CalcCRC(Uart_Frame_t *pFrame);
uint8_t UP_IsFrameValid(Uart_Frame_t *pFrame);

uint8_t testValue;





void UP_WaitNextFrame(void)
{
	HAL_UART_Receive_DMA(&HUART_HANDLE, DMA_Rxbuffer, sizeof(DMA_Rxbuffer));
}

void UP_ReceiveHandle(void)
{
	uartRxFrame.head = DMA_Rxbuffer[0] | DMA_Rxbuffer[1] << 8;
	uartRxFrame.id = DMA_Rxbuffer[2];
	uartRxFrame.len = DMA_Rxbuffer[3];
	uartRxFrame.code = DMA_Rxbuffer[4];
	for (uint16_t i = 0; i < uartRxFrame.len -2; i++)
	{
		uartRxFrame.buffer[i] = DMA_Rxbuffer[i + 5]; 
	}
	uartRxFrame.frameCRC = DMA_Rxbuffer[uartRxFrame.len + 3];
	testValue = UP_IsFrameValid(&uartRxFrame);
	if (UP_IsFrameValid(&uartRxFrame))
		FrameAnalysis(&uartRxFrame);
}

void UP_SendHandle(void)
{
	
	DMA_Txbuffer[0] = 0xff;
	DMA_Txbuffer[1] = 0xff;
	DMA_Txbuffer[2] = uartTxFrame.id;
	DMA_Txbuffer[3] = uartTxFrame.len;
	DMA_Txbuffer[4] = uartTxFrame.code;
	for (uint16_t i = 0; i < uartTxFrame.len - 2; i++)
	{
		DMA_Txbuffer[i + 5] = uartTxFrame.buffer[i];
	}
	DMA_Txbuffer[uartTxFrame.len + 3] = UP_CalcCRC(&uartTxFrame);
	HAL_UART_Transmit_DMA(&huart2, DMA_Txbuffer,uartTxFrame.len + 4);	
	
}

void FrameAnalysis(Uart_Frame_t *pFrame)
{
	bool RequireAck = true;
  bool bNoError = true; // Default is error
  uint8_t bErrorCode;
  uint8_t id = pFrame->id;  
  uint8_t code = pFrame->code;
  uint8_t* buf = pFrame->buffer;
  uint8_t len = pFrame->len - 2;
	
	
	switch(code)
				{
					case MC_PROTOCOL_CODE_PING:
						break;
					case MC_PROTOCOL_CODE_GET_TABLE:
					{
						MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buf[0];
						switch (bRegID)
						{
							case MC_PROTOCOL_REG_NodeID:
							case MC_PROTOCOL_REG_MotorOperation:
							case MC_PROTOCOL_REG_MOS_ProtectTemp:
							case MC_PROTOCOL_REG_MotorProtectTemp:
							case MC_PROTOCOL_REG_cUnderVoltage:
							case MC_PROTOCOL_REG_OverVoltage :
							case MC_PROTOCOL_REG_DissipationOn:
							case MC_PROTOCOL_REG_DissipationOff:
							case MC_PROTOCOL_REG_OverCurrent:
							case MC_PROTOCOL_REG_ReductionRatio :
							case MC_PROTOCOL_REG_MaxSpeed:
							case MC_PROTOCOL_REG_MaxPosition:
							case MC_PROTOCOL_REG_MaxCurrent:
							case MC_PROTOCOL_REG_IdLoopKp: 
							case MC_PROTOCOL_REG_IdLoopKi:
							case MC_PROTOCOL_REG_IdLoopKd:
							case MC_PROTOCOL_REG_IdLoopKaw:
							case MC_PROTOCOL_REG_IdLoopLimit:
							case MC_PROTOCOL_REG_IqLoopKp:
							case MC_PROTOCOL_REG_IqLoopKi:
							case MC_PROTOCOL_REG_IqLoopKd:
							case MC_PROTOCOL_REG_IqLoopKaw:                  
							case MC_PROTOCOL_REG_IqLoopLimit:
							case MC_PROTOCOL_REG_SpeedLoopKp:
							case MC_PROTOCOL_REG_SpeedLoopKi:
							case MC_PROTOCOL_REG_SpeedLoopKd:
							case MC_PROTOCOL_REG_SpeedLoopKaw:
							case MC_PROTOCOL_REG_SpeedLoopLimit:
							case MC_PROTOCOL_REG_PositionLoopKp:
							case MC_PROTOCOL_REG_PositionLoopKi:
							case MC_PROTOCOL_REG_PositionLoopKd:
							case MC_PROTOCOL_REG_PositionLoopKaw:
							case MC_PROTOCOL_REG_PositionLoopLimit:
							case MC_PROTOCOL_REG_ActualPosition:
								break;
							case MC_PROTOCOL_REG_ActualSpeed:
							{
								uint16_t hValue = cActualSpeed;
								uartTxFrame.id = id;							
								uartTxFrame.len = 4;
								uartTxFrame.code = 0;
								uartTxFrame.buffer[0] = 0xff & hValue;
								uartTxFrame.buffer[1] = hValue >> 8;
								uartTxFrame.frameCRC = UP_CalcCRC(&uartTxFrame);
								UP_SendHandle();
								RequireAck = false;
							}
							break;
							case MC_PROTOCOL_REG_ActualCurrent:
							case MC_PROTOCOL_REG_ActualMotorTemp:
							{
								uint16_t hValue = cActualMotorTemp;
								uartTxFrame.id = id;							
								uartTxFrame.len = 4;
								uartTxFrame.code = 0;
								uartTxFrame.buffer[0] = 0xff & hValue;
								uartTxFrame.buffer[1] = hValue >> 8;
								uartTxFrame.frameCRC = UP_CalcCRC(&uartTxFrame);
								UP_SendHandle();
								RequireAck = false;
							}
							break;							
							case MC_PROTOCOL_REG_ActualVoltage:
							case MC_PROTOCOL_REG_ActualMosTemp:
							case MC_PROTOCOL_REG_ReferencePosition:
							case MC_PROTOCOL_REG_ReferenceSpeed:
							case MC_PROTOCOL_REG_ReferenceCurrent:
							case MC_PROTOCOL_REG_MotorStatus:
							{
								uint16_t hValue = cMotorStatus;
								uartTxFrame.id = id;							
								uartTxFrame.len = 4;
								uartTxFrame.code = 0;
								uartTxFrame.buffer[0] = 0xff & hValue;
								uartTxFrame.buffer[1] = hValue >> 8;
								uartTxFrame.frameCRC = UP_CalcCRC(&uartTxFrame);
								UP_SendHandle();
								RequireAck = false;
							}
							break;								
							default:
								break;							
						}
					}
						break;
					case MC_PROTOCOL_CODE_SET_TABLE: 
					{
						MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buf[0];
						switch (bRegID)
						{
							case MC_PROTOCOL_REG_NodeID:
							case MC_PROTOCOL_REG_MotorOperation:
							{
								uint16_t hValue = buf[1] + (buf[2] << 8);
								cMotorOperation = hValue;
								sendOnePDOevent(&master_node_Data,0);								
							}
							break;							
							case MC_PROTOCOL_REG_MOS_ProtectTemp:
							case MC_PROTOCOL_REG_MotorProtectTemp:
							case MC_PROTOCOL_REG_cUnderVoltage:
							case MC_PROTOCOL_REG_OverVoltage :
							case MC_PROTOCOL_REG_DissipationOn:
							case MC_PROTOCOL_REG_DissipationOff:
							case MC_PROTOCOL_REG_OverCurrent:
							case MC_PROTOCOL_REG_ReductionRatio :
							case MC_PROTOCOL_REG_MaxSpeed:
							case MC_PROTOCOL_REG_MaxPosition:
							case MC_PROTOCOL_REG_MaxCurrent:
							case MC_PROTOCOL_REG_IdLoopKp: 
							case MC_PROTOCOL_REG_IdLoopKi:
							case MC_PROTOCOL_REG_IdLoopKd:
							case MC_PROTOCOL_REG_IdLoopKaw:
							case MC_PROTOCOL_REG_IdLoopLimit:
							case MC_PROTOCOL_REG_IqLoopKp:
							case MC_PROTOCOL_REG_IqLoopKi:
							case MC_PROTOCOL_REG_IqLoopKd:
							case MC_PROTOCOL_REG_IqLoopKaw:                  
							case MC_PROTOCOL_REG_IqLoopLimit:
							case MC_PROTOCOL_REG_SpeedLoopKp:
							case MC_PROTOCOL_REG_SpeedLoopKi:
							case MC_PROTOCOL_REG_SpeedLoopKd:
							case MC_PROTOCOL_REG_SpeedLoopKaw:
							case MC_PROTOCOL_REG_SpeedLoopLimit:
							case MC_PROTOCOL_REG_PositionLoopKp:
							case MC_PROTOCOL_REG_PositionLoopKi:
							case MC_PROTOCOL_REG_PositionLoopKd:
							case MC_PROTOCOL_REG_PositionLoopKaw:
							case MC_PROTOCOL_REG_PositionLoopLimit:
							case MC_PROTOCOL_REG_ActualPosition:
							case MC_PROTOCOL_REG_ActualSpeed:
							case MC_PROTOCOL_REG_ActualCurrent:
							case MC_PROTOCOL_REG_ActualMotorTemp:
							case MC_PROTOCOL_REG_ActualVoltage:
							case MC_PROTOCOL_REG_ActualMosTemp:
							case MC_PROTOCOL_REG_ReferencePosition:
							case MC_PROTOCOL_REG_ReferenceSpeed:
							{
								uint16_t hValue = buf[1] + (buf[2] << 8);
								cReferenceSpeed = hValue;
								cMotorOperation = 9;
								sendOnePDOevent(&master_node_Data,0);
							}
							break;
							case MC_PROTOCOL_REG_ReferenceCurrent:
							case MC_PROTOCOL_REG_MotorStatus:
							default:
								break;							
						}
					}						
						break;
					
					default:
						break;				
					
	    	}
				
				  if (RequireAck)
				{
					bNoError = true;
						if (bNoError)
						{
								uartTxFrame.id = id;							
								uartTxFrame.len = 3;
								uartTxFrame.code = code;
								uartTxFrame.buffer[0] = 0;
								uartTxFrame.frameCRC = UP_CalcCRC(&uartTxFrame);
								UP_SendHandle();					
						}
						else
						{
      //pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
						}
				}
}





uint8_t UP_CalcCRC(Uart_Frame_t *pFrame)
{
	uint8_t nSum = 0;
	nSum += 0xff;
	nSum += 0xff;
	nSum += pFrame->id;
	nSum += pFrame->len;
	nSum += pFrame->code;
	for (uint16_t i = 0; i < pFrame->len - 2; i++)
	{
		nSum += pFrame->buffer[i]; 
	}
	
	return nSum;
}

uint8_t UP_IsFrameValid(Uart_Frame_t *pFrame)
{
	return UP_CalcCRC(pFrame) == pFrame->frameCRC;
}





//void Motor_Disable(void){DEBUG_PRINTF("Disable\r\n");SendPDO();}
//void Motor_Enable(void){DEBUG_PRINTF("Enable\r\n");SendPDO();}
//void Break_Enable(void){DEBUG_PRINTF("Break_Enable\r\n");SendPDO();}
//void Break_Disable(void){DEBUG_PRINTF("Break_Disable\r\n");SendPDO();}
//void Motor_TorqueMode(void){DEBUG_PRINTF("Torque\r\n");SendPDO();}
//void Motor_SpeedMode(void){DEBUG_PRINTF("Speed\r\n");SendPDO();}
//void Motor_PositionMode(void){DEBUG_PRINTF("Motor_PositionMode\r\n");SendPDO();}
//void Motor_Encoder_Align(void){DEBUG_PRINTF("Motor_Encoder_Align\r\n");SendPDO();}
//void Motor_WriteSpeed(void){DEBUG_PRINTF("Write\r\n");}




//void SendPDO(void)
//{
//	uint16_t temp=0x0000;
//	cMotorOperation=temp+DMA_buffer[1];
//	cReferenceCurrent=((temp+DMA_buffer[2])<<8)+DMA_buffer[3];//
//	cReferenceSpeed=((temp+DMA_buffer[4])<<8)+DMA_buffer[5];
//	cReferencePosition=((temp+DMA_buffer[6])<<8)+DMA_buffer[7];	
//	sendOnePDOevent(&master_node_Data,0);//

//}

void UP_PackTransmit(void)
{
				uint8_t nSum = 0;
				DMA_MltiTxbuf[0]=0xFF;
				DMA_MltiTxbuf[1]=0xFF;
				DMA_MltiTxbuf[2]=0x01;
				DMA_MltiTxbuf[3]=16;
				DMA_MltiTxbuf[4]=0x08;
	
				
				DMA_MltiTxbuf[5]=((uint8_t*)(&cMotorStatus))[0];
				DMA_MltiTxbuf[6]=((uint8_t*)(&cMotorStatus))[1];	
	
				DMA_MltiTxbuf[7]=((uint8_t*)(&cActualCurrent))[0];
				DMA_MltiTxbuf[8]=((uint8_t*)(&cActualCurrent))[1];	
	
				DMA_MltiTxbuf[9]=((uint8_t*)(&cActualSpeed))[0];
				DMA_MltiTxbuf[10]=((uint8_t*)(&cActualSpeed))[1];
	
				DMA_MltiTxbuf[11]=((uint8_t*)(&cActualPosition))[0];
				DMA_MltiTxbuf[12]=((uint8_t*)(&cActualPosition))[1];
	
				DMA_MltiTxbuf[13]=((uint8_t*)(&cActualBusVoltage))[0];
				DMA_MltiTxbuf[14]=((uint8_t*)(&cActualBusVoltage))[1];
	

         
        DMA_MltiTxbuf[15]=((uint8_t*)(&cActualMotorTemp))[0];
				DMA_MltiTxbuf[16]=0;							 
						
				DMA_MltiTxbuf[17]=((uint8_t*)(&cActualMotorTemp))[1];
				DMA_MltiTxbuf[18]=0;
				
				for (uint16_t i = 0; i < 19; i++)
					nSum += DMA_MltiTxbuf[i];								
				DMA_MltiTxbuf[19]=nSum;
				
				HAL_UART_Transmit_DMA(&huart2, DMA_MltiTxbuf, 20);
}




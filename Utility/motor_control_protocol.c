/**
  ******************************************************************************
  * @file    motor_control_protocol.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the motor_control_protocol component of the Motor Control SDK.
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

/* Includes ------------------------------------------------------------------*/
#include "user_interface.h"
#include "motor_control_protocol.h"
#include "stdio.h"
#include "protocol.h"

/**
 * @addtogroup MCSDK
 * @{
 */

/**
 * @addtogroup MCUI
 * @{
 */

/**
 * @defgroup motor_control_protocol Motor Control Protocol
 *
 * @brief Transmission protocol designed to report Motor Control subsystem status and to control motors.
 *
 * The Motor Control Protocol defines a transmission mechanism that allows a Motor Control
 * Application to send in real time the values of a defined set of internal variables to an
 * external recipient and to receive Motor Control commands from this recipient.
 *
 * The Commands that can be sent with the Motor Control Protocol are defined in **TBD**.
 *
 * The variables which values can be exchanged through the Motor Control Protocol are listed in
 * @ref MC_Protocol_REG.
 *
 * An example of external recipient is the Motor Control Monitor tool that is part of the Motor
 * Control Workbench.
 *
 * The Motor Control Protocol defines frames that contain either commands or variables values.
 * These frames are the exchanged between the Application and the external recipient. To that end,
 * the Motor Control Protocol relies on a lower level transport protocol to actually send them.
 *
 * @todo Complete documentation
 * @{
 */

/* Private define ------------------------------------------------------------*/

#define ACK_NOERROR 0xF0
#define ACK_ERROR   0xFF
#define ATR_FRAME_START 0xE0

#define MC_PROTOCOL_CODE_NONE        0x00

/* List of error codes */
typedef enum ERROR_CODE_e
{
	ERROR_NONE = 0,             /**<  0x00 - No error */
	ERROR_BAD_FRAME_ID,         /**<  0x01 - BAD Frame ID. The Frame ID has not been recognized by the firmware. */
	ERROR_CODE_OVERRUN,         /**<  0x08 - Overrun error. Transmission speed too fast, frame not received correctly */
	ERROR_CODE_BAD_CRC,         /**<  0x0A - The computed CRC is not equal to the received CRC byte. */
	ERROR_CODE_HOT,         /**<  0x0A - The computed CRC is not equal to the received CRC byte. */
	ERROR_CODE_OVERLIMIT,         /**<  0x0A - The computed CRC is not equal to the received CRC byte. */
} ERROR_CODE;

MPInfo_t MPInfo = {0, 0};

joint_control joint;
//extern joint_state joint_s;

/**
* @brief  Initializes  MCP component parameters
*
* @param  pHandle Pointer on the handle of the component to initialize.
* @param  pFCP Pointer on Frame communication protocol component's handle to use.
* @param  fFcpSend Pointer on FCP's send message function
* @param  fFcpReceive Pointer on FCP's receive message function
* @param  fFcpAbortReceive Pointer on FCP's abort receive message function
* @param  pDAC Pointer on DAC component.
* @param  s_fwVer Pointer on string containing FW release version.
*/
__weak void MCP_Init( MCP_Handle_t *pHandle, 
               FCP_Handle_t * pFCP,
               FCP_SendFct_t fFcpSend, 
               FCP_ReceiveFct_t fFcpReceive, 
               FCP_AbortReceiveFct_t fFcpAbortReceive)
{
  pHandle->pFCP = pFCP;
  FCP_SetClient( pFCP, pHandle,
                 (FCP_SentFrameCallback_t) & MCP_SentFrame,
                 (FCP_ReceivedFrameCallback_t) & MCP_ReceivedFrame,
                 (FCP_RxTimeoutCallback_t) & MCP_OnTimeOut );
  pHandle->fFcpSend = fFcpSend;
  pHandle->fFcpReceive = fFcpReceive;
  pHandle->fFcpAbortReceive = fFcpAbortReceive;

  MCP_WaitNextFrame(pHandle);
}

/**
* @brief  Function used to set and report the Time Out.
*
* @param  pHandle Pointer on the handle of the component.
*/
__weak void MCP_OnTimeOut(MCP_Handle_t *pHandle)
{
     MCP_WaitNextFrame(pHandle);
}

/**
* @brief  Function used to check next reception frame
*
* @param  pHandle Pointer on the handle of the component.
*/
__weak void MCP_WaitNextFrame(MCP_Handle_t *pHandle)
{
  pHandle->fFcpAbortReceive(pHandle->pFCP);
  pHandle->BufferSize = FCP_MAX_PAYLOAD_SIZE;
  pHandle->fFcpReceive(pHandle->pFCP);
}

/**
* @brief  Function used to transmit the data
*
* @param  pHandle Pointer on the handle of the component.
* @param  Code code value of frame to send.
* @param  buffer frame data buffer.
* @param  Size size of data frame.
*/
__weak void MCP_SentFrame(MCP_Handle_t *pHandle, uint8_t Code, uint8_t *buffer, uint8_t Size)
{
    MCP_WaitNextFrame(pHandle);
}

/**
* @brief  Function used to decode received data
*
* @param  pHandle Pointer on the handle of the component.
* @param  Code code value of frame to send.
* @param  buffer frame data buffer.
* @param  Size size of data frame.
*/
__weak void MCP_ReceivedFrame(MCP_Handle_t *pHandle)
{
  bool RequireAck = true;
  bool bNoError = true; // Default is error
  uint8_t bErrorCode;
  uint8_t id = pHandle->pFCP->RxFrame.ID;  
  uint8_t code = pHandle->pFCP->RxFrame.Code;
  uint8_t* buf = pHandle->pFCP->RxFrame.Buffer;
  uint8_t len = pHandle->pFCP->RxFrame.Size - 2;
	

	
	switch (code)
	{
	case MC_PROTOCOL_CODE_PING:
	{
		pHandle->pFCP->TxFrame.ID = id;
    pHandle->pFCP->TxFrame.Size = 2;		
    pHandle->pFCP->TxFrame.Code = 0;
    pHandle->pFCP->TxFrame.FrameCRC = FCP_CalcCRC( & pHandle->pFCP->TxFrame );
		printf("ping%d\r\n",id);
    pHandle->fFcpSend(pHandle->pFCP);
		RequireAck = false;
	}
		
		break;
		
	case MC_PROTOCOL_CODE_GET_TABLE:
				{
			MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buf[0];
			switch (bRegID)
			{
			case MC_PROTOCOL_REG_TYPE_L:
			case MC_PROTOCOL_REG_TYPE_H:
			case MC_PROTOCOL_REG_FW_VERSION:
			case MC_PROTOCOL_REG_ID:
			case MC_PROTOCOL_REG_BAUDRATE:
			case MC_PROTOCOL_REG_MIN_POS_L:
			case MC_PROTOCOL_REG_MIN_POS_H:
			case MC_PROTOCOL_REG_MAX_POS_L:
			case MC_PROTOCOL_REG_MAX_POS_H:
			case MC_PROTOCOL_REG_MAX_TEMP:
			case MC_PROTOCOL_REG_MIN_VOLT:
			case MC_PROTOCOL_REG_MAX_VOLT:
			case MC_PROTOCOL_REG_MAX_CURRENT_L:
			case MC_PROTOCOL_REG_MAX_CURRENT_H:
			case MC_PROTOCOL_REG_OFFSET_L:
			case MC_PROTOCOL_REG_OFFSET_H:
				break;
/**************RAM******************/
			case MC_PROTOCOL_REG_TURNS:
				break;
			case MC_PROTOCOL_REG_EN:
			{
				if (1 == buf[1])
				{
					cmd_EnterMotorMode(id);
					printf("enable\r\n");
				}
				else
				{
					cmd_ExitMotorMode(id);
					printf("disable\r\n");
				}					
			
			}
			break;
			case MC_PROTOCOL_REG_LED:
				break;
			case MC_PROTOCOL_REG_KD:
			case MC_PROTOCOL_REG_KI:
			case MC_PROTOCOL_REG_KP:
			
				break;				
               
			case MC_PROTOCOL_REF_POS_L:
			case MC_PROTOCOL_REF_POS_H:
			{
				uint16_t wValue = buf[1] + (buf[2] << 8);
			  float pos = 0.001534*wValue - 3.1415;
				
				joint.p_des = pos;
				joint.v_des = 0;
				joint.kp = 1;
				joint.kd = 0.1;
				joint.t_ff = 0;
				
				cmd_control(id, joint);
			  printf("pos=%f\r\n",pos);
			}
			    
				//motor_ctrl()
				break;
			case MC_PROTOCOL_REF_VEL_L:
			case MC_PROTOCOL_REF_VEL_H:
				break;
	
			case MC_PROTOCOL_ACTUAL_POS_L:
			case MC_PROTOCOL_ACTUAL_POS_H:
			{
				uint16_t pos = 0;			
				pos = 0.5333*cmd_GetPos(id)+400;	
        				
				printf("acpos%d=%d\r\n",id,pos);				
				
				pHandle->pFCP->TxFrame.ID = id;				
				pHandle->pFCP->TxFrame.Size = 4;		
				pHandle->pFCP->TxFrame.Code = 0;
				pHandle->pFCP->TxFrame.Buffer[0] = 0xff & pos;
				pHandle->pFCP->TxFrame.Buffer[1] = pos >> 8;				
				pHandle->pFCP->TxFrame.FrameCRC = FCP_CalcCRC( & pHandle->pFCP->TxFrame );
				pHandle->fFcpSend(pHandle->pFCP);
				RequireAck = false;
			}
				break;
			case MC_PROTOCOL_ACTUAL_VEL_L:
			case MC_PROTOCOL_ACTUAL_VEL_H:
				break;
	
			case MC_PROTOCOL_ACTUAL_VOL:
				break;
			case MC_PROTOCOL_ACTUAL_TEMP:
			{
				uint16_t temp = 25;
				printf("temp=%d\r\n",temp);
				
				pHandle->pFCP->TxFrame.ID = id;
        pHandle->pFCP->TxFrame.Size = 3;		
				pHandle->pFCP->TxFrame.Code = 0;
				pHandle->pFCP->TxFrame.Buffer[0] = 20;
				pHandle->pFCP->TxFrame.FrameCRC = FCP_CalcCRC( & pHandle->pFCP->TxFrame );
				pHandle->fFcpSend(pHandle->pFCP);
				RequireAck = false;
			}
			break;
			
			case MC_PROTOCOL_ACTUAL_CURRENT_L:
			case MC_PROTOCOL_ACTUAL_CURRENT_H:
			{
				uint16_t current = 0;
				current = cmd_GetCurrent(id);
				printf("current=%d\r\n",current);
				//current = 2048;
				
				pHandle->pFCP->TxFrame.ID = id;
				pHandle->pFCP->TxFrame.Size = 4;		
				pHandle->pFCP->TxFrame.Code = 0;
				pHandle->pFCP->TxFrame.Buffer[0] = 0xff & current;
				pHandle->pFCP->TxFrame.Buffer[1] = current >> 8;				
				pHandle->pFCP->TxFrame.FrameCRC = FCP_CalcCRC( & pHandle->pFCP->TxFrame );
				pHandle->fFcpSend(pHandle->pFCP);
				RequireAck = false;
			}
			break;
			case MC_PROTOCOL_REF_ACC:	
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
			case MC_PROTOCOL_REG_TYPE_L:
			case MC_PROTOCOL_REG_TYPE_H:
			case MC_PROTOCOL_REG_FW_VERSION:
			case MC_PROTOCOL_REG_ID:
			case MC_PROTOCOL_REG_BAUDRATE:
			case MC_PROTOCOL_REG_MIN_POS_L:
			case MC_PROTOCOL_REG_MIN_POS_H:
			case MC_PROTOCOL_REG_MAX_POS_L:
			case MC_PROTOCOL_REG_MAX_POS_H:
			case MC_PROTOCOL_REG_MAX_TEMP:
			case MC_PROTOCOL_REG_MIN_VOLT:
			case MC_PROTOCOL_REG_MAX_VOLT:
			case MC_PROTOCOL_REG_MAX_CURRENT_L:
			case MC_PROTOCOL_REG_MAX_CURRENT_H:
			case MC_PROTOCOL_REG_OFFSET_L:
			case MC_PROTOCOL_REG_OFFSET_H:
				break;
/**************RAM******************/
			case MC_PROTOCOL_REG_TURNS:
				break;
			case MC_PROTOCOL_REG_EN:
			{
				if (1 == buf[1])
				{
					cmd_EnterMotorMode(id);
					printf("enable\r\n");
				}
				else
				{
					cmd_ExitMotorMode(id);
					printf("disable\r\n");
				}					
			
			}
			break;
			case MC_PROTOCOL_REG_LED:
				break;
			case MC_PROTOCOL_REG_KD:
			case MC_PROTOCOL_REG_KI:
			case MC_PROTOCOL_REG_KP:
			
				break;				
               
			case MC_PROTOCOL_REF_POS_L:
			case MC_PROTOCOL_REF_POS_H:
			{
				uint16_t wValue = buf[1] + (buf[2] << 8);
			  float pos = 0.001534*wValue - 3.1415;
				
				joint.p_des = pos;
				joint.v_des = 0;
				joint.kp = 1;
				joint.kd = 0.1;
				joint.t_ff = 0;
				
				cmd_control(id, joint);
			  printf("pos=%f\r\n",pos);
			}
			    
				//motor_ctrl()
				break;
			case MC_PROTOCOL_REF_VEL_L:
			case MC_PROTOCOL_REF_VEL_H:
				break;
	
			case MC_PROTOCOL_ACTUAL_POS_L:
			case MC_PROTOCOL_ACTUAL_POS_H:
				break;
			case MC_PROTOCOL_ACTUAL_VEL_L:
			case MC_PROTOCOL_ACTUAL_VEL_H:
				break;
	
			case MC_PROTOCOL_ACTUAL_VOL:
				break;
			case MC_PROTOCOL_ACTUAL_TEMP:
			break;
			
			case MC_PROTOCOL_ACTUAL_CURRENT_L:
			case MC_PROTOCOL_ACTUAL_CURRENT_H:
			break;
			case MC_PROTOCOL_REF_ACC:	
				break;
			default:
				break;						
			}
		
		
		}
		break;			
	case MC_PROTOCOL_CODE_SET_REG:
		break;			
	case MC_PROTOCOL_CODE_EN_REG:  
		break;			
	case MC_PROTOCOL_CODE_DEFAULT: 
		break;			
	case MC_PROTOCOL_CODE_SET_MIDDLE:
	{
		cmd_zero(id);
		printf("zero\r\n");
	}
		break;			
	case MC_PROTOCOL_CODE_CLE_MIDDLE: 
		break;			
	case MC_PROTOCOL_CODE_MUL_SET:
		{
		  if (id == 0xfe )
			{
				printf("multi\r\n");
				MC_Protocol_REG_t bRegID = (MC_Protocol_REG_t)buf[0];
				switch (bRegID)
				{
				case MC_PROTOCOL_REF_POS_H:
				case MC_PROTOCOL_REF_POS_L:
			{
				uint16_t wValue;
			  float pos;
				uint8_t mid;
				
				joint.v_des = 0;
				joint.kp = 0.5;
				joint.kd = 0.1;
				joint.t_ff = 0;
								
				wValue = buf[3] + (buf[4] << 8);
				pos = 0.001534*wValue - 3.1415;
				joint.p_des = pos;
				mid = buf[2];
				cmd_control(mid, joint);
			  printf("pos%d=%f\r\n",mid, pos);
				
				wValue = buf[6] + (buf[7] << 8);
				pos = 0.001534*wValue - 3.1415;
				joint.p_des = pos;
				mid = buf[5];
				cmd_control(mid, joint);
			  printf("pos%d=%f\r\n",mid, pos);
				
				wValue = buf[9] + (buf[10] << 8);
				pos = 0.001534*wValue - 3.1415;
				joint.p_des = pos;
				mid = buf[8];
				cmd_control(mid, joint);
			  printf("pos%d=%f\r\n",mid, pos);
				
			}
			break;
			case MC_PROTOCOL_REG_EN:
			{
				uint8_t enable =0;
				uint8_t mid =0;
				
				mid = buf[2];
				enable = buf[3];				
				if (1 == enable )
				{
					cmd_EnterMotorMode(mid);
					printf("enable%d\r\n", mid);
					printf("*********enable************%d\r\n", enable);
				}
//				else
//				{
//					cmd_ExitMotorMode(mid);
//					printf("disable%d\r\n", mid);
//				}
				
				mid = buf[4];
				enable = buf[5];				
				if(1 == enable )
				{
					cmd_EnterMotorMode(mid);
					printf("enable%d\r\n", mid);
				}
//				else
//				{
//					cmd_ExitMotorMode(mid);
//					printf("disable%d\r\n", mid);
//				}					
				mid = buf[6];
				enable = buf[7];				
				if (1 == enable )
				{
					cmd_EnterMotorMode(mid);
					printf("enable%d\r\n", mid);
				}
//				else
//				{
//					cmd_ExitMotorMode(mid);
//					printf("disable%d\r\n", mid);
//				}									
			
			}
			break;
			default:
				break;
		}
			
			
			
				
		    
			
			}	
		 RequireAck = false;
		 MCP_WaitNextFrame(pHandle);			
      
	 }
		break;
	default:
    {
      bErrorCode = ERROR_BAD_FRAME_ID;
    }
    break;	
	}

  
  if (RequireAck)
  {
    if (bNoError)
    {
      pHandle->pFCP->TxFrame.ID = id;
      pHandle->pFCP->TxFrame.Size = 2;		
      pHandle->pFCP->TxFrame.Code = 0;
      pHandle->pFCP->TxFrame.FrameCRC = FCP_CalcCRC( & pHandle->pFCP->TxFrame );
      pHandle->fFcpSend(pHandle->pFCP);
    }
    else
    {
      //pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
    }
  }
}

/**
* @brief  Allow to report the overrun error message.
*
* Called when received frame has not been received correctly due to the
* transmission speed too fast.
*
* @param  pHandle Pointer on the handle of the component.
*/
__weak void MCP_SendOverrunMessage(MCP_Handle_t *pHandle)
{
  //uint8_t bErrorCode = ERROR_CODE_OVERRUN;
  //pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
}

/**
* @brief  Allow to report the time out error message.
*
* @param  pHandle Pointer on the handle of the component.
*/
__weak void MCP_SendTimeoutMessage(MCP_Handle_t *pHandle)
{
  //uint8_t bErrorCode = ERROR_CODE_TIMEOUT;
  //pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
}



/**
* @brief  Allow to send back a BAD CRC message.
*
* @param  pHandle Pointer on the handle of the component.
*/
__weak void MCP_SendBadCRCMessage(MCP_Handle_t *pHandle)
{
  uint8_t bErrorCode = ERROR_CODE_BAD_CRC;
 // pHandle->fFcpSend(pHandle->pFCP, ACK_ERROR, &bErrorCode, 1);
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

/**
 * @}
 */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

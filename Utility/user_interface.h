/**
  ******************************************************************************
  * @file    user_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          user interface component of the Motor Control SDK.
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
  * @ingroup MCUI
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USERINTERFACE_H
#define __USERINTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
//#include "mc_type.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCUI
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Motor control protocol registers
 *
 * Enumerates the variables and information that can be exchanged across the Motor Control Protocol
 */
typedef enum
{
/**************flash***************/
	MC_PROTOCOL_REG_NodeID = 0x01,
	MC_PROTOCOL_REG_MotorOperation = 0x02,
	MC_PROTOCOL_REG_MOS_ProtectTemp = 0x03,
	MC_PROTOCOL_REG_MotorProtectTemp = 0x04,
	MC_PROTOCOL_REG_cUnderVoltage = 0x05,
	MC_PROTOCOL_REG_OverVoltage = 0x06,
	MC_PROTOCOL_REG_DissipationOn = 0x07,
	MC_PROTOCOL_REG_DissipationOff = 0x08,
	MC_PROTOCOL_REG_OverCurrent = 0x09,
	MC_PROTOCOL_REG_ReductionRatio = 0x0A,
	MC_PROTOCOL_REG_MaxSpeed = 0x0B,
	MC_PROTOCOL_REG_MaxPosition = 0x0C,
	MC_PROTOCOL_REG_MaxCurrent = 0x0D,
	MC_PROTOCOL_REG_IdLoopKp = 0x0E,
	MC_PROTOCOL_REG_IdLoopKi = 0x0F,
	MC_PROTOCOL_REG_IdLoopKd = 0x10,
	MC_PROTOCOL_REG_IdLoopKaw = 0x11,
	MC_PROTOCOL_REG_IdLoopLimit = 0x12,
	MC_PROTOCOL_REG_IqLoopKp = 0x13,
	MC_PROTOCOL_REG_IqLoopKi = 0x14,
	MC_PROTOCOL_REG_IqLoopKd = 0x15,
	MC_PROTOCOL_REG_IqLoopKaw = 0x16,                         
	MC_PROTOCOL_REG_IqLoopLimit = 0x17,
	MC_PROTOCOL_REG_SpeedLoopKp = 0x18,
	MC_PROTOCOL_REG_SpeedLoopKi = 0x19,
	MC_PROTOCOL_REG_SpeedLoopKd = 0x1A,
	MC_PROTOCOL_REG_SpeedLoopKaw = 0x1B,
	MC_PROTOCOL_REG_SpeedLoopLimit = 0x1C,
	MC_PROTOCOL_REG_PositionLoopKp = 0x1D,
	MC_PROTOCOL_REG_PositionLoopKi = 0x1E,	
	MC_PROTOCOL_REG_PositionLoopKd = 0x1F,
	MC_PROTOCOL_REG_PositionLoopKaw = 0x20,
	MC_PROTOCOL_REG_PositionLoopLimit = 0x21,
	
	MC_PROTOCOL_REG_ActualPosition = 0x22,
	MC_PROTOCOL_REG_ActualSpeed = 0x23,	
	MC_PROTOCOL_REG_ActualCurrent = 0x24,
	MC_PROTOCOL_REG_ActualMotorTemp = 0x25,
	MC_PROTOCOL_REG_ActualVoltage = 0x26,
	MC_PROTOCOL_REG_ActualMosTemp = 0x27,
	MC_PROTOCOL_REG_ReferencePosition = 0x28,
	MC_PROTOCOL_REG_ReferenceSpeed = 0x29,
	MC_PROTOCOL_REG_ReferenceCurrent = 0x2A,
	MC_PROTOCOL_REG_MotorStatus = 0x2B, 
} MC_Protocol_REG_t;


#define MC_PROTOCOL_CODE_PING             0x01
#define MC_PROTOCOL_CODE_GET_TABLE          0x02
#define MC_PROTOCOL_CODE_SET_TABLE          0x03
#define MC_PROTOCOL_CODE_MUL_READ          0x08

//#define MC_PROTOCOL_CODE_SET_REG          0x04
//#define MC_PROTOCOL_CODE_EN_REG          0x05
//#define MC_PROTOCOL_CODE_DEFAULT          0x06
//#define MC_PROTOCOL_CODE_SET_MIDDLE          0x07
//#define MC_PROTOCOL_CODE_CLE_MIDDLE          0x08
//#define MC_PROTOCOL_CODE_MUL_SET            0x83




/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__USERINTERFACE_H*/

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

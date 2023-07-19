/**
 ******************************************************************************
 * @file     car_control.h
 * @author
 * @brief
 * @date
 ******************************************************************************
 */

#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#include "stdint.h"

enum 
{
	CAR_IDLE = 0,
	CAR_RUN = 1,
	CAR_STOP =2
};

void Car_Enable(uint8_t enableOperation);
void Car_SetSpeed(float velRadial, float velCircle);
void Car_Forward(float speed);
void Car_Turn(float spinSpeed);
void Car_ErrAck(void);
void WheelSetPosition (int32_t targetPos);
void Car_SetPosition(float position);
void Car_SetMode (uint8_t mode);
void brake(void);
#endif /* __CAR_CONTROL_H */

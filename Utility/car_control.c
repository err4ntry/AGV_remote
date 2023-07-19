/**
 ******************************************************************************
 * @file     car_control.c
 * @author
 * @brief
 * @date
 ******************************************************************************
 */

#include "car_control.h"
#include "canfestival.h"
#include "master_node.h"
#include "master_node_aux.h"
#include "debug_printf.h"

#define L 1.0f
#define SPEEDRATIO 1562.5f
#define POSITIONRATIO 104303.78f
#define SPEEDMODE 0
#define POSITIONMODE 1

/**
 * @description:
 * @param {uint8_t} enableOperation equel 0 or 1
 * @return {*}
 */
void Car_Enable(uint8_t enableOperation)
{
	if (enableOperation == 1)
	{
		cMotorOperation = 0x01;
		sendOnePDOevent(&master_node_Data, 0);
				cMotorOperation = 0x01;
		sendOnePDOevent(&master_node_Data, 0);
				cMotorOperation = 0x01;
		sendOnePDOevent(&master_node_Data, 0);
				cMotorOperation = 0x01;
		sendOnePDOevent(&master_node_Data, 0);
				cMotorOperation = 0x01;
		sendOnePDOevent(&master_node_Data, 0);
		cMotorOperation = 0x06;
		sendOnePDOevent(&master_node_Data, 0);
		cMotorOperation = 0x06;
		sendOnePDOevent(&master_node_Data, 0);
		cMotorOperation = 0x06;
		sendOnePDOevent(&master_node_Data, 0);
		cMotorOperation = 0x06;
		sendOnePDOevent(&master_node_Data, 0);
		cMotorOperation = 0x06;
		sendOnePDOevent(&master_node_Data, 0);
		// HAL_Delay(1);
		cMotorOperation_aux = 0x01;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x01;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x01;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x01;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x01;
		sendOnePDOevent(&master_node_aux_Data, 0);
		cMotorOperation_aux = 0x06;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x06;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x06;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x06;
		sendOnePDOevent(&master_node_aux_Data, 0);
				cMotorOperation_aux = 0x06;
		sendOnePDOevent(&master_node_aux_Data, 0);
	}
	else if (enableOperation == 0)
	{
		cMotorOperation = 0x02;
		sendOnePDOevent(&master_node_Data, 0);
		cMotorOperation_aux = 0x02;
		sendOnePDOevent(&master_node_aux_Data, 0);
	}
	else
	{
	}
}

void Car_ErrAck(void)
{
	cMotorOperation = 0xf1;
	cMotorOperation_aux = 0xf1;
	sendOnePDOevent(&master_node_Data, 0);
	sendOnePDOevent(&master_node_aux_Data, 0);
}

/**
 * @description:
 * @param {float} speed < 1.6m/s
 * @return {*}
 */
void WheelLeftSetSpeed(float speed)
{
//	uint16_t speedValue = 0x8000 - (int16_t)(speed * SPEEDRATIO);
	uint16_t speedValue = 0;
	if(speed >= 0)
	{
		speedValue = speed * SPEEDRATIO;
	}
	else
	{
		speedValue = 0x8000 - speed * SPEEDRATIO;
	}
	cMotorOperation = 0x09;
	cReferenceSpeed = speedValue;
	sendOnePDOevent(&master_node_Data, 0);
}

/**
 * @description:
 * @param {float} speed < 1.6m/s
 * @return {*}
 */
void WheelRightSetSpeed(float speed)
{
//	uint16_t speedValue = (int16_t)(speed * SPEEDRATIO);
	
	uint16_t speedValue = 0;
	if(speed < 0)
	{
		speedValue = -speed * SPEEDRATIO;
	}
	else
	{
		speedValue = 0x8000 + speed * SPEEDRATIO;
	}
	cMotorOperation_aux = 0x09;
	cReferenceSpeed_aux = speedValue;
	sendOnePDOevent(&master_node_aux_Data, 0);
}

/**
 * @description:
 * @param {float} velRadial < 1.6m/s
 * @param {float} velCircle < 3.2rad/s
 * @return {*}
 */

void WheelSetPosition (int32_t targetPos)
{
	cMotorOperation = 0x0A;
	cMotorOperation_aux = 0x0A;
	cReferencePosition = -1*targetPos;
	cReferencePosition_aux = targetPos;
	sendOnePDOevent(&master_node_Data, 0);
	sendOnePDOevent(&master_node_aux_Data, 0);
}

void Car_SetPosition(float position)
{
	//周长0.62831853 65536   1m 104303.78f
	int32_t wTargetPos = position * POSITIONRATIO;
	WheelSetPosition(wTargetPos);
}

void Car_SetMode (uint8_t mode)
{
	
	if (mode == SPEEDMODE)
	{
		cMotorOperation = 0x06;//速度模式
	cMotorOperation_aux = 0x06;
	}

	if (mode == POSITIONMODE)
	{
		cMotorOperation = 0x07;//位置模式
	cMotorOperation_aux = 0x07;
	}
		
	sendOnePDOevent(&master_node_Data, 0);
	sendOnePDOevent(&master_node_aux_Data, 0);
	
	
}

void Car_SetSpeed(float velRadial, float velCircle)
{
	float liftSpeed, rightSpeed;
	liftSpeed = velRadial + 0.5f * velCircle * L;
	rightSpeed = velRadial - 0.5f * velCircle * L;
	WheelLeftSetSpeed(liftSpeed);
	WheelRightSetSpeed(rightSpeed);
}

/**
 * @description:
 * @param {float} speed < 1.6m/s
 * @return {*}
 */
void Car_Forward(float speed)
{
	Car_SetSpeed(speed, 0);
}

/**
 * @description:
 * @param {float} spinSpeed < 3.2rad/s
 * @return {*}
 */
void Car_Turn(float spinSpeed)
{
	Car_SetSpeed(0, spinSpeed);
}

void brake(void)
{
	cMotorOperation = 0x03;
	cMotorOperation_aux = 0x03;
	sendOnePDOevent(&master_node_Data, 0);
	sendOnePDOevent(&master_node_aux_Data, 0);
}
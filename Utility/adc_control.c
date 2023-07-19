/**
  ******************************************************************************
  * @file     adc_control.c  
  * @author  
	* @brief
	* @date
	******************************************************************************
  */
	
#include "adc_control.h"
#include "canfestival.h"
#include "adc.h"
#include "master_node.h"
#include "master_node_aux.h"

#define VDD_APPLI                      ((uint32_t)3000)    /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t)4095)    /* Max value with a full range of 12 bits */
#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)                        \
  ( (ADC_DATA) * VDD_APPLI / RANGE_12BITS)
	
uint16_t uhADCxConvertedValue = 0;
uint16_t valueArray[32] = {0}; 

uint16_t valueFileter(uint16_t value);
void SendSpeed(uint16_t value);
//uint16_t valueFilter;
//uint16_t speed;




float AC_Handle(void)
{
	  uint16_t valueFilter;
	  static uint16_t preValue;
	  float carSpeed = 0;
		HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
		uhADCxConvertedValue = HAL_ADC_GetValue(&hadc1);
	  valueFilter = valueFileter(uhADCxConvertedValue);
//	  if ((valueFilter >> 4) != (preValue >> 4))
//		{
//			uint16_t speed;
//			speed = (valueFilter & 0xFFC0) + 0x8000;
//			preValue = (valueFilter & 0xFFC0);
//		    carSpeed = valueFilter/4095 * 1.6f;
//			//SendSpeed(speed);
//		}
	carSpeed = (valueFilter & 0xFFC0)/4095.0f * 1.6f;
	return carSpeed;				  	
}

uint16_t valueFileter(uint16_t value)
{
	static uint16_t index;
	static uint32_t valueSum;
	uint16_t valueFilter;
	valueArray[index++] = value;
	valueSum += value;
	valueFilter = valueSum >> 5;
	if (index >= 32)
		index = 0;
	valueSum -= valueArray[index];
	return valueFilter;
}


void SendSpeed(uint16_t value)
{
	cMotorOperation = 0x09;
	cReferenceSpeed = value;
	sendOnePDOevent(&master_node_Data,0);
	cMotorOperation_aux = 0x09;
	cReferenceSpeed_aux = value;
	sendOnePDOevent(&master_node_aux_Data,0);
}


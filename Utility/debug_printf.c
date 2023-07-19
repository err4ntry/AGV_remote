/**
  ******************************************************************************
  * @file     debug_printf.c  
  * @author  
	* @brief
	* @date
	******************************************************************************
  */

#include "debug_printf.h"
#include "stm32f4xx.h"

#define DEBUG_USART USART2

#pragma import(__use_no_semihosting)             
            
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
  
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{ 	
	while((DEBUG_USART->SR&0X40)==0);//
	DEBUG_USART->DR = (uint8_t) ch;      
	return ch;
}

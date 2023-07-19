/**
  ******************************************************************************
  * @file     debug_printf.h  
  * @author  
	* @brief
	* @date
	******************************************************************************
  */

#ifndef __DEBUG_PRINTF_H
#define __DEBUG_PRINTF_H

#include "stdio.h"

#define DEBUG_STDIO 1
#if DEBUG_STDIO
#   define DEBUG_PRINTF(...) do { printf(__VA_ARGS__); } while(0)
#else
#   define DEBUG_PRINTF(...) {}
#endif


#endif /* __UART_PROTOCOL_H */


	
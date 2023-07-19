/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug_printf.h"
#include "uart_protocol.h"
#include "canFestival.h"
#include "adc_control.h"
#include "master_node.h"
#include "master_node_aux.h"
#include "can_handle.h"
#include "multi_button.h"
#include "car_control.h"
#include "mylog.h"
#include "multi_timer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
UNS8 node_id = 0x15;
UNS8 node_id_aux = 0x16;
uint16_t keyFlag;
struct Button button0;
struct Button button_emcy;
struct Button button_green;
struct Button button_red;
struct Button button_yellow;
uint16_t carState = CAR_STOP;
float carSpeeds;
float carPosition;
int speed_flag;
int position_flag;

uint16_t pre_value = 0;
uint16_t rem_frontorrear=0;

extern SBUS_CH_Struct SBUS_CH;

enum
{
	BUTTON0 = 0,
	BUTTON_EMERGENCY = 1,
	BUTTON_GREEN = 2,
	BUTTON_RED = 3,
	BUTTON_YELLOW = 4,
};

typedef struct CHxData
{
	uint16_t preValue;
	uint16_t curValue;
} CHxData_t;

CHxData_t ch1_data;
CHxData_t ch2_data;
CHxData_t ch3_data;
CHxData_t ch5_data;
CHxData_t ch7_data;
CHxData_t ch10_data;


uint16_t IsChanged (CHxData_t* chx_data,uint16_t value);








struct Timer buttonTimer;
void ButtonTimer_callback(void *arg)
{
    button_ticks();
	//DEBUG_PRINTF("timer callback\r\n");
}

struct Timer loopTriggerTimer1;
struct Timer loopTriggerTimer2;
struct Timer loopTriggerTimer3;

void LoopTrigger(void* arg)
{
	uint16_t numTimer = (uint16_t)arg;

	switch(numTimer)
	{
		case 1:
			DEBUG_PRINTF("time1\r\n");
			timer_init(&loopTriggerTimer2, LoopTrigger, 1000, 0, (void*)2); //1s loop
			timer_start(&loopTriggerTimer2);
			break;
		case 2:
			DEBUG_PRINTF("timer2\r\n");
			timer_init(&loopTriggerTimer3, LoopTrigger, 1000, 0, (void*)3); //1s loop
			timer_start(&loopTriggerTimer3);
			break;
		case 3:
			DEBUG_PRINTF("timer3\r\n");
			timer_init(&loopTriggerTimer1, LoopTrigger, 1000, 0, (void*)1); //1s loop
			timer_start(&loopTriggerTimer1);
			break;
		default:
			break;
	}
}




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t read_button_GPIO(uint8_t button_id)
{
	switch (button_id)
	{
	case BUTTON0:
		return HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);
		break;
	case BUTTON_EMERGENCY:
		return HAL_GPIO_ReadPin(Button_Emergency_GPIO_Port, Button_Emergency_Pin);
		break;
	case BUTTON_GREEN:
		return HAL_GPIO_ReadPin(Button_Green_GPIO_Port, Button_Green_Pin);
		break;
	case BUTTON_RED:
		return HAL_GPIO_ReadPin(Button_Red_GPIO_Port, Button_Red_Pin);
		break;
	case BUTTON_YELLOW:
		return HAL_GPIO_ReadPin(Button_Yellow_GPIO_Port, Button_Yellow_Pin);
		break;
	default:
		return 0;
		break;
	}
}

void btn_press_up_Handler(void *btn)
{
	struct Button *button = btn;
	switch (button->button_id)
	{
	case BUTTON0:
		break;
	case BUTTON_EMERGENCY:
		Car_ErrAck();
		DEBUG_PRINTF("--->Error ACK<---\r\n");
		break;
	case BUTTON_GREEN:
		carState = CAR_STOP;
		Car_Forward(0);
		Car_Turn(0);
		DEBUG_PRINTF("--->Car stop<---\r\n");
		break;
	case BUTTON_RED:
		break;
	case BUTTON_YELLOW:
		carState = CAR_STOP;
		Car_Forward(0);
		Car_Turn(0);
		DEBUG_PRINTF("--->Car stop<---\r\n");
		break;
	default:
		break;
	}
	// DEBUG_PRINTF("--->key0 press down! <---\r\n");
}

void btn_press_down_Handler(void *btn)
{
	struct Button *button = btn;
	static uint16_t key0State = 0;
	static uint16_t keyRedState = 0;
	static uint16_t mode_SpinOrForward = 0;
	switch (button->button_id)
	{
	case BUTTON0:
		
		if (0 == key0State)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			Car_Enable(1);
			DEBUG_PRINTF("--->Car enable<---\r\n");
		}
		else
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			Car_Enable(0);
			DEBUG_PRINTF("--->Car disable<---\r\n");
		}
		key0State = ~key0State;
		break;
	case BUTTON_EMERGENCY:
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		Car_Enable(0);
		DEBUG_PRINTF("--->Emergency stop<---\r\n");
		break;
	case BUTTON_GREEN:
		carState = CAR_RUN;

		if (0 == mode_SpinOrForward)
		{
			Car_Forward(carSpeeds);
			DEBUG_PRINTF("--->Car forward<---\r\n");
		}
		else
		{
			Car_Turn(0.2);
			DEBUG_PRINTF("--->Car turn<---\r\n");
		}
		break;
	case BUTTON_RED:
		if (0 == keyRedState)
		{
			mode_SpinOrForward = 1;
			DEBUG_PRINTF("--->Spin mode<---\r\n");
		}
		else
		{
			mode_SpinOrForward = 0;
			DEBUG_PRINTF("--->Forward mode<---\r\n");
		}
		keyRedState = ~keyRedState;
		break;
	case BUTTON_YELLOW:
		carState = CAR_RUN;
		if (0 == mode_SpinOrForward)
		{
			Car_Forward(-1.0*carSpeeds);
			DEBUG_PRINTF("--->Car reverse<---\r\n");
		}
		else
		{
			Car_Turn(-0.2);
			DEBUG_PRINTF("--->Car reverse turn<---\r\n");
		}
		break;
	default:
		break;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
	
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_CAN1_Init();
	MX_USART2_UART_Init();
	
	uart_init(100000);              //初始化USART1
  stop_Init();
	
	
	MX_ADC1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	CanFilterConfig();
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_TIM_Base_Start_IT(&htim2);
	UP_WaitNextFrame();
	InitNodes(&master_node_Data, node_id);
	// HAL_Delay(10);
	

	InitNodes(&master_node_aux_Data, node_id_aux);


	button_init(&button0, read_button_GPIO, 0, BUTTON0);
	button_init(&button_emcy, read_button_GPIO, 0, BUTTON_EMERGENCY);
	button_init(&button_green, read_button_GPIO, 0, BUTTON_GREEN);
	button_init(&button_red, read_button_GPIO, 0, BUTTON_RED);
	button_init(&button_yellow, read_button_GPIO, 0, BUTTON_YELLOW);

	button_attach(&button0, PRESS_DOWN, btn_press_down_Handler);
	button_attach(&button_emcy, PRESS_DOWN, btn_press_down_Handler);
	button_attach(&button_green, PRESS_DOWN, btn_press_down_Handler);
	button_attach(&button_red, PRESS_DOWN, btn_press_down_Handler);
	button_attach(&button_yellow, PRESS_DOWN, btn_press_down_Handler);

	button_attach(&button_green, PRESS_UP, btn_press_up_Handler);
	button_attach(&button_yellow, PRESS_UP, btn_press_up_Handler);
	button_attach(&button_emcy, PRESS_UP, btn_press_up_Handler);

	button_start(&button0);
	button_start(&button_emcy);
	button_start(&button_green);
	button_start(&button_red);
	button_start(&button_yellow);
	
	timer_init(&buttonTimer, ButtonTimer_callback, 5, 5, NULL); //1s loop
	timer_start(&buttonTimer);
	

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

		
	while (1)
	{
		if (IsChanged(&ch10_data,SBUS_CH.CH10))
		{
			if(SBUS_CH.CH10>1000)
			{
				brake();
			}
		}
		if (IsChanged(&ch5_data,SBUS_CH.CH5))
		{
			if(SBUS_CH.CH5>1000)
			{
				Car_Enable(1);
			}
	//		if (IsChanged(&ch5_data,SBUS_CH.CH5))
		//	{
					if(SBUS_CH.CH5<200)
				{
					Car_Enable(0);
					Car_ErrAck();//清除错误
				}
		//	}

		}
		

		
			if(SBUS_CH.CH7<1000)
			{
				rem_frontorrear=0;
				//前进
			}
			else
			{
				rem_frontorrear=1;
			  //后退
			  //Car_SetMode(1);//转弯
			}
		
//		
//		if(IsChanged(&ch3_data,SBUS_CH.CH3))
//		{
			//DEBUG_PRINTF("ch3=%d\r\n",SBUS_CH.CH3);
//			if(rem_frontorrear==0)
//			{
				
//				Car_Forward(carSpeeds);

			
			float scale_flag=(SBUS_CH.CH1-172)/1639.0f;
			int radial_flag=SBUS_CH.CH1/10-17;
			float rem_Radial=radial_flag/102.5f;
			int circle_flag=SBUS_CH.CH2/10-98;
			float rem_Circle=circle_flag/102.5f*scale_flag;
			
			if(rem_frontorrear==0)
			{
				printf("debug");
				Car_SetSpeed(rem_Radial,rem_Circle);
			}
			else
			{
				Car_SetSpeed(-1.0*rem_Radial,rem_Circle);
			}
	
			//printf("Radial=%f  ",rem_Radial);
			//printf("Circle=%f\r\n",rem_Circle);
//			}
			 
//		}
			 
//if(IsChanged(&ch2_data,SBUS_CH.CH2)|IsChanged(&ch2_data,SBUS_CH.CH3))//转弯
//	if(IsChanged(&ch2_data,SBUS_CH.CH2))//位置模式
//		{
//				if(rem_spinorforword==1)
//				{
//					if(SBUS_CH.CH2>1800)
//					{
//						
//						Car_SetPosition(carPosition);
//						printf("carPosition=%f\r\n",carPosition);
//					}

////					int radial_flag=SBUS_CH.CH3/10-99;
////					float rem_Radial=radial_flag/102.5f;
////					int circle_flag=SBUS_CH.CH2/10-98;
////					float rem_Circle=circle_flag/51.25f;
////					Car_SetSpeed(rem_Radial,rem_Circle);
////					printf("Radial=%f  ",rem_Radial);
////					printf("Circle=%f\r\n",rem_Circle);
//				}
//			}

		
//		if(SBUS_CH.CH3<1800&&SBUS_CH.CH3>180&&SBUS_CH.CH7<1000)
//		{
//			carState = CAR_STOP;
//		}
//	else
//		carState =CAR_RUN;
			
		HAL_Delay(100);
	
//	rem_flag=SBUS_CH.CH3/10-99;
//	carSpeeds =rem_flag/51.25f;
	//printf("car=%f\r\n",carSpeeds);
	
		//button_ticks();
//		if (carState == CAR_STOP)
//		{
//			position_flag=SBUS_CH.CH11/10-17;
//			carPosition =position_flag/16.4f;//位置范围0-10m
//	
//			speed_flag=SBUS_CH.CH12/10-17;
//			carSpeeds =speed_flag/102.5f;//速度范围0-1.6m/s
//			printf("car=%f\r\n",carSpeeds);
//		}
//		carSpeeds =  AC_Handle();		
		timer_loop();		

		//	UP_PackTransmit();
		// UP_SendHandle();
		 //DEBUG_PRINTF("hello\r\n");
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
uint16_t IsChanged (CHxData_t* chx_data,uint16_t value)
{
	chx_data->curValue = value;
	if((chx_data->curValue>>4)==(chx_data->preValue>>4))
		return 0;
	else
	{ 
		chx_data->preValue = chx_data->curValue;
		return 1;
	}
		
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

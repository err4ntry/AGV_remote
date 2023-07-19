/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "stdio.h"
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//#if 1
//#pragma import(__use_no_semihosting)             
////��׼����Ҫ��֧�ֺ���                 
//struct __FILE 
//{ 
//	int handle; 
//}; 

//FILE __stdout;       
////����_sys_exit()�Ա���ʹ�ð�����ģʽ    
//void _sys_exit(int x) 
//{ 
//	x = x; 
//} 
////�ض���fputc���� 
//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
//	USART1->DR = (uint8_t) ch;      
//	return ch;
//}
//#endif 


UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

UART_HandleTypeDef UART1_Handler; //UART���
uint8_t aRxBuffer[1];//HAL��ʹ�õĴ��ڽ��ջ���

uint8_t USART_RX_BUF[100];     //���ջ���,���100���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
uint16_t USART_RX_STA=0;       //����״̬���	

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}


void uart_init(uint32_t bound)
{	
	//UART ��ʼ������
	UART1_Handler.Instance=USART1;					    //USART1
	UART1_Handler.Init.BaudRate=bound;				    //������
	UART1_Handler.Init.WordLength=UART_WORDLENGTH_9B;   //�ֳ�Ϊ8λ���ݸ�ʽ
	UART1_Handler.Init.StopBits=UART_STOPBITS_1;	    //һ��ֹͣλ
	UART1_Handler.Init.Parity=UART_PARITY_EVEN;		    //����żУ��λ
	UART1_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
	UART1_Handler.Init.Mode=UART_MODE_TX_RX;		    //�շ�ģʽ
	HAL_UART_Init(&UART1_Handler);					    //HAL_UART_Init()��ʹ��UART1
	
	HAL_UART_Receive_IT(&UART1_Handler, (uint8_t *)aRxBuffer, 1);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
  
}





void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
	    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(uartHandle->Instance==USART1)//����Ǵ���1�����д���1 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART1_CLK_ENABLE();			//ʹ��USART1ʱ��
	
		GPIO_Initure.Pin=GPIO_PIN_9;			//PA9
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FAST;		//����
		GPIO_Initure.Alternate=GPIO_AF7_USART1;	//����ΪUSART1
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA9

		GPIO_Initure.Pin=GPIO_PIN_10;			//PA10
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��PA10
		
#if EN_USART1_RX
		HAL_NVIC_EnableIRQ(USART1_IRQn);				//ʹ��USART1�ж�ͨ��
		HAL_NVIC_SetPriority(USART1_IRQn,3,0);			//��ռ���ȼ�3�������ȼ�3
#endif	
	}
	

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



SBUS_CH_Struct SBUS_CH;

uint8_t update_sbus(uint8_t *buf)
{
    int i;
    if (buf[23] == 0)
    {
        SBUS_CH.ConnectState = 1;
        SBUS_CH.CH1 = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
        SBUS_CH.CH2 = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
        SBUS_CH.CH3 = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 ) | (int16_t)buf[ 5] << 10 ) & 0x07FF;
        SBUS_CH.CH4 = ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
        SBUS_CH.CH5 = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
        SBUS_CH.CH6 = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 ) | (int16_t)buf[9] << 9 ) & 0x07FF;
        SBUS_CH.CH7 = ((int16_t)buf[ 9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
        SBUS_CH.CH8 = ((int16_t)buf[10] >> 5 | ((int16_t)buf[11] << 3 )) & 0x07FF;
        SBUS_CH.CH9 = ((int16_t)buf[12] << 0 | ((int16_t)buf[13] << 8 )) & 0x07FF;
        SBUS_CH.CH10 = ((int16_t)buf[13] >> 3 | ((int16_t)buf[14] << 5 )) & 0x07FF;
        SBUS_CH.CH11 = ((int16_t)buf[14] >> 6 | ((int16_t)buf[15] << 2 ) | (int16_t)buf[16] << 10 ) & 0x07FF;
        SBUS_CH.CH12 = ((int16_t)buf[16] >> 1 | ((int16_t)buf[17] << 7 )) & 0x07FF;
        SBUS_CH.CH13 = ((int16_t)buf[17] >> 4 | ((int16_t)buf[18] << 4 )) & 0x07FF;
        SBUS_CH.CH14 = ((int16_t)buf[18] >> 7 | ((int16_t)buf[19] << 1 ) | (int16_t)buf[20] << 9 ) & 0x07FF;
        SBUS_CH.CH15 = ((int16_t)buf[20] >> 2 | ((int16_t)buf[21] << 6 )) & 0x07FF;
        SBUS_CH.CH16 = ((int16_t)buf[21] >> 5 | ((int16_t)buf[22] << 3 )) & 0x07FF;
        return 1;
    }
    else 
    {
        SBUS_CH.ConnectState = 0;
        return 0;
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int i;
	while (huart->Instance==USART1) 
	{
		USART_RX_BUF[USART_RX_STA]=aRxBuffer[0] ;
		if (USART_RX_STA == 0 && USART_RX_BUF[USART_RX_STA] != 0x0F) break; //֡ͷ���ԣ�����
		USART_RX_STA++;
		if (USART_RX_STA > 25) USART_RX_STA = 0;  
		if (USART_RX_STA == 25)
		{
			if (USART_RX_BUF[0] == 0x0F && USART_RX_BUF[24] == 0x00)
			{
				update_sbus(USART_RX_BUF);
				
				
				
		if(SBUS_CH.CH9>800)//��ͣ
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);	//Pc8��1
		}
		else
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	//Pc8��0
		}
		
//			printf("%d  ",SBUS_CH.CH1);
//	    printf("%d  ",SBUS_CH.CH2);
//	    printf("%d  ",SBUS_CH.CH3);
//	    printf("%d  ",SBUS_CH.CH4);
//			printf("%d  ",SBUS_CH.CH5);
//			printf("%d  ",SBUS_CH.CH6);
//			printf("%d  ",SBUS_CH.CH7);
//			printf("%d  ",SBUS_CH.CH8);
//			printf("%d  ",SBUS_CH.CH9);
//			printf("%d  ",SBUS_CH.CH10);
//			printf("%d  ",SBUS_CH.CH11);
//			printf("%d  ",SBUS_CH.CH12);
//			printf("%d  ",SBUS_CH.CH13);
//			printf("%d  ",SBUS_CH.CH14);
//			printf("%d  ",SBUS_CH.CH15);
//			printf("%d  \r\n",SBUS_CH.CH16);
				for (i = 0; i<25; i++)	//��ջ�����
					USART_RX_BUF[i] = 0;
				USART_RX_STA = 0;
			}
		}
		

		break;
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==USART1)//����Ǵ���1
//	{
//		if((USART_RX_STA&0x8000)==0)//����δ���
//		{
//			if(USART_RX_STA&0x4000)//���յ��˱�ʶ��
//			{
//				if(aRxBuffer[0]==0x00)USART_RX_STA|=0x8000;	//��������� 
//				else
//				{
//					USART_RX_STA&=0X0FFF;
//					USART_RX_BUF[USART_RX_STA]=aRxBuffer[0] ;
//	        USART_RX_STA++;
//				}
//			}
//			else//δ�յ���ʶ��
//			{
//				if(aRxBuffer[0]==0x0c||aRxBuffer[0]==0x00)
//				{
//					USART_RX_STA|=0x4000;
//				}
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
//	        USART_RX_STA++;
//				}
//			}
//		}

//	}
//	
//}

void USART1_IRQHandler(void)
{
//if(USART_RX_STA&0x8000)
//{
//	printf("%d  ",SBUS_CH.CH1);
//	printf("%d  ",SBUS_CH.CH2);
//	printf("%d  ",SBUS_CH.CH3);
//	printf("%d  ",SBUS_CH.CH4);
//	printf("%d  ",SBUS_CH.CH5);
//	printf("%d  ",SBUS_CH.CH6);
//	printf("%d  ",SBUS_CH.CH7);
//	printf("%d  ",SBUS_CH.CH8);
//	printf("%d  ",SBUS_CH.CH9);
//	printf("%d  ",SBUS_CH.CH10);
//	printf("%d  ",SBUS_CH.CH11);
//	printf("%d  ",SBUS_CH.CH12);
//	printf("%d  ",SBUS_CH.CH13);
//	printf("%d  ",SBUS_CH.CH14);
//	printf("%d  ",SBUS_CH.CH15);
//	printf("%d  \r\n",SBUS_CH.CH16);
//	
//	printf("%d  ",USART_RX_BUF[0]);
//	printf("%d  ",USART_RX_BUF[1]);
//	printf("%d  ",USART_RX_BUF[2]);
//	printf("%d  ",USART_RX_BUF[3]);
//	printf("%d  ",USART_RX_BUF[4]);
//	printf("%d  ",USART_RX_BUF[5]);
//	printf("%d  ",USART_RX_BUF[6]);
//	printf("%d  ",USART_RX_BUF[7]);
//	printf("%d  ",USART_RX_BUF[8]);
//	printf("%d  ",USART_RX_BUF[9]);
//	printf("%d  ",USART_RX_BUF[10]);
//	printf("%d  ",USART_RX_BUF[11]);
//	printf("%d  ",USART_RX_BUF[12]);
//	printf("%d  ",USART_RX_BUF[13]);
//	printf("%d  ",USART_RX_BUF[14]);
//	printf("%d  ",USART_RX_BUF[15]);
//	printf("%d  ",USART_RX_BUF[16]);
//	printf("%d  ",USART_RX_BUF[17]);
//	printf("%d  ",USART_RX_BUF[18]);
//	printf("%d  ",USART_RX_BUF[19]);
//	printf("%d  ",USART_RX_BUF[20]);
//	printf("%d  ",USART_RX_BUF[21]);
//	printf("%d  ",USART_RX_BUF[22]);
//	printf("%d  ",USART_RX_BUF[23]);
//	printf("%d  ",USART_RX_BUF[25]);
//	printf("%d  ",USART_RX_BUF[26]);
//	printf("%d  ",USART_RX_BUF[27]);
//	printf("%d  ",USART_RX_BUF[28]);
//	printf("%d  ",USART_RX_BUF[29]);
//	printf("%d  ",USART_RX_BUF[30]);
//	printf("%d  ",USART_RX_BUF[31]);
//	printf("%d  ",USART_RX_BUF[32]);
//	printf("%d  \r\n",USART_RX_BUF[24]);
//	
//	USART_RX_STA=0;
//}

	
	
	
	uint32_t timeout=0;
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntEnter();    
#endif
	
	HAL_UART_IRQHandler(&UART1_Handler);	//����HAL���жϴ����ú���
	
	timeout=0;
    while (HAL_UART_GetState(&UART1_Handler) != HAL_UART_STATE_READY)//�ȴ�����
	{
	 timeout++;////��ʱ����
     if(timeout>HAL_MAX_DELAY) break;		
	
	}
     
	timeout=0;
	while(HAL_UART_Receive_IT(&UART1_Handler, (uint8_t *)aRxBuffer, 1) != HAL_OK)//һ�δ������֮�����¿����жϲ�����RxXferCountΪ1
	{
	 timeout++; //��ʱ����
	 if(timeout>HAL_MAX_DELAY) break;	
	}
#if SYSTEM_SUPPORT_OS	 	//ʹ��OS
	OSIntExit();  											 
#endif

}



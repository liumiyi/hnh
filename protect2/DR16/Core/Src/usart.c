/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "timers.h"

uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc = rc_Init;

RemoteControlState_t g_RemoteControlState; //全局变量，用于追踪遥控器的状态
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

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
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
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

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/************************************************************************************/

static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{//用于接收数据的函数，使用DMA方式来接收UART数据
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
	//创建一个临时变量tmp1，并将UART的接收状态赋值给它
	if (tmp1 == HAL_UART_STATE_READY)//判断UART是否处于就绪状态
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//启动DMA接收，源地址为UART数据寄存器，目标地址为接收缓冲区
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//使能UART的DMA接收功能
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}


void dbus_uart_init(void)//DBUS串口初始化
{
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);//用于清除UART的空闲标志，空闲标志指示UART在接收数据时处于空闲状态，通常在接收完成后设置
	//清除这个标志是为了确保后续的接收操作能够正确检测到新的空闲状态
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);//使能UART的空闲中断，当UART处于空闲状态并且接收缓冲区没有数据时，会触发这个中断
	//使能这个中断后，可以在中断服务例程中处理空闲状态
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);//调用之前的函数，用DMA来接收串口数据
}



void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{//用于处理接收到的遥控器（rc）数据，将接收到的字节解码
  rc->ch0 = (buff[0] | buff[1] << 8) & 0x07FF;//将buff[0]和buff[1]的值组合为ch0通道的值，并将其限制在11位（通过与0x07FF按位与）
  rc->ch0 -= 1024;//由于数据在364到1684，将解码后的数据减去1024，使其中心值为0
  rc->ch1 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch3 -= 1024;
  rc->roll = (buff[16] | (buff[17] << 8)) & 0x07FF;  //左上角滚轮
  rc->roll -= 1024;
 
  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;//sw1和sw2的值分别由相应的位计算得出
	
  if ((abs(rc->ch0) > 660) || \
      (abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660))
	  
  {
    memset(rc, 0, sizeof(rc_info_t));//如果任一通道的绝对值超过660，表示接收到异常数据，则将rc结构体的所有内容清零
  }		
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{//返回DMA预定义的缓冲区剩余的长度，方便了解传输过程中还有多少数据尚未传输
  return ((uint16_t)(dma_stream->NDTR));
}
 
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//清除UART的空闲标志，以便下一次接收时能够正确检测到空闲状态
	
	if (huart == &DBUS_HUART)//确保只处理DBUS串口
	{
		__HAL_DMA_DISABLE(huart->hdmarx);//失能DMA接收，防止下一次接收的数据在上一次数据的尾部，而不是全新的数据
 
		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{//计算当前接收的数据长度，如果接收到的数据长度等于18字节，则调用处理数据函数
			rc_callback_handler(&rc, dbus_buf);	//处理接收的数据并解码
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);//设置DMA接收预定义的缓冲区的长度，以便为下一次接收做好准备
		__HAL_DMA_ENABLE(huart->hdmarx);//重新启用DMA接收，以便继续接收数据
	}
}


void uart_receive_handler(UART_HandleTypeDef *huart)
{//用于检查UART接收状态并在接收到空闲状态时调用相应的回调函数
    
    g_RemoteControlState.isRemoteActive=true;                         //将遥控状态设为活跃
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTimerResetFromISR(g_RemoteControlState.xNoSignalTimer,&xHigherPriorityTaskWoken);//重置软件定时器信号，喂狗

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //检查UART是否设置了空闲标志，表示UART接收完成并进入空闲状态
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//检查UART空闲中断是否被使能，只有在中断使能的情况下，才会处理空闲状态
	{
		uart_rx_idle_callback(huart);//调用之前定义的函数，处理接收到的数据
	}
}

void vNoSignalTimerCallback(TimerHandle_t xTimer) 
{
    // 当定时器超时时，意味着失去了遥控信号
	  
	 
    RemoteControlState_t *pState = (RemoteControlState_t *)pvTimerGetTimerID(xTimer);
	 
    pState->isRemoteActive = false;  //将状态置为不活跃          
	                                   //关闭电机输出......
	 
	  
}

void InitializeRemoteControl(RemoteControlState_t *pState) 
{
    // 初始化遥控状态为不活跃
    pState->isRemoteActive = false;

    // 创建定时器
    pState->xNoSignalTimer = xTimerCreate
	  (
        "NoSignalTimer",          // 定时器名称
        pdMS_TO_TICKS(16),         // 定时器周期(超时时间)
        pdFALSE,                   // 是否重复
        (void *)pState,           // 定时器ID，传递结构体指针
        vNoSignalTimerCallback    // 回调函数
    );

    if (pState->xNoSignalTimer == NULL) //创建定时器失败
	{
        // 处理错误
    }

}
/* USER CODE END 1 */

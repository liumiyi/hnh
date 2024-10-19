/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "FreeRTOS.h"
#include "timers.h"

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart2


typedef __packed struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
  uint8_t flag;
} rc_info_t;
 
#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}

typedef struct
{
    bool isRemoteActive;  // 遥控信号是否活跃
    TimerHandle_t xNoSignalTimer;
} RemoteControlState_t;

static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void dbus_uart_init(void);
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
void uart_receive_handler(UART_HandleTypeDef *huart);

void InitializeRemoteControl(RemoteControlState_t *pState) ; //初始化遥控状态结构体并创建对应的软件定时器

extern uint8_t dbus_buf[DBUS_BUFLEN];
extern rc_info_t rc;

extern RemoteControlState_t g_RemoteControlState;
/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t   dbus_buf[DBUS_BUFLEN];

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


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

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define BUFF_SIZE	100
extern uint8_t rx_buff[BUFF_SIZE];
extern uint8_t Serial_RxFlag;
extern uint8_t ByteRecv;
void hhSerialSendByte(uint8_t Byte);
uint8_t Serial_GetRxFlag(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef  *huart);
extern DMA_HandleTypeDef hdma_usart1_rx;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart);

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */


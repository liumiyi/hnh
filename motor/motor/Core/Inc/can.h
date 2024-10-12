/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
extern CAN_FilterTypeDef sFilterConfig;

extern CAN_TxHeaderTypeDef TxMessage;
extern CAN_RxHeaderTypeDef RxMessage;

extern uint8_t TxData[8];
extern uint8_t RxData[8];

extern uint32_t TxMailbox;//store the Tx message
extern uint8_t  aData[8];    //data that can receive
//çľćşäżĄćŻć°ćŽçťćä˝ĺŽäš?
typedef struct
{
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  motor_temperature;
    int16_t  last_ecd;
} Motor_Info;
extern Motor_Info motor_info;
void Configure_Filter(void);
void CAN_Transmit(int16_t v);

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


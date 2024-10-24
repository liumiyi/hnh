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

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
extern CAN_FilterTypeDef sFilterConfig;

extern CAN_TxHeaderTypeDef TxMessage;
extern CAN_RxHeaderTypeDef RxMessage;

extern uint8_t can1_0x200_TxData[8];

extern uint32_t TxMailbox;//store the Tx message
extern uint8_t  aData[8];    //data that can receive

typedef struct
{
    int16_t  rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  motor_temperature;
    int16_t  last_ecd;
    uint8_t  c_Init;
    int16_t round_cnt;
    int16_t  c_offset;
    int64_t  total_encoder;
    uint8_t  gearbox_rate;
} Motor_Info;
extern Motor_Info motor_info1;//右摩擦轮
extern Motor_Info motor_info2;//左摩擦轮

extern Motor_Info motor_info3;//拨盘电机的结构体

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void Configure_Filter(void);
void CAN_Transmit_1(int16_t v);//右摩擦轮
void CAN_Transmit_2(int16_t v);//左摩擦轮
void CAN_Transmit_3(int16_t v);//拨盘电机
void update_angle(Motor_Info* motor_info);
//void dial_angle(Motor_Info* motor_info);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


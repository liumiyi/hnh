/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "usart.h"
#include <stdio.h>

CAN_FilterTypeDef sFilterConfig;

CAN_TxHeaderTypeDef TxMessage;
CAN_TxHeaderTypeDef TxMessage2;
CAN_TxHeaderTypeDef TxMessage3;
uint8_t can1_0x200_TxData[8];
uint8_t can1_0x1FF_TxData[8];
uint8_t can2_0x2FF_TxData[8];
uint32_t TxMailbox;

CAN_RxHeaderTypeDef RxMessage;
uint8_t  aData[8];

Motor_Info motor_info1;
Motor_Info motor_info2;//两个摩擦轮电机信息的结构�?

Motor_Info motor_info3;//拨盘电机的结构体

Motor_Info motor_info4;//pitch电机的结构体

Motor_Info motor_info5;//yaw电机的结构体

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 7;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void Configure_Filter(void)
{
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//把接收到的报文放入到FIFO0
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;//为从CAN实例选择启动筛鿉器组㿂对于单个CAN实例，此参数没有意义。对于双CAN实例，所有具有较低索引的过滤器组都被分配给主CAN实例，迌�?有具有较大索引的过滤器组都被分配给从CAN实例。该参数必须为Min_Data = O和Max_Data =27之间的一个数�?. 
    //配置过滤�?
    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig)!=HAL_OK)
    {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__); 
     } 
     //弿启CAN
     if(HAL_CAN_Start(&hcan1)!=HAL_OK)
     {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);  
     }
     //当FIFO0中有消息的时候进入中�?
     if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
     {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__); 
     }
}

void Configure_Filter2(void)//配置can2
{
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//把接收到的报文放入到FIFO0
    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;//为从CAN实例选择启动筛鿉器组㿂对于单个CAN实例，此参数没有意义。对于双CAN实例，所有具有较低索引的过滤器组都被分配给主CAN实例，迌承有具有较大索引的过滤器组都被分配给从CAN实例。该参数必须为Min_Data = O和Max_Data =27之间的一个数孿. 
    //配置过滤噿
    if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig)!=HAL_OK)
    {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__); 
     } 
     //弿启CAN
     if(HAL_CAN_Start(&hcan2)!=HAL_OK)
     {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);  
     }
     //当FIFO0中有消息的时候进入中斿
     if(HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
     {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__); 
     }
}

void CAN_Transmit_1(int16_t v)//同时控制左右摩擦轮电机，id�?2
{
  TxMessage.IDE=CAN_ID_STD;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.DLC=8;
  
  TxMessage.StdId=0x200;
  
  can1_0x200_TxData[0] = 0;
  can1_0x200_TxData[1] = 0;
  can1_0x200_TxData[2] = (v>>8);
  can1_0x200_TxData[3] = (v);
  can1_0x200_TxData[4] = (v>>8);
  can1_0x200_TxData[5] = (v);
  can1_0x200_TxData[6] = 0;
  can1_0x200_TxData[7] = 0;
    
  if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,can1_0x200_TxData,&TxMailbox)!= HAL_OK)
  {
        Error_Handler();
  }
}

//void CAN_Transmit_2(int16_t v)//控制左摩擦轮电机，id�?3
//{
//  TxMessage.IDE=CAN_ID_STD;
//  TxMessage.RTR=CAN_RTR_DATA;
//  TxMessage.DLC=8;
//  
//  TxMessage.StdId=0x200;
//  
//  can1_0x200_TxData[0] = 0;
//  can1_0x200_TxData[1] = 0;
//  can1_0x200_TxData[2] = 0;
//  can1_0x200_TxData[3] = 0;
//  can1_0x200_TxData[4] = (v>>8);
//  can1_0x200_TxData[5] = (v);
//  can1_0x200_TxData[6] = 0;
//  can1_0x200_TxData[7] = 0;
//    
//  if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,can1_0x200_TxData,&TxMailbox)!= HAL_OK)
//  {
//        Error_Handler();
//  }
//}

void CAN_Transmit_3(int16_t v)//控制拨盘电机，id�?1
{
  TxMessage.IDE=CAN_ID_STD;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.DLC=8;
  
  TxMessage.StdId=0x200;
  
  can1_0x200_TxData[0] = (v>>8);
  can1_0x200_TxData[1] = (v);
  can1_0x200_TxData[2] = 0;
  can1_0x200_TxData[3] = 0;
  can1_0x200_TxData[4] = 0;
  can1_0x200_TxData[5] = 0;
  can1_0x200_TxData[6] = 0;
  can1_0x200_TxData[7] = 0;
    
  if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,can1_0x200_TxData,&TxMailbox)!= HAL_OK)
  {
        Error_Handler();
  }
}

void CAN_Transmit_4(int16_t v)//控制pitch电机
{
  TxMessage2.IDE=CAN_ID_STD;
  TxMessage2.RTR=CAN_RTR_DATA;
  TxMessage2.DLC=8;
  
  TxMessage2.StdId=0x1FF;
  
    can1_0x1FF_TxData[0] = (v>>8);
    can1_0x1FF_TxData[1] = (v);
    can1_0x1FF_TxData[2] = (v>>8);
    can1_0x1FF_TxData[3] = (v);
    can1_0x1FF_TxData[4] = (v>>8);
    can1_0x1FF_TxData[5] = (v);
    can1_0x1FF_TxData[6] = (v>>8);
    can1_0x1FF_TxData[7] = (v);
    
  if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage2,can1_0x1FF_TxData,&TxMailbox)!= HAL_OK)
  {
        Error_Handler();
  }
}

void CAN_Transmit_5(int16_t v)//控制yaw电机
{
  TxMessage3.IDE=CAN_ID_STD;
  TxMessage3.RTR=CAN_RTR_DATA;
  TxMessage3.DLC=8;
  
  TxMessage3.StdId=0x2FF;
  
    can2_0x2FF_TxData[0] = (v>>8);
    can2_0x2FF_TxData[1] = (v);
    can2_0x2FF_TxData[2] = (v>>8);
    can2_0x2FF_TxData[3] = (v);
    can2_0x2FF_TxData[4] = (v>>8);
    can2_0x2FF_TxData[5] = (v);
    can2_0x2FF_TxData[6] = (v>>8);
    can2_0x2FF_TxData[7] = (v);
    
  if(HAL_CAN_AddTxMessage(&hcan2,&TxMessage3,can2_0x2FF_TxData,&TxMailbox)!= HAL_OK)
  {
        Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if(hcan->Instance==CAN1)
    {
        if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,aData)==HAL_OK)
        {
            switch(RxMessage.StdId)
            {
                case 0x201://拨盘电机
                    motor_info3.rotor_angle       = ((aData[0] << 8) | aData[1]);
                    motor_info3.rotor_speed       = ((aData[2] << 8) | aData[3]);
                    motor_info3.torque_current    = ((aData[4] << 8) | aData[5]);
                    motor_info3.motor_temperature =   aData[6];
                    update_angle(&motor_info3);
                    break;
                
                case 0x202://右摩擦轮
                    motor_info1.rotor_angle       = ((aData[0] << 8) | aData[1]);
                    motor_info1.rotor_speed       = ((aData[2] << 8) | aData[3]);
                    motor_info1.torque_current    = ((aData[4] << 8) | aData[5]);
                    motor_info1.motor_temperature =   aData[6];
                    break;

                case 0x203://左摩擦轮
                    motor_info2.rotor_angle       = ((aData[0] << 8) | aData[1]);
                    motor_info2.rotor_speed       = ((aData[2] << 8) | aData[3]);
                    motor_info2.torque_current    = ((aData[4] << 8) | aData[5]);
                    motor_info2.motor_temperature =   aData[6];
                    break;
                
                case 0x206://pitch电机
                    motor_info4.rotor_angle       = ((aData[0] << 8) | aData[1]);
                    motor_info4.rotor_speed       = ((aData[2] << 8) | aData[3]);
                    motor_info4.torque_current    = ((aData[4] << 8) | aData[5]);
                    motor_info4.motor_temperature =   aData[6];
                    break;
            }
//           HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // The FIFO0 receive interrupt function was enabled again
        }
    }
    else if(hcan->Instance==CAN2)
    {
        if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,aData)==HAL_OK)
        {   
            motor_info5.rotor_angle       = ((aData[0] << 8) | aData[1]);
            motor_info5.rotor_speed       = ((aData[2] << 8) | aData[3]);
            motor_info5.torque_current    = ((aData[4] << 8) | aData[5]);
            motor_info5.motor_temperature =   aData[6];
            update_angle(&motor_info5);
//            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }
    
}

void update_angle(Motor_Info* motor_info)//单环变多环，给绝对式编码器使�?
{
      if (motor_info->c_Init)
      {
          if(motor_info->rotor_angle - motor_info->last_ecd > 4096) 
              motor_info->round_cnt --;
          else if(motor_info->rotor_angle - motor_info->last_ecd < -4096)
              motor_info->round_cnt ++;
      }
      else
      {
          motor_info->c_offset = motor_info->rotor_angle;
          motor_info->c_Init = 1;
      }
        motor_info->last_ecd = motor_info->rotor_angle;
        motor_info->total_encoder = motor_info->round_cnt * 8192 + motor_info->rotor_angle - motor_info->c_offset;   
}

//void dial_angle(Motor_Info* motor_info)//拨盘电机角度和�?�度编码器�?�解�?
//{
//    if(motor_info->rotor_angle - motor_info->last_ecd > 4096) 
//      motor_info->round_cnt --;
//    else if(motor_info->rotor_angle - motor_info->last_ecd < -4096)
//      motor_info->round_cnt ++;

//    motor_info->last_ecd = motor_info->rotor_angle;
//    motor_info->total_encoder = motor_info->round_cnt * 8192 + motor_info->rotor_angle;   
//}

/* USER CODE END 1 */

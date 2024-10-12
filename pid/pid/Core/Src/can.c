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
uint8_t TxData[8];
uint32_t TxMailbox;

CAN_RxHeaderTypeDef RxMessage;
uint8_t  aData[8];

Motor_Info motor_info;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

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

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

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
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

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
    sFilterConfig.SlaveStartFilterBank = 0;//为从CAN实例选择启动筛选器组。对于单个CAN实例，此参数没有意义。对于双CAN实例，所有具有较低索引的过滤器组都被分配给主CAN实例，而所有具有较大索引的过滤器组都被分配给从CAN实例。该参数必须为Min_Data = O和Max_Data =27之间的一个数字. 
    //配置过滤器
    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig)!=HAL_OK)
    {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__); 
     } 
     //开启CAN
     if(HAL_CAN_Start(&hcan1)!=HAL_OK)
     {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);  
     }
     //当FIFO0中有消息的时候进入中断
     if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
     {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__); 
     }
}

void CAN_Transmit(int16_t v)
{
  TxMessage.IDE=CAN_ID_STD;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.DLC=8;
  
  TxMessage.StdId=0x1FF;//一个帧只能驱动四个电机
  
  TxData[0] = (v>>8);
  TxData[1] = (v);
  TxData[2] = (v>>8);
  TxData[3] = (v);
  TxData[4] = (v>>8);
  TxData[5] = (v);
  TxData[6] = (v>>8);
  TxData[7] = (v);
    
  if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData,&TxMailbox)!= HAL_OK)
  {
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    if(hcan->Instance==CAN1)
    {
        if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,aData)==HAL_OK)
        {
            /*输出接收报文的header*/
//            printf("IDE     : %s\r\n",RxMessage.IDE == CAN_ID_STD ? "CAN_ID_STD" : "CAN_ID_EXT");
//            printf("RTR     : %s\r\n",RxMessage.RTR == CAN_RTR_DATA ? "CAN_RTR_DATA" : "CAN_RTR_REMOTE");
//            printf("DLC     : %lu\r\n", RxMessage.DLC);
//            printf("StdId   : 0x%lx\r\n",RxMessage.StdId);

            motor_info.rotor_angle       = ((aData[0] << 8) | aData[1]);
            motor_info.rotor_speed       = ((aData[2] << 8) | aData[3]);
            motor_info.torque_current    = ((aData[4] << 8) | aData[5]);
            motor_info.motor_temperature =   aData[6];
            
            //串口输出:电机的转子机械电角度(0-8191对应0-360°)、转子转速(RPM)、实际转矩电流(A)
//            printf("rotor_angle          : %d\n",motor_info.rotor_angle);
//            printf("rotor_speed          : %d\n",motor_info.rotor_speed);
//            printf("torque_current       : %d\n",motor_info.torque_current);

//            switch(RxMessage.StdId)
//                case 0x204:
//                {
//                    break;
//                }

//            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // The FIFO0 receive interrupt function was enabled again
        }
    }
}
/* USER CODE END 1 */

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
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
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

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

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
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
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
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */



/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief 配置CAN1的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类�?
 * @param ID ID
 * @param Mask_ID 屏蔽�?(0x3ff, 0x1fffffff)
 */
void CAN1_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef can_filter_init_structure;

  // �?测关键传�?
  assert_param(hcan != NULL);

  if ((Object_Para & 0x02))
  {
    // 标准�?
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
  }
  else
  {
    // 扩展�?
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 5;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
  }

  // 滤波器序�?, 0-27, �?28个滤波器, can1�?0~13, can2�?14~27
  can_filter_init_structure.FilterBank = 0;
  // 滤波器绑定FIFOx, 只能绑定�?�?
  can_filter_init_structure.FilterFIFOAssignment = 1;
  // 使能滤波�?
  can_filter_init_structure.FilterActivation = ENABLE;
  // 滤波器模�?, 设置ID掩码模式
  can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
  // 32位滤�?
  can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择�?始单�?
  can_filter_init_structure.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 配置CAN1的滤波器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类�?
 * @param ID ID
 * @param Mask_ID 屏蔽�?(0x3ff, 0x1fffffff)
 */
void CAN2_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef can_filter_init_structure;

  // �?测关键传�?
  assert_param(hcan != NULL);

  if ((Object_Para & 0x02))
  {
    // 标准�?
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 3 >> 16;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 3 << 16;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
  }
  else
  {
    // 扩展�?
    // 掩码后ID的高16bit
    can_filter_init_structure.FilterIdHigh = ID << 5;
    // 掩码后ID的低16bit
    can_filter_init_structure.FilterIdLow = ((Object_Para & 0x03) << 1);
    // ID掩码值高16bit
    can_filter_init_structure.FilterMaskIdHigh = Mask_ID << 5;
    // ID掩码值低16bit
    can_filter_init_structure.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
  }

  // 滤波器序�?, 0-27, �?28个滤波器, can1�?0~13, can2�?14~27
  can_filter_init_structure.FilterBank = 14;
  // 滤波器绑定FIFOx, 只能绑定�?�?
  can_filter_init_structure.FilterFIFOAssignment = 1;
  // 使能滤波�?
  can_filter_init_structure.FilterActivation = ENABLE;
  // 滤波器模�?, 设置ID掩码模式
  can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
  // 32位滤�?
  can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    //从机模式选择�?始单�?
  can_filter_init_structure.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

/**
 * @brief 发�?�数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状�??
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
  CAN_TxHeaderTypeDef tx_header;
  uint32_t used_mailbox;

  // �?测关键传�?
  assert_param(hcan != NULL);

  tx_header.StdId = ID;
  tx_header.ExtId = 0;
  tx_header.IDE = 0;
  tx_header.RTR = 0;
  tx_header.DLC = Length;

  return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data;

  if(HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &header, &data)==HAL_OK)
  {
    if(hcan->Instance == CAN1)
    {
        uint8_t a = 6;
        HAL_UART_Transmit(&huart6, &a, 1, 10);
    }
     if(hcan->Instance == CAN2)
    {
        uint8_t b = 7;
        HAL_UART_Transmit(&huart6, &b, 1, 10);
    }
  }
}

/* USER CODE END 1 */

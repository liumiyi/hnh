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
#include "FreeRTOS.h"
#include "usart.h"
CAN_FilterTypeDef sFilterConfig;

CAN_TxHeaderTypeDef TxMessage;
CAN_RxHeaderTypeDef RxMessage;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;//Txé‚®ç®�?,å�¯å¡�? (uint32_t*)CAN_TX_MAILBOX0 ï¼ŒCAN_TX_MAILBOX1 ï¼? CAN_TX_MAILBOX3
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
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
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
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void Configure_Filter(void)
{
//    u1_printf("4444\r\n");
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;//æŠŠæŽ¥æ”¶åˆ°çš�?�æŠ¥æ�?��?�æ�?�¾å�?�¥åˆ°FIFO0
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;//ä¸ºä»ŽCANå®žä¾‹é€‰æ�?�©å�¯åŠ¨ç­›�??‰å™¨ç»„�??‚å¯¹äºŽå�•ä¸ªCANå®žä¾‹ï¼Œæ­¤å�‚æ�?�°æ²¡æœ�?�æ�?��ä¹‰ã€‚å¯¹äºŽå�ŒCANå®žä¾‹ï¼Œæ�?�€æœ‰å�?�·æœ�?�è¾ƒä½Žç´¢å¼�?�çš�?�è¿�?�æ»¤å™¨ç»„éƒ½è¢«åˆ�?�é�?��ç»™ä¸»CANå®žä¾‹ï¼Œ�??Œæ‰€æœ‰å�?�·æœ�?�è¾ƒå¤§ç´¢å¼�?�çš�?�è¿�?�æ»¤å™¨ç»„éƒ½è¢«åˆ�?�é�?��ç»™ä»ŽCANå®žä¾‹ã€‚è¯¥å�‚æ�?�°å¿�?�é¡»ä¸ºMin_Data = Oå’ŒMax_Data =27ä¹‹é�?�´çš�?�ä¸€ä¸ªæ•°å�??.

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig)!=HAL_OK)                     //creat CanFilter
    {
        //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);
    }
    if(HAL_CAN_Start(&hcan1)!=HAL_OK)                                            //initialize can
    {
        //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);
    }
    //å½“FIFO0ä¸­æœ‰æ¶ˆæ�¯çš„æ�?�¶å€™è¿�?�å�?�¥ä¸�?
    if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)//The FIFO0 receive interrupt function was enabled
    {
        //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);
    }
}

//Transmit data
void CAN_Transmit(int16_t v)
{
    //u1_printf("5555\r\n");
    TxMessage.StdId=0x1FF;//0x2FF or 0x1FF
    //TxMessage.ExtId=0x00001314;
    TxMessage.IDE=CAN_ID_STD;
    TxMessage.RTR=CAN_RTR_DATA;
    TxMessage.DLC=8;
    //TxMessage.TransmitGlobalTime = DISABLE;
    //data
    TxData[0] = (v>>8);//&0xff;
    TxData[1] = (v);//&0xff;
    TxData[2] = (v>>8);
    TxData[3] = (v);
    TxData[4] = (v>>8);
    TxData[5] = (v);
    TxData[6] = (v>>8);
    TxData[7] = (v);
    
    
//    uint8_t a = 1;
//    HAL_UART_Transmit(&huart6, &a, 1, 10);
    //start to transmit
    /*hcan
      TxMessage:specifies the data format to be transferred
      TxData:   specifies the data to be transferred
      TxMailbox:store the Tx message
    */
    if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData,&TxMailbox)!= HAL_OK)//if(HAL_CAN_AddTxMessage(&hcan,&TxMessage,TxData,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
    {
//        u1_printf("7777\r\n");
        //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
        Error_Handler();//_Error_Handler(__FILE__, __LINE__);
    }
}

//Receive data(need to open CAN RX0 interrupts)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
//    u1_printf("8888\r\n");
    if(hcan->Instance==CAN1)
    {
        /*
          hcan
          CAN_RX_FIFO0:  [ sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  ]
          RxMessage:
          aData:the payload of the Rx frame will be stored
        */
//        u1_printf("6666\r\n");
        if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMessage,aData)==HAL_OK)
        {
            //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
            /*è¾“å�?�ºæŽ¥æ�?�¶æŠ¥æ�?��?�çš�?�header*/
//            u1_printf("IDE     : %s\r\n",RxMessage.IDE == CAN_ID_STD ? "CAN_ID_STD" : "CAN_ID_EXT");
//            u1_printf("RTR     : %s\r\n",RxMessage.RTR == CAN_RTR_DATA ? "CAN_RTR_DATA" : "CAN_RTR_REMOTE");
//            u1_printf("DLC     : %lu\r\n", RxMessage.DLC);
//            u1_printf("StdId   : 0x%lx\r\n",RxMessage.StdId);
//            u1_printf("ExtId   : 0x%lx\r\n", RxMessage.ExtId);
//            u1_printf("Data0    : %d\r\n", aData[0]);
//            u1_printf("Data1    : %d\r\n", aData[1]);
//            u1_printf("Data2    : %d\r\n", aData[2]);
//            u1_printf("Data3    : %d\r\n", aData[3]);
//            u1_printf("Data4    : %d\r\n", aData[4]);


            motor_info.rotor_angle       = ((aData[0] << 8) | aData[1]);
            motor_info.rotor_speed       = ((aData[2] << 8) | aData[3]);
            motor_info.torque_current    = ((aData[4] << 8) | aData[5]);
            motor_info.motor_temperature =   aData[6];
            /*ä¸²å�£è¾�?�å�?��?:ç”µæœºçš�?�è½¬å­�æœºæ¢°ç”µè§�?�åº�?(0-8191å¯¹åº�?0-360Â°)ã€�è½¬å­�è½¬é€?(RPM)ã€�å®žé™�?�è½¬çŸ©ç�?�µæ�??(A)ã€�ç”µæœºæ¸©å�??(Â°C)*/

            u1_printf("rotor_angle        ： %hu\r\n",motor_info.rotor_angle);
            vTaskDelay(1);
            u1_printf("rotor_speed       : %hu\r\n",motor_info.rotor_speed);
            vTaskDelay(1);
            u1_printf("torque_current    : %hu\r\n",motor_info.torque_current);
            vTaskDelay(1);
            u1_printf("motor_temperature : %hu\r\n",motor_info.motor_temperature);
            vTaskDelay(1);
            u1_printf("\n");
            vTaskDelay(1);
//            switch(RxMessage.StdId)
//                case 0x203:
//                {
//                    break;
//                }


            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // The FIFO0 receive interrupt function was enabled again
        }
    }
}



/* USER CODE END 1 */

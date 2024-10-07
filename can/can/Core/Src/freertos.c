/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "usart.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */



/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask01 */
  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelete(NULL);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask01 */
/**
* @brief Function implementing the myTask01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN StartTask01 */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
    xLastWakeTime = xTaskGetTickCount();
    
    TxMessage1.StdId = 0x201;
    TxMessage1.ExtId = 0x00;
    TxMessage1.RTR = CAN_RTR_DATA;
    TxMessage1.IDE = CAN_ID_STD;
    TxMessage1.DLC = 8;
    TxMessage1.TransmitGlobalTime = DISABLE;
    
    TxMessage2.StdId = 0x202;
    TxMessage2.ExtId = 0x00;
    TxMessage2.RTR = CAN_RTR_DATA;
    TxMessage2.IDE = CAN_ID_STD;
    TxMessage2.DLC = 8;
    TxMessage2.TransmitGlobalTime = DISABLE;
    
    uint32_t pTxMailbox = 0;

    uint8_t adata[8] = {0};
    
    BaseType_t xReturn;
    uint16_t temp;
    uint16_t flag = 0;
    
    uint8_t a = 6;
    uint8_t c = 9;
    
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);	
    
    uint8_t count=0;
    
  /* Infinite loop */
  for(;;)
  {
      xReturn=xQueueReceive(myQueue01Handle,&temp,1);
      #if 0
    if(temp == 1)
    {
        HAL_CAN_AddTxMessage(&hcan1, &TxMessage1, adata, &pTxMailbox);
    }
    else if(temp == 2)
    {
        HAL_CAN_AddTxMessage(&hcan2, &TxMessage2, adata, &pTxMailbox);
    }
    
//    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage1, adata);
//    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxMessage2, adata);
    
    
    if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxMessage1, adata) == HAL_OK )
    {
        HAL_UART_Transmit(&huart1, &a, 1, 0xffff);
    }
    
    
    else if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage2, adata) == HAL_OK )
    {
        HAL_UART_Transmit(&huart1, &c, 1, 0xffff);
    }
    #else
    for(;count<2;count++)
    {
        HAL_CAN_AddTxMessage(&hcan1, &TxMessage1, adata, &pTxMailbox);
    }
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage1, adata);
    uint32_t msgCount = HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0);
    osDelay(1);
    if(RxMessage1.StdId == 0x201)
    {
        osDelay(1);
    }
    
    #endif
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  /* USER CODE END StartTask01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


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
#include "MPU6050.h"
#include "OLED.h"
#include "queue.h"
#include "tim.h"
#include "usart.h"

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
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
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
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);

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
  myTask01Handle = osThreadNew(StartTask02, NULL, &myTask01_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask03, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask04, NULL, &myTask03_attributes);

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

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
     TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
      xLastWakeTime = xTaskGetTickCount();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buff, BUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);	

    
    /* Infinite loop */
  for(;;)
  {
    
     vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
      xLastWakeTime = xTaskGetTickCount();
    BaseType_t xReturn;
    uint16_t temp;
  /* Infinite loop */
  for(;;)
  {
      xReturn=xQueueReceive(myQueue01Handle,&temp,1);
      if( temp == 1)
    {
          for(int i = 0;i < 100;i++)
        {
            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,i);
            vTaskDelay(1);
        }
        for(int i = 0;i < 100;i++)
        {
            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,100-i);
            vTaskDelay(1);
        }
    }
    else if(temp == 2)
    {
          for(int i = 0;i < 100;i++)
        {
            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,i);
            vTaskDelay(10);
        }
        for(int i = 0;i < 100;i++)
        {
            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,100-i);
            vTaskDelay(10);
        }
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;
      xLastWakeTime = xTaskGetTickCount();
    int res=1;
    float pitch;
    float roll;
    float yaw;
    int16_t gx,gy,gz,ax,ay,az;
    while(res)
    {
        res = MPU6050_DMP_Init();
        OLED_ShowString(1, 1, " ");
        OLED_ShowString(1, 1, "InitNG:");
        OLED_ShowSignedNum(1, 8,res,1);
        vTaskDelay(1000);
    }
    OLED_ShowString(1, 1, " ");
    OLED_ShowString(1, 1, "InitOK");
  /* Infinite loop */
  for(;;)
  {
      if(MPU6050_DMP_Get_Data(&pitch, &roll, &yaw,&gx,&gy,&gz,&ax,&ay,&az))
    {
        OLED_ShowSignedNum(2, 1,ax, 5);
        OLED_ShowSignedNum(3, 1, ay, 5);
        OLED_ShowSignedNum(4, 1, az, 5);
        OLED_ShowSignedNum(2, 8, gx, 5);
        OLED_ShowSignedNum(3, 8, gy, 5);
        OLED_ShowSignedNum(4, 8, gz, 5);
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


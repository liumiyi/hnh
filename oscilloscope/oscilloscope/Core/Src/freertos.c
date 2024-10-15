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
#include "can.h"
#include "pid.h"
#include "usart.h"
#include <stdio.h>
#include "tim.h"

pid_struct_t motor_pid_Single;//定义单个pid结构�?
float speedTarget = 300;//速度目标位置（单级）(0-380rpm左右)

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
/* Definitions for task01 */
osThreadId_t task01Handle;
const osThreadAttr_t task01_attributes = {
  .name = "task01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task02 */
osThreadId_t task02Handle;
const osThreadAttr_t task02_attributes = {
  .name = "task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task03 */
osThreadId_t task03Handle;
const osThreadAttr_t task03_attributes = {
  .name = "task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void CAN001(void *argument);
void USART002(void *argument);
void PWM003(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of task01 */
  task01Handle = osThreadNew(CAN001, NULL, &task01_attributes);

  /* creation of task02 */
  task02Handle = osThreadNew(USART002, NULL, &task02_attributes);

  /* creation of task03 */
  task03Handle = osThreadNew(PWM003, NULL, &task03_attributes);

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

/* USER CODE BEGIN Header_CAN001 */
/**
* @brief Function implementing the task01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN001 */
void CAN001(void *argument)
{
  /* USER CODE BEGIN CAN001 */
//    Configure_Filter();
//    pid_init(&motor_pid_Single,40,3,0,500,25000);
  /* Infinite loop */
  for(;;)
  {
    CAN_Transmit(25000);
//    int16_t set_voltage = pid_calc(&motor_pid_Single, speedTarget, motor_info.rotor_speed);
//    CAN_Transmit(set_voltage);
//    printf("speed: %d,%f\n",motor_info.rotor_speed,speedTarget);
    vTaskDelay(1);
  }
  /* USER CODE END CAN001 */
}

/* USER CODE BEGIN Header_USART002 */
/**
* @brief Function implementing the task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USART002 */
void USART002(void *argument)
{
  /* USER CODE BEGIN USART002 */
  /* Infinite loop */
  for(;;)
  {
    printf("LMY\n");
    vTaskDelay(1);
  }
  /* USER CODE END USART002 */
}

/* USER CODE BEGIN Header_PWM003 */
/**
* @brief Function implementing the task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PWM003 */
void PWM003(void *argument)
{
  /* USER CODE BEGIN PWM003 */
   HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  /* Infinite loop */
  for(;;)
  {
      for(int i = 0;i < 100;i++)
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,i);
        vTaskDelay(10);
    }
    for(int i = 0;i < 100;i++)
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,100-i);
        vTaskDelay(10);
    }
    vTaskDelay(1);
  }
  /* USER CODE END PWM003 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


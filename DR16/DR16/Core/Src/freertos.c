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

pid_struct_t motor_pid_Single;//定义单个pid结构使
float speedTarget = 0;//速度目标位置（单级）(0-380rpm左右)
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
/* Definitions for DR16 */
osThreadId_t DR16Handle;
const osThreadAttr_t DR16_attributes = {
  .name = "DR16",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DR16motor */
osThreadId_t DR16motorHandle;
const osThreadAttr_t DR16motor_attributes = {
  .name = "DR16motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void tDR16(void *argument);
void tDR16motor(void *argument);

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

  /* creation of DR16 */
  DR16Handle = osThreadNew(tDR16, NULL, &DR16_attributes);

  /* creation of DR16motor */
  DR16motorHandle = osThreadNew(tDR16motor, NULL, &DR16motor_attributes);

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

/* USER CODE BEGIN Header_tDR16 */
/**
* @brief Function implementing the DR16 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tDR16 */
void tDR16(void *argument)
{
  /* USER CODE BEGIN tDR16 */
    Configure_Filter();
    pid_init(&motor_pid_Single,40,3,0,25000,25000);
  /* Infinite loop */
  for(;;)
  {
    int16_t set_voltage = pid_calc(&motor_pid_Single, speedTarget, motor_info.rotor_speed);
    CAN_Transmit(set_voltage);
    if(rc.ch0 >= 600||rc.ch1 >= 600)
    {
        speedTarget++;
    }
    else if(rc.ch0 <= -600||rc.ch1 <= -600)
    {
        speedTarget--;
    }
    vTaskDelay(1);
  }
  /* USER CODE END tDR16 */
}

/* USER CODE BEGIN Header_tDR16motor */
/**
* @brief Function implementing the DR16motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tDR16motor */
void tDR16motor(void *argument)
{
  /* USER CODE BEGIN tDR16motor */
    
  /* Infinite loop */
  for(;;)
  {
    
    vTaskDelete(NULL);
  }
  /* USER CODE END tDR16motor */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


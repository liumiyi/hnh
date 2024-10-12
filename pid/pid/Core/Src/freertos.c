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

//速度环（单级）
pid_struct_t motor_pid_Single;//定义单个pid结构体
float speedTarget = 300;//速度目标位置（单级）(0-320rpm)

//角度环（串级）
pid_Cascade_t motor_pid_Cas;//定义串级pid结构体
float angleTarget = 180;//角度目标位置（串级）


#define ANGLE//SPEED or ANGLE or OPEN

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
    Configure_Filter();
    
    #ifdef SPEED
    pid_init(&motor_pid_Single,15,3,0,25000,25000);
    #endif
    
    #ifdef ANGLE
    pid_init(&motor_pid_Cas.inner, 40, 3, 0, 25000, 320);
    pid_init(&motor_pid_Cas.outer, 60, 0, 0, 10000, 320);
    #endif
    
  /* Infinite loop */
  for(;;)
  {
#ifdef OPEN //开环控制
      CAN_Transmit(25000);//[-25000,25000]
#endif
      
#ifdef SPEED//速度单闭环控制
      /*open loop control*/
      taskENTER_CRITICAL();//进入临界区
      int16_t set_voltage = pid_calc(&motor_pid_Single, speedTarget, motor_info.rotor_speed);
      CAN_Transmit(set_voltage);
      taskEXIT_CRITICAL();
      printf("speed: %d,%f\n",motor_info.rotor_speed,speedTarget);
#endif
      
#ifdef ANGLE//角度双闭环控制
    /*closed loop control*/
      taskENTER_CRITICAL();//进入临界区
      double Rotor_angle = (double)(motor_info.rotor_angle *360.0)/8192.0;//将角度规范到0~360
      int16_t set_voltage = pid_CascadeCalc(&motor_pid_Cas,angleTarget,Rotor_angle,motor_info.rotor_speed);
      CAN_Transmit(set_voltage);
      taskEXIT_CRITICAL();
      printf("angle: %f,%f\n",Rotor_angle,angleTarget);
#endif

//    printf("angle: %f,%f\n",Rotor_angle,angleTarget);
//    printf("rotor_speed        : %d\n\n",motor_info.rotor_speed);
//    printf("torque_current     : %d\n\n\n",motor_info.torque_current);
      
    vTaskDelay(5);
  }
  /* USER CODE END StartTask01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


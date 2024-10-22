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

pid_struct_t motor_pid_Single;//定义摩擦轮pid结构体
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
/* Definitions for friction */
osThreadId_t frictionHandle;
const osThreadAttr_t friction_attributes = {
  .name = "friction",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Dial */
osThreadId_t DialHandle;
const osThreadAttr_t Dial_attributes = {
  .name = "Dial",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void vfriction(void *argument);
void vDial(void *argument);

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

  /* creation of friction */
  frictionHandle = osThreadNew(vfriction, NULL, &friction_attributes);

  /* creation of Dial */
  DialHandle = osThreadNew(vDial, NULL, &Dial_attributes);

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
  InitializeRemoteControl(&g_RemoteControlState); //初始化遥控器状�?�并创建对应定时�?
  /* Infinite loop */
  for(;;)
  {
    vTaskDelete(NULL);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_vfriction */
/**
* @brief Function implementing the friction thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vfriction */
void vfriction(void *argument)//控制右摩擦轮和左摩擦轮
{
  /* USER CODE BEGIN vfriction */
    Configure_Filter();
    pid_init(&motor_pid_Single,40,3,0,500,25000);
  /* Infinite loop */
  for(;;)
  {
      int16_t set_voltage1 = pid_calc(&motor_pid_Single, speedTarget, motor_info1.rotor_speed);//右摩擦轮
      int16_t set_voltage2 = pid_calc(&motor_pid_Single, speedTarget, motor_info2.rotor_speed);//左摩擦轮
      
      if(g_RemoteControlState.isRemoteActive==false)
    {
        CAN_Transmit_1(0);
        CAN_Transmit_2(0);
    }
    else
    {
        CAN_Transmit_1(set_voltage1);
        CAN_Transmit_2(set_voltage2);
    }
    
    if(rc.ch0>100)
    {
        speedTarget = 6000;
    }
    else
    {
        speedTarget = 0;
    }
    
    vTaskDelay(1);
  }
  /* USER CODE END vfriction */
}

/* USER CODE BEGIN Header_vDial */
/**
* @brief Function implementing the Dial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vDial */
void vDial(void *argument)
{
  /* USER CODE BEGIN vDial */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(1);
  }
  /* USER CODE END vDial */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


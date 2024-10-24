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
float speedTarget = 0;//摩擦轮使用

pid_Cascade_t motor_pid_Cas;//定义串级pid结构体用于拨盘电机
float angleTarget = 0;//拨盘电机使用

float Rotor_angle;//拨盘电机多圈角度

uint8_t singleflag = 0;//拨盘电机单发标志

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
  Configure_Filter();
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
//    Configure_Filter();//在默认任务中配置过滤器
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
    pid_init(&motor_pid_Cas.inner, 20, 3, 0, 500, 16384);
    pid_init(&motor_pid_Cas.outer, 30, 1, 0, 500, 20000);
    motor_info3.c_Init = 0;
    motor_info3.round_cnt = 0;
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;//使拨盘单机的频率为20Hz，即1发等50ms
    xLastWakeTime = xTaskGetTickCount();
      
  /* Infinite loop */
  for(;;)
  {
    Rotor_angle = (motor_info3.total_encoder *360)/8192/36;//将编码器刻度换成角度
    int16_t set_current = pid_CascadeCalc(&motor_pid_Cas,angleTarget,Rotor_angle,motor_info3.rotor_speed);
    if(g_RemoteControlState.isRemoteActive==false)
    {
        CAN_Transmit_3(0);
    }
    else
    {
        CAN_Transmit_3(set_current);
    }
    
    //连发
    if(rc.sw2 == 1)
    {
        if(rc.ch1 >= 400)
        {
            angleTarget = angleTarget+40;
            vTaskDelayUntil( &xLastWakeTime, xFrequency );
        }
        else
        {
            angleTarget = angleTarget;
        }
    }
    //单发
    else if(rc.sw2 == 2)
    {
        if(rc.sw1 == 2)
        singleflag = 0;

        if(rc.sw1 == 1 && singleflag == 0)
        {
            angleTarget = angleTarget+40;
            singleflag = 1;//将单发标志位罿1，使其无法发尿
        }
    }
    vTaskDelay(1);
  }
  /* USER CODE END vDial */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


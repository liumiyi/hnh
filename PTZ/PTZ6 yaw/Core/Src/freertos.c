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

pid_struct_t motor_pid_Single;//定义摩擦轮pid结构�?
float speedTarget = 0;//摩擦轮使�?

pid_Cascade_t motor_pid_Cas;//定义串级pid结构体用于拨盘电�?
float angleTarget = 0;//拨盘电机使用

float Rotor_angle;//拨盘电机多圈角度

uint8_t singleflag = 0;//拨盘电机单发标志

pid_Cascade_t motor_pid_Cas2;//定义串级pid结构体用于pitch电机
float angleTarget2 = 60;//pitch电机使用  57.1~107.0(全向轮)
float Rotor_angle2;//pitch电机当前角度
uint8_t pitchflag = 0;

pid_Cascade_t motor_pid_Cas3;//定义串级pid结构体用于yaw电机
float angleTarget3 = 0;//yaw电机使用
float Rotor_angle3;//yaw电机当前角度

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
/* Definitions for pitch */
osThreadId_t pitchHandle;
const osThreadAttr_t pitch_attributes = {
  .name = "pitch",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for yaw */
osThreadId_t yawHandle;
const osThreadAttr_t yaw_attributes = {
  .name = "yaw",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void vfriction(void *argument);
void vDial(void *argument);
void vpitch(void *argument);
void vyaw(void *argument);

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

  /* creation of pitch */
  pitchHandle = osThreadNew(vpitch, NULL, &pitch_attributes);

  /* creation of yaw */
  yawHandle = osThreadNew(vyaw, NULL, &yaw_attributes);

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
  Configure_Filter2();
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
void vfriction(void *argument)
{
  /* USER CODE BEGIN vfriction */
//    Configure_Filter();//在默认任务中配置过滤�?
    pid_init(&motor_pid_Single,40,3,0,500,25000);
  /* Infinite loop */
  for(;;)
  {
      int16_t set_voltage1 = pid_calc(&motor_pid_Single, speedTarget, motor_info1.rotor_speed);//右摩擦轮
//      int16_t set_voltage2 = pid_calc(&motor_pid_Single, speedTarget, motor_info2.rotor_speed);//左摩擦轮
      
      if(g_RemoteControlState.isRemoteActive==false)
    {
        CAN_Transmit_1(0);
    }
    else
    {
        CAN_Transmit_1(set_voltage1);
    }
    
    if(rc.ch0>100)
    {
        speedTarget = 6000;
    }
    else
    {
        speedTarget = 0;
    }
    
    vTaskDelay(5);
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
    const TickType_t xFrequency = 50;//使拨盘单机的频率�?20Hz，即1发等50ms
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
            singleflag = 1;//将单发标志位�?1，使其无法发�?
        }
    }
    vTaskDelay(5);
  }
  /* USER CODE END vDial */
}

/* USER CODE BEGIN Header_vpitch */
/**
* @brief Function implementing the pitch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vpitch */
void vpitch(void *argument)
{
  /* USER CODE BEGIN vpitch */
    pid_init(&motor_pid_Cas2.inner, 40, 0, 0, 500, 25000);
    pid_init(&motor_pid_Cas2.outer, 10, 3, 0, 500, 10000);
    motor_info4.c_Init = 0;
    motor_info4.round_cnt = 0;
  /* Infinite loop */
  for(;;)
  {
      Rotor_angle2 = (motor_info4.rotor_angle *360)/8192;
        int16_t set_voltage3 = pid_CascadeCalc(&motor_pid_Cas2,angleTarget2,Rotor_angle2,motor_info4.rotor_speed);

        if(g_RemoteControlState.isRemoteActive==false)
        {
            CAN_Transmit_4(0);
        }
        else
        {
//            CAN_Transmit_4(set_voltage3);
        }

//        if((rc.ch3 >= 400) && (motor_info4.rotor_angle <= 1198))//舵轮
//        {
//            angleTarget2 = angleTarget2+30;
//            vTaskDelay(3000);
//        }
//        else if((rc.ch3 <= -400) && (motor_info4.rotor_angle >= 87))
//        {
//            angleTarget2 = angleTarget2-30;
//            vTaskDelay(3000);
//        }
        
        if((rc.ch3 >= 400) && (motor_info4.rotor_angle <= 2412))//全向轮
        {
            angleTarget2 = angleTarget2+30;
            vTaskDelay(3000);
        }
        else if((rc.ch3 <= -400) && (motor_info4.rotor_angle >= 1272))
        {
            angleTarget2 = angleTarget2-30;
            vTaskDelay(3000);
        }

    vTaskDelay(1);
  }
  /* USER CODE END vpitch */
}

/* USER CODE BEGIN Header_vyaw */
/**
* @brief Function implementing the yaw thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vyaw */
void vyaw(void *argument)
{
  /* USER CODE BEGIN vyaw */
    pid_init(&motor_pid_Cas3.inner, 40, 0, 0, 500, 25000);
    pid_init(&motor_pid_Cas3.outer, 10, 0, 0, 500, 10000);
    motor_info5.c_Init = 0;
    motor_info5.round_cnt = 0;
    
  /* Infinite loop */
  for(;;)
  {
        Rotor_angle3 = (motor_info5.rotor_angle *360)/8192;
        int16_t set_voltage4 = pid_CascadeCalc(&motor_pid_Cas3,angleTarget3,Rotor_angle3,motor_info5.rotor_speed);

        if(g_RemoteControlState.isRemoteActive==false)
        {
            CAN_Transmit_5(0);
        }
        else
        {
            CAN_Transmit_5(set_voltage4);
        }
        
        if(rc.ch2 >= 400)
        {
            angleTarget3 = angleTarget3+30;
            vTaskDelay(3000);
        }
        else if(rc.ch2 <= -400)
        {
            angleTarget3 = angleTarget3-30;
            vTaskDelay(3000);
        }
        
    vTaskDelay(1);
  }
  /* USER CODE END vyaw */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


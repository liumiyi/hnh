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
#include <stdio.h>
#include "usart.h"
#include "tim.h"

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
/* Definitions for CAN01 */
osThreadId_t CAN01Handle;
const osThreadAttr_t CAN01_attributes = {
  .name = "CAN01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for USART02 */
osThreadId_t USART02Handle;
const osThreadAttr_t USART02_attributes = {
  .name = "USART02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PWM03 */
osThreadId_t PWM03Handle;
const osThreadAttr_t PWM03_attributes = {
  .name = "PWM03",
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

  /* creation of CAN01 */
  CAN01Handle = osThreadNew(CAN001, NULL, &CAN01_attributes);

  /* creation of USART02 */
  USART02Handle = osThreadNew(USART002, NULL, &USART02_attributes);

  /* creation of PWM03 */
  PWM03Handle = osThreadNew(PWM003, NULL, &PWM03_attributes);

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
* @brief Function implementing the CAN01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN001 */
void CAN001(void *argument)
{
  /* USER CODE BEGIN CAN001 */
  /* Infinite loop */
  for(;;)
  {
    CAN_Transmit(25000);
    vTaskDelay(1);
  }
  /* USER CODE END CAN001 */
}

/* USER CODE BEGIN Header_USART002 */
/**
* @brief Function implementing the USART02 thread.
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
* @brief Function implementing the PWM03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PWM003 */
void PWM003(void *argument)
{
  /* USER CODE BEGIN PWM003 */
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


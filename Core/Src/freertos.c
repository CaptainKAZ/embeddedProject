/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
uint32_t chassisTaskBuffer[ 128 ];
osStaticThreadDef_t chassisTaskControlBlock;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .cb_mem = &chassisTaskControlBlock,
  .cb_size = sizeof(chassisTaskControlBlock),
  .stack_mem = &chassisTaskBuffer[0],
  .stack_size = sizeof(chassisTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for yawTask */
osThreadId_t yawTaskHandle;
uint32_t ahrsTaskBuffer[ 512 ];
osStaticThreadDef_t ahrsTaskControlBlock;
const osThreadAttr_t yawTask_attributes = {
  .name = "yawTask",
  .cb_mem = &ahrsTaskControlBlock,
  .cb_size = sizeof(ahrsTaskControlBlock),
  .stack_mem = &ahrsTaskBuffer[0],
  .stack_size = sizeof(ahrsTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for feedbackTask */
osThreadId_t feedbackTaskHandle;
uint32_t feedbackTaskBuffer[ 128 ];
osStaticThreadDef_t feedbackTaskControlBlock;
const osThreadAttr_t feedbackTask_attributes = {
  .name = "feedbackTask",
  .cb_mem = &feedbackTaskControlBlock,
  .cb_size = sizeof(feedbackTaskControlBlock),
  .stack_mem = &feedbackTaskBuffer[0],
  .stack_size = sizeof(feedbackTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tofTask */
osThreadId_t tofTaskHandle;
uint32_t tofTaskBuffer[ 512 ];
osStaticThreadDef_t tofTaskControlBlock;
const osThreadAttr_t tofTask_attributes = {
  .name = "tofTask",
  .cb_mem = &tofTaskControlBlock,
  .cb_size = sizeof(tofTaskControlBlock),
  .stack_mem = &tofTaskBuffer[0],
  .stack_size = sizeof(tofTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void chassis_task(void *argument);
void yaw_task(void *argument);
void feedback_task(void *argument);
void tof_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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
  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(chassis_task, NULL, &chassisTask_attributes);

  /* creation of yawTask */
  yawTaskHandle = osThreadNew(yaw_task, NULL, &yawTask_attributes);

  /* creation of feedbackTask */
  feedbackTaskHandle = osThreadNew(feedback_task, NULL, &feedbackTask_attributes);

  /* creation of tofTask */
  tofTaskHandle = osThreadNew(tof_task, NULL, &tofTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_chassis_task */
/**
  * @brief  Function implementing the chassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_yaw_task */
/**
* @brief Function implementing the yawTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_yaw_task */
__weak void yaw_task(void *argument)
{
  /* USER CODE BEGIN yaw_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END yaw_task */
}

/* USER CODE BEGIN Header_feedback_task */
/**
* @brief Function implementing the feedbackTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_feedback_task */
__weak void feedback_task(void *argument)
{
  /* USER CODE BEGIN feedback_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END feedback_task */
}

/* USER CODE BEGIN Header_tof_task */
/**
* @brief Function implementing the tofTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tof_task */
__weak void tof_task(void *argument)
{
  /* USER CODE BEGIN tof_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END tof_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


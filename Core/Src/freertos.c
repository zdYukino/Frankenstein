/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "vofa.h"
#include "BMI088driver.h"
#include "Attitude.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float gyro[3], accel[3], attitute[3], temp;
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
osThreadId defaultTaskHandle;
osThreadId vofaTaskHandle;
osThreadId mcuTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartVofaTask(void const * argument);
void StartMcuTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of vofaTask */
  osThreadDef(vofaTask, StartVofaTask, osPriorityIdle, 0, 128);
  vofaTaskHandle = osThreadCreate(osThread(vofaTask), NULL);

  /* definition and creation of mcuTask */
  osThreadDef(mcuTask, StartMcuTask, osPriorityIdle, 0, 128);
  mcuTaskHandle = osThreadCreate(osThread(mcuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
      HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);

      osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartVofaTask */
/**
* @brief Function implementing the vofaTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVofaTask */
void StartVofaTask(void const * argument)
{
  /* USER CODE BEGIN StartVofaTask */
  /* Infinite loop */
  for(;;)
  {
      tempFloat[0] = gyro[0];
      tempFloat[1] = gyro[1];
      tempFloat[2] = gyro[2];
      tempFloat[3] = accel[0];
      tempFloat[4] = accel[1];
      tempFloat[5] = accel[2];
      tempFloat[6] = attitute[0];
      tempFloat[7] = attitute[1];
      tempFloat[8] = attitute[2];
      tempFloat[9] = temp;
      Vofa_Uart_Transmit(&huart4,10);
      osDelay(2);
  }
  /* USER CODE END StartVofaTask */
}

/* USER CODE BEGIN Header_StartMcuTask */
/**
* @brief Function implementing the mcuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMcuTask */
void StartMcuTask(void const * argument)
{
  /* USER CODE BEGIN StartMcuTask */
    while(BMI088_init())
    {
        BMI088_read(gyro, accel, &temp);
    }
    Attitude_Init(500, &accel[3]);
  /* Infinite loop */
  for(;;)
  {
      BMI088_read(gyro, accel, &temp);
      Attitude_Calculate(gyro, accel, attitute);
      osDelay(2);
  }
  /* USER CODE END StartMcuTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
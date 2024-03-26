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
#include "music.h"
#include "bsp_delay.h"
#include "CAN_receive_dm.h"
#include "can.h"
#include "lqr_wbr.h"
#include "vofa_setting.h"
#include "usart.h"
#include "ddt_m6_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t tep[10];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId initTaskHandle;
osThreadId imuTaskHandle;
osThreadId vofaTaskHandle;
osThreadId lqrTaskHandle;

uint8_t board_init_flag = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void InitBoardTask(void const * argument);
void StartImuTask(void const * argument);
void VofaOutputTask(void const * argument);
void LqrControlTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(initTask, InitBoardTask, osPriorityIdle, 0, 128);                 //INIT task start
  initTaskHandle = osThreadCreate(osThread(initTask), NULL);

  osThreadDef(vofaTask, VofaOutputTask, osPriorityLow, 0, 128);                 //VOFA task start
  vofaTaskHandle = osThreadCreate(osThread(vofaTask), NULL);

  osThreadDef(imuTask, StartImuTask, osPriorityNormal, 0, 128);                 //IMU task start
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  osThreadDef(lqrTask, LqrControlTask, osPriorityHigh, 0, 512);               //CONTROL task start
  lqrTaskHandle = osThreadCreate(osThread(lqrTask), NULL);
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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
      osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
* @brief Function implementing the InitBoardTask thread.
* @param argument: Not used
* @retval None
*/
/**
  * @brief Function 初始化
  * @param argument: Not used
  * @retval none
  */
void InitBoardTask(void const * argument)
{
    delay_init();                                                //延时函数初始化
    CAN_Filter_Init(2);                              //CAN初始化

    for(uint8_t i=0;i<4;i++)
    {
        while (DM_Motor_measure[i].T_coil == 0)
        {
            start_motor(&hcan1,i+1);
            osDelay(2);
        }
        osDelay(2);
    }
    board_init_flag = 1;                                         //初始化完成
    init_music();                                                //一阵强劲的音乐响起
    /* Infinite loop */
    for(;;)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        if(VofaData[1] == 1)
        {
            MIT_motor_CTRL(&hcan1,1, 0, 0, 0, 0, -lqr_data_L.Tj1);//lqr_data_L.Tj1
            osDelay(1);
            MIT_motor_CTRL(&hcan1,2, 0, 0, 0, 0, -lqr_data_L.Tj2);//lqr_data_L.Tj2
            osDelay(1);
            DDT_motor_toq_CTRL(&huart2, 0x01,  -lqr_data_L.T*0.01f);

            MIT_motor_CTRL(&hcan1,3, 0, 0, 0, 0,  lqr_data_R.Tj2);
            osDelay(1);
            MIT_motor_CTRL(&hcan1,4, 0, 0, 0, 0,  lqr_data_R.Tj1);
            osDelay(1);
            DDT_motor_toq_CTRL(&huart2, 0x02, lqr_data_R.T*0.01f);
        }
        else
        {
            MIT_motor_CTRL(&hcan1,1, 0, 0, 0, 0, 0);
            osDelay(1);
            MIT_motor_CTRL(&hcan1,2, 0, 0, 0, 0, 0);
            osDelay(1);
            DDT_motor_toq_CTRL(&huart2, 0x01, 0);

            MIT_motor_CTRL(&hcan1,3, 0, 0, 0, 0, 0);
            osDelay(1);
            MIT_motor_CTRL(&hcan1,4, 0, 0, 0, 0, 0);
            osDelay(1);
            DDT_motor_toq_CTRL(&huart2, 0x02, 0);
        }
    }
}
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE END Application */

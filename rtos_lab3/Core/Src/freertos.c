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
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_RANDS 1000
#define NUMBER_OF_TASKS 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
means_s allMeans;
int counter = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId taskAHandle;
osThreadId taskBHandle;
osThreadId taskCHandle;
osThreadId taskUARTHandle;
osMutexId meanMutexHandle;
osSemaphoreId meanSemaphoreHandle;
osSemaphoreId UARTSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void randomizeAndCalculate(float* randsArray, float* sum, float* squareSum);
void acquireSemaphoreOrMutex();
void releaseSemaphoreOrMutex();
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTaskA(void const * argument);
void StartTaskB(void const * argument);
void StartTaskC(void const * argument);
void StartTaskUART(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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
  /* Create the mutex(es) */
  /* definition and creation of meanMutex */
  osMutexDef(meanMutex);
  meanMutexHandle = osMutexCreate(osMutex(meanMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of meanSemaphore */
  osSemaphoreDef(meanSemaphore);
  meanSemaphoreHandle = osSemaphoreCreate(osSemaphore(meanSemaphore), 1);

  /* definition and creation of UARTSemaphore */
  osSemaphoreDef(UARTSemaphore);
  UARTSemaphoreHandle = osSemaphoreCreate(osSemaphore(UARTSemaphore), 3);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of taskA */
  osThreadDef(taskA, StartTaskA, osPriorityBelowNormal, 0, 1280);
  taskAHandle = osThreadCreate(osThread(taskA), NULL);

  /* definition and creation of taskB */
  osThreadDef(taskB, StartTaskB, osPriorityNormal, 0, 1280);
  taskBHandle = osThreadCreate(osThread(taskB), NULL);

  /* definition and creation of taskC */
  osThreadDef(taskC, StartTaskC, osPriorityHigh, 0, 1280);
  taskCHandle = osThreadCreate(osThread(taskC), NULL);

  /* definition and creation of taskUART */
  osThreadDef(taskUART, StartTaskUART, osPriorityLow, 0, 4096);
  taskUARTHandle = osThreadCreate(osThread(taskUART), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  //initialization of SystemView (here and not in main because it has to be done after init of OS but before start and in
  //main code would be deleted by CubeMX
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

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

/* USER CODE BEGIN Header_StartTaskA */
/**
* @brief Function implementing the taskA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskA */
void StartTaskA(void const * argument)
{
  /* USER CODE BEGIN StartTaskA */
  float randValue[NUMBER_OF_RANDS];
  float sum = 0;
  float squareSum = 0;
  /* Infinite loop */
  for(;;)
  {
    randomizeAndCalculate(randValue, &sum, &squareSum);

    acquireSemaphoreOrMutex();

	allMeans.taskA_mean = sum / NUMBER_OF_RANDS;
	allMeans.taskA_meanSquare = sqrt(squareSum / NUMBER_OF_RANDS);

	releaseSemaphoreOrMutex();

    //osDelay(1);
	//osThreadYield();
	//xTaskNotifyGive(taskUARTHandle);
//	xTaskNotifyGiveIndexed(taskUARTHandle, 0);
	osSemaphoreRelease(UARTSemaphoreHandle);
	vTaskSuspend(NULL);
  }
  /* USER CODE END StartTaskA */
}

/* USER CODE BEGIN Header_StartTaskB */
/**
* @brief Function implementing the taskB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskB */
void StartTaskB(void const * argument)
{
  /* USER CODE BEGIN StartTaskB */
  float randValue[NUMBER_OF_RANDS];
  float sum = 0;
  float squareSum = 0;
  /* Infinite loop */
  for(;;)
  {
	randomizeAndCalculate(randValue, &sum, &squareSum);

	acquireSemaphoreOrMutex();

	allMeans.taskB_mean = sum / NUMBER_OF_RANDS;
	allMeans.taskB_meanSquare = sqrt(squareSum / NUMBER_OF_RANDS);

	releaseSemaphoreOrMutex();

    //osDelay(1);
    //osThreadYield();
//	xTaskNotifyGiveIndexed(taskUARTHandle, 1);
	osSemaphoreRelease(UARTSemaphoreHandle);
    vTaskSuspend(NULL);
  }
  /* USER CODE END StartTaskB */
}

/* USER CODE BEGIN Header_StartTaskC */
/**
* @brief Function implementing the taskC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskC */
void StartTaskC(void const * argument)
{
  /* USER CODE BEGIN StartTaskC */
  float randValue[NUMBER_OF_RANDS];
  float sum = 0;
  float squareSum = 0;

  /* Infinite loop */
  for(;;)
  {
	randomizeAndCalculate(randValue, &sum, &squareSum);

	acquireSemaphoreOrMutex();

	allMeans.taskC_mean = sum / NUMBER_OF_RANDS;
	allMeans.taskC_meanSquare = sqrt(squareSum / NUMBER_OF_RANDS);

	releaseSemaphoreOrMutex();

    //osDelay(1);
	//osThreadYield();
//	xTaskNotifyGiveIndexed(taskUARTHandle, 2);
	osSemaphoreRelease(UARTSemaphoreHandle);
	vTaskSuspend(NULL);
  }
  /* USER CODE END StartTaskC */
}

/* USER CODE BEGIN Header_StartTaskUART */
/**
* @brief Function implementing the taskUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUART */
void StartTaskUART(void const * argument)
{
  /* USER CODE BEGIN StartTaskUART */
  float finalMean = 0;
  float finalMeanSquare = 0;
  /* Infinite loop */
  for(;;)
  {
//	ulTaskNotifyTakeIndexed(0, pdFALSE, portMAX_DELAY);
//	ulTaskNotifyTakeIndexed(1, pdFALSE, portMAX_DELAY);
//	ulTaskNotifyTakeIndexed(2, pdFALSE, portMAX_DELAY);
//	ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
	osSemaphoreWait(UARTSemaphoreHandle, osWaitForever);
	osSemaphoreWait(UARTSemaphoreHandle, osWaitForever);
	osSemaphoreWait(UARTSemaphoreHandle, osWaitForever);

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    acquireSemaphoreOrMutex();

	finalMean = (allMeans.taskA_mean + allMeans.taskB_mean + allMeans.taskC_mean) / NUMBER_OF_TASKS;
	finalMeanSquare = (allMeans.taskA_meanSquare + allMeans.taskB_meanSquare + allMeans.taskC_meanSquare) / NUMBER_OF_TASKS;
	printf("Hello from uart, calculated data:\r\n mean: %.8f\r\n mean square: %.8f\r\n\r\n", finalMean, finalMeanSquare);

	releaseSemaphoreOrMutex();

	vTaskResume(taskAHandle);
	vTaskResume(taskBHandle);
	vTaskResume(taskCHandle);

    osDelay(1);
  }
  /* USER CODE END StartTaskUART */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void randomizeAndCalculate(float* randsArray, float* sum, float* squareSum)
{
	*sum = 0;
	*squareSum = 0;
	srand(osKernelSysTick());
	for(int i = 0; i < NUMBER_OF_RANDS; i++)
	{
		randsArray[i] = (float)rand()/(float)(RAND_MAX/2) - 1;
		*sum += randsArray[i];
		*squareSum += ((randsArray[i]) * (randsArray[i]));
	}
}

void acquireSemaphoreOrMutex()
{
#ifdef SEMAPHORE_VARIANT
	osSemaphoreWait(meanSemaphoreHandle, osWaitForever);
#else
	osMutexWait(meanMutexHandle, osWaitForever);
#endif
}

void releaseSemaphoreOrMutex()
{
#ifdef SEMAPHORE_VARIANT
	osSemaphoreRelease(meanSemaphoreHandle);
#else
	osMutexRelease(meanMutexHandle);
#endif
}
/* USER CODE END Application */

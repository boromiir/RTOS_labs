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
#include "adc.h"
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
dataStruct measuredData;
dataStruct rcvMessage;
uint16_t average;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataTask */
osThreadId_t dataTaskHandle;
const osThreadAttr_t dataTask_attributes = {
  .name = "dataTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_task */
osThreadId_t UART_taskHandle;
const osThreadAttr_t UART_task_attributes = {
  .name = "UART_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
  .name = "dataQueue"
};
/* Definitions for UART_queue */
osMessageQueueId_t UART_queueHandle;
const osMessageQueueAttr_t UART_queue_attributes = {
  .name = "UART_queue"
};
/* Definitions for ADCTimer */
osTimerId_t ADCTimerHandle;
const osTimerAttr_t ADCTimer_attributes = {
  .name = "ADCTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartDataTask(void *argument);
void StartUART_task(void *argument);
void Callback_ADCTimer(void *argument);

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

  /* Create the timer(s) */
  /* creation of ADCTimer */
  ADCTimerHandle = osTimerNew(Callback_ADCTimer, osTimerPeriodic, NULL, &ADCTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(ADCTimerHandle, 1000);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (10, sizeof(uint32_t), &dataQueue_attributes);

  /* creation of UART_queue */
  UART_queueHandle = osMessageQueueNew (10, sizeof(uint32_t), &UART_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of dataTask */
  dataTaskHandle = osThreadNew(StartDataTask, NULL, &dataTask_attributes);

  /* creation of UART_task */
  UART_taskHandle = osThreadNew(StartUART_task, NULL, &UART_task_attributes);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDataTask */
/**
* @brief Function implementing the dataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataTask */
void StartDataTask(void *argument)
{
  /* USER CODE BEGIN StartDataTask */
  uint32_t count = 0;
  uint32_t sum = 0;
  /* Infinite loop */
  for(;;)
  {
	if (osMessageQueueGetCount(dataQueueHandle) != 0)
	{
		HAL_GPIO_WritePin(analyzer_GPIO_Port, analyzer_Pin, GPIO_PIN_SET);
		osMessageQueueGet(dataQueueHandle, &rcvMessage, 0, 0);
		sum += rcvMessage.tempValue;
		count++;
		average = sum/count;
		osMessageQueuePut(UART_queueHandle, &average, 0, 0);
		HAL_GPIO_WritePin(analyzer_GPIO_Port, analyzer_Pin, GPIO_PIN_RESET);
	}
    osDelay(1);
  }
  /* USER CODE END StartDataTask */
}

/* USER CODE BEGIN Header_StartUART_task */
/**
* @brief Function implementing the UART_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_task */
void StartUART_task(void *argument)
{
  /* USER CODE BEGIN StartUART_task */
  uint16_t dataToPrint;
  /* Infinite loop */
  for(;;)
  {
	if (osMessageQueueGetCount(UART_queueHandle) != 0)
	{
	  osMessageQueueGet(UART_queueHandle, &dataToPrint, 0, 0);
	  printf("average temp: %d\n\r", dataToPrint);
	}
    osDelay(1);
  }
  /* USER CODE END StartUART_task */
}

/* Callback_ADCTimer function */
void Callback_ADCTimer(void *argument)
{
  /* USER CODE BEGIN Callback_ADCTimer */
  uint16_t tempRaw = 0;
  uint16_t freqRaw = 0;
  float avg_slope = 4.3;
  uint16_t V25 = 1430;

  HAL_ADC_Start(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, 1);
  tempRaw = HAL_ADC_GetValue(&hadc1);
  measuredData.tempValue = (uint16_t)((V25 - tempRaw) / avg_slope) + 25;

  HAL_ADC_PollForConversion(&hadc1, 1);
  freqRaw = HAL_ADC_GetValue(&hadc1);
  measuredData.freqValue = freqRaw;

  HAL_ADC_Stop(&hadc1);

  osMessageQueuePut(dataQueueHandle, &measuredData, 0, 0);
  /* USER CODE END Callback_ADCTimer */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


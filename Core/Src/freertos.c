/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "mpu6050.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {

	ON_FLIGHT,
	APOGEE_REACHED


}FlightState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


extern char serialBuffer[];
extern char serialBuffer2[];

extern double rollOffset;

extern double pitchOffset;

extern MPU6050_t mpuSensor;



extern double rollFinal;
extern double pitchFinal;
extern FlightState currentState;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SensorOkumasiHandle;
osThreadId SeriPortYazdirHandle;
osMutexId myMutex01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void readSensor(void const * argument);
void serialWrite(void const * argument);

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
  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

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

  /* definition and creation of SensorOkumasi */
  osThreadDef(SensorOkumasi, readSensor, osPriorityRealtime, 0, 512);
  SensorOkumasiHandle = osThreadCreate(osThread(SensorOkumasi), NULL);

  /* definition and creation of SeriPortYazdir */
  osThreadDef(SeriPortYazdir, serialWrite, osPriorityHigh, 0, 512);
  SeriPortYazdirHandle = osThreadCreate(osThread(SeriPortYazdir), NULL);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_readSensor */
/**
* @brief Function implementing the SensorOkumasi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readSensor */
void readSensor(void const * argument)
{
  /* USER CODE BEGIN readSensor */
  /* Infinite loop */
  for(;;)
  {



	  if (osMutexWait(myMutex01Handle, osWaitForever) == osOK) {




		  MPU6050_Read_All(&hi2c1, &mpuSensor);



		  		rollFinal= mpuSensor.KalmanAngleX + rollOffset;
		  	    pitchFinal= mpuSensor.KalmanAngleY + pitchOffset;

		  switch(currentState){

		  case ON_FLIGHT:

		  	if(fabs(pitchFinal)>45.00){
		  		//sprintf( serialBuffer2, "Pitch degerine ulasildi \r\n");

		  			//	HAL_UART_Transmit(&huart2, (uint8_t*)serialBuffer2, strlen(serialBuffer2),2000);
		  	  currentState = APOGEE_REACHED;
		  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  	}

		  	else{
		  		//sprintf( serialBuffer2, "ULASILMADI \r\n");

		  			//		HAL_UART_Transmit(&huart2, (uint8_t*)serialBuffer2, strlen(serialBuffer2),2000);

		  					currentState = ON_FLIGHT;
		  	}

		  	break;




		  case APOGEE_REACHED:

	//aşağıda sensçr yazmasında burayı doldurduk.
		  	break;


		  }

		  osMutexRelease(myMutex01Handle);
	  }

	  osDelay(50);
  }
  /* USER CODE END readSensor */
}

/* USER CODE BEGIN Header_serialWrite */
/**
* @brief Function implementing the SeriPortYazdir thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serialWrite */
void serialWrite(void const * argument)
{
  /* USER CODE BEGIN serialWrite */
  /* Infinite loop */
  for(;;)
  {
	  FlightState temporaryState;

	  if (osMutexWait(myMutex01Handle, osWaitForever) == osOK) {
	  sprintf( serialBuffer, "Pitch: %.2f | Roll: %.2f | Acc(G): X:%.2f Y:%.2f Z:%.2f | Gyro: X:%.2f Y:%.2f Z:%.2f\r\n",
	  pitchFinal,
	  rollFinal,
	  mpuSensor.Ax,
	  mpuSensor.Ay,
	  mpuSensor.Az,
	  mpuSensor.Gx,
	  mpuSensor.Gy,
	  mpuSensor.Gz
	  );



	  temporaryState= currentState;

    osMutexRelease(myMutex01Handle);
  }




	  HAL_UART_Transmit(&huart2, (uint8_t*)serialBuffer, strlen(serialBuffer),2000);


	 if(temporaryState==APOGEE_REACHED){
	  sprintf(serialBuffer2, "APOGEE ULASILDI \r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*)serialBuffer2, strlen(serialBuffer2),2000);


	  }
	 osDelay(100);


	  }
  /* USER CODE END serialWrite */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

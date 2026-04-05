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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Uçuş state machine mantığı
typedef enum {
    WAITING_ON_PAD,        // Rampada fırlatma bekliyor
    ON_FLIGHT,                // Roket fırlatıldı, irtifa artıyor
    FALLING,               // Roket düşüşte
    LANDING                 // Yere inildi, hareket bitti
} FlightState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SİMÜLASYON VE UÇUŞ PARAMETRELERİ
#define LAUNCH_ALTITUDE_REACHED	15.0f
#define APOGEE_DROP_CONFIRM	5.0f
#define MAIN_PARACHUTE_ALTITUDE	2000.0f
#define DRAG_PARACHUTE_ALTITUDE	8000.0f
#define LANDING_ALTITUDE_REACHED	10.0f

#define G_FORCE_DELAY_MS  3000 //başta G kuvveti yüzünden bmp180'in verebileceği hatalı verilerden daha az etkilenmek amacıyla belirli bir süre low pass'daki

#define LOW_PASS_ALPHA_NORMAL	0.2f    // low pass filtre katsayısı
#define LOW_PASS_ALPHA_SECURE	0.05f


#define NEGATIVE_VELOCITY_CONFIRM_VALUE	-2.0f
#define LANDING_VELOCITY_CONFIRM_VALUE  0.5f
// BMP180 sensörmüzn moduna göre maksimum okuma yapma sürelerini buldum ve bu sürelere göre bekleme yaptık döngü sonundaki HAL_Delay fonksiyonunda

#define BMP180_MODE	3

#if BMP180_MODE == 0
#define SENSOR_WAIT_MS 10

#elif BMP180_MODE == 1
#define SENSOR_WAIT_MS 15

#elif BMP180_MODE == 2
#define SENSOR_WAIT_MS 25

#elif BMP180_MODE == 3
#define SENSOR_WAIT_MS 40

#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
FlightState currentState = WAITING_ON_PAD;

float groundAltitude = 0.0f;     // Rampa kalibrasyon için iritifa değeri
float currentRelativeAltitude = 0.0f; // Yere göre güncel filtrelenmiş irtifa
float maxRelativeAltitude = 0.0f;     // Uçuş boyunca ulaşılan maksimum irtifa
float previousAltitude = 0.0f;
float verticalV = 0.0f;
uint32_t lastVTime = 0.0f;

// Low-Pass Filtre Değişkeni
float filteredAltitude = 0.0f;
uint32_t launchTime = 0;
float currentAlpha = LOW_PASS_ALPHA_NORMAL;



extern UART_HandleTypeDef huart2; //usartı tanısın diye exterb ile tanıttık
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId sensorOkuHandle;
osThreadId drag_pHandle;
osThreadId main_pHandle;
osThreadId fsmHandle;
osSemaphoreId dragSemHandle;
osSemaphoreId mainSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
float getAltitude(int32_t pressure, float temperature);
float lowpassfilter(float rawAltitude, float alpha);
void launch_drag_parachute(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}
void launch_main_parachute(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

}

void sendTelemetry(char* message);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void sensorRead(void const * argument);
void dragParachute(void const * argument);
void mainParachute(void const * argument);
void stateMachine(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of dragSem */
  osSemaphoreDef(dragSem);
  dragSemHandle = osSemaphoreCreate(osSemaphore(dragSem), 0);

  /* definition and creation of mainSem */
  osSemaphoreDef(mainSem);
  mainSemHandle = osSemaphoreCreate(osSemaphore(mainSem), 0);

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

  /* definition and creation of sensorOku */
  osThreadDef(sensorOku, sensorRead, osPriorityRealtime, 0, 128);
  sensorOkuHandle = osThreadCreate(osThread(sensorOku), NULL);

  /* definition and creation of drag_p */
  osThreadDef(drag_p, dragParachute, osPriorityAboveNormal, 0, 128);
  drag_pHandle = osThreadCreate(osThread(drag_p), NULL);

  /* definition and creation of main_p */
  osThreadDef(main_p, mainParachute, osPriorityAboveNormal, 0, 128);
  main_pHandle = osThreadCreate(osThread(main_p), NULL);

  /* definition and creation of fsm */
  osThreadDef(fsm, stateMachine, osPriorityHigh, 0, 128);
  fsmHandle = osThreadCreate(osThread(fsm), NULL);

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

/* USER CODE BEGIN Header_sensorRead */
/**
* @brief Function implementing the sensorOku thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensorRead */
void sensorRead(void const * argument)
{
  /* USER CODE BEGIN sensorRead */
  /* Infinite loop */
  for(;;)
  {

	  int32_t temperature = BMP180_GetRawTemperature();
	        int32_t pressure = BMP180_GetPressure();
	        float rawAltitude = getAltitude(pressure,temperature);

	        lowpassfilter(rawAltitude, currentAlpha);
	        currentRelativeAltitude = filteredAltitude - groundAltitude;

	        uint32_t currentTime = HAL_GetTick();
	        if (currentTime - lastVTime >= 100) {
	            verticalV = (currentRelativeAltitude - previousAltitude) / ((currentTime - lastVTime) / 1000.0f);
	            previousAltitude = currentRelativeAltitude;
	            lastVTime = currentTime;
	        }

	        if(currentRelativeAltitude > maxRelativeAltitude){
	            maxRelativeAltitude = currentRelativeAltitude;
	        }

	        osDelay(SENSOR_WAIT_MS); // akilli bekleme
  }
  /* USER CODE END sensorRead */
}

/* USER CODE BEGIN Header_dragParachute */
/**
* @brief Function implementing the drag_p thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dragParachute */
void dragParachute(void const * argument)
{
  /* USER CODE BEGIN dragParachute */
  /* Infinite loop */
  for(;;)
  {

	  if(osSemaphoreWait(dragSemHandle, osWaitForever) == osOK){
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	            sendTelemetry("SURUKLENME PARASUTU ACILDI");
	        }

  }
  /* USER CODE END dragParachute */
}

/* USER CODE BEGIN Header_mainParachute */
/**
* @brief Function implementing the main_p thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mainParachute */
void mainParachute(void const * argument)
{
  /* USER CODE BEGIN mainParachute */
  /* Infinite loop */
  for(;;)
  {

	  // FSM'den semafor gelene kadar burada sonsuza dek bekleyecek
	        if(osSemaphoreWait(mainSemHandle, osWaitForever) == osOK){
	            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	            sendTelemetry("ANA PARASUT ACILDI");
	        }
  }
  /* USER CODE END mainParachute */
}

/* USER CODE BEGIN Header_stateMachine */
/**
* @brief Function implementing the fsm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stateMachine */
void stateMachine(void const * argument)
{
  /* USER CODE BEGIN stateMachine */
  /* Infinite loop */
  for(;;)
  {
 //eğer fırlatmadan bu yana geçen zaman G_FORCE_DELAY_MS değerimizden küçükse henüz güvenli G kuvveti bölgesinde değiliz demektir ve buradayken BMP verilerine daha az güveneceğiz. (low pass filtermizdeki alfa değeriyle oynayarak)
 if(currentState==ON_FLIGHT && (HAL_GetTick()-launchTime) < G_FORCE_DELAY_MS ){
	 currentAlpha= LOW_PASS_ALPHA_SECURE;
 }
 else{
currentAlpha = LOW_PASS_ALPHA_NORMAL;
 }


	switch(currentState){


	case WAITING_ON_PAD:

		if(currentRelativeAltitude > LAUNCH_ALTITUDE_REACHED){
        currentState= ON_FLIGHT;
        launchTime=HAL_GetTick(); //burada fırlatma anındaki geçen zamanı aldık ve launchTime değişkenine atadık.
        sendTelemetry("ROKET FIRLATILDI");

		}
    break;





	case ON_FLIGHT:

		 if((HAL_GetTick()-launchTime) > G_FORCE_DELAY_MS ){ //eğer o riskli G kuvveti bölgesindeysek burada apogee kontrolü de yapmıyoruz, break ile çıkış yapıyoruz doğrudan.
   if(maxRelativeAltitude-currentRelativeAltitude>APOGEE_DROP_CONFIRM && verticalV < NEGATIVE_VELOCITY_CONFIRM_VALUE ){ //apogee kontrol

	   osSemaphoreRelease(dragSemHandle);// eğer apogee ulaşıldıysa ilk ayrılmayı gerçekleştirdik ve doğrudan FALLING state'ine geçtik
	   currentState= FALLING;
	   // semaphore ile çağıracağımız fonskiyonda yazdıracağız zaten sendTelemetry("SURUKLENME PARASUTU ACILDI");

   }

		 }

   break;

	case FALLING:

      if(currentRelativeAltitude<MAIN_PARACHUTE_ALTITUDE){
    	  osSemaphoreRelease(mainSemHandle);
    	 currentState= LANDING;
    	// sendTelemetry("ANA PARASUT ACILDI");

      }

		break;





	case LANDING:
    if(currentRelativeAltitude<LANDING_ALTITUDE_REACHED && fabsf(verticalV)<LANDING_VELOCITY_CONFIRM_VALUE){// ek olarak dikey hızın mutlak değerine de baktık, sensör gürültüsünü hesaba katarak LANDING_VELOCITY_CONFIRM_VALUE değeriyle dikey hızı kontrol ettik

    	//landing gerçekleşti, gerekli uyarılar verilebilir.
    	sendTelemetry("INIS GERCEKLESTI - ROKET RAMPADA");
    	osDelay(500);
    }

		break;







	}//switch sonu








osDelay(20);

  }
  /* USER CODE END stateMachine */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// basınç ve sıcaklığı alarak daha keskin irtifa hesabı yaptık sadece basınç üzerinden hesap yapmaya kıyasla
float getAltitude(int32_t pressure, float temperature) {



	if(pressure <=0){
		return filteredAltitude;
	}
	float T_kelvin = temperature + 273.15f;
	    return ((pow((101325.0f / (float)pressure), 1.0f/5.257f) - 1.0f) * T_kelvin) / 0.0065f;
}

// Sensör Gürültüsünü Engellemek İçin Low-Pass Filtresi
float lowpassfilter(float rawAltitude, float alpha){
	filteredAltitude = (alpha*rawAltitude) + ((1.0f-alpha) * filteredAltitude);
	return filteredAltitude;
}


void sendTelemetry(char* message) {
    char buffer[200];
    sprintf(buffer, "DURUM: %s\r\n"
                    "Current Altitude: %.2f | Max Altitude: %.2f | State: %d | Temperature: %.2f\r\n",
            message, currentRelativeAltitude, maxRelativeAltitude, (int)currentState, BMP180_GetTemperature());
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}
/* USER CODE END Application */

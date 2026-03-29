/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : YRT Rocket Recovery Flight Software (Low-Pass Filter)
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "bmp180_for_stm32_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Uçuş state machine mantığı
typedef enum {
    WAITING_ON_PAD,        // Rampada fırlatma bekliyor
    ON_FLIGHT,                // Roket fırlatıldı, irtifa artıyor
   // APOGEE_REACHED,        // Tepe noktası tespit edildi //şu an gereksiz
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

/* USER CODE BEGIN PV */
FlightState currentState = WAITING_ON_PAD;

float groundAltitude = 0.0f;     // Rampa kalibrasyon için iritifa değeri
float currentRelativeAltitude = 0.0f; // Yere göre güncel filtrelenmiş irtifa
float maxRelativeAltitude = 0.0f;     // Uçuş boyunca ulaşılan maksimum irtifa

// Low-Pass Filtre Değişkeni
float filteredAltitude = 0.0f;
uint32_t launchTime = 0;
float currentAlpha = LOW_PASS_ALPHA_NORMAL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
float getAltitude(int32_t pressure, float temperature);
float lowpassfilter(float rawAltitude, float alpha);
void launch_drag_parachute(){

}
void launch_main_parachute(){

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Basınçtan Mutlak İrtifa Hesaplama
float getAltitude(int32_t pressure, float temperature) {

	float T_kelvin = temperature + 273.15f;
	    return ((pow((101325.0f / (float)pressure), 1.0f/5.257f) - 1.0f) * T_kelvin) / 0.0065f;
}

// Sensör Gürültüsünü Engellemek İçin Low-Pass Filtresi
float lowpassfilter(float rawAltitude, float alpha){
	filteredAltitude = (alpha*rawAltitude) + ((1.0f-alpha) * filteredAltitude);
	return filteredAltitude;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  BMP180_Init(&hi2c1); //bmp ayarlarını yaptık
  BMP180_UpdateCalibrationData();
  BMP180_SetOversampling(BMP180_ULTRA);

  //RAMPA KALİBRASYONU VE FİLTRE OTURTMA
  // gerçek yüksekliğe ulaşmak için 25 kere çalıştırdık low pass filtresini
  for (int i = 0; i < 25; i++) {
	  int32_t temperature = BMP180_GetRawTemperature();
	  int32_t pressure = BMP180_GetPressure();
	       float rawAltitude = getAltitude(pressure,temperature);// ham yüksekliği aldıktan sonra aşağıda low pass filtresine sokuyoruz
    lowpassfilter(rawAltitude, LOW_PASS_ALPHA_NORMAL); // Filtreyi besliyoruz

      HAL_Delay(SENSOR_WAIT_MS); // Sensörün kendini toplaması için kısa bekleme - yine akıllı delay
  }


 groundAltitude = filteredAltitude;//ilk başta sensörü 25 kere çalıştırıp rampa yüksekliğimiz aldık, bu artık groundAltitude oldu.



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      // 1. Sensörden Verileri Okuma
      int32_t temperature = BMP180_GetRawTemperature();
      int32_t pressure = BMP180_GetPressure();
      float rawAltitude = getAltitude(pressure,temperature);// ham yüksekliği aldıktan sonra aşağıda low pass filtresine sokuyoruz
      lowpassfilter(rawAltitude, currentAlpha);
//lowpass filtresi bize filteredAltitude'yi verecek, onu ve rampa yüksekliğini kullanarak göreceli- aslında gerçek irtifayı alacağız
 currentRelativeAltitude= filteredAltitude - groundAltitude;





 if(currentRelativeAltitude>maxRelativeAltitude){
	  maxRelativeAltitude=currentRelativeAltitude;
 }


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



		}
    break;





	case ON_FLIGHT:

		 if((HAL_GetTick()-launchTime) > G_FORCE_DELAY_MS ){ //eğer o riskli G kuvveti bölgesindeysek burada apogee kontrolü de yapmıyoruz, break ile çıkış yapıyoruz doğrudan.
   if(maxRelativeAltitude-currentRelativeAltitude>APOGEE_DROP_CONFIRM ){ //apogee kontrol

	   launch_drag_parachute();// eğer apogee ulaşıldıysa ilk ayrılmayı gerçekleştirdik ve doğrudan FALLING state'ine geçtik
	   currentState= FALLING;
   }
		 }
   break;



/*	case APOGEE_REACHED: //burası gereksiz şu an çünkü yanlış bir sosnuz döngü oluşturuyordu


		if(currentRelativeAltitude<DRAG_PARACHUTE_ALTITUDE){
		//apogee ulaştığımızda burada bir drag paraşütü açacağız
		launch_drag_parachute();
        HAL_Delay(10); // paraşüt açıldıktan sonra ufak bir bekletme sonrasında state değiştireceğiz
        currentState = FALLING;

		}

		else{
		currentState = ON_FLIGHT;
		}


		break;
*/

	case FALLING:

      if(currentRelativeAltitude<MAIN_PARACHUTE_ALTITUDE){
    	 launch_main_parachute();
    	 HAL_Delay(10);
    	 currentState= LANDING;

      }

		break;





	case LANDING:
    if(currentRelativeAltitude<LANDING_ALTITUDE_REACHED){
    	//landing gerçekleşti, gerekli uyarılar verilebilir.
    }

		break;







	}//switch sonu








HAL_Delay(SENSOR_WAIT_MS);//Döngümüz BMP'nin okuma hızını aşıp bmp'den boş veri aldığını zannetmesin diye akıllı bir delay koyduk
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


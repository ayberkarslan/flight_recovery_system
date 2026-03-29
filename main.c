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
    APOGEE_REACHED,        // Tepe noktası tespit edildi
    FALLING,               // Roket düşüşte
    LANDING                 // Yere inildi, hareket bitti
} FlightState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// SİMÜLASYON VE UÇUŞ PARAMETRELERİ
#define LAUNCH_ALTITUDE_REACHED  15.0f
#define APOGEE_DROP_CONFIRM      5.0f
#define MAIN_PARACHUTE_ALTITUDE         2000.0f
#define DRAG_PARACHUTE_ALTITUDE     8000.0f
#define LANDING_ALTITUDE_REACHED  10.0f

#define LOW_PASS_ALPHA              0.2f    // low pass filtre katsayısı
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
float getAltitude(int32_t pressure);
float lowpassfilter(float rawAltitude);
void launch_drag_parachute(){

}
void launch_main_parachute(){

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Basınçtan Mutlak İrtifa Hesaplama
float getAltitude(int32_t pressure) {
    // 101325: Deniz seviyesi standart basıncı (pascal)
    return 44330.0f * (1.0f - pow((float)pressure / 101325.0f, 0.190295f));
}

// Sensör Gürültüsünü Engellemek İçin Low-Pass Filtresi
float lowpassfilter(float rawAltitude){
	filteredAltitude = (LOW_PASS_ALPHA*rawAltitude) + ((1.0f-LOW_PASS_ALPHA) * filteredAltitude);
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

  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n--- YRT ROKET KURTARMA SISTEMI ---\r\n", 40, 100);
  HAL_UART_Transmit(&huart2, (uint8_t*)"Rampa kalibrasyonu yapiliyor...\r\n", 33, 100);

  // 1. ADIM: RAMPA KALİBRASYONU VE FİLTRE OTURTMA
  // gerçek yüksekliğe ulaşmak için 25 kere çalıştırdık low pass filtresini
  for (int i = 0; i < 25; i++) {
      int32_t pressure = BMP180_GetPressure();
      float rawAlt = getAltitude(pressure);
      lowpassfilter(rawAlt); // Filtreyi besliyoruz

      HAL_Delay(50); // Sensörün kendini toplaması için kısa bekleme
  }


 groundAltitude = filteredAltitude;//ilk başta sensörü 25 kere çalıştırıp rampa yüksekliğimiz aldık, bu artık groundAltitude oldu.

  currentRelativeAltitude = filteredAltitude-groundAltitude;


  if(currentRelativeAltitude>maxRelativeAltitude){
	  maxRelativeAltitude=currentRelativeAltitude;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      // 1. Sensörden Verileri Okuma
     // int32_t temperature = BMP180_GetRawTemperature();   gerek yok sıcaklığa şu an
      int32_t pressure = BMP180_GetPressure();
      float rawAltitude = getAltitude(pressure);// ham yüksekliği aldıktan sonra aşağıda low pass filtresine sokuyoruz
      lowpassfilter(rawAltitude);


	switch(currentState){


	case WAITING_ON_PAD:

		if(currentRelativeAltitude > LAUNCH_ALTITUDE_REACHED){
        currentState= ON_FLIGHT;

		}
    break;





	case ON_FLIGHT:

   if(maxRelativeAltitude-currentRelativeAltitude<APOGEE_DROP_CONFIRM && currentRelativeAltitude<maxRelativeAltitude ){ //apogee kontrol
 // hem apogee kontrolü için gereken düşüş yaşandı mı hem de bu max yüksekliğe ulaşıldaıktan sınra mı yapıldı diye kontrol ettik.
	   currentState= APOGEE_REACHED;
   }
   break;



	case APOGEE_REACHED:


		if(currentRelativeAltitude<DRAG_PARACHUTE_ALTITUDE){
		//apogee ulaştığımızda burada bir drag paraşütü açacağız
		launch_drag_parachute();
        HAL_Delay(50); // paraşüt açıldıktan sonra ufak bir bekletme sonrasında state değiştireceğiz
        currentState = FALLING;

		}

		else{
		currentState = ON_FLIGHT;
		}


		break;





	case FALLING:

      if(currentRelativeAltitude<MAIN_PARACHUTE_ALTITUDE){
    	 launch_main_parachute();
    	 HAL_Delay(50);
    	 currentState= LANDING;

      }

		break;





	case LANDING:
    if(currentRelativeAltitude<LANDING_ALTITUDE_REACHED){
    	//landing gerçekleşti, gerekli uyarılar verilebilir.
    }

		break;







	}//switch sonu









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


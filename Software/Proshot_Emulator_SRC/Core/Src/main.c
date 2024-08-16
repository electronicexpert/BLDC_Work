/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint32_t Impuls_Start       = 300;    // Start-Versatz
uint32_t Impuls_Delta       =  22;    // fuer 125 ns
uint32_t Impuls_Grundbreite = 181;    // 1 us
uint32_t Impuls_Abstand     = 732;    // 4.04 us
uint32_t Impuls_Breite_1    =   0;
uint32_t Impuls_Breite_2    =   0;
uint32_t Impuls_Breite_3    =   0;
uint32_t Impuls_Breite_4    =   0;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile unsigned char OC_CH1_N = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// interrupt handler for TIM1 capture/compare event
void TIM1_CC_IRQHandler(void) {
	uint32_t sr = TIM1->SR; // read interrupt flags
	TIM1->SR = 0; // clear interrupt flags before proceeding
	if(sr & TIM_SR_CC1IF) {									// capture event
		switch (OC_CH1_N)									// erster Output Compare kam:
		{
		  case (0):
				OC_CH1_N = 1;  								// naechster Zustand
		        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,0);     // setze PG13-Pin auf High
				TIM1->CCR1 = Impuls_Start;                  // bereite naechsten Output Compare vor
		  break;
		  case (1):
				OC_CH1_N = 2;			                    // naechster Zustand
		        HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);     // setze PG13-Pin auf Low
			   TIM1->CCR1 = Impuls_Start + Impuls_Grundbreite + Impuls_Breite_1;  // bereite naechsten Output Compare vor
		  break;
		  case (2):
				OC_CH1_N = 3;			                    // naechster Zustand
			    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,0);     // setze PG13-Pin auf High
			    TIM1->CCR1 = Impuls_Start + Impuls_Abstand;
		  break;
		  case (3):
				OC_CH1_N = 4;
		 	    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);     // setze PG13-Pin auf Low
		 	   TIM1->CCR1 = Impuls_Start + Impuls_Abstand + Impuls_Grundbreite + Impuls_Breite_2;
		  break;
		  case (4):
				OC_CH1_N = 5;			                    // naechster Zustand
				HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,0);     // setze PG13-Pin auf High
			    TIM1->CCR1 = Impuls_Start + 2 * Impuls_Abstand;
		  break;
		  case (5):
				OC_CH1_N = 6;
		 	    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);     // setze PG13-Pin auf Low
		 	   TIM1->CCR1 = Impuls_Start + 2 * Impuls_Abstand + Impuls_Grundbreite + Impuls_Breite_3;
		  break;
		  case (6):
			   OC_CH1_N = 7;			                    // naechster Zustand
			   HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,0);     // setze PG13-Pin auf High
			   TIM1->CCR1 = Impuls_Start + 3 * Impuls_Abstand;
		  break;
		  case (7):
				OC_CH1_N = 0;
		 	    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);     // setze PG13-Pin auf Low
		 	   TIM1->CCR1 = Impuls_Start + 3 * Impuls_Abstand + Impuls_Grundbreite + Impuls_Breite_4;;
		  break;

		}
	}
}

//void TIM1_UP_TIM10_IRQHandler(void) {
//	uint32_t sr = TIM1->SR; // read interrupt flags
//	TIM1->SR = 0; // clear interrupt flags before proceeding
////	if(sr & TIM_SR_UIF) {									// update event
//		HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
//
//	}
//}
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
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  RCC->APB2ENR = RCC_APB2ENR_TIM1EN; // enable timer clock in RCC
  RCC->APB2ENR;
  TIM1->PSC = 0; //
  TIM1->EGR = TIM_EGR_UG; // update event to write prescaler shadow register
  TIM1->SR = 0; // clear interrupt flasg set by the update event
  TIM1->ARR = 11584 - 1; // must be > CCR1 (fuer 64us)
  TIM1->CCR1 = 100 - 1; //
  TIM1->DIER = TIM_DIER_UIE | TIM_DIER_CC1IE; // enable CC and update interrupts in timer
  TIM1->CR1 = TIM_CR1_CEN; // start timer
  NVIC_EnableIRQ(TIM1_CC_IRQn); // enable CC interrupt in NVIC
//  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // enable update interrupt in NVIC



  uint8_t stellwert_l;
  uint8_t stellwert_m;
  uint8_t stellwert_h;

  // zerlege in 3 mal 4 bit:

//  stellwert_l = stellwert & 0x0f;
//  stellwert_m = (stellwert >> 4) & 0x0f;
//  stellwert_h = (stellwert >> 8) & 0x0f;



//  Impuls_Breite_1 = stellwert_h * Impuls_Delta;
//  Impuls_Breite_2 = stellwert_m * Impuls_Delta;
//  Impuls_Breite_3 = stellwert_l * Impuls_Delta;
  Impuls_Breite_4 = 15 * Impuls_Delta;


// als Beispiel: 1 Inkrement, bedenke, dass Stellwert auf Empfangsseite durch 4 geteilt wird

//  stellwert_l = (uint32_t) 0x4;
//  stellwert_m = (uint32_t)0x0;
//  stellwert_h = (uint32_t)0x0;
//
//  Impuls_Breite_1 = stellwert_h * Impuls_Delta;
//  Impuls_Breite_2 = stellwert_m * Impuls_Delta;
//  Impuls_Breite_3 = stellwert_l * Impuls_Delta;
//   HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



    /* USER CODE END WHILE */
   for(int i = 0; i < 1000; i++)
   {
	   stellwert_l = (uint32_t)i & 0x0f;
	   stellwert_m = ((uint32_t)i >> 4) & 0x0f;
	   stellwert_h = ((uint32_t)i >> 8) & 0x0f;

	   Impuls_Breite_1 = stellwert_h * Impuls_Delta;
	   Impuls_Breite_2 = stellwert_m * Impuls_Delta;
	   Impuls_Breite_3 = stellwert_l * Impuls_Delta;
	   HAL_Delay(10);
   }
   for(int i = 1000; i > 0; i--)
     {
  	   stellwert_l = (uint32_t)i & 0x0f;
  	   stellwert_m = ((uint32_t)i >> 4) & 0x0f;
  	   stellwert_h = ((uint32_t)i >> 8) & 0x0f;

  	   Impuls_Breite_1 = stellwert_h * Impuls_Delta;
  	   Impuls_Breite_2 = stellwert_m * Impuls_Delta;
  	   Impuls_Breite_3 = stellwert_l * Impuls_Delta;
  	   HAL_Delay(10);
     }

   // toggle DIRECTION

   HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  *         where the assert_param error has occurred.
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

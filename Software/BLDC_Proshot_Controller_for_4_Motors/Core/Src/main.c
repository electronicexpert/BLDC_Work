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
  *  10.09.2024  JK
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t test = 0;
uint8_t ready = 0;
uint8_t Rx_data[1];



const struct Impuls imp = {.Impuls_Delta=22,.Impuls_Base=181,.Impuls_Distance=732};

struct Motor m1;
struct Motor m2;
struct Motor m3;
struct Motor m4;

volatile GPIO_PinState Pintable[8] = {GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET,GPIO_PIN_RESET,GPIO_PIN_SET};
volatile uint32_t Pulsetable_M1[8] = {0,0,0,0,0,0,0,0};
volatile uint32_t Pulsetable_M2[8] = {0,0,0,0,0,0,0,0};
volatile uint32_t Pulsetable_M3[8] = {0,0,0,0,0,0,0,0};
volatile uint32_t Pulsetable_M4[8] = {0,0,0,0,0,0,0,0};


volatile unsigned char OC_CH1_N = 0;
volatile unsigned char OC_CH2_N = 0;
volatile unsigned char OC_CH3_N = 0;
volatile unsigned char OC_CH4_N = 0;





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
  MX_UART5_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  // Timer 1 pre-settings:

   RCC->APB2ENR = RCC_APB2ENR_TIM1EN; // enable timer 1 in RCC
   TIM1->PSC = 0; //
   TIM1->EGR = TIM_EGR_UG; // update event to write prescaler shadow register
   TIM1->SR = 0; // clear interrupt flag set by the update event
   TIM1->ARR = 11584 - 1; // must be > CCR1 (fuer 64us)
   TIM1->CCR1 = 100 - 1;  // initial OC for motor1
   TIM1->CCR2 = 3010 - 1; // initial OC for motor2
   TIM1->CCR3 = 5820 - 1; // initial OC for motor3
   TIM1->CCR4 = 8630 - 1; // initial OC for motor4
   TIM1->DIER = TIM_DIER_UIE | TIM_DIER_CC4IE | TIM_DIER_CC3IE | TIM_DIER_CC2IE | TIM_DIER_CC1IE; // enable all CC and update interrupts
   NVIC_EnableIRQ(TIM1_CC_IRQn); // enable CC interrupt in NVIC
   NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); // enable update interrupt in NVIC

   // Timer will be started in  main.cpp


  maincpp();


  while (1)
  {
    /* USER CODE END WHILE */

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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Dir4_Pin|Dir3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Dir2_Pin|Dir1_Pin|Proshot4_Pin|Proshot3_Pin
                          |Proshot2_Pin|Proshot1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Relais_GPIO_Port, Relais_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Dir4_Pin Dir3_Pin */
  GPIO_InitStruct.Pin = Dir4_Pin|Dir3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Dir2_Pin Dir1_Pin Proshot4_Pin Proshot3_Pin
                           Proshot2_Pin Proshot1_Pin */
  GPIO_InitStruct.Pin = Dir2_Pin|Dir1_Pin|Proshot4_Pin|Proshot3_Pin
                          |Proshot2_Pin|Proshot1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : Relais_Pin */
  GPIO_InitStruct.Pin = Relais_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Relais_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	test = Rx_data[0];
	ready = 1;
	HAL_UART_Receive_IT (&huart5, Rx_data, 1);

}

void TIM1_UP_TIM10_IRQHandler(void)
{
	TIM1->SR = 0; // clear interrupt flags before proceeding
}


// interrupt handler for TIM1 capture/compare event

void TIM1_CC_IRQHandler(void)
{
	uint32_t sr = TIM1->SR; // read interrupt flags
   	TIM1->SR = 0; // clear interrupt flags before proceeding
   	if(sr & TIM_SR_CC1IF)    // Motor 1 will be serviced , CC1 triggered
   	{
   	   HAL_GPIO_WritePin(Proshot1_GPIO_Port,Proshot1_Pin,Pintable[OC_CH1_N]);
	   TIM1->CCR1 = Pulsetable_M1[OC_CH1_N];
	   OC_CH1_N++;
	   if(OC_CH1_N == 8)
	   {
		   OC_CH1_N = 0;
	   }
   	}
	if(sr & TIM_SR_CC2IF)    // Motor 2 will be serviced , CC2 triggered
   	{
	   HAL_GPIO_WritePin(Proshot2_GPIO_Port,Proshot2_Pin,Pintable[OC_CH2_N]);
	   TIM1->CCR2 = Pulsetable_M2[OC_CH2_N];
	   OC_CH2_N++;
	   if(OC_CH2_N == 8)
	   {
		   OC_CH2_N = 0;
	   }
   	}

	if(sr & TIM_SR_CC3IF)    // Motor 3 will be serviced , CC3 triggered
	{
       HAL_GPIO_WritePin(Proshot3_GPIO_Port,Proshot3_Pin,Pintable[OC_CH3_N]);
	   TIM1->CCR3 = Pulsetable_M3[OC_CH3_N];
	   OC_CH3_N++;
	   if(OC_CH3_N == 8)
	   {
		   OC_CH3_N = 0;
	   }
	}

	if(sr & TIM_SR_CC4IF)    // Motor 4 will be serviced , CC4 triggered
	{
		HAL_GPIO_WritePin(Proshot4_GPIO_Port,Proshot4_Pin,Pintable[OC_CH4_N]);
		TIM1->CCR4 = Pulsetable_M4[OC_CH4_N];
		OC_CH4_N++;
		if(OC_CH4_N == 8)
		{
		  OC_CH4_N = 0;
	    }
	}

}



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

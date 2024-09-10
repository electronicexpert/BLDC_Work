/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

struct Impuls
{
	uint32_t Impuls_Delta;        // for 125 ns
	uint32_t Impuls_Base;         // for 1 us
	uint32_t Impuls_Distance;    // for 4.04 us
};

struct Motor
{
	uint32_t Impuls_Start;
	uint32_t Impuls_Width_1;
	uint32_t Impuls_Width_2;
	uint32_t Impuls_Width_3;
	uint32_t Impuls_Width_4;
};

extern volatile uint32_t Pulsetable_M1[8];
extern volatile uint32_t Pulsetable_M2[8];
extern volatile uint32_t Pulsetable_M3[8];
extern volatile uint32_t Pulsetable_M4[8];

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void maincpp();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Dir4_Pin GPIO_PIN_6
#define Dir4_GPIO_Port GPIOD
#define Dir3_Pin GPIO_PIN_7
#define Dir3_GPIO_Port GPIOD
#define Dir2_Pin GPIO_PIN_9
#define Dir2_GPIO_Port GPIOG
#define Dir1_Pin GPIO_PIN_10
#define Dir1_GPIO_Port GPIOG
#define Proshot4_Pin GPIO_PIN_11
#define Proshot4_GPIO_Port GPIOG
#define Proshot3_Pin GPIO_PIN_12
#define Proshot3_GPIO_Port GPIOG
#define Proshot2_Pin GPIO_PIN_13
#define Proshot2_GPIO_Port GPIOG
#define Proshot1_Pin GPIO_PIN_14
#define Proshot1_GPIO_Port GPIOG
#define Relais_Pin GPIO_PIN_3
#define Relais_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

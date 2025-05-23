/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_A_Pin GPIO_PIN_13
#define SW_A_GPIO_Port GPIOC
#define SW_B_Pin GPIO_PIN_14
#define SW_B_GPIO_Port GPIOC
#define WAKE_Pin GPIO_PIN_7
#define WAKE_GPIO_Port GPIOE
#define INL1_Pin GPIO_PIN_8
#define INL1_GPIO_Port GPIOE
#define INH1_Pin GPIO_PIN_9
#define INH1_GPIO_Port GPIOE
#define INL2_Pin GPIO_PIN_10
#define INL2_GPIO_Port GPIOE
#define INH2_Pin GPIO_PIN_11
#define INH2_GPIO_Port GPIOE
#define INL3_Pin GPIO_PIN_12
#define INL3_GPIO_Port GPIOE
#define INH3_Pin GPIO_PIN_13
#define INH3_GPIO_Port GPIOE
#define READY_Pin GPIO_PIN_14
#define READY_GPIO_Port GPIOE
#define NFAULT_Pin GPIO_PIN_15
#define NFAULT_GPIO_Port GPIOE
#define SW_Pin GPIO_PIN_10
#define SW_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOA
#define LED_ALIVE_Pin GPIO_PIN_2
#define LED_ALIVE_GPIO_Port GPIOD
#define SPI1_NSS_Pin GPIO_PIN_6
#define SPI1_NSS_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

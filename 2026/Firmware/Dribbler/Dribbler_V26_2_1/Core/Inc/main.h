/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define USER_SW_Pin GPIO_PIN_3
#define USER_SW_GPIO_Port GPIOB
#define BS_IN_Pin GPIO_PIN_6
#define BS_IN_GPIO_Port GPIOA
#define ENC_IN1_Pin GPIO_PIN_0
#define ENC_IN1_GPIO_Port GPIOA
#define ENC_IN2_Pin GPIO_PIN_1
#define ENC_IN2_GPIO_Port GPIOA
#define MD_SO_Pin GPIO_PIN_4
#define MD_SO_GPIO_Port GPIOA
#define MD_nSLEEP_Pin GPIO_PIN_8
#define MD_nSLEEP_GPIO_Port GPIOA
#define MD_INB_Pin GPIO_PIN_0
#define MD_INB_GPIO_Port GPIOB
#define MD_INA_Pin GPIO_PIN_1
#define MD_INA_GPIO_Port GPIOB
#define CAN_LED_Pin GPIO_PIN_10
#define CAN_LED_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_7
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define BS_LED_Pin GPIO_PIN_10
#define BS_LED_GPIO_Port GPIOA
#define BS_OUT_Pin GPIO_PIN_7
#define BS_OUT_GPIO_Port GPIOA
#define ENC_POWER_Pin GPIO_PIN_15
#define ENC_POWER_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

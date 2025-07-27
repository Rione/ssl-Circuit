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
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MVC_VSCAN1_Pin GPIO_PIN_0
#define MVC_VSCAN1_GPIO_Port GPIOA
#define MVC_VSCAN2_Pin GPIO_PIN_1
#define MVC_VSCAN2_GPIO_Port GPIOA
#define MVC_IOUT2_Pin GPIO_PIN_2
#define MVC_IOUT2_GPIO_Port GPIOA
#define MVC_IOUT1_Pin GPIO_PIN_3
#define MVC_IOUT1_GPIO_Port GPIOA
#define MVC_TMP_Pin GPIO_PIN_4
#define MVC_TMP_GPIO_Port GPIOA
#define MVC_DIR_Pin GPIO_PIN_5
#define MVC_DIR_GPIO_Port GPIOA
#define MVC_EN2_Pin GPIO_PIN_6
#define MVC_EN2_GPIO_Port GPIOA
#define MVC_ISETD_Pin GPIO_PIN_7
#define MVC_ISETD_GPIO_Port GPIOA
#define MVC_UVLO_Pin GPIO_PIN_0
#define MVC_UVLO_GPIO_Port GPIOB
#define MVC_EN1_Pin GPIO_PIN_1
#define MVC_EN1_GPIO_Port GPIOB
#define SVC_EN_Pin GPIO_PIN_10
#define SVC_EN_GPIO_Port GPIOB
#define FAN_EN_Pin GPIO_PIN_8
#define FAN_EN_GPIO_Port GPIOA
#define LED_CAN_Pin GPIO_PIN_4
#define LED_CAN_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_5
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_6
#define LED_2_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_7
#define LED_1_GPIO_Port GPIOB
#define USW_Pin GPIO_PIN_8
#define USW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

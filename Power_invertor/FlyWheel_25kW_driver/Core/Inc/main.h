/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define PWR_ON_1_Pin GPIO_PIN_13
#define PWR_ON_1_GPIO_Port GPIOC
#define PWR_ON_2_Pin GPIO_PIN_14
#define PWR_ON_2_GPIO_Port GPIOC
#define PWR_ON_2C15_Pin GPIO_PIN_15
#define PWR_ON_2C15_GPIO_Port GPIOC
#define TOUCH_INT_Pin GPIO_PIN_0
#define TOUCH_INT_GPIO_Port GPIOC
#define BTN_1_Pin GPIO_PIN_1
#define BTN_1_GPIO_Port GPIOC
#define BTN_2_Pin GPIO_PIN_2
#define BTN_2_GPIO_Port GPIOC
#define BTN_3_Pin GPIO_PIN_3
#define BTN_3_GPIO_Port GPIOC
#define PWR_ON_4_Pin GPIO_PIN_4
#define PWR_ON_4_GPIO_Port GPIOC
#define PWR_ON_5_Pin GPIO_PIN_5
#define PWR_ON_5_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_0
#define LED_3_GPIO_Port GPIOB
#define FAN_PWM_Pin GPIO_PIN_1
#define FAN_PWM_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOB
#define CAPT_2_Pin GPIO_PIN_10
#define CAPT_2_GPIO_Port GPIOB
#define DIRECT_Pin GPIO_PIN_8
#define DIRECT_GPIO_Port GPIOC
#define CS_5_Pin GPIO_PIN_9
#define CS_5_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOA
#define CAPT_1_Pin GPIO_PIN_15
#define CAPT_1_GPIO_Port GPIOA
#define CS_1_Pin GPIO_PIN_10
#define CS_1_GPIO_Port GPIOC
#define CS_2_Pin GPIO_PIN_11
#define CS_2_GPIO_Port GPIOC
#define CS_3_Pin GPIO_PIN_12
#define CS_3_GPIO_Port GPIOC
#define CS_4_Pin GPIO_PIN_2
#define CS_4_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_Pin GPIO_PIN_13
#define START_GPIO_Port GPIOC
#define START_EXTI_IRQn EXTI15_10_IRQn
#define STOP_Pin GPIO_PIN_14
#define STOP_GPIO_Port GPIOC
#define STOP_EXTI_IRQn EXTI15_10_IRQn
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOF
#define LED_3_Pin GPIO_PIN_1
#define LED_3_GPIO_Port GPIOF
#define INT_Pin GPIO_PIN_1
#define INT_GPIO_Port GPIOA
#define INT_EXTI_IRQn EXTI1_IRQn
#define U_in_volt_Pin GPIO_PIN_3
#define U_in_volt_GPIO_Port GPIOA
#define SENS_1_Pin GPIO_PIN_4
#define SENS_1_GPIO_Port GPIOA
#define SENS_2_Pin GPIO_PIN_5
#define SENS_2_GPIO_Port GPIOA
#define SENS_3_Pin GPIO_PIN_6
#define SENS_3_GPIO_Port GPIOA
#define SENS_4_Pin GPIO_PIN_7
#define SENS_4_GPIO_Port GPIOA
#define INP_1_Pin GPIO_PIN_0
#define INP_1_GPIO_Port GPIOB
#define INP_2_Pin GPIO_PIN_1
#define INP_2_GPIO_Port GPIOB
#define Current_Pin GPIO_PIN_2
#define Current_GPIO_Port GPIOB
#define Port_2_Pin GPIO_PIN_10
#define Port_2_GPIO_Port GPIOB
#define Port_1_Pin GPIO_PIN_11
#define Port_1_GPIO_Port GPIOB
#define Port_3_Pin GPIO_PIN_12
#define Port_3_GPIO_Port GPIOB
#define Port_4_Pin GPIO_PIN_13
#define Port_4_GPIO_Port GPIOB
#define Direct_Pin GPIO_PIN_10
#define Direct_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

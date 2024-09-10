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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define LED_1_ON    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);
#define LED_1_OFF   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET);
#define LED_2_ON    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET);
#define LED_2_OFF   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET);
#define LED_3_ON    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET);
#define LED_3_OFF   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_RESET);
#define LED_4_ON    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);
#define LED_4_OFF   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET);
#define LED_5_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
#define LED_5_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_RESET);
#define LED_6_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,GPIO_PIN_SET);
#define LED_6_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,GPIO_PIN_RESET);
#define LED_7_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET);
#define LED_7_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET);


#define LED_1_Togle   HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
#define LED_2_Togle   HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
#define LED_3_Togle   HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
#define LED_4_Togle   HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_7);
#define LED_5_Togle   HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);
#define LED_6_Togle   HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
#define LED_7_Togle   HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_5);

#define TOGLE_TIME 300

#define TX_2    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
#define RX_2    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);


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
#define LED_1_Pin GPIO_PIN_13
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_7
#define LED_4_GPIO_Port GPIOA
#define LED_7_Pin GPIO_PIN_11
#define LED_7_GPIO_Port GPIOB
#define DIRECT_Pin GPIO_PIN_4
#define DIRECT_GPIO_Port GPIOB
#define LED_5_Pin GPIO_PIN_5
#define LED_5_GPIO_Port GPIOB
#define LED_6_Pin GPIO_PIN_9
#define LED_6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

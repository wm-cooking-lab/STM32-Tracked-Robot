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
#include "stm32l4xx_hal.h"

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
#define BP_Start_Pin GPIO_PIN_13
#define BP_Start_GPIO_Port GPIOC
#define BP_Start_EXTI_IRQn EXTI15_10_IRQn
#define Terminal_TX_Pin GPIO_PIN_2
#define Terminal_TX_GPIO_Port GPIOA
#define Terminal_RX_Pin GPIO_PIN_3
#define Terminal_RX_GPIO_Port GPIOA
#define Alert_Batt_Pin GPIO_PIN_5
#define Alert_Batt_GPIO_Port GPIOA
#define Tension_Batt_Pin GPIO_PIN_5
#define Tension_Batt_GPIO_Port GPIOC
#define Direction_Gauche_Pin GPIO_PIN_2
#define Direction_Gauche_GPIO_Port GPIOB
#define Moteur_Droit_Pin GPIO_PIN_11
#define Moteur_Droit_GPIO_Port GPIOB
#define Direction_Droite_Pin GPIO_PIN_8
#define Direction_Droite_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Moteur_Gauche_Pin GPIO_PIN_15
#define Moteur_Gauche_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

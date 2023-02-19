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
#include "stm32f1xx_hal.h"

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
#define BOARD_LED_Pin GPIO_PIN_13
#define BOARD_LED_GPIO_Port GPIOC
#define ADC1_POT_Pin GPIO_PIN_1
#define ADC1_POT_GPIO_Port GPIOB
#define SSEG_G_Pin GPIO_PIN_12
#define SSEG_G_GPIO_Port GPIOB
#define SSEG_C_Pin GPIO_PIN_13
#define SSEG_C_GPIO_Port GPIOB
#define SSEG_DOT_Pin GPIO_PIN_14
#define SSEG_DOT_GPIO_Port GPIOB
#define SSEG_D_Pin GPIO_PIN_15
#define SSEG_D_GPIO_Port GPIOB
#define SSEG_E_Pin GPIO_PIN_8
#define SSEG_E_GPIO_Port GPIOA
#define SSEG_4_Pin GPIO_PIN_9
#define SSEG_4_GPIO_Port GPIOA
#define SSEG_1_Pin GPIO_PIN_10
#define SSEG_1_GPIO_Port GPIOA
#define SSEG_A_Pin GPIO_PIN_11
#define SSEG_A_GPIO_Port GPIOA
#define SSEG_F_Pin GPIO_PIN_12
#define SSEG_F_GPIO_Port GPIOA
#define SYS_JTMS_SWDIO_Pin GPIO_PIN_13
#define SYS_JTMS_SWDIO_GPIO_Port GPIOA
#define SSEG_3_Pin GPIO_PIN_15
#define SSEG_3_GPIO_Port GPIOA
#define SSEG_2_Pin GPIO_PIN_3
#define SSEG_2_GPIO_Port GPIOB
#define SSEG_B_Pin GPIO_PIN_4
#define SSEG_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

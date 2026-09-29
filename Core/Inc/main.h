/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_C13_Pin GPIO_PIN_13
#define BTN_C13_GPIO_Port GPIOC
#define MOTOR_LEFT_R_EN_Pin GPIO_PIN_1
#define MOTOR_LEFT_R_EN_GPIO_Port GPIOC
#define MOTOR_RIGHT_R_EN_Pin GPIO_PIN_2
#define MOTOR_RIGHT_R_EN_GPIO_Port GPIOC
#define MOTOR_LEFT_L_EN_Pin GPIO_PIN_3
#define MOTOR_LEFT_L_EN_GPIO_Port GPIOC
#define MOTOR_RIGHT_L_EN_Pin GPIO_PIN_0
#define MOTOR_RIGHT_L_EN_GPIO_Port GPIOA
#define MOTOR_LEFT_PWM_Pin GPIO_PIN_1
#define MOTOR_LEFT_PWM_GPIO_Port GPIOA
#define MOTOR_RIGHT_PWM_Pin GPIO_PIN_2
#define MOTOR_RIGHT_PWM_GPIO_Port GPIOA
#define BTN_B0_Pin GPIO_PIN_0
#define BTN_B0_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_2
#define LED_STATUS_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

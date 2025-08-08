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
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOE
#define I2C2_SCL_MPU_Pin GPIO_PIN_10
#define I2C2_SCL_MPU_GPIO_Port GPIOB
#define I2C2_SDA_MPU_Pin GPIO_PIN_11
#define I2C2_SDA_MPU_GPIO_Port GPIOB
#define LoRa_Aux_Pin GPIO_PIN_15
#define LoRa_Aux_GPIO_Port GPIOB
#define LoRa_M0_Pin GPIO_PIN_10
#define LoRa_M0_GPIO_Port GPIOD
#define LoRa_M1_Pin GPIO_PIN_11
#define LoRa_M1_GPIO_Port GPIOD
#define PC817_D1_Pin GPIO_PIN_6
#define PC817_D1_GPIO_Port GPIOD
#define PC817_D2_Pin GPIO_PIN_7
#define PC817_D2_GPIO_Port GPIOD
#define I2C1_SCL_MS_Pin GPIO_PIN_6
#define I2C1_SCL_MS_GPIO_Port GPIOB
#define I2C1_SDA_MS_Pin GPIO_PIN_7
#define I2C1_SDA_MS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

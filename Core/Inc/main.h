/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define HW_OUT_5_Pin GPIO_PIN_14
#define HW_OUT_5_GPIO_Port GPIOC
#define Green_light_Pin GPIO_PIN_15
#define Green_light_GPIO_Port GPIOC
#define HW_OUT_1_Pin GPIO_PIN_5
#define HW_OUT_1_GPIO_Port GPIOA
#define HC_SR04_Echo_Pin GPIO_PIN_6
#define HC_SR04_Echo_GPIO_Port GPIOA
#define HW_OUT_2_Pin GPIO_PIN_7
#define HW_OUT_2_GPIO_Port GPIOA
#define HW_OUT_3_Pin GPIO_PIN_0
#define HW_OUT_3_GPIO_Port GPIOB
#define HW_OUT_4_Pin GPIO_PIN_1
#define HW_OUT_4_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_12
#define OLED_SDA_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_13
#define AIN1_GPIO_Port GPIOB
#define Red_light_Pin GPIO_PIN_14
#define Red_light_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOA
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define OLED_SCL_Pin GPIO_PIN_15
#define OLED_SCL_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_3
#define BIN1_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOB
#define KEY1_EXTI_IRQn EXTI4_IRQn
#define HC_SR04_Trig_Pin GPIO_PIN_5
#define HC_SR04_Trig_GPIO_Port GPIOB
#define SCL_6050_Pin GPIO_PIN_8
#define SCL_6050_GPIO_Port GPIOB
#define SDA_6050_Pin GPIO_PIN_9
#define SDA_6050_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

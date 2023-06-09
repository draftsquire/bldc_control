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
#define MOTOR_DIR_Pin GPIO_PIN_1
#define MOTOR_DIR_GPIO_Port GPIOA
#define TIM2_PWM_CH1_Pin GPIO_PIN_5
#define TIM2_PWM_CH1_GPIO_Port GPIOA
#define TIM3_OPT_CH1_Pin GPIO_PIN_6
#define TIM3_OPT_CH1_GPIO_Port GPIOA
#define TIM3_OPT_CH2_Pin GPIO_PIN_7
#define TIM3_OPT_CH2_GPIO_Port GPIOA
#define TS_A_IN_Pin GPIO_PIN_8
#define TS_A_IN_GPIO_Port GPIOC
#define TS_A_IN_EXTI_IRQn EXTI9_5_IRQn
#define TS_B_IN_Pin GPIO_PIN_9
#define TS_B_IN_GPIO_Port GPIOC
#define TS_B_IN_EXTI_IRQn EXTI9_5_IRQn
#define TIM1_MECH_CH1_Pin GPIO_PIN_8
#define TIM1_MECH_CH1_GPIO_Port GPIOA
#define TIM1_MECH_CH2_Pin GPIO_PIN_9
#define TIM1_MECH_CH2_GPIO_Port GPIOA
#define EN_BUTTON_IN_Pin GPIO_PIN_6
#define EN_BUTTON_IN_GPIO_Port GPIOB
#define EN_BUTTON_IN_EXTI_IRQn EXTI9_5_IRQn
#define TIM10_IC_CH1_Pin GPIO_PIN_8
#define TIM10_IC_CH1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

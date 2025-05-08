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
#include "stm32g0xx_hal.h"

#include "stm32g0xx_nucleo.h"
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
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOC
#define BSP_button_Pin GPIO_PIN_13
#define BSP_button_GPIO_Port GPIOC
#define BSP_button_EXTI_IRQn EXTI4_15_IRQn
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define INT_ENCODER1_Pin GPIO_PIN_1
#define INT_ENCODER1_GPIO_Port GPIOC
#define INT_ENCODER1_EXTI_IRQn EXTI0_1_IRQn
#define INT_ENCODER2_Pin GPIO_PIN_2
#define INT_ENCODER2_GPIO_Port GPIOC
#define INT_ENCODER2_EXTI_IRQn EXTI2_3_IRQn
#define SPI_BNRG_EXTI_Pin GPIO_PIN_0
#define SPI_BNRG_EXTI_GPIO_Port GPIOA
#define SPI_BNRG_EXTI_EXTI_IRQn EXTI0_1_IRQn
#define SPI_CS_Pin GPIO_PIN_1
#define SPI_CS_GPIO_Port GPIOA
#define BLNRG_RST_Pin GPIO_PIN_4
#define BLNRG_RST_GPIO_Port GPIOC
#define NUCLEO_LED_Pin GPIO_PIN_5
#define NUCLEO_LED_GPIO_Port GPIOC
#define ADC_BATTERY_INDICATOR_Pin GPIO_PIN_2
#define ADC_BATTERY_INDICATOR_GPIO_Port GPIOB
#define PHA_Pin GPIO_PIN_14
#define PHA_GPIO_Port GPIOB
#define PHB_Pin GPIO_PIN_8
#define PHB_GPIO_Port GPIOA
#define REF_Pin GPIO_PIN_9
#define REF_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_7
#define RST_GPIO_Port GPIOC
#define EN_IHM_Pin GPIO_PIN_10
#define EN_IHM_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_2
#define LED5_GPIO_Port GPIOD
#define ENCODER1_DATA_Pin GPIO_PIN_3
#define ENCODER1_DATA_GPIO_Port GPIOB
#define IHM_PWMA_Pin GPIO_PIN_4
#define IHM_PWMA_GPIO_Port GPIOB
#define IHM_PWMB_Pin GPIO_PIN_5
#define IHM_PWMB_GPIO_Port GPIOB
#define ENCODER2_DATA_Pin GPIO_PIN_6
#define ENCODER2_DATA_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

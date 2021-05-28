/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l0xx_hal.h"

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
#define NSS_Pin GPIO_PIN_15
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_3
#define SCK_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_5
#define MOSI_GPIO_Port GPIOB
#define QI_EN_Pin GPIO_PIN_9
#define QI_EN_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_4
#define MISO_GPIO_Port GPIOB
#define QI_INT_Pin GPIO_PIN_8
#define QI_INT_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_10
#define GREEN_LED_GPIO_Port GPIOA
#define QI_PDETB_Pin GPIO_PIN_7
#define QI_PDETB_GPIO_Port GPIOB
#define ACC_INT1_Pin GPIO_PIN_14
#define ACC_INT1_GPIO_Port GPIOC
#define ACC_INT2_Pin GPIO_PIN_15
#define ACC_INT2_GPIO_Port GPIOC
#define RED_LED_Pin GPIO_PIN_15
#define RED_LED_GPIO_Port GPIOB
#define BATT_CHARGING_Pin GPIO_PIN_2
#define BATT_CHARGING_GPIO_Port GPIOC
#define SDA_Pin GPIO_PIN_14
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_13
#define SCL_GPIO_Port GPIOB
#define BUSY_Pin GPIO_PIN_10
#define BUSY_GPIO_Port GPIOB
#define LR_NRESET_Pin GPIO_PIN_0
#define LR_NRESET_GPIO_Port GPIOB
#define HALL_SOUTH_Pin GPIO_PIN_6
#define HALL_SOUTH_GPIO_Port GPIOA
#define LR_IRQ__DIO9_Pin GPIO_PIN_5
#define LR_IRQ__DIO9_GPIO_Port GPIOA
#define VBAT_MCU_Pin GPIO_PIN_3
#define VBAT_MCU_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

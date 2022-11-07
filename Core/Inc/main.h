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
#include "stm32l1xx_hal.h"

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
void SystemClock_Config(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOF_SHUT_N_Pin GPIO_PIN_3
#define TOF_SHUT_N_GPIO_Port GPIOE
#define TOF_INT_Pin GPIO_PIN_1
#define TOF_INT_GPIO_Port GPIOE
#define LED_RD_Pin GPIO_PIN_7
#define LED_RD_GPIO_Port GPIOD
#define ACC_CS_N_Pin GPIO_PIN_15
#define ACC_CS_N_GPIO_Port GPIOA
#define CEL_RST_N_Pin GPIO_PIN_12
#define CEL_RST_N_GPIO_Port GPIOA
#define ACL_INT2_Pin GPIO_PIN_4
#define ACL_INT2_GPIO_Port GPIOE
#define GNSS_RX_Pin GPIO_PIN_7
#define GNSS_RX_GPIO_Port GPIOB
#define GNSS_TX_Pin GPIO_PIN_6
#define GNSS_TX_GPIO_Port GPIOB
#define CEL_MAIN_DTR_Pin GPIO_PIN_11
#define CEL_MAIN_DTR_GPIO_Port GPIOA
#define LED_GR_Pin GPIO_PIN_5
#define LED_GR_GPIO_Port GPIOE
#define BLE_RST_Pin GPIO_PIN_0
#define BLE_RST_GPIO_Port GPIOE
#define CEL_NET_STATUS_Pin GPIO_PIN_10
#define CEL_NET_STATUS_GPIO_Port GPIOA
#define CE_PON_TRG_Pin GPIO_PIN_8
#define CE_PON_TRG_GPIO_Port GPIOA
#define CEL_USB_BOOT_Pin GPIO_PIN_9
#define CEL_USB_BOOT_GPIO_Port GPIOC
#define CEL_MAIN_RI_Pin GPIO_PIN_6
#define CEL_MAIN_RI_GPIO_Port GPIOC
#define CEL_MAIN_DCD_Pin GPIO_PIN_14
#define CEL_MAIN_DCD_GPIO_Port GPIOD
#define CEL_PSM_Pin GPIO_PIN_13
#define CEL_PSM_GPIO_Port GPIOD
#define CEL_AP_RDY_Pin GPIO_PIN_1
#define CEL_AP_RDY_GPIO_Port GPIOC
#define CEL_PWR_KEY_Pin GPIO_PIN_2
#define CEL_PWR_KEY_GPIO_Port GPIOC
#define CEL_STATUS_Pin GPIO_PIN_10
#define CEL_STATUS_GPIO_Port GPIOD
#define GNSS_PWR_EN_Pin GPIO_PIN_5
#define GNSS_PWR_EN_GPIO_Port GPIOC
#define TOF_P_EN_Pin GPIO_PIN_1
#define TOF_P_EN_GPIO_Port GPIOB
#define CEL_PWR_EN_Pin GPIO_PIN_11
#define CEL_PWR_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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
#include "stm32h5xx_hal.h"

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
#define ACC_Staging_Pin GPIO_PIN_13
#define ACC_Staging_GPIO_Port GPIOC
#define CS_Pin GPIO_PIN_3
#define CS_GPIO_Port GPIOC
#define HVIL_In_Pin GPIO_PIN_1
#define HVIL_In_GPIO_Port GPIOA
#define HVIL_Out_Pin GPIO_PIN_2
#define HVIL_Out_GPIO_Port GPIOA
#define HV_Staging_Pin GPIO_PIN_3
#define HV_Staging_GPIO_Port GPIOA
#define LVGPIO1_Pin GPIO_PIN_4
#define LVGPIO1_GPIO_Port GPIOA
#define LVGPIO2_Pin GPIO_PIN_5
#define LVGPIO2_GPIO_Port GPIOA
#define LVGPIO3_Pin GPIO_PIN_6
#define LVGPIO3_GPIO_Port GPIOA
#define LVGPIO4_Pin GPIO_PIN_7
#define LVGPIO4_GPIO_Port GPIOA
#define SW5V_2_Pin GPIO_PIN_4
#define SW5V_2_GPIO_Port GPIOC
#define SW5V_1_Pin GPIO_PIN_5
#define SW5V_1_GPIO_Port GPIOC
#define SW12V_1_Pin GPIO_PIN_0
#define SW12V_1_GPIO_Port GPIOB
#define SW12V_2_Pin GPIO_PIN_1
#define SW12V_2_GPIO_Port GPIOB
#define PCAN_FLT_Pin GPIO_PIN_10
#define PCAN_FLT_GPIO_Port GPIOB
#define LED_1_Ctrl_Pin GPIO_PIN_6
#define LED_1_Ctrl_GPIO_Port GPIOC
#define LED_2_Ctrl_Pin GPIO_PIN_7
#define LED_2_Ctrl_GPIO_Port GPIOC
#define USB_VSENSE_Pin GPIO_PIN_8
#define USB_VSENSE_GPIO_Port GPIOC
#define LED_3_Ctrl_Pin GPIO_PIN_8
#define LED_3_Ctrl_GPIO_Port GPIOA
#define LED_4_Ctrl_Pin GPIO_PIN_9
#define LED_4_Ctrl_GPIO_Port GPIOA
#define SDMMC1_CD_Pin GPIO_PIN_2
#define SDMMC1_CD_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_4
#define IMU_INT1_GPIO_Port GPIOB
#define IMU_INT2_Pin GPIO_PIN_5
#define IMU_INT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

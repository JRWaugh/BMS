/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum {
    Success = 0, Fail
};
/* USER CODE BEGIN PV */
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
#define Led0_Pin GPIO_PIN_0
#define Led0_GPIO_Port GPIOC
#define Led1_Pin GPIO_PIN_1
#define Led1_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_2
#define Led2_GPIO_Port GPIOC
#define Led3_Pin GPIO_PIN_3
#define Led3_GPIO_Port GPIOC
#define IO_0_Pin GPIO_PIN_7
#define IO_0_GPIO_Port GPIOA
#define IO_1_Pin GPIO_PIN_4
#define IO_1_GPIO_Port GPIOC
#define IO_2_Pin GPIO_PIN_5
#define IO_2_GPIO_Port GPIOC
#define PreCharge_Pin GPIO_PIN_0
#define PreCharge_GPIO_Port GPIOB
#define BMSrelay_Pin GPIO_PIN_1
#define BMSrelay_GPIO_Port GPIOB
#define FansPWM_Pin GPIO_PIN_2
#define FansPWM_GPIO_Port GPIOB
#define Det_Pin GPIO_PIN_8
#define Det_GPIO_Port GPIOA
#define Lock_Pin GPIO_PIN_9
#define Lock_GPIO_Port GPIOA
#define Det_Lock_Pin GPIO_PIN_10
#define Det_Lock_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SOS_Pin GPIO_PIN_9
#define SOS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/*** Test enable/disable ***/
#define CAN_DEBUG                           1
#define CAN_ENABLED                         1
#define SD_CARD_DEBUG                       1
#define FAN_DEBUG                           1
#define TEST_OVERVOLTAGE                    1
#define TEST_UNDERVOLTAGE                   1
#define TEST_OVERTEMPERATURE                1
#define TEST_UNDERTEMPERATURE               1
#define TEST_OVERPOWER                      1
#define TEST_ACCU_UNDERVOLTAGE              1 // This is for testing undervoltage with IVT
#define CHECK_IVT                           1 // To completely disable IVT TEST_ACCU_UNDERVOLTAGE this needs to be set to 0
#define TEST_OVERTEMPERATURE_CHARGING       1
#define TEST_OVERCURRENT                    1
#define IVT_TIMEOUT                         1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

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
#define CAN_ID_UPTIME		8
#define CAN_ID_OPMODE		9
#define CAN_ID_ERROR		10 // might change to Data which is basically volt_total with temp data jammed in // voltage data 0 - 3, min 4, min 5, temp_max byte 6
#define CAN_ID_TMP_TESTING	77

#define CAN_ID_IVT_I		1313
#define CAN_ID_IVT_U1		1314
#define CAN_ID_IVT_U2		1315
#define CAN_ID_IVT_U3		1315

#define CAN_ID_NLGA_STAT	1552
#define CAN_ID_NLGA_CTRL	1560
#define CAN_ID_NLGB_STAT	1568
#define CAN_ID_NLGB_CTRL	1576

#define CANID_FIRST			1900
#define CAN_ID_SETTING		1902
#define CAN_ID_DISHB		1909

#define CAN_ID_LOGGER_REQ	1972
#define CAN_ID_LOGGER_RESP	1973
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE BEGIN PV */
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

/*** BMS.H function prototypes ***/
int8_t 	core_routine();
void 	datalog_routine(void);
void    set_fan_duty_cycle(uint8_t dc, uint8_t manual_mode_bit);
void    SetCharger(void);
void 	fan_energize(void);
int32_t can0_test(void);
int8_t  CANTxVoltage(void);
int8_t CANTxVoltageLimpTotal(void);
int32_t CANTxUptime(void);
int32_t CanTxOpMode(void);
int32_t CanTxError(void);
int8_t  CANTxTemperature();
int32_t CANTxDCfg(void);
int32_t CANTxNLGAControl(void);
int32_t CANTxNLGBControl(void);
int32_t CANTxVolumeSize(uint32_t size_of_log);
void canresp_get_volume_size(void);
void canresp_delete_logfile(void);
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
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

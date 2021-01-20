#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
enum CAN0_ID {
    TMPTesting = 77,
    DateTime = 0x7B,
    IVT_I = 0x521, IVT_U1, IVT_U2, IVT_U3, IVT_T, IVT_W, IVT_As // From https://www.isabellenhuette.de/fileadmin/Daten/Praezisionsmesstechnik/Datasheet_IVT-S.pdf
};
enum CAN1_ID {
    OpMode = 8, PEC_Error, Data, VoltTotal,
    NLGAStat = 1552,
    NLGACtrl = 1560,
    NLGBStat = 1568,
    NLGBCtrl = 1576,
    Setting = 1902,
    DishB = 1909,
    Volt = 1912,
    Temp = 1948,
    LoggerReq = 1972, LoggerResp
};

// Behaviour when updating discharge config
enum DischargeMode { GTMinPlusDelta, MaxOnly, GTMeanPlusDelta };

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

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
#define NSS_Pin GPIO_PIN_6
#define NSS_GPIO_Port GPIOB
#define SOS_Pin GPIO_PIN_9
#define SOS_GPIO_Port GPIOB

/* * * Debug functionality enable/disable * * */
#define BMS_RELAY_CTRL_BYPASS               0
#define STOP_CORE_ON_SAFE_STATE             1
#define START_DEBUG_ON_SAFE_STATE           1
#define BYPASS_INITIAL_CHECK                1
#define SKIP_PEC_ERROR_ACTIONS              1
#define CAN_DEBUG                           1
#define CAN_ENABLED                         1
#define SD_CARD_DEBUG                       1
#define FAN_DEBUG                           1
#define TEST_OVERVOLTAGE                    1
#define TEST_UNDERVOLTAGE                   1
#define TEST_OVERTEMPERATURE                1
#define TEST_UNDERTEMPERATURE               1
#define TEST_OVERPOWER                      1
#define TEST_ACCU_UNDERVOLTAGE              1
#define CHECK_IVT                           1
#define TEST_OVERTEMPERATURE_CHARGING       1
#define TEST_OVERCURRENT                    1
#define IVT_TIMEOUT                         1

#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

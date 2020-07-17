/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include "fatfs.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IVT.h"
#include "Status.h"
#include "NLG5.h"
#include "LTC6811.h"
#include "PWM_Fan.h"
#include "DWTWrapper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
class RTClock {
public:
    [[nodiscard]] static RTClock& getInstance() noexcept {
        static RTClock rtc;
        return rtc;
    }

    volatile uint8_t year{ 0 };
    volatile uint8_t month{ 0 };
    volatile uint8_t days{ 0 };
    volatile uint8_t hours{ 0 };
    volatile uint8_t minutes{ 0 };
    volatile uint8_t seconds{ 0 };

    void tick() noexcept {
        if (++seconds >= 60) {
            seconds = 0;
            if (++minutes >= 60) {
                minutes = 0;
                if (++hours >= 24) {
                    hours = 0;
                }
            }
        }
    }

    RTClock(RTClock const&)       = delete;
    void operator=(RTClock const&)   = delete;

private:
    constexpr RTClock() {};
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static constexpr auto kFile = "/hpf20/data.txt";
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1; // CAN1
CAN_HandleTypeDef hcan2; // CAN0. Confusing, I know! Blame the electrical boys.
SD_HandleTypeDef hsd;
SPI_HandleTypeDef hspi1;


/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;
CAN_TxHeaderTypeDef TxHeader{ 0, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE };
NLG5* nlg5{ nullptr };
Status* status{ nullptr };
LTC6811* ltc6811{ nullptr };
IVT* ivt{ nullptr };
PWM_Fan* pwm_fan{ nullptr };
RTClock& rtc = RTClock::getInstance();
uint32_t mailbox{ 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM2_Init(void);
uint32_t CANTxTest(void);
uint32_t CANTxData(uint16_t const v_min, uint16_t const v_max, int16_t const t_max);
uint32_t CANTxVoltageLimpTotal(uint32_t const sum_of_cells, bool const limping);
uint32_t CANTxDCCfg(LTC6811::RegisterGroup<uint8_t> const& slave_cfg_rx);
uint32_t CANTxVoltage(std::array<LTC6811::RegisterGroup<uint16_t>, 4> const& cell_data);
uint32_t CANTxTemperature(std::array<LTC6811::RegisterGroup<int16_t>, 2> const& temp_data);
uint32_t CANTxStatus(void);
uint32_t CANTxPECError(void);
uint32_t CANTxVolumeSize(uint32_t const size_of_log);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" { void HAL_IncTick(void) {
    uwTick += uwTickFreq;

    if (status != nullptr) {
        status->tick();

        if (nlg5 != nullptr && status->getOpMode() & Status::Charging)
            nlg5->tick();
    }

    if (ivt != nullptr)
        ivt->tick();

    rtc.tick();
}}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */


    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_FATFS_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_SDIO_SD_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

    nlg5 = new NLG5(hcan1, TxHeader);
    ivt = new IVT;
    status = new Status(Status::Core | Status::Logging);
    ltc6811 = new LTC6811(hspi1);
    pwm_fan = new PWM_Fan;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
#if BYPASS_INITIAL_CHECK
    status->setAIRState(Closed);
#endif
    HAL_Delay(5000);
#if BYPASS_INITIAL_CHECK
    status->setPrechargeState(Closed);
#endif

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_GPIO_TogglePin(Led0_GPIO_Port, Led0_Pin);

        auto const op_mode = status->getOpMode();

        /*  Core routine for monitoring voltage and temperature of the cells.  */
        if (op_mode & Status::Core) {
            auto const voltage_status = ltc6811->checkVoltageStatus();
            auto const temp_status = ltc6811->checkTemperatureStatus();

            if (!status->isError(Status::PECError, !voltage_status) && !status->isError(Status::PECError, !temp_status)) {
                status->isError(Status::Limping, voltage_status->min < Status::kLimpMinVoltage);
                nlg5->setChargeCurrent(voltage_status->max);

                if (pwm_fan->getMode() == PWM_Fan::Automatic)
                    pwm_fan->setDutyCycle(PWM_Fan::calcDutyCycle(temp_status->max));


                if (op_mode & Status::Balance)
                    ltc6811->writeConfigRegisterGroup(ltc6811->makeDischargeConfig(*voltage_status));

#if CHECK_IVT
                if (!ivt->isLost()) { // This, if anything, will be the cause of error false positives
                    switch (ivt->comparePrecharge(voltage_status->sum)) {
                    case IVT::Charged:
                        status->setPrechargeState(Closed);
                        break;

                    case IVT::NotCharged:
                        status->setPrechargeState(Open);
                        break;

                    case IVT::Hysteresis:
                        // Do nothing
                        break;

                    default:
                        break;
                    }
                }
#endif

                if ( // NOTE: Bitwise & will not short circuit like Logical &&. We want all isError() calls to happen, so do not replace & with &&.
#if CHECK_IVT
#if IVT_TIMEOUT
                        !status->isError(Status::IVTLost, ivt->isLost()) &
#endif
#if TEST_OVERPOWER
                        !status->isError(Status::OverPower, voltage_status->sum * ivt->getCurrent() > Status::kMaxPower) &
#endif
#if TEST_OVERCURRENT
                        !status->isError(Status::OverCurrent, ivt->getCurrent() > Status::kMaxCurrent) &
#endif
#if TEST_ACCU_UNDERVOLTAGE
                        !status->isError(Status::AccuUnderVoltage, ivt->getVoltage2() < Status::kAccuMinVoltage) &
#endif
#endif
#if TEST_UNDERVOLTAGE
                        !status->isError(Status::UnderVoltage, voltage_status->min < Status::kMinVoltage) &
#endif
#if TEST_OVERVOLTAGE
                        !status->isError(Status::OverVoltage, voltage_status->max > Status::kMaxVoltage) &
#endif
#if TEST_UNDERTEMPERATURE
                        !status->isError(Status::UnderTemp, temp_status->min < Status::kMinTemp) &
#endif
#if TEST_OVERTEMPERATURE
                        !status->isError(Status::OverTemp, temp_status->max > Status::kMaxTemp) &
#endif
#if TEST_OVERTEMPERATURE_CHARGING
                        !status->isError(Status::OverTempCharging, (op_mode & Status::Charging) && (temp_status->max > Status::kMaxChargeTemp)) &
#endif
                        true
                ) {
                    status->setAIRState(Closed);
#if !CHECK_IVT
                    status->setPrechargeState(Closed);
#endif
                }
#if CAN_ENABLED
                CANTxData(voltage_status->min, voltage_status->max, temp_status->max);
                CANTxVoltageLimpTotal(voltage_status->sum, status->isErrorOverLimit(Status::Limping));
#endif
            }
#if CAN_ENABLED
            CANTxStatus();
            CANTxPECError();
        }

#if CAN_DEBUG
        /*  Functions for debugging and untested code.  */
        if (op_mode & Status::Debug) {
            CANTxVoltage(ltc6811->getCellData());
            CANTxTemperature(ltc6811->getTempData());
            CANTxDCCfg(ltc6811->getSlaveCfg());
        }
#endif
#endif

        /*  Log data to SD card.  */
        if (op_mode & Status::Logging) {
            if (retSD == FR_OK) {
                if (f_size(&SDFile) < 524288000 && f_open(&SDFile, kFile, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
                    HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin);

                    /* NOTE: f_printf might be pretty slow compared to f_write. */
                    f_printf(&SDFile, "%u,", status->getUptime());
                    /* ISO 8601 Notation (yyyy-mm-ddThh:mm:ss) */
                    // TODO Not implemented.
                    f_printf(&SDFile, "%02u-%02u-%02uT%02u:%02u:%02u,",
                            status->rtc.tm_year, status->rtc.tm_mon, status->rtc.tm_mday, status->rtc.tm_hour, status->rtc.tm_min, status->rtc.tm_sec);

                    UINT number_written{ 0 };
                    uint16_t buffer[4 * LTC6811::kDaisyChainLength * 3]{ 0 };
                    size_t position{ 0 };

                    auto const cell_data = ltc6811->getCellData();
                    for (auto const& register_group : cell_data) // 4 voltage register groups
                        for (auto const& IC : register_group) // N ICs in daisy chain, determined by kDaisyChainLength
                            for (auto const voltage : IC.data) // 3 voltages in IC.data
                                buffer[position++] = voltage;
                    f_write(&SDFile, buffer, sizeof(buffer), &number_written);

                    position = 0;

                    auto const temp_data = ltc6811->getTempData();
                    for (auto const& register_group : temp_data) // 2 temperature register groups
                        for (auto const& IC : register_group) // N ICs in daisy chain, determined by kDaisyChainLength
                            for (auto const temperature : IC.data) // 3 temperatures in IC.data
                                buffer[position++] = temperature;
                    f_write(&SDFile, buffer, sizeof(buffer) / 2, &number_written);

                    f_sync(&SDFile);
                    f_close(&SDFile);
                }
            }
        }
    }
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 50;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
    PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void)
{

    /* USER CODE BEGIN CAN1_Init 0 */

    /* USER CODE END CAN1_Init 0 */

    /* USER CODE BEGIN CAN1_Init 1 */


    /* USER CODE END CAN1_Init 1 */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 16;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
        Error_Handler();
    /* USER CODE BEGIN CAN1_Init 2 */
    CAN_FilterTypeDef  sFilterConfig;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // allows 4 IDs to be set to one filter with IDLIST
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterIdHigh = Setting << 5;
    sFilterConfig.FilterIdLow = NLGAStat << 5;
    sFilterConfig.FilterMaskIdHigh = NLGBStat << 5;
    sFilterConfig.FilterMaskIdLow = LoggerReq << 5;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
    /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

    /* USER CODE BEGIN CAN2_Init 0 */
    /* USER CODE END CAN2_Init 0 */

    /* USER CODE BEGIN CAN2_Init 1 */

    /* USER CODE END CAN2_Init 1 */
    hcan2.Instance = CAN2;
    hcan2.Init.Prescaler = 16;
    hcan2.Init.Mode = CAN_MODE_NORMAL;
    hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan2.Init.TimeTriggeredMode = DISABLE;
    hcan2.Init.AutoBusOff = DISABLE;
    hcan2.Init.AutoWakeUp = DISABLE;
    hcan2.Init.AutoRetransmission = DISABLE;
    hcan2.Init.ReceiveFifoLocked = DISABLE;
    hcan2.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN2_Init 2 */
    CAN_FilterTypeDef  sFilterConfig;
    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // allows two IDs to be set to one filter with IDLIST
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.SlaveStartFilterBank = 14;

    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterIdHigh = IVT_I << 5;
    sFilterConfig.FilterIdLow = IVT_U1 << 5;
    sFilterConfig.FilterMaskIdHigh = IVT_U2 << 5;
    sFilterConfig.FilterMaskIdLow = IVT_U3 << 5;
    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    /* USER CODE END CAN2_Init 2 */

}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{

    /* USER CODE BEGIN SDIO_Init 0 */

    /* USER CODE END SDIO_Init 0 */

    /* USER CODE BEGIN SDIO_Init 1 */

    /* USER CODE END SDIO_Init 1 */
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 0;
    /* USER CODE BEGIN SDIO_Init 2 */

    /* USER CODE END SDIO_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 1;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    // In the code below, prescaler is 800 as 16MHz / 800 == 20kHz.

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 800;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
    HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, Led0_Pin|Led1_Pin|Led2_Pin|Led3_Pin
            |IO_1_Pin|IO_2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(IO_0_GPIO_Port, IO_0_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, PreCharge_Pin|BMSrelay_Pin|NSS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : Led0_Pin Led1_Pin Led2_Pin Led3_Pin
                           IO_1_Pin IO_2_Pin */
    GPIO_InitStruct.Pin = Led0_Pin|Led1_Pin|Led2_Pin|Led3_Pin
            |IO_1_Pin|IO_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : IO_0_Pin */
    GPIO_InitStruct.Pin = IO_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IO_0_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PreCharge_Pin BMSrelay_Pin NSS_Pin */
    GPIO_InitStruct.Pin = PreCharge_Pin|BMSrelay_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NSS_Pin;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : Det_Pin Lock_Pin Det_Lock_Pin */
    GPIO_InitStruct.Pin = Det_Pin|Lock_Pin|Det_Lock_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : SOS_Pin */
    GPIO_InitStruct.Pin = SOS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SOS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// CAN0 / CAN2
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8]{ 0 };

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
        switch(RxHeader.StdId) {
        case IVT_I:
            ivt->setCurrent(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f);
            break;

        case IVT_U1:
            ivt->setVoltage1(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f);
            break;

        case IVT_U2:
            ivt->setVoltage2(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f);
            break;

        default:
            break;
        }
    }
}

// CAN1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8]{ 0 };

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, data) == HAL_OK) {
        switch(RxHeader.StdId) {
        case DateTime:
            rtc.year = data[0] + 2000;
            rtc.month = data[1];
            rtc.days = data[2];
            rtc.hours = data[3];
            rtc.minutes = data[4];
            rtc.seconds = data[5];
            break;

        case NLGAStat: // possible the order of this array is backwards
            nlg5->a_buffer[0] = data[0];
            nlg5->a_buffer[1] = data[1];
            nlg5->a_buffer[2] = data[2];
            nlg5->a_buffer[3] = data[3];
            break;

        case NLGBStat:
            nlg5->b_buffer[0] = data[0];
            nlg5->b_buffer[1] = data[1];
            nlg5->b_buffer[2] = data[2];
            nlg5->b_buffer[3] = data[3];
            break;

        case LoggerReq: {
            // NOTE: It's possible the switch should happen on data[0] if I got the endianness wrong.
            switch (data[3]) {
            case 0:
                /* Call appropriate logger function */
                break;

            case 1:
                if (retSD == FR_OK)
                    CANTxVolumeSize(f_size(&SDFile));
                else
                    CANTxVolumeSize(0);
                break;

            case 2:
                if (retSD == FR_OK)
                    f_unlink(kFile);
                break;

            case 3:
                // TODO Not implemented. Don't know how to implement yet. Also Case 0-3 should have names.
                // rtc_set_date_time(data);
                break;

            default:
                break;
            }
            break;
        }

        case Setting:
            status->setOpMode(data[2]);
            ltc6811->setDischargeMode(static_cast<LTC6811::DischargeMode>(data[3]));
            nlg5->oc_limit = data[6];
            pwm_fan->setMode(static_cast<PWM_Fan::Mode>(data[7] & 0x80));

            if (pwm_fan->getMode() == PWM_Fan::Manual)
                pwm_fan->setDutyCycle(data[7]);

            break;

        default:
            break;
        }
    }
}

uint32_t CANTxTest(void) {
    TxHeader.StdId = TMPTesting;
    TxHeader.DLC = 8;

    uint8_t data[]{ 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &mailbox) == HAL_OK)
        return Success;
    else
        return Fail;
}

uint32_t CANTxStatus(void) {
    TxHeader.StdId = OpMode;
    TxHeader.DLC = 8;

    uint32_t const uptime = status->getUptime();

    uint8_t data[] = {
            static_cast<uint8_t>(uptime >> 24),
            static_cast<uint8_t>(uptime >> 16),
            static_cast<uint8_t>(uptime >>  8),
            static_cast<uint8_t>(uptime >>  0),
            status->getOpMode(),
            status->getLastError(),
            status->getPrechargeState(),
            status->getAIRState()
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) == HAL_OK)
        return Success;
    else
        return Fail;
}

uint32_t CANTxPECError(void) {
    TxHeader.StdId = PECError;
    TxHeader.DLC = 8;

    static uint32_t last_error;

    uint32_t const total_error{ status->getErrorCount(Status::PECError) };
    uint32_t const error_change = total_error - last_error;
    uint8_t data[] = {
            static_cast<uint8_t>(total_error >> 24),
            static_cast<uint8_t>(total_error >> 16),
            static_cast<uint8_t>(total_error >>  8),
            static_cast<uint8_t>(total_error >>  0),
            static_cast<uint8_t>(error_change >> 24),
            static_cast<uint8_t>(error_change >> 16),
            static_cast<uint8_t>(error_change >>  8),
            static_cast<uint8_t>(error_change >>  0),
    };

    last_error = total_error;

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) == HAL_OK)
        return Success;
    else
        return Fail;
}

uint32_t CANTxData(uint16_t const v_min, uint16_t const v_max, int16_t const t_max) {
    TxHeader.StdId = Data;
    TxHeader.DLC = 8;

    uint16_t U1 = static_cast<uint16_t>(ivt->getVoltage1()); // TODO this is bad
    uint8_t data[] = {
            static_cast<uint8_t>(U1 >> 8),
            static_cast<uint8_t>(U1 >> 0),
            static_cast<uint8_t>(v_min >> 8),
            static_cast<uint8_t>(v_min >> 0),
            static_cast<uint8_t>(v_max >> 8),
            static_cast<uint8_t>(v_max >> 0),
            static_cast<uint8_t>(t_max >> 8),
            static_cast<uint8_t>(t_max >> 0)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) == HAL_OK)
        return Success;
    else
        return Fail;

}

uint32_t CANTxVoltage(std::array<LTC6811::RegisterGroup<uint16_t>, 4> const& cell_data) {
    TxHeader.StdId = Volt;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    uint8_t byte_position{ 0 };

    for (size_t current_ic = 0; current_ic < LTC6811::kDaisyChainLength; ++current_ic) {
        for (const auto& register_group : cell_data) { // 4 voltage register groups
            for (const auto voltage : register_group[current_ic].data) { // 3 voltages per IC
                data[byte_position++] = static_cast<uint8_t>(voltage >> 8);
                data[byte_position++] = static_cast<uint8_t>(voltage);

                if (byte_position == 8) {
                    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) != HAL_OK)
                        return Fail;

                    byte_position = 0;
                    ++TxHeader.StdId;
                }
            } // 4 * 3 == 12 voltages associated with each LTC6811 in the daisy chain
        }
    } // 4 * 3 * kDaisyChainLength == all voltages associated with the daisy chain
    return Success;
}

uint32_t CANTxTemperature(std::array<LTC6811::RegisterGroup<int16_t>, 2> const& temp_data) {
    TxHeader.StdId = Temp;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    uint8_t byte_position{ 0 };

    for (size_t current_ic = 0; current_ic < LTC6811::kDaisyChainLength; ++current_ic) {
        for (const auto& register_group : temp_data) { // 2 voltage register groups
            for (const auto temperature : register_group[current_ic].data) { // 3 temperatures per IC
                data[byte_position++] = static_cast<uint8_t>(temperature >> 8);
                data[byte_position++] = static_cast<uint8_t>(temperature);

                if (byte_position == 8) {
                    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) != HAL_OK)
                        return Fail;

                    byte_position = 0;
                    ++TxHeader.StdId;
                }
            } // 2 * 3 == 6 temperatures associated with each LTC6811 in the daisy chain
        }
    } // 2 * 3 * kDaisyChainLength == all temperatures associated with the daisy chain

    return Success;
}

uint32_t CANTxVoltageLimpTotal(uint32_t const sum_of_cells, bool const limping) {
    TxHeader.StdId = VoltTotal;
    TxHeader.DLC = 8;

    uint8_t data[8] {
        static_cast<uint8_t>(sum_of_cells >> 24),
                static_cast<uint8_t>(sum_of_cells >> 16),
                static_cast<uint8_t>(sum_of_cells >>  8),
                static_cast<uint8_t>(sum_of_cells >>  0),
                limping,
                0x0,
                0x0,
                0x0
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) == HAL_OK)
        return Success;
    else
        return Fail;
}

/* Put discharge flag data on CAN bus. */
uint32_t CANTxDCCfg(LTC6811::RegisterGroup<uint8_t> const& slave_cfg_rx) {
    TxHeader.StdId = DishB;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    uint8_t byte_position{ 0 };

    for (const auto& IC : slave_cfg_rx) {
        data[byte_position++] = IC.data[5];
        data[byte_position++] = IC.data[4];

        if (byte_position == 8) {
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) != HAL_OK)
                return Fail;

            byte_position = 0;
            ++TxHeader.StdId;
        }
    }

    return Success;
}

uint32_t CANTxVolumeSize(uint32_t const size_of_log) {
    TxHeader.StdId = LoggerResp;
    TxHeader.DLC = 4;

    uint8_t data[] = {
            static_cast<uint8_t>(size_of_log >> 24),
            static_cast<uint8_t>(size_of_log >> 16),
            static_cast<uint8_t>(size_of_log >>  8),
            static_cast<uint8_t>(size_of_log >>  0)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) == HAL_OK)
        return Success;
    else
        return Fail;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

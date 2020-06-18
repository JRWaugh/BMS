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
#include "Status.h"
#include "LTC6811.h"
#include "PWM_Fan.h"
#include <optional>
#include <atomic>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct IVT {
    enum { Charged, NotCharged, Hysteresis };
    std::atomic<float> U1; // Called pre in old code
    std::atomic<float> U2; // Called air_p in old code
    std::atomic<float> I;
    std::atomic<uint32_t> tick{ 0 }; // time in ms

    static constexpr uint32_t kMaxDelay{ 500 }; // time in ms
    static constexpr float kPrechargeMinStartVoltage{ 470.0f };
    static constexpr float kPrechargeMaxEndVoltage{ 450.0f };
    static constexpr uint8_t kHysteresis{ 10 };

    int prechargeCompare(uint32_t const sum_of_cells) const {
        float percentage = U1 * 100 / U2;
        float match_percentage = U2 * 100 / sum_of_cells - 100;
        bool voltage_match = match_percentage < kHysteresis && match_percentage > -kHysteresis;

        if (percentage >= 95 && voltage_match && U1 > kPrechargeMinStartVoltage && U2 > kPrechargeMinStartVoltage)
            return Charged;
        else if (U1 < kPrechargeMaxEndVoltage || U2 < kPrechargeMaxEndVoltage)
            return NotCharged;
        else
            return Hysteresis;
    }

    bool isLost() const {
        return tick > kMaxDelay;
    }
};

enum { Success, Fail };

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static constexpr auto kDirectory = "/hpf20";
static constexpr auto kFile = "/hpf20/data.txt";
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2; // CAN0
SD_HandleTypeDef hsd;
SPI_HandleTypeDef hspi1;


/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim2;
CAN_TxHeaderTypeDef TxHeader;
NLG5* nlg5{ nullptr };
Status* status{ nullptr };
LTC6811* ltc6811{ nullptr };
IVT* ivt{ nullptr };
PWM_Fan* pwm_fan{ nullptr };
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
uint32_t CANTxData(uint16_t const v_min, uint16_t const v_max, int16_t const t_max);
uint32_t CANTxVoltageLimpTotal(uint32_t sum_of_cells, bool limping);
uint32_t CANTxDCCfg(const LTC6811RegisterGroup<uint8_t>& slave_cfg_rx);
uint32_t CANTxVoltage(const std::array<LTC6811RegisterGroup<uint16_t>, 4>& cell_data);
uint32_t CANTxTemperature(const std::array<LTC6811RegisterGroup<int16_t>, 2>& temp_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern "C" { void HAL_IncTick(void) {
    uwTick += uwTickFreq;

    if (status != nullptr)
        ++status->tick;

    if (ivt != nullptr)
        ++ivt->tick;

    if (nlg5 != nullptr)
        ++nlg5->tick;
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
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_SDIO_SD_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_FATFS_Init();

    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    nlg5 = new NLG5;
    ivt = new IVT;
    status = new Status(Status::Core | Status::Logging);
    ltc6811 = new LTC6811(hspi1);
    pwm_fan = new PWM_Fan;
    f_mount(&SDFatFS, "", 0);
    f_open(&SDFile, kFile, FA_WRITE | FA_OPEN_APPEND);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
#if BYPASS_INITIAL_CHECK
    status->CloseAIR();
#endif
    HAL_Delay(5000);
#if BYPASS_INITIAL_CHECK
    status->ClosePre();
#endif

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_GPIO_TogglePin(Led0_GPIO_Port, Led0_Pin);

        if (status->op_mode & Status::Core) {
            auto const voltage_status = ltc6811->GetVoltageStatus();
            auto const temp_status = ltc6811->GetTemperatureStatus();

            if (!status->isError(Status::PECError, !voltage_status.has_value()) && !status->isError(Status::PECError, !temp_status.has_value())) {
                status->isError(Status::Limping, voltage_status.value().min < Status::kLimpMinVoltage);
                nlg5->SetChargeCurrent(voltage_status.value().max);
                pwm_fan->SetFanDutyCycle(pwm_fan->CalcDutyCycle(temp_status.value().max));
#if CHECK_IVT
                if (!ivt->isLost()) { // This, if anything, will be the cause of error false positives
                    switch (ivt->prechargeCompare(voltage_status.value().sum)) {
                    case IVT::Charged:
                        status->ClosePre();
                        break;

                    case IVT::NotCharged:
                        status->OpenPre();
                        break;

                    case IVT::Hysteresis: // Do nothing
                        break;

                    default:
                        break;
                    }
                }
#endif
                // NOTE: Bitwise & will not short circuit like Logical &&. We want all isError() calls to happen, so do not replace & with &&.
                if (
#if CHECK_IVT
#if IVT_TIMEOUT
                        !status->isError(Status::IVTLost, ivt->isLost()) &
#endif
#if TEST_OVERPOWER
                        !status->isError(Status::OverPower, voltage_status.value().sum * ivt->I > Status::kMaxPower) &
#endif
#if TEST_OVERCURRENT
                        !status->isError(Status::OverCurrent, ivt->I > Status::kMaxCurrent) &
#endif
#if TEST_ACCU_UNDERVOLTAGE
                        !status->isError(Status::AccuUnderVoltage, ivt->U2 < Status::kAccuMinVoltage) &
#endif
#endif
#if TEST_UNDERVOLTAGE
                        !status->isError(Status::UnderVoltage, voltage_status.value().min < Status::kMinVoltage) &
#endif
#if TEST_OVERVOLTAGE
                        !status->isError(Status::OverVoltage, voltage_status.value().max > Status::kMaxVoltage) &
#endif
#if TEST_UNDERTEMPERATURE
                        !status->isError(Status::UnderTemp, temp_status.value().min < Status::kMinTemp) &
#endif
#if TEST_OVERTEMPERATURE
                        !status->isError(Status::OverTemp, temp_status.value().max > Status::kMaxTemp) &
#endif
#if TEST_OVERTEMPERATURE_CHARGING
                        !status->isError(Status::OverTempCharging, (status->op_mode & Status::Charging) && (temp_status.value().max > Status::kMaxChargeTemp)) &
#endif
                        true
                ) {
                    status->CloseAIR();
#if !CHECK_IVT
                    status->ClosePre();
#endif
                }

                CANTxData(voltage_status.value().min, voltage_status.value().max, temp_status.value().max);
                CANTxVoltageLimpTotal(voltage_status.value().sum, true);
            }

            CanTxOpMode();
            CanTxError();


            if (status->op_mode & Status::Balance)
                ltc6811->BuildDischargeConfig(voltage_status.value());
        }

#if CAN_ENABLED
        /* Send charger command message on CAN bus.
         * Every fifth time the timeout occurs a reset command is sent if charger is in fault state.
         * Otherwise a charge command is sent. */
        if (status->op_mode & Status::Charging && nlg5->isChargerEvent()) {
            if ((nlg5->a_buffer[0] == 136 || nlg5->a_buffer[0] == 152) && (nlg5->b_buffer[0] == 136 || nlg5->b_buffer[0] == 152)) {
                // This condition is just here to avoid annoyingly long and obtuse boolean operations. This is the non-fault state.
            } else if (nlg5->event_counter++ > 4) {
                nlg5->ctrl = NLG5::C_C_EL;
                nlg5->event_counter = 0;
            } else
                nlg5->ctrl = NLG5::C_C_EN;

            CANTxNLGAControl();
            CANTxNLGBControl();
            nlg5->previous_tick = nlg5->tick.load();
        }
#endif

#if CAN_DEBUG
        /*  Functions for debugging and untested code. */
        if (status->op_mode & Status::Debug) {
            CANTxVoltage(ltc6811->GetCellData());
            CANTxTemperature(ltc6811->GetTempData());
            CANTxDCCfg(ltc6811->GetSlaveCfg());
            CanTxOpMode();
        }
#endif

        if (status->op_mode & Status::Logging) {
            FILINFO inf;
            if (BSP_SD_IsDetected()) {
                HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin);

                if (f_stat(kDirectory, &inf) == FR_NO_FILE)
                    f_mkdir(kDirectory);

                // Magic number below is probably 500MB
                if (f_size(&SDFile) < 524288000 && f_open(&SDFile, kFile, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
                    f_printf(&SDFile, "%u,", status->tick / 100); // TODO If the uptime number seems wrong, just change or remove the 100.
                    /* ISO 8601 Notation (yyyy-mm-ddThh:mm:ss) */
                    f_printf(&SDFile, "%02u-%02u-%02uT%02u:%02u:%02u,",
                            status->rtc.tm_year, status->rtc.tm_mon, status->rtc.tm_mday, status->rtc.tm_hour, status->rtc.tm_min, status->rtc.tm_sec);
                    // TODO Not implemented.

                    UINT number_written = 0;
                    uint8_t write_error{ 0 };

                    // TODO need to chop out the PEC data and write to file
#if 0
                    for (auto& reg : cell_data) {
                        auto serialized_reg = reinterpret_cast<uint8_t*>(reg.data.data()->data());

                        f_write(&SDFile, serialized_reg, kBytesPerRegister * kDaisyChainLength, &number_written);

                        if (number_written != kBytesPerRegister * kDaisyChainLength)
                            write_error = 1;
                    }

                    for (auto& reg : temp_data) {
                        auto serialized_reg = reinterpret_cast<uint8_t*>(reg.data.data()->data());

                        f_write(&SDFile, serialized_reg, kBytesPerRegister * kDaisyChainLength, &number_written);

                        if (number_written != kBytesPerRegister * kDaisyChainLength)
                            write_error = 1;
                    }
#endif
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
    CAN_FilterTypeDef  sFilterConfig;

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

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // allows two IDs to be set to one filter with IDLIST
    sFilterConfig.FilterIdHigh = 0x0000; // first ID
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000; //second ID
    sFilterConfig.FilterMaskIdLow = 0x0000; // don't think anything goes here
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }


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
    CAN_FilterTypeDef  sFilterConfig;
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
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // allows two IDs to be set to one filter with IDLIST
    sFilterConfig.FilterIdHigh = 0x0000; // first ID
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000; //second ID
    sFilterConfig.FilterMaskIdLow = 0x0000; // don't think anything goes here
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }
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
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT; // correct
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; // correct
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE; // I THINK this is now correct (was 0)
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
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
    /* In the code below, prescaler is 800 as 16MHz / 800 == 20kHz. */
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
            |IO_1_Pin|IO_2_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(IO_0_GPIO_Port, IO_0_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, PreCharge_Pin|BMSrelay_Pin, GPIO_PIN_RESET);

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

    /*Configure GPIO pins : PreCharge_Pin BMSrelay_Pin */
    GPIO_InitStruct.Pin = PreCharge_Pin|BMSrelay_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : FansPWM_Pin */
    GPIO_InitStruct.Pin = FansPWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(FansPWM_GPIO_Port, &GPIO_InitStruct);

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
    CAN_RxHeaderTypeDef   RxHeader;
    uint8_t data[8] = { 0 };

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
        switch(static_cast<CAN0_ID>(RxHeader.StdId)) {
        case IVT_I:
            ivt->I = static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f;
            ivt->tick = 0;
            break;

        case IVT_U1:
            ivt->U1 = static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f;
            ivt->tick = 0;
            break;

        case IVT_U2:
            ivt->U2 = static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f;
            ivt->tick = 0;
            break;


        default:
            break;
        }
    }
}

// CAN1
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef   RxHeader;
    uint8_t data[8] = { 0 };

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
        switch(static_cast<CAN1_ID>(RxHeader.StdId)) {
        case CAN1_ID::NLGAStat: // possible the order of this array is backwards
            nlg5->a_buffer[0] = data[0];
            nlg5->a_buffer[1] = data[1];
            nlg5->a_buffer[2] = data[2];
            nlg5->a_buffer[3] = data[3];
            break;

        case CAN1_ID::NLGBStat:
            nlg5->b_buffer[0] = data[0];
            nlg5->b_buffer[1] = data[1];
            nlg5->b_buffer[2] = data[2];
            nlg5->b_buffer[3] = data[3];
            break;

        case CAN1_ID::LoggerReq:
            // CANBus stuff is big endian so this should be right, but it's possible the switch should happen on data[0].
            switch (data[3]) {
            case 0:
                /* Call appropriate logger function */
                break;

            case 1:
                if (f_mount(&SDFatFS, "", 0) == FR_OK) {
                    CANTxVolumeSize(f_size(&SDFile));
                    f_mount(NULL, "", 0); /* Unmount */
                } else
                    CANTxVolumeSize(0);
                break;

            case 2:
                if (f_mount(&SDFatFS, "", 0) == FR_OK) {
                    f_unlink(kFile);
                    f_mount(NULL, "", 0); /* Unmount */
                }
                break;

            case 3:
                // TODO this didn't work on old BMS and will be different now. Worry about it at the office.
                // rtc_set_date_time(data);
                break;

            default:
                break;
            }
            break;

            case CAN1_ID::Setting:
                status->op_mode = data[2];
                ltc6811->SetDischargeMode(static_cast<LTC6811::DischargeMode>(data[3]));
                nlg5->oc_limit = data[6];
                pwm_fan->manual_mode = static_cast<bool>(data[7] & 0x80);
                pwm_fan->SetFanDutyCycle(data[7]);
                break;

            default:
                break;
        }
    }
}

uint32_t CAN0_Test(void) {
    TxHeader.StdId = CAN0_ID::TMPTesting;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

uint32_t CanTxOpMode(void) {
    TxHeader.StdId = CAN1_ID::OpMode;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint32_t uptime = status->tick / 100; // TODO petition to just give uptime in ms instead of ds

    uint8_t data[] = {
            static_cast<uint8_t>(uptime >> 24),
            static_cast<uint8_t>(uptime >> 16),
            static_cast<uint8_t>(uptime >>  8),
            static_cast<uint8_t>(uptime >>  0),
            status->op_mode,
            status->last_error,
            status->precharge_flag,
            status->AIR_flag
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return Fail;
    else
        return Success;
}

uint32_t CanTxError(void) {
    TxHeader.StdId = CAN1_ID::PECError;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    static uint32_t last_error;

    uint32_t total_error{ status->getPECError() };
    uint32_t error_change = total_error - last_error;
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

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return Fail;
    else
        return Success;
}

uint32_t CANTxData(uint16_t const v_min, uint16_t const v_max, int16_t const t_max) {
    TxHeader.StdId = CAN1_ID::Data;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint16_t U1 = static_cast<uint16_t>(ivt->U1); // TODO this is bad
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

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return Fail;
    else
        return Success;
}

uint32_t CANTxVoltage(const std::array<LTC6811RegisterGroup<uint16_t>, 4>& cell_data) {
    TxHeader.StdId = CAN1_ID::Volt;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    uint8_t byte_position{ 0 };

    for (size_t current_ic = 0; current_ic < kDaisyChainLength; ++current_ic) {
        for (const auto& register_group : cell_data) { // 4 voltage register groups
            for (const auto voltage : register_group.ICDaisyChain[current_ic].data) { // 3 voltages per IC
                data[byte_position++] = static_cast<uint8_t>(voltage >> 8);
                data[byte_position++] = static_cast<uint8_t>(voltage);

                if (byte_position == 8) {
                    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
                        return Fail;

                    byte_position = 0;
                    ++TxHeader.StdId;
                }
            } // 4 * 3 == 12 voltages associated with each LTC6811 in the daisy chain
        }
    } // 4 * 3 * kDaisyChainLength == all voltages associated with the daisy chain
    return Success;
}

uint32_t CANTxTemperature(const std::array<LTC6811RegisterGroup<int16_t>, 2>& temp_data) {
    TxHeader.StdId = CAN1_ID::Temp;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    uint8_t byte_position{ 0 };

    for (size_t current_ic = 0; current_ic < kDaisyChainLength; ++current_ic) {
        for (const auto& register_group : temp_data) { // 2 voltage register groups
            for (const auto temperature : register_group.ICDaisyChain[current_ic].data) { // 3 temperatures per IC
                data[byte_position++] = static_cast<uint8_t>(temperature >> 8);
                data[byte_position++] = static_cast<uint8_t>(temperature);

                if (byte_position == 8) {
                    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
                        return Fail;

                    byte_position = 0;
                    ++TxHeader.StdId;
                }
            } // 2 * 3 == 6 temperatures associated with each LTC6811 in the daisy chain
        }
    } // 2 * 3 * kDaisyChainLength == all temperatures associated with the daisy chain

    return Success;
}

uint32_t CANTxVoltageLimpTotal(uint32_t sum_of_cells, bool limping) {
    TxHeader.StdId = CAN1_ID::VoltTotal;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[8] = {
            // Swap endian-ness of SOC value
            static_cast<uint8_t>(sum_of_cells >> 24),
            static_cast<uint8_t>(sum_of_cells >> 16),
            static_cast<uint8_t>(sum_of_cells >>  8),
            static_cast<uint8_t>(sum_of_cells >>  0),
            limping,
            0x0,
            0x0,
            0x0
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return Fail;
    else
        return Success;
}

/* Put discharge flag data on CAN bus. */
uint32_t CANTxDCCfg(const LTC6811RegisterGroup<uint8_t>& slave_cfg_rx) {
    TxHeader.StdId = CAN1_ID::DishB;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    uint8_t byte_position{ 0 };

    for (auto& IC : slave_cfg_rx.ICDaisyChain) {
        data[byte_position++] = IC.data[5];
        data[byte_position++] = IC.data[4];

        if (byte_position == 8) {
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
                return Fail;

            byte_position = 0;
            ++TxHeader.StdId;
        }
    }

    return Success;
}

/* Checks specified chargers MOB status */
uint32_t CANTxNLGAControl(void) {
    TxHeader.StdId = CAN1_ID::NLGACtrl;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 7;

    uint8_t data[7]{
        nlg5->ctrl,
        static_cast<uint8_t>(nlg5->mc_limit >> 8),
        static_cast<uint8_t>(nlg5->mc_limit),
        static_cast<uint8_t>(nlg5->ov_limit >> 8),
        static_cast<uint8_t>(nlg5->ov_limit),
        static_cast<uint8_t>(nlg5->oc_limit >> 8),
        static_cast<uint8_t>(nlg5->oc_limit)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return Fail;
    else
        return Success;
}

// TODO This is exactly the same as the function above?
uint32_t CANTxNLGBControl(void) {
    TxHeader.StdId = CAN1_ID::NLGBCtrl;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 7;

    uint8_t data[7]{
        nlg5->ctrl,
        static_cast<uint8_t>(nlg5->mc_limit >> 8),
        static_cast<uint8_t>(nlg5->mc_limit),
        static_cast<uint8_t>(nlg5->ov_limit >> 8),
        static_cast<uint8_t>(nlg5->ov_limit),
        static_cast<uint8_t>(nlg5->oc_limit >> 8),
        static_cast<uint8_t>(nlg5->oc_limit)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return Fail;
    else
        return Success;
}

uint32_t CANTxVolumeSize(uint32_t const size_of_log) {
    TxHeader.StdId = CAN1_ID::LoggerResp;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4;

    uint8_t data[] = {
            static_cast<uint8_t>(size_of_log >> 24),
            static_cast<uint8_t>(size_of_log >> 16),
            static_cast<uint8_t>(size_of_log >>  8),
            static_cast<uint8_t>(size_of_log >>  0)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return Fail;
    else
        return Success;
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

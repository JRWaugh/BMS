/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <cmath>
#include "fatfs.h"
#include "IVT.h"
#include "NLG5.h"
#include "LTC6811.h"
#include "PWM_Fan.h"
#include "Status.h"
#include "DWTWrapper.h"

/* Private typedef -----------------------------------------------------------*/
struct RTClock {
    volatile uint16_t year{ 0 };
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
} rtc;

struct VoltageStatus {
    uint32_t sum{ 0 };
    uint16_t min{ std::numeric_limits<uint16_t>::max() };
    size_t min_id{ 0 };
    uint16_t max{ std::numeric_limits<uint16_t>::min() };
    size_t max_id{ 0 };
};

struct TempStatus {
    int16_t min{ std::numeric_limits<int16_t>::max() };
    size_t min_id{ 0 };
    int16_t max{ std::numeric_limits<int16_t>::min() };
    size_t max_id{ 0 };
};

static constexpr auto kFile = "/hpf20/data.txt";

CAN_HandleTypeDef hcan1; // CAN1
CAN_HandleTypeDef hcan2; // CAN0. Confusing, I know! Blame the electrical boys.
SD_HandleTypeDef hsd;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
CAN_TxHeaderTypeDef TxHeader{ 0, 0, CAN_ID_STD, CAN_RTR_DATA, 8, DISABLE };
NLG5* nlg5{ nullptr };
LTC6811* ltc6811;
PWM_Fan* pwm_fan{ nullptr };
IVT ivt;
uint32_t mailbox{ 0 };
Status status{ Core | Charging };
DischargeMode volatile discharge_mode{ GTMinPlusDelta };

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config();
static void MX_GPIO_Init();
static void MX_CAN1_Init();
static void MX_CAN2_Init();
static void MX_SDIO_SD_Init();
static void MX_SPI1_Init();
static void MX_TIM2_Init();
void balance_cells(LTC6811::CellData const &, VoltageStatus const &, DischargeMode) noexcept;
[[nodiscard]] std::optional<VoltageStatus> read_cell_data(LTC6811::CellData&) noexcept;
[[nodiscard]] std::optional<TempStatus> read_temp_data(LTC6811::TempData&) noexcept;
HAL_StatusTypeDef CANTxData(uint16_t const, uint16_t const, int16_t const);
HAL_StatusTypeDef CANTxVoltageLimpTotal(uint32_t const);
HAL_StatusTypeDef CANTxDCCfg();
HAL_StatusTypeDef CANTxVoltage(LTC6811::CellData const &);
HAL_StatusTypeDef CANTxTemperature(LTC6811::TempData const &);
HAL_StatusTypeDef CANTxStatus();
HAL_StatusTypeDef CANTxPECError();
HAL_StatusTypeDef CANTxVolumeSize(uint32_t const);
inline auto get_uptime() { return uwTick / 10; }

extern "C" { void HAL_IncTick() {
    uwTick += uwTickFreq;
    if (nlg5 != nullptr && status.get_op_mode() & Charging)
        nlg5->tick();
}}

int main() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    //MX_FATFS_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    //MX_SDIO_SD_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
    nlg5 = new NLG5(hcan1, TxHeader);
    pwm_fan = new PWM_Fan;

#if BYPASS_INITIAL_CHECK
    status.set_AIR_state(GPIO_PIN_SET);
    //HAL_Delay(5000);
    status.set_precharge_state(GPIO_PIN_SET);
#endif

    ltc6811 = new LTC6811{
        [](bool level) {
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, static_cast<GPIO_PinState>(level));
        },
        [](uint8_t const * tx_buffer, std::size_t size) {
            HAL_SPI_Transmit(&hspi1, tx_buffer, size, HAL_MAX_DELAY);
        },
        [](uint8_t* rx_buffer, std::size_t size) {
            HAL_SPI_Receive(&hspi1, rx_buffer, size, HAL_MAX_DELAY);
        }
    };

    LTC6811::RegisterGroup<uint8_t> default_config{ 0 };
    for (auto& IC : default_config)
        IC.data[0] = LTC6811::CFGR0;
    ltc6811->WRCFG(LTC6811::A, default_config);
    ltc6811->RDCFG(LTC6811::A, default_config);

    LTC6811::CellData cell_data{ 0 };
    LTC6811::TempData temp_data{ 0 };

    while (true) {
        HAL_GPIO_TogglePin(Led0_GPIO_Port, Led0_Pin);
        auto const op_mode = status.get_op_mode();

        /*  Core routine for monitoring voltage and temperature of the cells.  */
        if (op_mode & Core) {
            auto const voltage_status = read_cell_data(cell_data);
            auto const temp_status = read_temp_data(temp_data);
            // The boolean logic here is confusing, I know, but it's correct. The error handling needs a full rewrite.
            if (!status.is_error(PECError, !voltage_status.has_value()) &&
                    !status.is_error(PECError, !temp_status.has_value())) { // If no PEC errors and we have both statuses
                status.is_error(Limping, voltage_status->min < kLimpMinVoltage);
                nlg5->setChargeCurrent(voltage_status->max);

                if (op_mode & Balance)
                    balance_cells(cell_data, *voltage_status, discharge_mode);

                if (pwm_fan->getMode() == PWM_Fan::Automatic)
                    pwm_fan->calcDutyCycle(temp_status->max);

#if CHECK_IVT
                switch (ivt.compare_precharge(voltage_status->sum)) {
                case ivt.Charged:
                status.set_precharge_state(GPIO_PIN_SET);
                break;

                case ivt.NotCharged:
                status.set_precharge_state(GPIO_PIN_RESET);
                break;

                case ivt.Hysteresis:
                case ivt.Lost:
                default:
                    // Do nothing.
                    break;
                }
#endif

                if ( // NOTE: Bitwise & will not short circuit like Logical &&. We want all status.is_error() calls to happen, so do not replace & with &&.
#if CHECK_IVT
#if IVT_TIMEOUT
                        !status.is_error(IVTLost, ivt.is_lost()) &
#endif
#if TEST_OVERPOWER
                        !status.is_error(OverPower, voltage_status->sum * ivt.get_current() > kMaxPower) &
#endif
#if TEST_OVERCURRENT
                        !status.is_error(OverCurrent, ivt.get_current() > kMaxCurrent) &
#endif
#if TEST_ACCU_UNDERVOLTAGE
                        !status.is_error(AccuUnderVoltage, ivt.get_voltage2() < kAccuMinVoltage) &
#endif
#endif
#if TEST_UNDERVOLTAGE
                        !status.is_error(UnderVoltage, voltage_status->min < kMinVoltage) &
#endif
#if TEST_OVERVOLTAGE
                        !status.is_error(OverVoltage, voltage_status->max > kMaxVoltage) &
#endif
#if TEST_UNDERTEMPERATURE
                        !status.is_error(UnderTemp, temp_status->min < kMinTemp) &
#endif
#if TEST_OVERTEMPERATURE
                        !status.is_error(OverTemp, temp_status->max > kMaxTemp) &
#endif
#if TEST_OVERTEMPERATURE_CHARGING
                        !status.is_error(OverTempCharging, (op_mode & Charging) && (temp_status->max > kMaxChargeTemp)) &
#endif
                        true
                ) {
                    // If no errors occurred, including PEC errors, since that's the condition of entering this scope.
                    status.set_AIR_state(GPIO_PIN_SET);
#if !CHECK_IVT
                    status.set_precharge_state(GPIO_PIN_SET);
#endif
                }
#if CAN_ENABLED
                CANTxData(voltage_status->min, voltage_status->max, temp_status->max);
                CANTxVoltageLimpTotal(voltage_status->sum);
#endif
            }
#if CAN_ENABLED
            CANTxStatus();
            CANTxPECError();
        }

#if CAN_DEBUG
        /*  Functions for debugging and untested code.  */
        if (op_mode & Debug) {
            CANTxVoltage(cell_data);
            CANTxTemperature(temp_data);
            CANTxDCCfg();
        }
#endif
#endif
        /*  Log data to SD card.  */
        if (op_mode & Logging) {
            if (retSD == FR_OK) {
                if (f_size(&SDFile) < 524288000 && f_open(&SDFile, kFile, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
                    HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin);

                    /* NOTE: f_printf might be pretty slow compared to f_write. */
                    f_printf(&SDFile, "%u,", get_uptime());
                    /* ISO 8601 Notation (yyyy-mm-ddThh:mm:ss) */
                    // TODO Not implemented.
                    f_printf(&SDFile, "%04u-%02u-%02uT%02u:%02u:%02u\n", rtc.year, rtc.month, rtc.days, rtc.hours, rtc.minutes, rtc.seconds);

                    UINT number_written{ 0 };
                    uint16_t buffer[4 * LTC6811::kDaisyChainLength * 3]{ 0 };
                    size_t position{ 0 };

                    for (auto const& register_group : cell_data) // 4 voltage register groups
                        for (auto const& IC : register_group) // N ICs in daisy chain, determined by kDaisyChainLength
                            for (auto const voltage : IC.data) // 3 voltages in IC.data
                                buffer[position++] = voltage;
                    f_write(&SDFile, buffer, sizeof(buffer), &number_written);

                    position = 0;

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

static void SystemClock_Config() {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    // Initializes the CPU, AHB and APB busses clocks
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
        Error_Handler();

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
    PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        Error_Handler();
}

static void MX_CAN1_Init() {
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
    CAN_FilterTypeDef sFilterConfig = {
            .FilterIdHigh = Setting << 5,
            .FilterIdLow = NLGAStat << 5,
            .FilterMaskIdHigh = NLGBStat << 5,
            .FilterMaskIdLow = LoggerReq << 5,
            .FilterFIFOAssignment = CAN_RX_FIFO1,
            .FilterBank = 0,
            .FilterMode = CAN_FILTERMODE_IDLIST,
            .FilterScale = CAN_FILTERSCALE_16BIT,
            .FilterActivation = ENABLE,
            .SlaveStartFilterBank = 14
    };
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

static void MX_CAN2_Init() {
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
        Error_Handler();
    CAN_FilterTypeDef sFilterConfig = {
            .FilterIdHigh = IVT_I << 5,
            .FilterIdLow = IVT_U1 << 5,
            .FilterMaskIdHigh = IVT_U2 << 5,
            .FilterMaskIdLow = IVT_U3 << 5,
            .FilterFIFOAssignment = CAN_RX_FIFO0,
            .FilterBank = 14,
            .FilterMode = CAN_FILTERMODE_IDLIST,
            .FilterScale = CAN_FILTERSCALE_16BIT,
            .FilterActivation = ENABLE,
            .SlaveStartFilterBank = 14,
    };
    HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
}

static void MX_SDIO_SD_Init() {
    hsd.Instance = SDIO;
    hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = 0;
}

static void MX_SPI1_Init() {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 1;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        Error_Handler();
}

static void MX_TIM2_Init() {
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 800; // Prescaler is 800 as 16MHz / 800 == 20kHz, I guess?
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 19999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        Error_Handler();

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
        Error_Handler();

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        Error_Handler();

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
        Error_Handler();
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_GPIO_Init() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Levels */
    HAL_GPIO_WritePin(GPIOC, Led0_Pin|Led1_Pin|Led2_Pin|Led3_Pin|IO_1_Pin|IO_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IO_0_GPIO_Port, IO_0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, PreCharge_Pin|BMSrelay_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = Led0_Pin | Led1_Pin | Led2_Pin | Led3_Pin | IO_1_Pin | IO_2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = IO_0_Pin;
    HAL_GPIO_Init(IO_0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PreCharge_Pin | BMSrelay_Pin | NSS_Pin;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Det_Pin |Lock_Pin | Det_Lock_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SOS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(SOS_GPIO_Port, &GPIO_InitStruct);
}

void balance_cells(LTC6811::CellData const & cell_data, VoltageStatus const & voltage_status, DischargeMode discharge_mode) noexcept {
    static constexpr uint8_t kDelta{ 100 };
    uint16_t DCC{ 0 };
    uint8_t current_cell{ 0 }, current_ic{ LTC6811::kDaisyChainLength - 1 };
    LTC6811::RegisterGroup<uint8_t> config_register_group{ 0 };

    switch (discharge_mode) {
    case GTMinPlusDelta:
        for (auto& IC : config_register_group) { // 12 register groups
            DCC = 0;
            current_cell = 0;
            for (auto const& register_group : cell_data) { // 4 voltage register groups
                for (auto const voltage : register_group[current_ic].data) { // 3 voltages per IC
                    if (voltage > voltage_status.min + kDelta)
                        DCC |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }
            --current_ic;
            IC.data[4] = DCC >> 0 & 0xFF;
            IC.data[5] = DCC >> 8 & 0x0F;
        } // 12 * 12 = 144 voltages associated with the entire daisy chain
        break;

    case MaxOnly:
        if (voltage_status.max - voltage_status.min > kDelta) {
            current_ic = voltage_status.max_id / 3 % 12;
            DCC |= 1 << voltage_status.max_id % 11;
            config_register_group[current_ic].data[4] = DCC >> 0 & 0xFF;
            config_register_group[current_ic].data[5] = DCC >> 8 & 0x0F;
        }
        break;

    case GTMeanPlusDelta: {
        size_t const average_voltage{ voltage_status.sum / (4 * LTC6811::kDaisyChainLength * 3) };
        for (auto& IC : config_register_group) {
            DCC = 0;
            current_cell = 0;
            for (auto const& register_group : cell_data) { // 4 voltage register groups
                for (auto const voltage : register_group[current_ic].data) { // 3 voltages per IC
                    if (voltage > average_voltage + kDelta)
                        DCC |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }
            --current_ic;
            IC.data[4] = DCC >> 0 & 0xFF;
            IC.data[5] = DCC >> 8 & 0x0F;
        }
        break;
    }
    }

    ltc6811->WRCFG(LTC6811::A, config_register_group);
}

/* Read cell data and generate a status report of the cell voltage register groups.
 * Returns a VoltageStatus on success, nullopt if error. */
[[nodiscard]] std::optional<VoltageStatus> read_cell_data(LTC6811::CellData& cell_data) noexcept {
    VoltageStatus status;
    size_t count{ 0 };

    //WRCFG[0](config_register_group);
    //DWTWrapper::delay_us(500);
    //RDCFG[0](rd_cfgr_dump);
    LTC6811::RegisterGroup<uint8_t> config_register_group;
    ltc6811->ADCV();

    for (size_t i = 0; i < cell_data.size(); ++i)
        if (ltc6811->RDCV(static_cast<LTC6811::Group>(i), cell_data[i]) == true)
            return std::nullopt;

    for (const auto& register_group : cell_data) {
        for (const auto& IC : register_group) {
            for (const auto voltage : IC.data) {
                status.sum += voltage;
                if (voltage < status.min) {
                    status.min = voltage;
                    status.min_id = count;
                } else if (voltage > status.max) {
                    status.max = voltage;
                    status.max_id = count;
                }
                ++count;
            }
        }
    }
    status.sum /= 10000; // Convert centiDegC to DegC (with rounding errors, but this is what the old code did...)
    return status;
}

/* Read temp data and generate a status report of the current temperatures from aux voltage register groups.
 * Returns a TempStatus on success, nullopt if error. */
[[nodiscard]] std::optional<TempStatus> read_temp_data(LTC6811::TempData& temp_data) noexcept {
    static constexpr auto steinharthart = [](int16_t const NTC_voltage) noexcept {
        constexpr auto Vin = 30000.0f; // 3[V], or 30000[V * 10-5]
        constexpr auto KtoC = 27315; // centiKelvin to centiDegCelsius
        constexpr auto A = 0.003354016f;
        constexpr auto B = 0.000256524f;
        constexpr auto C = 0.00000260597f;
        constexpr auto D = 0.0000000632926f;
        auto log = -logf(Vin / NTC_voltage - 1);
        return static_cast<int16_t>(100.0f / (A + log * ( B + log * (C + D * log))) - KtoC);
    };

    TempStatus status;
    size_t count{ 0 };

    ltc6811->ADAX();

    for (size_t i = 0; i < temp_data.size(); ++i)
        if (ltc6811->RDAUX(static_cast<LTC6811::Group>(i), temp_data[i]) == true)
            return std::nullopt;

    for (auto& register_group : temp_data) {
        for (auto& IC : register_group) {
            for (auto& temperature : IC.data) {
                temperature = steinharthart(temperature);
                if (temperature < status.min) {
                    status.min = temperature;
                    status.min_id = count;
                } else if (temperature > status.max) {
                    status.max = temperature;
                    status.max_id = count;
                }
                ++count;
            }
        }
    }
    return status;
}

/* USER CODE BEGIN 4 */
// CAN0 / CAN2
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8]{ 0 };

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
        switch(RxHeader.StdId) {
        case IVT_I:
            ivt.set_current(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f);
            break;

        case IVT_U1:
            ivt.set_voltage1(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f);
            break;

        case IVT_U2:
            ivt.set_voltage2(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]) / 1000.0f);
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
            status.set_op_mode(static_cast<Op_Mode>(data[2]));
            discharge_mode = static_cast<DischargeMode>(data[3]);
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

HAL_StatusTypeDef CANTxStatus() {
    TxHeader.StdId = OpMode;
    TxHeader.DLC = 8;

    uint32_t const uptime = get_uptime();

    uint8_t data[] = {
            static_cast<uint8_t>(uptime >> 24),
            static_cast<uint8_t>(uptime >> 16),
            static_cast<uint8_t>(uptime >>  8),
            static_cast<uint8_t>(uptime >>  0),
            status.get_op_mode(),
            status.get_last_error(),
            status.get_precharge_state(),
            status.get_AIR_state()
    };

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox);
}

HAL_StatusTypeDef CANTxPECError() {
    TxHeader.StdId = PECError;
    TxHeader.DLC = 8;

    static uint32_t last_error{ 0 };

    uint32_t const total_error{ status.get_error_count(PECError) };
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

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox);
}

HAL_StatusTypeDef CANTxData(uint16_t const v_min, uint16_t const v_max, int16_t const t_max) {
    TxHeader.StdId = Data;
    TxHeader.DLC = 8;

    uint16_t U1 = static_cast<uint16_t>(ivt.get_voltage1()); // TODO this is bad
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
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox);
}

HAL_StatusTypeDef CANTxVoltage(LTC6811::CellData const & cell_data) {
    TxHeader.StdId = Volt;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    for (size_t current_ic = 0, byte_position = 0; current_ic < LTC6811::kDaisyChainLength; ++current_ic) {
        for (const auto& register_group : cell_data) { // 4 voltage register groups
            for (const auto value : register_group[current_ic].data) { // 3 voltages per IC
                data[byte_position++] = static_cast<uint8_t>(value >> 8);
                data[byte_position++] = static_cast<uint8_t>(value);

                if (byte_position == 8) {
                    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) != HAL_OK)
                        return HAL_ERROR;
                    byte_position = 0;
                    ++TxHeader.StdId;
                }
            } // 4 * 3 == 12 voltages associated with each LTC6811 in the daisy chain
        }
    } // 4 * 3 * kDaisyChainLength == all voltages associated with the daisy chain
    return HAL_OK;
}

HAL_StatusTypeDef CANTxTemperature(LTC6811::TempData const & temp_data) {
    TxHeader.StdId = Temp;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    for (size_t current_ic = 0, byte_position = 0; current_ic < LTC6811::kDaisyChainLength; ++current_ic) {
        for (const auto& register_group : temp_data) { // 2 voltage register groups
            for (const auto temperature : register_group[current_ic].data) { // 3 temperatures per IC
                data[byte_position++] = static_cast<uint8_t>(temperature >> 8);
                data[byte_position++] = static_cast<uint8_t>(temperature);
                if (byte_position == 8) {
                    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) != HAL_OK)
                        return HAL_ERROR;
                    byte_position = 0;
                    ++TxHeader.StdId;
                }
            } // 2 * 3 == 6 temperatures associated with each LTC6811 in the daisy chain
        }
    } // 2 * 3 * kDaisyChainLength == all temperatures associated with the daisy chain
    return HAL_OK;
}

HAL_StatusTypeDef CANTxVoltageLimpTotal(uint32_t const sum_of_cells) {
    TxHeader.StdId = VoltTotal;
    TxHeader.DLC = 8;
    uint8_t data[8] {
        static_cast<uint8_t>(sum_of_cells >> 24),
                static_cast<uint8_t>(sum_of_cells >> 16),
                static_cast<uint8_t>(sum_of_cells >>  8),
                static_cast<uint8_t>(sum_of_cells >>  0),
                status.get_error_over_limit(Limping),
                0x0,
                0x0,
                0x0
    };
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox);
}

/* Put discharge flag data on CAN bus. */
HAL_StatusTypeDef CANTxDCCfg() {
    TxHeader.StdId = DishB;
    TxHeader.DLC = 8;

    uint8_t data[8]{ 0 };
    uint8_t byte_position{ 0 };
    LTC6811::RegisterGroup<uint8_t> config_register_group;
    ltc6811->RDCFG(LTC6811::A, config_register_group);
    for (const auto& IC : config_register_group) {
        data[byte_position++] = IC.data[5];
        data[byte_position++] = IC.data[4];
        if (byte_position == 8) {
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox) != HAL_OK)
                return HAL_ERROR;
            byte_position = 0;
            ++TxHeader.StdId;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef CANTxVolumeSize(uint32_t const size_of_log) {
    TxHeader.StdId = LoggerResp;
    TxHeader.DLC = 4;

    uint8_t data[] = {
            static_cast<uint8_t>(size_of_log >> 24),
            static_cast<uint8_t>(size_of_log >> 16),
            static_cast<uint8_t>(size_of_log >>  8),
            static_cast<uint8_t>(size_of_log >>  0)
    };

    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &mailbox);
}

void Error_Handler() {}
/* USER CODE END 4 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

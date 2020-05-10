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
#include "main.h"
#include "fatfs.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Status.h"
#include "LTC6811.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
SD_HandleTypeDef hsd;
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
uint8_t rtc_event_flag;
NLG5* nlg5;
Status* status;
LTC6811* ltc6811;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    MX_FATFS_Init();
    /* USER CODE BEGIN 2 */
    nlg5 = new NLG5;
    status = new Status(Status::Core | Status::Logging, *nlg5);
    ltc6811 = new LTC6811(hspi1, *status); // TODO could be hcan2!
    f_mount(&SDFatFS, "", 0);
    f_open(&SDFile, "data.csv", FA_WRITE | FA_OPEN_APPEND);
#if SPITEST
    while (true) {
        const uint8_t bytes[2] { 1, 0 };
        const uint16_t test = bytes[1] << 8 | bytes[0];
        const uint16_t test2 = *reinterpret_cast<uint16_t const *>(bytes);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, reinterpret_cast<uint8_t const *>(&bytes), 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
#endif

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
#if BYPASS_INITIAL_CHECK
    status->CloseAIR();
    HAL_Delay(5000);
    status->ClosePRE();
#else
    //if (!core_routine()) {    // Initial check before closing AIRs
    //  HAL_Delay(1000);    // This small delay may not be necessary
    //status->CloseAIR(); //bmsrelay
#if !CHECK_IVT
    HAL_Delay(5000);
    status->closePRE();
#endif
    //}
#endif
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);

        /* Each bit of opmode represents a different mode. */
        if (status->op_mode & Status::Core) {
            auto const voltage_status = ltc6811->GetVoltageStatus();
            if (!voltage_status.has_value()) {
                status->IncreasePecCounter();
            } else {
                // Test limits
                voltage_status.value().max;
                voltage_status.value().min;
            }

            auto const temp_status = ltc6811->GetTemperatureStatus();
            if (!temp_status.has_value())
                status->IncreasePecCounter();
            else {
                // Test limits
                temp_status.value().max;
                temp_status.value().min;
            }

            status->SetFanDutyCycle(status->CalcDutyCycle());
            CANTxUptime();
            CanTxOpMode();
            CanTxError();
            CANTxVoltageLimpTotal();
            status->CheckIVTLost();

            if (status->op_mode & Status::Balance)
                ltc6811->BuildDischargeConfig(voltage_status.value());

        }



#if CAN_ENABLED
        /*  Charging routine. CAN buffers for charger messages are checked, and charger command message is sent. */
        if (status->op_mode & Status::Charging)
            SetCharger();
#endif
#if CAN_DEBUG
        /*  Functions for debugging and untested code. */
        if (status->op_mode & Status::Debug) {
            CANTxVoltage();
            CANTxTemperature();
            CANTxDCfg();
            CANTxUptime();
            CanTxOpMode();
        }
#endif
        if (status->op_mode & Status::Logging) {
            FILINFO inf;
            if (BSP_SD_IsDetected()) {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_2); // led 2
                if (f_stat("/hpf20", &inf) == FR_NO_FILE)
                    f_mkdir("/hpf20");

                // TODO Magic number below that needs fixing
                if (f_size(&SDFile) < 524288000 && f_open(&SDFile, "/hpf20/data.csv", FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
                    f_printf(&SDFile, "%u,", status->uptime);
                    /* ISO 8601 Notation (yyyy-mm-ddThh:mm:ss) */
                    f_printf(&SDFile, "%02u-%02u-%02uT%02u:%02u:%02u,",
                            status->rtc.tm_year, status->rtc.tm_mon, status->rtc.tm_mday, status->rtc.tm_hour, status->rtc.tm_min, status->rtc.tm_sec);

                    UINT number_written = 0;
                    uint8_t write_error{ 0 };

                    // TODO error handling isn't really done yet
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
    /* USER CODE END 3 */
}

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
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CAN1_Init 2 */

    // Need to find what filter INDEX might be.
    /* Copying from ECU code for now. */
    /* BMS filter */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // allows two IDs to be set to one filter with IDLIST
    sFilterConfig.FilterIdHigh = 0x20 << 5; // first ID
    sFilterConfig.FilterIdLow = 0x0; // don't think anything goes here
    sFilterConfig.FilterMaskIdHigh = 0x21 << 5; //second ID
    sFilterConfig.FilterMaskIdLow = 0x0;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        Error_Handler();

    //sFilterConfig.FilterIdHigh = 0x8 << 5; // first ID
    //sFilterConfig.FilterIdLow = 0x0;

    sFilterConfig.FilterBank++; // ECU CAN 1
    sFilterConfig.FilterIdHigh = 0x20 << 5;
    sFilterConfig.FilterIdLow = 0x21;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        Error_Handler();

    sFilterConfig.FilterBank++; // PDM CAN 1
    sFilterConfig.FilterIdHigh = 0x520 << 5;
    sFilterConfig.FilterIdLow = 0x0;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        Error_Handler();

    sFilterConfig.FilterBank++; // ADC ID Range
    sFilterConfig.FilterIdHigh = 0x600 << 5;
    sFilterConfig.FilterIdLow = 0x605;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        Error_Handler();

    sFilterConfig.FilterBank++;
    sFilterConfig.FilterIdHigh = 0x610 << 5;
    sFilterConfig.FilterIdLow = 0x614;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        Error_Handler();

    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterBank++; // Front Wheelspeed Filter
    sFilterConfig.FilterIdHigh = 0x70;
    sFilterConfig.FilterIdLow = 0x7F;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
        Error_Handler();

#ifndef ONECAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();

    // Start CANRX interrupt for CAN1
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
#endif

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
        Error_Handler();

    //TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    //TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    //TxHeader.TxEventFifoControl = FDCAN_NO_Tx_EVENTS;
    //TxHeader.MessageMarker = 0;


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
// CAN1
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef   RxHeader;
    uint8_t data[8] = { 0 };

    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
        //Placeholder switch statement
        switch(RxHeader.StdId) {
        case 1:
            break;
        default:
            break;
        }
    }
}

// CAN2
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef   RxHeader;
    uint8_t data[8] = { 0 };

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
        //Placeholder switch statement
        switch(RxHeader.StdId) {
        case CAN_ID_NLGA_STAT:
            nlg5->a_buffer[0] = data[0];
            nlg5->a_buffer[1] = data[1];
            nlg5->a_buffer[2] = data[2];
            nlg5->a_buffer[3] = data[3];
            break;

        case CAN_ID_NLGB_STAT:
            nlg5->b_buffer[0] = data[0];
            nlg5->b_buffer[1] = data[1];
            nlg5->b_buffer[2] = data[2];
            nlg5->b_buffer[3] = data[3];
            break;

        case CAN_ID_LOGGER_REQ:
            /* Old BMS code sorted 32-bit ints into a byte array. It used the LS8Bits of the first 32-bit int in the switch case. In the new BMS, that means data[3] */
            switch (data[3]) {
            case 0:
                /* Call appropriate logger function */
                break;
            case 1:
                canresp_get_volume_size();
                break;
            case 2:
                canresp_delete_logfile();
                break;
            case 3:
                // Data will likely not be in the right order yet.
                // rtc_set_date_time(data);
                break;
            default:
                break;
            }
            break;

            case CAN_ID_SETTING:
                ltc6811->SetDischargeMode(data[0]);
                status->op_mode = data[1];

                /* data[4] == fanduty */
                status->manual_mode = data[4] & 0x80;
                status->SetFanDutyCycle(data[4]); // TODO set_fan_duty_cycle(data[4], 1);
                nlg5->oc_limit = data[5];
                break;

#if CHECK_IVT
            case CAN_ID_IVT_U1:
                // Data[0] and Data[1] contain information that is not relevant.
                status->SetAccuVoltage(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]));
                break;

            case CAN_ID_IVT_U2:
                status->SetAccuVoltage2(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]));
                break;

            case CAN_ID_IVT_I:
                status->SetCurrent(static_cast<int32_t>(data[2] << 24 | data[3] << 16 | data[4] << 8 | data[5]));
                break;
#endif

            default:
                break;
        }
    }
}

/* Send charger command message on CAN bus. Every fifth time the charger_event_flag is set a reset command is sent,
 * if charger is in fault state. Otherwise a charge command is sent. */
void SetCharger(void) {
    uint8_t charger_event_flag = true; // TODO would be set to true every 1s in the old system by a timer
    if (charger_event_flag) {
        if((nlg5->a_buffer[0] == 136 || nlg5->a_buffer[0] == 152) && (nlg5->b_buffer[0] == 136 || nlg5->b_buffer[0] == 152)) {
            // Empty for now. No need for this check.
        } else if (status->charger_event_counter++ > 4) {
            nlg5->ctrl = NLG5::C_C_EL;
            status->charger_event_counter = 0;
        } else
            nlg5->ctrl = NLG5::C_C_EN;

        CANTxNLGAControl();
        CANTxNLGBControl();
        charger_event_flag = false;
    }
}



int32_t CAN0_Test(void) {
    TxHeader.StdId = CAN_ID_TMP_TESTING;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

int8_t CANTxVoltage(void) {
    TxHeader.StdId = CAN_ID_VOLT;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

#if NOT_WORKING
    for (auto it = cell_data.begin(); it != cell_data.end(); it += 8) {
        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, it, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
            return -1;

        ++TxHeader.StdId;
    }
    return 0;


    for (auto& reg : cell_data) {
        for (auto& ic : reg) { //TODO This used to not send PEC data
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, ic.data(), (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
                return -1;

            ++TxHeader.StdId;
        }
    }
#endif
    return 0;
}

int8_t CANTxVoltageLimpTotal(void) {
    TxHeader.StdId = CAN_ID_VOLT_TOTAL;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    auto sum_of_cells = status->sum_of_cells / 10000; // TODO was being divided by 10000 on old system, I believe

    uint8_t data[] = {
            static_cast<uint8_t>(sum_of_cells),
            static_cast<uint8_t>(sum_of_cells >> 8),
            static_cast<uint8_t>(sum_of_cells >> 16),
            static_cast<uint8_t>(sum_of_cells >> 24),
            0xCD,
            0xAB,
            0,
            status->GetLimping()
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

int8_t CANTxTemperature(void) {
    TxHeader.StdId = CAN_ID_TEMP;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

#if 0
    uint8_t data[8];
    uint8_t byte_position = 0;

    for (auto& reg : temp_data) {
        for (auto& ic : reg) {
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, ic.data()->data(), (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
                return -1;

            ++TxHeader.StdId;
            byte_position = 0;

        }
    }

    if (byte_position != 0)
        TxHeader.DLC = byte_position; // I think this is unnecessary, but it was in the old code...

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;
#endif
    return 0;
}

int32_t CANTxUptime(void) {
    static uint32_t upCounter;
    upCounter++; // nicer way of saving the value between function calls

    TxHeader.StdId = CAN_ID_UPTIME;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[] = {
            static_cast<uint8_t>(upCounter),
            static_cast<uint8_t>(upCounter >> 8),
            static_cast<uint8_t>(upCounter >> 16),
            static_cast<uint8_t>(upCounter >> 24),

            static_cast<uint8_t>(status->uptime),
            static_cast<uint8_t>(status->uptime >> 8),
            static_cast<uint8_t>(status->uptime >> 16),
            static_cast<uint8_t>(status->uptime >> 24)
    };


    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

int32_t CanTxOpMode(void) {
    TxHeader.StdId = CAN_ID_OPMODE;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    // I believe the order of these data arrays is wrong
    uint8_t data[8] = {
            status->precharge_flag,
            static_cast<uint8_t>(status->max_temp >> 8), // Why are we ruining a 16 bit int like this?
            status->last_error,
            status->safe_state_executed,
            static_cast<uint8_t>(status->min_voltage & 0xFF), // This one too
            static_cast<uint8_t>(status->min_voltage >> 8),
            status->min_voltage_index, // TODO this is messed up for now
            status->op_mode
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

int32_t CanTxError(void) {
    auto pec_change = status->GetPecChange();

    TxHeader.StdId = 0xBEEF;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[] = {
            static_cast<uint8_t>(status->pec_counter >> 24),
            static_cast<uint8_t>(status->pec_counter >> 16),
            static_cast<uint8_t>(status->pec_counter >> 8),
            static_cast<uint8_t>(status->pec_counter),

            static_cast<uint8_t>(pec_change >> 24),
            static_cast<uint8_t>(pec_change >> 16),
            static_cast<uint8_t>(pec_change >> 8),
            static_cast<uint8_t>(pec_change)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

/*!
    \brief Puts discharge flag data on CAN bus.
 */
int32_t CANTxDCfg(void) {
    TxHeader.StdId = CAN_ID_DISHB;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;

    uint8_t data[8] = { 0 };
    uint8_t byte_position = 0;

    /* TODO
    for (auto& ic : slave_cfg_rx) {
        data[byte_position++] = ic[5];
        data[byte_position++] = ic[4];

        if (byte_position == 8) {
            if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
                return -1;

            byte_position = 0;
            ++TxHeader.StdId;
        }
    }*/

    return 0;
}

/* Checks specified chargers MOB status */
int32_t CANTxNLGAControl(void) {
    TxHeader.StdId = CAN_ID_NLGA_CTRL;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 7;

    uint8_t data[7] = {
            nlg5->ctrl,
            static_cast<uint8_t>(nlg5->mc_limit >> 8),
            static_cast<uint8_t>(nlg5->mc_limit),
            static_cast<uint8_t>(nlg5->ov_limit >> 8),
            static_cast<uint8_t>(nlg5->ov_limit),
            static_cast<uint8_t>(nlg5->oc_limit >> 8),
            static_cast<uint8_t>(nlg5->oc_limit)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

// TODO This is exactly the same as the function above?
int32_t CANTxNLGBControl(void) {
    TxHeader.StdId = CAN_ID_NLGB_CTRL;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 7;

    uint8_t data[7] = {
            nlg5->ctrl,
            static_cast<uint8_t>(nlg5->mc_limit >> 8),
            static_cast<uint8_t>(nlg5->mc_limit),
            static_cast<uint8_t>(nlg5->ov_limit >> 8),
            static_cast<uint8_t>(nlg5->ov_limit),
            static_cast<uint8_t>(nlg5->oc_limit >> 8),
            static_cast<uint8_t>(nlg5->oc_limit)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

int32_t CANTxVolumeSize(uint32_t size_of_log) {
    TxHeader.StdId = CAN_ID_LOGGER_RESP;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4;

    uint8_t data[] = {
            static_cast<uint8_t>(size_of_log >> 24),
            static_cast<uint8_t>(size_of_log >> 16),
            static_cast<uint8_t>(size_of_log >> 8),
            static_cast<uint8_t>(size_of_log)
    };

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, (uint32_t *)CAN_TX_MAILBOX0) != HAL_OK)
        return -1;

    return 0;
}

void canresp_get_volume_size(void) {
    if (f_mount(&SDFatFS, "", 0) == FR_OK) {
        CANTxVolumeSize(f_size(&SDFile));
        f_mount(NULL, "", 0); /* Unmount */
    }
}

void canresp_delete_logfile(void) {
    if (f_mount(&SDFatFS, "", 0) == FR_OK) {
        f_unlink("/hpf17/data.txt");
        f_mount(NULL, "", 0); /* Unmount */
    }
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

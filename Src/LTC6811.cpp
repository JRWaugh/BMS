/*
 * LTC6811.cpp
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#include "LTC6811.h"

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void DWT_Delay(uint32_t us) {
    uint32_t startTick{ DWT->CYCCNT }, delayTicks{ us * SystemCoreClock / 1000000 };

    while (DWT->CYCCNT - startTick < delayTicks);
}

void LTC6811::WakeFromSleep(void) {
    for (uint8_t i = 0; i < kDaisyChainLength; ++i) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        DWT_Delay(T_WAKE_MAX); // Guarantees the LTC681x will be in standby
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        DWT_Delay(10);
    }
}

void LTC6811::WakeFromIdle(void) {
    uint8_t data = 0xFF;

    for (uint8_t i = 0; i < kDaisyChainLength; ++i) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi, &data, 1, 10); //Guarantees the isoSPI will be in ready mode
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

}

/* Read the raw data from the LTC6811 cell voltage register and verifies that the data was received correctly.
 * Returns 0 on success, 1 if either PEC or SPI error. */
uint8_t LTC6811::ReadVoltageRegisterGroup(Group const group) {
    return ReadRegisterGroup(cell_data[group]);
}

/* Reads the raw data from the LTC6811 auxiliary register and verifies that the data was received correctly.
 * Returns 0 on success, 1 if either PEC or SPI error. */
uint8_t LTC6811::ReadAuxRegisterGroup(Group const group) {
    return ReadRegisterGroup(cell_data[group]);
}

/* Read status register group of a LTC6811 daisy chain. */
uint8_t LTC6811::ReadStatusRegisterGroup(Group const group) {
    return ReadRegisterGroup(status_registers[group]);
}

/* Read configuration registers of a LTC6811 daisy chain */
uint8_t LTC6811::ReadConfigRegisterGroup(void) {
    return ReadRegisterGroup(slave_cfg_rx);
}

/* Write to the configuration registers of the LTC6811s in the daisy chain. */
uint8_t LTC6811::WriteConfigRegisterGroup(void) {
    return WriteRegister(slave_cfg_tx);
}

/* Clear the LTC6811 cell voltage registers. */
void LTC6811::ClearVoltageRegisters(void) {
    static constexpr LTC6811Command command{ 7, 17, 201, 192 };

    WakeFromIdle();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, command.data(), sizeof(command), 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

/* Clear the LTC6811 Auxiliary registers. */
void LTC6811::ClearAuxRegisters(void) {
    static constexpr LTC6811Command command{ 7, 18, 223, 164 };

    WakeFromIdle();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, command.data(), sizeof(command), 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

std::optional<LTC6811VoltageStatus> LTC6811::GetVoltageStatus(void) {
    StartConversion(ADCV);

    for (size_t group = A; group < D; ++group)
        if (!ReadVoltageRegisterGroup(static_cast<Group>(group)))
            return std::nullopt;

    LTC6811VoltageStatus status;
    size_t count{ 0 };

    for (const auto& register_group : cell_data) {
        for (const auto& Register : register_group.register_group) {
            for (const auto voltage : Register.data) {
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
    return status;
}

std::optional<LTC6811TempStatus> LTC6811::GetTemperatureStatus() {
    StartConversion(ADAX);

    for (size_t group = A; group < D; ++group)
        if (!ReadAuxRegisterGroup(static_cast<Group>(group)))
            return std::nullopt;

    auto steinharthart = [](int16_t const NTC_voltage) noexcept {
        constexpr auto Vin = 30000.0f; // 3[V], or 30000[V * 10-5]
        constexpr auto KtoC = 27315; // centiKelvin to centiDegCelsius
        constexpr auto A = 0.003354016f;
        constexpr auto B = 0.000256524f;
        constexpr auto C = 0.00000260597f;
        constexpr auto D = 0.0000000632926f;
        auto log = -logf(Vin / NTC_voltage - 1);

        return static_cast<int16_t>(100.0f / (A + log * ( B + log * (C + D * log))) - KtoC);
    };

    LTC6811TempStatus status;
    size_t count{ 0 };

    for (const auto& register_group : cell_data) {
        for (const auto& Register : register_group.register_group) {
            for (auto temperature : Register.data) {
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

void LTC6811::BuildDischargeConfig(const LTC6811VoltageStatus& voltage_status) {
    constexpr uint8_t kDelta = 100;
    uint16_t DCCx{ 0 };
    uint8_t current_cell{ 0 }, current_ic{ kDaisyChainLength - 1 };
    auto average_voltage{ voltage_status.sum / (12 * kDaisyChainLength) };

    switch (discharge_mode) {
    case 0: // Discharge all above (min_voltage + delta)
        for (auto& cfg_register : slave_cfg_tx.register_group) {
            DCCx = 0;
            current_cell = 0;

            for (const auto& register_group : cell_data) { // 4 voltage register chains
                for (const auto voltage : register_group.register_group[current_ic].data) { // 3 voltages per IC
                    if (voltage > voltage_status.min + kDelta)
                        DCCx |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }
            cfg_register.data[4] |= DCCx & 0xFF;
            cfg_register.data[5] |= DCCx >> 8 & 0xF;
            cfg_register.PEC = PEC15Calc(cfg_register.data);
            --current_ic;
        }
        break;

    case 1: // Discharge only the max_voltage cell.
        if (voltage_status.max - voltage_status.min > kDelta) {
            current_ic = voltage_status.max_id / 3 % 12;
            DCCx |= 1 << voltage_status.max_id % 11;
            slave_cfg_tx.register_group[current_ic].data[4] = DCCx & 0xFF;
            slave_cfg_tx.register_group[current_ic].data[5] = DCCx >> 8 & 0xF;
            slave_cfg_tx.register_group[current_ic].PEC = PEC15Calc(slave_cfg_tx.register_group[current_ic].data);
        }
        break;

    case 2: // Discharge all cells that are above average cell voltage + delta
        for (auto& cfg_register : slave_cfg_tx.register_group) {
            DCCx = 0;
            current_cell = 0;

            for (const auto& register_group : cell_data) { // 4 voltage register chains
                for (const auto voltage : register_group.register_group[current_ic].data) { // 3 voltages per IC
                    if (voltage > average_voltage + kDelta)
                        DCCx |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }
            cfg_register.data[4] |= DCCx & 0xFF;
            cfg_register.data[5] |= DCCx >> 8 & 0xF;
            cfg_register.PEC = PEC15Calc(cfg_register.data);
            --current_ic;
        }
        break;
    }

    WriteConfigRegisterGroup();
    DWT_Delay(500);
    ReadConfigRegisterGroup();
}


/* Start a conversion */
void LTC6811::StartConversion(const LTC6811Command& command) {
    WakeFromIdle(); // It's possible all of these can be removed

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, command.data(), sizeof(command), HAL_MAX_DELAY);        // Start cell voltage conversion.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    DWT_Delay(T_REFUP_MAX + T_CYCLE_FAST_MAX); // TODO we aren't in fast conversion mode??? Also these delays aren't in the Linduino library
}

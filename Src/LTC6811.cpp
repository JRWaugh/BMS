/*
 * LTC6811.cpp
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#include <DWTWrapper.h>
#include "LTC6811.h"

LTC6811::LTC6811(SPI_HandleTypeDef& hspi, Mode mode, DCP dcp, CellCh cell, AuxCh aux, STSCh sts) : hspi{ hspi } {
    uint8_t md_bits = (mode & 0x02) >> 1;
    uint16_t PEC{ 0 };

    ADCV[0]   = 0x02 | md_bits;
    ADAX[0]   = 0x04 | md_bits;
    ADSTAT[0] = 0x04 | md_bits;

    md_bits   = (mode & 0x01) << 7;
    ADCV[1]   = md_bits | 0x60 | dcp << 4 | cell;
    ADAX[1]   = md_bits | 0x60 | aux;
    ADSTAT[1] = md_bits | 0x68 | sts;

    PEC = PEC15Calc(ADCV, 2);
    ADCV[2] = static_cast<uint8_t>(PEC >> 8);
    ADCV[3] = static_cast<uint8_t>(PEC);

    PEC = PEC15Calc(ADAX, 2);
    ADAX[2] = static_cast<uint8_t>(PEC >> 8);
    ADAX[3] = static_cast<uint8_t>(PEC);

    PEC = PEC15Calc(ADSTAT, 2);
    ADSTAT[2] = static_cast<uint8_t>(PEC >> 8);
    ADSTAT[3] = static_cast<uint8_t>(PEC);

    WakeFromSleep(); // NOTE: Takes 2.2s to fall asleep so if this has to be called after this, we have problems
}

void LTC6811::WakeFromSleep() const noexcept {
    static constexpr uint16_t kMaxWakeTime{ 400 }; // Time in us

    for (size_t i = 0; i < kDaisyChainLength; ++i) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        DWTWrapper::getInstance().delay(kMaxWakeTime); // Guarantees the LTC6811 will be in standby
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        DWTWrapper::getInstance().delay(10);
    }
}

void LTC6811::WakeFromIdle() const noexcept {
    static constexpr uint8_t kData{ 0xFF };

    for (size_t i = 0; i < kDaisyChainLength; ++i) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi, &kData, 1, HAL_MAX_DELAY); //Guarantees the isoSPI will be in ready mode
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
}

/* Read a cell voltage register group of an LTC6811 daisy chain.
 * Returns 0 on success, 1 if either PEC or SPI error.
 */
bool LTC6811::readVoltageRegisterGroup(Group const group) noexcept {
    constexpr static std::array<Command, 4> kCommands{ Command{ 0, 4, 7, 194}, Command{ 0, 6, 154, 148 }, Command{ 0, 8, 94, 82 }, Command{ 0, 10, 195, 4 } };

    /* Checking array bounds allows us to declare the function noexcept */
    if (group <= D)
        return readRegisterGroup(kCommands[group], cell_data[group]);
    else
        return Fail;
}

/* Read an auxiliary register group of an LTC6811 daisy chain.
 * Returns 0 on success, 1 if either PEC or SPI error.
 */
bool LTC6811::readAuxRegisterGroup(Group const group) noexcept {
    constexpr static std::array<Command, 2> kCommands{ Command{ 0, 12, 239, 204 }, Command{ 0, 14, 114, 154 } };

    if (group <= B)
        return readRegisterGroup(kCommands[group], temp_data[group]);
    else
        return Fail;
}

/* Read a status register group of an LTC6811 daisy chain. */
bool LTC6811::readStatusRegisterGroup(Group const group) {
    constexpr static std::array<Command, 2> kCommands{ Command{ 0x00, 0x10, 0xED, 0x72 }, Command{ 0x00, 0x12, 0x70, 0x24 } };

    if (group <= B)
        return readRegisterGroup(kCommands[group], status_data[group]);
    else
        return Fail;
}

/* Read the configuration register group of an LTC6811 daisy chain */
bool LTC6811::readConfigRegisterGroup() noexcept {
    constexpr static Command kCommand{ 0x00, 0x02, 0x2B, 0x0A };

    return readRegisterGroup(kCommand, slave_cfg_rx);
}

/* Write to the configuration register group of an LTC6811 daisy chain. */
bool LTC6811::writeConfigRegisterGroup(RegisterGroup<uint8_t> const& cfg_register_group) noexcept {
    constexpr static Command kCommand{ 0x00, 0x01, 0x3D, 0x6E };

    if  (writeRegisterGroup(kCommand, cfg_register_group) == Success) {
        DWTWrapper::getInstance().delay(500);
        /* Funky place to do this, but fixing this would require substantially reworking the whole class.
         * The purpose is to read back the config register after writing to it to check that it was written to properly, apparently. */
        readConfigRegisterGroup();
        return Success;
    } else {
        return Fail;
    }
}

/* Clear the LTC6811 cell voltage registers. */
bool LTC6811::clearVoltageRegisterGroup() noexcept {
    constexpr static Command kCommand{ 7, 17, 201, 192 };

    return clearRegisterGroup(kCommand);
}

/* Clear the LTC6811 Auxiliary registers. */
bool LTC6811::clearAuxRegisterGroup() noexcept {
    constexpr static Command kCommand{ 7, 18, 223, 164 };

    return clearRegisterGroup(kCommand);
}

/* Generate a status report of the cell voltage register groups.
 * Returns an LTC6811VoltageStatus on success, nullopt if error. */
[[nodiscard]] std::optional<LTC6811::VoltageStatus> LTC6811::checkVoltageStatus(void) noexcept {
    LTC6811::VoltageStatus status;
    size_t count{ 0 };

    startConversion(ADCV);

    for (size_t group = A; group <= D; ++group)
        if (readVoltageRegisterGroup(static_cast<Group>(group)) == Fail)
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

/* Generate a status report of the current temperatures from aux voltage register groups.
 * Returns an LTC6811TempStatus on success, nullopt if error. */
[[nodiscard]] std::optional<LTC6811::TempStatus> LTC6811::checkTemperatureStatus() noexcept {
    LTC6811::TempStatus status;
    size_t count{ 0 };
    constexpr static auto steinharthart = [](int16_t const NTC_voltage) noexcept {
        constexpr auto Vin = 30000.0f; // 3[V], or 30000[V * 10-5]
        constexpr auto KtoC = 27315; // centiKelvin to centiDegCelsius
        constexpr auto A = 0.003354016f;
        constexpr auto B = 0.000256524f;
        constexpr auto C = 0.00000260597f;
        constexpr auto D = 0.0000000632926f;
        auto log = -logf(Vin / NTC_voltage - 1);

        return static_cast<int16_t>(100.0f / (A + log * ( B + log * (C + D * log))) - KtoC);
    };

    startConversion(ADAX);

    for (size_t group = A; group <= D; ++group)
        if (readAuxRegisterGroup(static_cast<Group>(group)) == Fail)
            return std::nullopt;

    for (auto const& register_group : cell_data) {
        for (auto const& IC : register_group) {
            for (auto temperature : IC.data) {
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

[[nodiscard]] LTC6811::RegisterGroup<uint8_t> LTC6811::makeDischargeConfig(VoltageStatus const& voltage_status) const noexcept {
    constexpr static uint8_t kDelta{ 100 };

    uint16_t DCCx{ 0 };
    uint8_t current_cell{ 0 }, current_ic{ kDaisyChainLength - 1 };
    RegisterGroup<uint8_t> cfg_register_group;

    switch (discharge_mode) {
    case GTMinPlusDelta:
        for (auto& IC : cfg_register_group) { // 12 register groups
            DCCx = 0;
            current_cell = 0;

            for (auto const& register_group : cell_data) { // 4 voltage register groups
                for (auto const voltage : register_group[current_ic].data) { // 3 voltages per IC
                    if (voltage > voltage_status.min + kDelta)
                        DCCx |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }
            --current_ic;

            IC.data[0] = 0xFE;
            IC.data[4] = DCCx & 0xFF;
            IC.data[5] = DCCx >> 8 & 0xF;
            IC.PEC = PEC15Calc(IC.data);
        } // 12 * 12 = 144 voltages associated with the entire daisy chain
        break;

    case MaxOnly:
        if (voltage_status.max - voltage_status.min > kDelta) {
            current_ic = voltage_status.max_id / 3 % 12;
            DCCx |= 1 << voltage_status.max_id % 11;

            cfg_register_group[current_ic].data[0] = 0xFE;
            cfg_register_group[current_ic].data[4] = DCCx & 0xFF;
            cfg_register_group[current_ic].data[5] = DCCx >> 8 & 0xF;
            cfg_register_group[current_ic].PEC = PEC15Calc(cfg_register_group[current_ic].data);
        }
        break;

    case GTMeanPlusDelta: {
        size_t average_voltage{ voltage_status.sum / (4 * kDaisyChainLength * 3) };

        for (auto& IC : cfg_register_group) {
            DCCx = 0;
            current_cell = 0;

            for (auto const& register_group : cell_data) { // 4 voltage register groups
                for (auto const voltage : register_group[current_ic].data) { // 3 voltages per IC
                    if (voltage > average_voltage + kDelta)
                        DCCx |= 1 << current_cell;
                    ++current_cell;
                } // 4 * 3 = 12 voltages associated with each LTC6811 in the daisy chain
            }
            --current_ic;

            IC.data[0] = 0xFE;
            IC.data[4] = DCCx & 0xFF;
            IC.data[5] = DCCx >> 8 & 0xF;
            IC.PEC = PEC15Calc(IC.data);
        }
    }
    break;
    }

    return cfg_register_group;
}


/* Start a conversion */
void LTC6811::startConversion(Command const& command) const noexcept {
    static constexpr uint16_t kMaxCycleTimeFast{ 1185 }; // Measure 12 Cells. Time in us.
    static constexpr uint16_t kMaxRefWakeupTime{ 4400 }; // Time in us.

    WakeFromIdle(); // It's possible all of these can be removed

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi, command.data(), sizeof(Command), HAL_MAX_DELAY); // Start cell voltage conversion.
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

    DWTWrapper::getInstance().delay(kMaxRefWakeupTime + kMaxCycleTimeFast); // TODO we aren't in fast conversion mode??? Also these delays aren't in the Linduino library
}

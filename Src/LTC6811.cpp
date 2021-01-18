/*
 * LTC6811.cpp
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#include "LTC6811.h"

namespace LTC6811 {
static constexpr uint16_t crc15Table[256] {
    0x0000, 0xc599, 0xceab, 0x0b32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac, 0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa,
    0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1, 0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d,
    0x5b2e, 0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b, 0x77e6, 0xb27f, 0xb94d, 0x7cd4,
    0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd, 0x2544, 0x02be, 0xc727, 0xcc15, 0x098c, 0xda71, 0x1fe8, 0x14da, 0xd143,
    0xf3c5, 0x365c, 0x3d6e, 0xf8f7, 0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x07c2, 0xc25b, 0xc969, 0x0cf0, 0xdf0d, 0x1a94, 0x11a6, 0xd43f,
    0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf, 0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8,
    0xa8eb, 0x6d72, 0x6640, 0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba, 0x4a88, 0x8f11,
    0x057c, 0xc0e5, 0xcbd7, 0x0e4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b, 0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286,
    0xa213, 0x678a, 0x6cb8, 0xa921, 0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070, 0x85e9,
    0x0f84, 0xca1d, 0xc12f, 0x04b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528, 0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e,
    0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59, 0x2ac0, 0x0d3a, 0xc8a3, 0xc391, 0x0608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7,
    0x54aa, 0x9133, 0x9a01, 0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9, 0x7350,
    0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a, 0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c,
    0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 0x2fbc, 0x0846, 0xcddf, 0xc6ed, 0x0374, 0xd089, 0x1510, 0x1e22, 0xdbbb,
    0x0af8, 0xcf61, 0xc453, 0x01ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b, 0x2d02,
    0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3, 0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

template <typename T, size_t S>
[[nodiscard]] constexpr static uint16_t PEC15Calc(std::array<T, S> const& data, size_t const size = S * sizeof(T)) noexcept {
    uint16_t PEC{ 16 }, addr{ 0 };

    // Reinterpret_cast isn't allowed in constexpr contexts so this has to be done to compute the conversion commands (and others) at compile time.
    if constexpr (sizeof(T) > 1) {
        auto serialized = reinterpret_cast<uint8_t const*>(&data);
        for (uint8_t i = 0; i < size; ++i) {
            addr = ((PEC >> 7) ^ serialized[i]) & 0xFF;
            PEC = (PEC << 8) ^ crc15Table[addr];
        }
    } else {
        for (uint8_t i = 0; i < size; ++i) {
            addr = ((PEC >> 7) ^ data[i]) & 0xFF;
            PEC = (PEC << 8) ^ crc15Table[addr];
        }
    }

    PEC <<= 1; // The final PEC is the 15-bit value in the PEC register with a 0 bit appended to its LSB.
    // return PEC;
    return ((PEC & 0xFF) << 8 | (PEC & 0xFF00) >> 8); // Swapping byte order just because it makes life easier by allowing "if (IC.PEC != PEC15Calc(IC.data))"
}

static void wakeup() noexcept {
    static constexpr uint16_t tREADY{ 10 }, tWAKE{ 400 }; // Time in us
    static constexpr uint16_t tIDLE{ 5 }, tSLEEP{ 2000 }; // Time in ms. tIDLE is rounded down from 5.5, just to be safe.
    static uint32_t last_tick = 0;
    auto const pulse_csb = [](uint16_t pulse_length) {
        for (size_t i = 0; i < kDaisyChainLength; ++i) {
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
            DWTWrapper::delay_us(pulse_length);
            HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
        }
    };

    if (last_tick == 0)
        pulse_csb(tWAKE);
    else if (HAL_GetTick() - last_tick > tIDLE) {
        if (HAL_GetTick() - last_tick > tSLEEP)
            pulse_csb(tWAKE);
        else
            pulse_csb(tREADY);
    }

    last_tick = HAL_GetTick();
}

static SPI_HandleTypeDef* spi;
static RegisterGroup<uint8_t> config_register_group = []() {
    RegisterGroup<uint8_t> config_register_group{ 0 };
    for (auto& IC : config_register_group) {
        IC.data[0] = CFGR0;
        IC.PEC = PEC15Calc(IC.data);
    }
    return config_register_group;
}();
static RegisterGroup<uint8_t> rd_cfgr_dump{ 0 };

class Command {
public:
    constexpr Command(uint16_t const command_code) : command{ generate_command(command_code) } {}
    void operator()() const noexcept {
        wakeup();

        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(spi, command.data(), sizeof(command), HAL_MAX_DELAY);
        //HAL_SPI_Transmit(spi, poll_adc.data(), sizeof(poll_adc), HAL_MAX_DELAY);
        for (uint8_t value = 0; value == 0; )
            HAL_SPI_Receive(spi, &value, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
    }

protected:
    std::array<uint8_t, 4> const command;
    //static constexpr std::array<uint8_t, 4> poll_adc{ generate_command(0b11100010100) };
private:
    static constexpr std::array<uint8_t, 4> generate_command(uint16_t const command_code) {
        std::array<uint8_t, 4> command{ static_cast<uint8_t>(command_code >> 8), static_cast<uint8_t>(command_code) };
        uint16_t const PEC = PEC15Calc(command, 2);
        command[2] = PEC >> 0 & 0xFF;
        command[3] = PEC >> 8 & 0xFF;
        return command;
    }
};

class RD_Command : public Command {
public:
    template <typename T>
    HAL_StatusTypeDef operator()(RegisterGroup<T>& register_group) const noexcept {
        wakeup();
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(spi, command.data(), sizeof(command), HAL_MAX_DELAY);
        HAL_SPI_Receive(spi, reinterpret_cast<uint8_t*>(&register_group), sizeof(register_group), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
        for (auto& IC : register_group)
            if (IC.PEC != PEC15Calc(IC.data))
                return HAL_ERROR; // TODO make this PEC error
        return HAL_OK;
    }
};

class WR_Command : public Command {
public:
    template <typename T>
    HAL_StatusTypeDef operator()(RegisterGroup<T>& register_group) const noexcept {
        wakeup();
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
        HAL_SPI_Transmit(spi, command.data(), sizeof(command), HAL_MAX_DELAY);
        HAL_SPI_Receive(spi, reinterpret_cast<uint8_t*>(&register_group), sizeof(register_group), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
        for (auto& IC : register_group)
            if (IC.PEC != PEC15Calc(IC.data))
                return HAL_ERROR;
        return HAL_OK;
    }
};

// Commands taken from Table 38. Command Codes from LTC6811 datasheet.
// Write Configuration Register Group A and B
static constexpr WR_Command WRCFG[]{ { 0b00000000001 }, { 0b00000100100 } };
// Read Configuration Register Group A and B
static constexpr RD_Command RDCFG[]{ { 0b00000000010 }, { 0b00000100110 } };
// Read Cell Voltage Register Group A to F
static constexpr RD_Command RDCV[] { { 0b00000000100 }, { 0b00000000110 }, { 0b00000001000 }, { 0b00000001010 }, { 0b00000001001 }, { 0b00000001011 } };
// Read Auxiliary Register Group A to D
static constexpr RD_Command RDAUX[]{ { 0b00000001100 }, { 0b00000001110 }, { 0b00000001101 }, { 0b00000001111 } };
// Start Cell Voltage ADC Conversion and Poll Status
static constexpr Command ADCV   { 0b01001100000 | md << 7 | dcp << 4 | ch };
// Start GPIOs ADC Conversion and Poll Status
static constexpr Command ADAX   { 0b10001100000 | md << 7 | chg };
// Start Status Group ADC Conversion and Poll Status
static constexpr Command ADSTAT { 0b10001101000 | md << 7 | chst };

void init(SPI_HandleTypeDef* spi_handle) {
    spi = spi_handle;
    //wakeup();
    WRCFG[0](config_register_group);
};

RegisterGroup<uint8_t> const & get_config_register_group() noexcept {
    return config_register_group; // Just replace with RDCFG since it's better to know what's in the registers for sure anyway
}

uint32_t update_config_register_group(CellData const & cell_data, VoltageStatus const & voltage_status, DischargeMode discharge_mode) noexcept {
    constexpr static uint8_t kDelta{ 100 };

    uint16_t DCCx{ 0 };
    uint8_t current_cell{ 0 }, current_ic{ kDaisyChainLength - 1 };

    switch (discharge_mode) {
    case GTMinPlusDelta:
        for (auto& IC : config_register_group) { // 12 register groups
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
            IC.data[4] = DCCx >> 0 & 0xFF;
            IC.data[5] = DCCx >> 8 & 0x0F;
            IC.PEC = LTC6811::PEC15Calc(IC.data);
        } // 12 * 12 = 144 voltages associated with the entire daisy chain
        break;

    case MaxOnly:
        if (voltage_status.max - voltage_status.min > kDelta) {
            current_ic = voltage_status.max_id / 3 % 12;
            DCCx |= 1 << voltage_status.max_id % 11;
            config_register_group[current_ic].data[4] = DCCx >> 0 & 0xFF;
            config_register_group[current_ic].data[5] = DCCx >> 8 & 0x0F;
            config_register_group[current_ic].PEC = LTC6811::PEC15Calc(config_register_group[current_ic].data);
        }
        break;

    case GTMeanPlusDelta: {
        size_t const average_voltage{ voltage_status.sum / (4 * LTC6811::kDaisyChainLength * 3) };

        for (auto& IC : config_register_group) {
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
            IC.data[4] = DCCx >> 0 & 0xFF;
            IC.data[5] = DCCx >> 8 & 0x0F;
            IC.PEC = LTC6811::PEC15Calc(IC.data);
        }
        break;
    }
    }

    return WRCFG[0](config_register_group);
}

/* Read cell data and generate a status report of the cell voltage register groups.
 * Returns a VoltageStatus on success, nullopt if error. */
[[nodiscard]] std::optional<VoltageStatus> read_cell_data(CellData& cell_data) noexcept {
    LTC6811::VoltageStatus status;
    size_t count{ 0 };

    // wake and cfgr write shouldn't be necessary.
    //wake_from_sleep();
    //WRCFG[0](config_register_group);
    //DWTWrapper::delay_us(500);
    //RDCFG[0](rd_cfgr_dump);
    ADCV();

    for (size_t i = 0; i < cell_data.size(); ++i)
        if (RDCV[i](cell_data[i]) != HAL_OK)
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
[[nodiscard]] std::optional<TempStatus> read_temp_data(TempData& temp_data) noexcept {
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

    LTC6811::TempStatus status;
    size_t count{ 0 };

    ADAX();

    for (size_t i = 0; i < temp_data.size(); ++i)
        if (RDAUX[i](temp_data[i]) != HAL_OK)
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
}

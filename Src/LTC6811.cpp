/*
 * LTC6811.cpp
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#include "LTC6811.h"

uint16_t LTC6811::crc15Table[256] = {
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

void LTC6811::adcv(void) {
    WakeFromIdle();
    HAL_SPI_Transmit(&hspi, ADCV.data(), 4, 10);
}


/* Start an GPIO Conversion. */
void LTC6811::adax(void) {
    WakeFromIdle();
    HAL_SPI_Transmit(&hspi, ADAX.data(), 4, 10);
}


/* Start a STATUS reg Conversion. */
void LTC6811::adstat(void) {
    WakeFromIdle();
    HAL_SPI_Transmit(&hspi, ADSTAT.data(), 4, 10);
}

/* Reads and parses the LTC6804 cell voltage registers. */
uint8_t LTC6811::ReadVoltageHelper(std::array<LTC6811Register<uint16_t>, 4>& cell_data) {
    uint8_t pec_error{ 0 }, reg_index{ 0 }, ic_index{ 0 }, cell_index{ 0 };
    status.sum_of_cells = 0;
    // TODO min and max might need to be reset
    for (auto& reg : cell_data) { 		// For each LTC6811 cell voltage register
        ic_index = 0;
        if (ReadVoltageRegister(reg_index + 1, reg))
            pec_error = 1;

        for (auto& ic : reg) { 			// For each LTC6811 in the daisy chain
            /* Starting index of cells within each LTC6811 register. Will repeat { 0, 1, 2 } on 0th register, { 3, 4, 5 } on 1st, and so on.
             * Note that this is here PURELY for recording the index of the cell with the max voltage so that the array does not need to be
             * iterated through later */
            cell_index = reg_index * kCellsInReg;

            std::for_each(ic.begin(), ic.begin() + kCellsInReg, [&](auto& voltage) { // For each voltage of current LTC6811
                status.sum_of_cells += voltage;

                if (voltage < status.min_voltage && voltage > 5000) // Ignore cells under 500 mV
                    status.SetMinVoltage(voltage, { ic_index, cell_index });
                else if (voltage > status.max_voltage)
                    status.SetMaxVoltage(voltage, { ic_index, cell_index });

                ++cell_index;
            });
            ++ic_index;
        }
        ++reg_index;
    }
    status.CalcPower(); // TODO: sum_of_cells on Atmel was being divided by 10000
    return pec_error;
}

/* Read the raw data from the LTC6804 cell voltage register. */
uint8_t LTC6811::ReadVoltageRegister(uint8_t reg_id, LTC6811Register<uint16_t>& reg) {
    LTC6811Command command = { 0, static_cast<uint8_t>(0x2 + reg_id * 0x2) };
    auto PEC = PEC15Calc(command);
    command[2] = static_cast<uint8_t>(PEC >> 8);
    command[3] = static_cast<uint8_t>(PEC);

    WakeFromIdle();
    ReadRegister(command, reg);

    PEC = 0;
    for (auto& ic : reg)
        if (ic.back() != PEC15Calc(ic))
            PEC = 1;
    return PEC;
}

/* Reads and parses the LTC6804 auxiliary registers. */
uint8_t LTC6811::ReadTemperatureHelper(std::array<LTC6811Register<int16_t>, 2>& temp_data) {
    uint8_t pec_error{ 0 }, reg_index{ 0 }, ic_index{ 0 }, temp_index{ 0 };
    for (auto& reg : temp_data) {
        ic_index = 0;

        if (ReadAuxRegister(reg_index + 1, reg))
            pec_error = 1;

        for (auto& ic : reg) {
            temp_index = reg_index * kCellsInReg;

            std::for_each(ic.begin(), ic.begin() + kCellsInReg, [&](auto& temperature) {
                temperature = CalcTemp(temperature);

                if (temperature < status.min_temp)
                    status.SetMinTemp(temperature, { ic_index, temp_index });
                else if (temperature > status.max_temp)
                    status.SetMaxTemp(temperature, { ic_index, temp_index });

                ++temp_index;
            });
            ++ic_index;
        }
        ++reg_index;
    }
    return pec_error;
}

/* Reads the raw data from the LTC-6811 auxiliary register,
 * then verifies that the data was received correctly.
 * */
uint8_t LTC6811::ReadAuxRegister(uint8_t reg_id, LTC6811Register<int16_t>& reg) {
    LTC6811Command command = { 0x0, reg_id == 2 ? static_cast<uint8_t>(0xE) : static_cast<uint8_t>(0xC) };
    auto PEC = PEC15Calc(command);
    command[2] = static_cast<uint8_t>(PEC >> 8);
    command[3] = static_cast<uint8_t>(PEC);

    WakeFromIdle();
    ReadRegister(command, reg);

    PEC = 0;
    for (auto& ic : reg)
        if (ic.back() != PEC15Calc(ic))
            PEC = 1;
    return PEC;
}

/* Calculates the temperature from thermistor voltage using lookup table. Linear equations are used for approximation. */
int16_t LTC6811::CalcTemp(uint16_t ntc_voltage) {
    static constexpr int16_t LUT[32] = {
            22962, 22401, 21819, 21218, 20599, 19966, 19319, 18663,
            18000, 17332, 16662, 15994, 15330, 14672, 14023, 13385,
            12760, 12150, 11556, 10980, 10423,  9885,  9367,  8870,
            8394,  7939,  7505,  7091,  6697,  6323,  5969,  5633
    };

    auto indices = std::equal_range(std::rbegin(LUT), std::rend(LUT), ntc_voltage);

    int16_t x0 = *indices.first;
    int16_t x1 = *indices.second;
    int32_t m = (2 << 20) / (x1 - x0);
    int32_t b = ((std::rend(LUT) - indices.first - 1) * 2) - m * x0; //TODO possibly off by 1, but I don't think so
    return (m * ntc_voltage + b) / 10486; // (2^20 / 100) == 1048576
}

/*
 * LTC6820.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua Waugh
 */

#ifndef LTC6811_H_
#define LTC6811_H_

#include "stm32f4xx_hal.h"
#include <array>

class LTC6811 {

public:
    /* * * CELL STACK SIZE * * */
    static constexpr auto kDaisyChainLength{ 1 };

    /* * * ENUMS AND TYPE DEFINITIONS * * */
    // ADC Mode
    enum MD   { Default, Fast, Normal, Filtered };
    // Discharge Permitted
    enum DCP  { NotPermitted, Permitted };
    // Cell Selection for ADC Conversion
    enum CH   { AllCells, OneAndSeven, TwoAndEight, ThreeAndNine, FourAndTen, FiveAndEleven, SixAndTwelve };
    // Self Test Mode Selection
    enum ST   { SelfTest1 = 1, SelfTest2 };
    // GPIO Selection for ADC Conversion
    enum CHG  { AllAux, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, VREF2 };
    // Status Group Selection
    enum CHST { AllStat, SOC, ITMP, VA, VD };

    enum Group { A, B, C, D, E, F };

    // A command is an 11-bit code followed by 2 bytes of PEC. All 11-bit codes can be found in Table 38 of the datasheet.
    class Command {
    public:
        constexpr Command(uint16_t const command_code) :
        command{ static_cast<uint8_t>(command_code >> 8), static_cast<uint8_t>(command_code) }, PEC{ PEC15Calc(command) } {};
    private:
        std::array<uint8_t, 2> const command;
        uint16_t const PEC;
    };

    // A Register is 8 bytes: 6 bytes of data, and 2 bytes of PEC.
    template <typename T>
    struct Register {
        std::array<T, 6 / sizeof(T)> data;
        uint16_t PEC;
        static_assert(sizeof(T) <= 2, "Register is of invalid type.");
    };

    // A RegisterGroup is an array of Register that is kDaisyChainLength long
    template<typename T>
    using RegisterGroup = std::array<Register<T>, kDaisyChainLength>;

    // These type aliases are arrays of RegisterGroups, representing all the register groups of certain data.
    using CellData = std::array<RegisterGroup<uint16_t>, 4>;
    using TempData = std::array<RegisterGroup<int16_t>, 2>;
    using StatusData = std::array<RegisterGroup<uint16_t>, 2>;

    /* * * CONFIG CONSTANTS * * */
    static constexpr MD md = Normal;
    static constexpr DCP dcp = NotPermitted;
    static constexpr CH ch = AllCells;
    static constexpr CHG chg = AllAux;
    static constexpr CHST chst = AllStat;
    /* Configuration for configuration register 0.
     * Enables GPIOs, sets REFON bit to 1 so that the references are powered and ADC conversions can be initiated more quickly,
     * and sets DTEN pin to 1 so it takes longer for the LTC6811 to re-enter SLEEP state. */
    static constexpr uint8_t CFGR0 = 0b11111110;

    LTC6811(void (*chip_select)(bool), void (*spi_transmit)(uint8_t const *, std::size_t), void (*spi_receive)(uint8_t*, std::size_t));

    // Write Configuration Register Group A and B
    void WRCFG(Group group, RegisterGroup<uint8_t>& register_group);

    // Read Configuration Register Group A and B
    bool RDCFG(Group group, RegisterGroup<uint8_t>& register_group);

    // Read Cell Voltage Register Group A to F
    bool RDCV(Group group, RegisterGroup<uint16_t>& register_group);

    // Read Auxiliary Register Group A to D
    bool RDAUX(Group group, RegisterGroup<int16_t>& register_group);

    // Read Status Register Group A to B
    bool RDSTAT(Group group, RegisterGroup<uint16_t>& register_group);

    // Start Cell Voltage ADC Conversion and Poll Status
    void ADCV();

    // Start GPIOs ADC Conversion and Poll Status
    void ADAX();

    // Start Status Group ADC Conversion and Poll Status
    void ADSTAT();

    // Clear Cell Voltage Register Groups
    void CLRCELL();

    // Clear Auxiliary Register Groups
    void CLRAUX();

    // Clear Status Register Groups
    void CLRSTAT();

private:
    void (*chip_select)(bool level);
    void (*spi_transmit)(uint8_t const * tx_buffer, std::size_t size);
    void (*spi_receive)(uint8_t* rx_buffer, std::size_t size);
    uint32_t last_tick{ 0 };

    void wake_up();

    template <typename T>
    bool read(Command const & command, RegisterGroup<T>& register_group) {
        wake_up();
        chip_select(false);
        spi_transmit(reinterpret_cast<uint8_t const *>(&command), sizeof(command));
        spi_receive(reinterpret_cast<uint8_t*>(&register_group), sizeof(register_group));
        chip_select(true);
        for (auto& IC : register_group)
            if (IC.PEC != PEC15Calc(IC.data))
                return true;
        return false;
    }

    void write(Command const & command);

    template <typename T>
    void write(Command const & command, RegisterGroup<T>& register_group) {
        for (auto& IC : register_group)
            IC.PEC = PEC15Calc(IC.data);
        wake_up();
        chip_select(false);
        spi_transmit(reinterpret_cast<uint8_t const *>(&command), sizeof(command));
        spi_transmit(reinterpret_cast<uint8_t const *>(&register_group), sizeof(register_group));
        chip_select(true);
    }

    static constexpr uint16_t crc15Table[256] {
        0x0000, 0xC599, 0xCEAB, 0x0B32, 0xD8CF, 0x1D56, 0x1664, 0xD3FD, 0xF407, 0x319E, 0x3AAC, 0xFF35, 0x2CC8, 0xE951, 0xE263, 0x27FA,
        0xAD97, 0x680E, 0x633C, 0xA6A5, 0x7558, 0xB0C1, 0xBBF3, 0x7E6A, 0x5990, 0x9C09, 0x973B, 0x52A2, 0x815F, 0x44C6, 0x4FF4, 0x8A6D,
        0x5B2E, 0x9EB7, 0x9585, 0x501C, 0x83E1, 0x4678, 0x4D4A, 0x88D3, 0xAF29, 0x6AB0, 0x6182, 0xA41B, 0x77E6, 0xB27F, 0xB94D, 0x7CD4,
        0xF6B9, 0x3320, 0x3812, 0xFD8B, 0x2E76, 0xEBEF, 0xE0DD, 0x2544, 0x02BE, 0xC727, 0xCC15, 0x098C, 0xDA71, 0x1FE8, 0x14DA, 0xD143,
        0xF3C5, 0x365C, 0x3D6E, 0xF8F7, 0x2B0A, 0xEE93, 0xE5A1, 0x2038, 0x07C2, 0xC25B, 0xC969, 0x0CF0, 0xDF0D, 0x1A94, 0x11A6, 0xD43F,
        0x5E52, 0x9BCB, 0x90F9, 0x5560, 0x869D, 0x4304, 0x4836, 0x8DAF, 0xAA55, 0x6FCC, 0x64FE, 0xA167, 0x729A, 0xB703, 0xBC31, 0x79A8,
        0xA8EB, 0x6D72, 0x6640, 0xA3D9, 0x7024, 0xB5BD, 0xBE8F, 0x7B16, 0x5CEC, 0x9975, 0x9247, 0x57DE, 0x8423, 0x41BA, 0x4A88, 0x8F11,
        0x057C, 0xC0E5, 0xCBD7, 0x0E4E, 0xDDB3, 0x182A, 0x1318, 0xD681, 0xF17B, 0x34E2, 0x3FD0, 0xFA49, 0x29B4, 0xEC2D, 0xE71F, 0x2286,
        0xA213, 0x678A, 0x6CB8, 0xA921, 0x7ADC, 0xBF45, 0xB477, 0x71EE, 0x5614, 0x938D, 0x98BF, 0x5D26, 0x8EDB, 0x4B42, 0x4070, 0x85E9,
        0x0F84, 0xCA1D, 0xC12F, 0x04B6, 0xD74B, 0x12D2, 0x19E0, 0xDC79, 0xFB83, 0x3E1A, 0x3528, 0xF0B1, 0x234C, 0xE6D5, 0xEDE7, 0x287E,
        0xF93D, 0x3CA4, 0x3796, 0xF20F, 0x21F2, 0xE46B, 0xEF59, 0x2AC0, 0x0D3A, 0xC8A3, 0xC391, 0x0608, 0xD5F5, 0x106C, 0x1B5E, 0xDEC7,
        0x54AA, 0x9133, 0x9A01, 0x5F98, 0x8C65, 0x49FC, 0x42CE, 0x8757, 0xA0AD, 0x6534, 0x6E06, 0xAB9F, 0x7862, 0xBDFB, 0xB6C9, 0x7350,
        0x51D6, 0x944F, 0x9F7D, 0x5AE4, 0x8919, 0x4C80, 0x47B2, 0x822B, 0xA5D1, 0x6048, 0x6B7A, 0xAEE3, 0x7D1E, 0xB887, 0xB3B5, 0x762C,
        0xFC41, 0x39D8, 0x32EA, 0xF773, 0x248E, 0xE117, 0xEA25, 0x2FBC, 0x0846, 0xCDDF, 0xC6ED, 0x0374, 0xD089, 0x1510, 0x1E22, 0xDBBB,
        0x0AF8, 0xCF61, 0xC453, 0x01CA, 0xD237, 0x17AE, 0x1C9C, 0xD905, 0xFEFF, 0x3B66, 0x3054, 0xF5CD, 0x2630, 0xE3A9, 0xE89B, 0x2D02,
        0xA76F, 0x62F6, 0x69C4, 0xAC5D, 0x7FA0, 0xBA39, 0xB10B, 0x7492, 0x5368, 0x96F1, 0x9DC3, 0x585A, 0x8BA7, 0x4E3E, 0x450C, 0x8095
    };

    template <typename T, size_t S>
    [[nodiscard]] static constexpr uint16_t PEC15Calc(std::array<T, S> const & data) noexcept {
        uint16_t PEC{ 16 }, addr{ 0 };

        // This has to be done to compute the ADC commands (and others) at compile time.
        if constexpr (sizeof(T) > 1) {
            auto serialized = reinterpret_cast<uint8_t const*>(&data);
            for (uint8_t i = 0; i < sizeof(data); ++i) {
                addr = (PEC >> 7 ^ serialized[i]) & 0xFF;
                PEC = PEC << 8 ^ crc15Table[addr];
            }
        } else {
            for (uint8_t i = 0; i < sizeof(data); ++i) {
                addr = (PEC >> 7 ^ data[i]) & 0xFF;
                PEC = PEC << 8 ^ crc15Table[addr];
            }
        }

        PEC <<= 1; // The final PEC is the 15-bit value in the PEC register with a 0 bit appended to its LSB.
        return (PEC << 8 & 0xFF00) | (PEC >> 8);
    }
};

#endif /* LTC6811_H_ */

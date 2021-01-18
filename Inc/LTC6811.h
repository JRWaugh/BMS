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
#include <cmath>
#include <optional>
#include "main.h"
#include "DWTWrapper.h"

namespace LTC6811 {
/* * * CELL STACK SIZE * * */
inline constexpr auto kDaisyChainLength{ 1 };

/* * * ENUMS AND TYPE DEFINITIONS * * */
/*  ADC Mode  */
enum MD   { Default, Fast, Normal, Filtered };
/*  Discharge Permitted  */
enum DCP  { NotPermitted, Permitted };
/*  Cell Selection for ADC Conversion  */
enum CH   { AllCells, OneAndSeven, TwoAndEight, ThreeAndNine, FourAndTen, FiveAndEleven, SixAndTwelve };
/*  Self Test Mode Selection  */
enum ST   { SelfTest1 = 1, SelfTest2 };
/*  GPIO Selection for ADC Conversion  */
enum CHG  { AllAux, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, VREF2 };
/*  Status Group Selection  */
enum CHST { AllStat, SOC, ITMP, VA, VD };
/* Behaviour when updating discharge config */
enum DischargeMode {
    GTMinPlusDelta, MaxOnly, GTMeanPlusDelta
};

template <typename T>
struct Register {
    /* An LTC6811 register is 8 bytes: 6 bytes of data, and 2 bytes of PEC. Type T must be one or two bytes. */
    std::array<T, 6 / sizeof(T)> data;
    uint16_t PEC;

    static_assert(std::is_same_v<T, uint8_t> || std::is_same_v<T, uint16_t> || std::is_same_v<T, int8_t> || std::is_same_v<T, int16_t>, "Register is of invalid type.");
};

template<typename T>
using RegisterGroup = std::array<Register<T>, kDaisyChainLength>;

using CellData = std::array<RegisterGroup<uint16_t>, 4>;
using TempData = std::array<RegisterGroup<int16_t>, 2>;
using StatusData = std::array<RegisterGroup<uint16_t>, 2>;

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

/* * * CONFIG CONSTANTS * * */
inline constexpr MD md = Normal;
inline constexpr DCP dcp = NotPermitted;
inline constexpr CH ch = AllCells;
inline constexpr CHG chg = AllAux;
inline constexpr CHST chst = AllStat;
/* Configuration for configuration register 0.
 * Enables GPIOs, sets REFON bit to 1 so that the references are powered and ADC conversions can be initiated more quickly,
 * and sets DTEN pin to 1 so it takes longer for the LTC6811 to re-enter SLEEP state. */
inline constexpr uint8_t CFGR0 = 0b11111110;

void init(SPI_HandleTypeDef*);
[[nodiscard]] std::optional<VoltageStatus> read_cell_data(CellData&) noexcept;
[[nodiscard]] std::optional<TempStatus> read_temp_data(TempData&) noexcept;
RegisterGroup<uint8_t> const & get_config_register_group() noexcept;
uint32_t update_config_register_group(CellData const &, VoltageStatus const &, DischargeMode) noexcept;
}

#endif /* LTC6811_H_ */

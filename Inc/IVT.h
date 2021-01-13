/*
 * IVT.h
 *
 *  Created on: 20 Jun 2020
 *      Author: Joshua
 */

#ifndef IVT_H_
#define IVT_H_

#include "stm32f4xx_hal.h"
#include <atomic>

struct IVT {
    enum State { Charged, NotCharged, Hysteresis, Lost };

    [[nodiscard]] State compare_precharge(uint32_t const sum_of_cells) noexcept;

    void set_current(float const I) noexcept;
    [[nodiscard]] float get_current() noexcept;

    void set_voltage1(float const) noexcept;
    [[nodiscard]] float get_voltage1() noexcept;

    void set_voltage2(float const) noexcept;
    [[nodiscard]] float get_voltage2() noexcept;

    [[nodiscard]] bool is_lost() noexcept;

private:
    float volatile voltage1; // Voltage1. pre in old code.
    float volatile voltage2; // Voltage2. air_p in old code.
    float volatile current;  // Current.
    uint32_t volatile last_update; // In ticks
};

#endif /* IVT_H_ */

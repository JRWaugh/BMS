/*
 * IVT.h
 *
 *  Created on: 20 Jun 2020
 *      Author: Joshua
 */

#ifndef IVT_H_
#define IVT_H_

#include <atomic>

class IVT {
public:
    enum {
        Charged, NotCharged, Hysteresis
    };

    IVT() {};

    std::atomic<float> U1; // Voltage1. pre in old code.
    std::atomic<float> U2; // Voltage2. air_p in old code.
    std::atomic<float> I;  // Current.
    std::atomic<uint32_t> tick{ 0 }; // Time in milliseconds.

    [[nodiscard]] int comparePrecharge(uint32_t const sum_of_cells) const noexcept {
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

    [[nodiscard]] bool isLost() const noexcept {
        return tick > kMaxDelay;
    }

private:
    static constexpr uint32_t kMaxDelay{ 500 }; // time in ms
    static constexpr float kPrechargeMinStartVoltage{ 470.0f };
    static constexpr float kPrechargeMaxEndVoltage{ 450.0f };
    static constexpr uint8_t kHysteresis{ 10 };
};

#endif /* IVT_H_ */

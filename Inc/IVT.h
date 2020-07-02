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

    [[nodiscard]] uint32_t comparePrecharge(uint32_t const sum_of_cells) const noexcept {
        static constexpr float kPrechargeMinStartVoltage{ 470.0f };
        static constexpr float kPrechargeMaxEndVoltage{ 450.0f };
        static constexpr uint8_t kHysteresis{ 10 };

        float const percentage = U1 * 100 / U2;
        float const match_percentage = U2 * 100 / sum_of_cells - 100;
        bool const is_voltage_match = match_percentage < kHysteresis && match_percentage > -kHysteresis;

        if (percentage >= 95 && is_voltage_match && U1 > kPrechargeMinStartVoltage && U2 > kPrechargeMinStartVoltage)
            return Charged;
        else if (U1 < kPrechargeMaxEndVoltage || U2 < kPrechargeMaxEndVoltage)
            return NotCharged;
        else
            return Hysteresis;
    }

    void setCurrent(float const I) noexcept {
        this->I = I;
        mCounter = 0;
    }

    [[nodiscard]] float getCurrent() const noexcept {
        return I;
    }

    void setVoltage1(float const U1) noexcept {
        this->U1 = U1;
        mCounter = 0;
    }

    [[nodiscard]] float getVoltage1() const noexcept {
        return U1;
    }

    void setVoltage2(float const U2) noexcept {
        this->U2 = U2;
        mCounter = 0;
    }

    [[nodiscard]] float getVoltage2() const noexcept {
        return U2;
    }

    void tick() noexcept {
        ++mCounter;

        /* Put anything else you want to happen inside this class each systick.
         * Remember, anything changed inside this function MUST be either:
         * volatile, if it is being either read from or written to separately, or
         * atomic, if it is being read from and written to at the same time (such as with the increment operator). */
    }

    [[nodiscard]] uint32_t getTicks() const noexcept {
        return mCounter;
    }

    [[nodiscard]] bool isLost() const noexcept {
        static constexpr uint32_t kMaxDelay{ 500 }; // time in ms

        return mCounter > kMaxDelay;
    }


private:
    float volatile U1; // Voltage1. pre in old code.
    float volatile U2; // Voltage2. air_p in old code.
    float volatile I;  // Current.
    std::atomic<uint32_t> mCounter{ 0 }; // Time in milliseconds.
};

#endif /* IVT_H_ */

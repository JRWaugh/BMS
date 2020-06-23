/*
 * IVT.h
 *
 *  Created on: 20 Jun 2020
 *      Author: Joshua
 */

#ifndef IVT_H_
#define IVT_H_

class IVT {
public:
    enum {
        Charged, NotCharged, Hysteresis
    };

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

        /* Put anything else you want to happen inside this class each systick (every millisecond) */
    }

    [[nodiscard]] uint32_t getTicks() const noexcept {
        return mCounter;
    }

    [[nodiscard]] bool isLost() const noexcept {
        return mCounter > kMaxDelay;
    }


private:
    float volatile U1; // Voltage1. pre in old code.
    float volatile U2; // Voltage2. air_p in old code.
    float volatile I;  // Current.
    uint32_t volatile mCounter{ 0 }; // Time in milliseconds.

    static constexpr uint32_t kMaxDelay{ 500 }; // time in ms
    static constexpr float kPrechargeMinStartVoltage{ 470.0f };
    static constexpr float kPrechargeMaxEndVoltage{ 450.0f };
    static constexpr uint8_t kHysteresis{ 10 };
};

#endif /* IVT_H_ */

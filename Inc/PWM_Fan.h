/*
 * PWM_Fan.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua Waugh
 */

#ifndef PWMFAN_H_
#define PWMFAN_H_

#include "stm32f4xx_hal.h"

class PWM_Fan {
public:
    enum Mode { Manual, Automatic };

    PWM_Fan(uint8_t const duty_cycle = kLowDutyCycle) {
        setDutyCycle(duty_cycle);
    }

    void setDutyCycle(uint8_t duty_cycle) const noexcept {
        if (duty_cycle > kMaxDutyCycle)
            duty_cycle = kMaxDutyCycle;
        else if (duty_cycle < kMinDutyCycle)
            duty_cycle = kMinDutyCycle;

        /* PWM period is 20000 cycles, so the duty cycle is:
         * (duty_cycle / 100) * 20000 or, duty_cycle * 200
         * TODO: This is probably not working right! */
        TIM2->CCR4 = duty_cycle * 200;
    }

    [[nodiscard]] static uint8_t calcDutyCycle(int16_t const max_temp) noexcept {
        if (max_temp > kHighTemp)
            return kMaxDutyCycle;
        else if (max_temp < kLowTemp)
            return kLowDutyCycle;
        else
            return kM * max_temp + kB; // y = mx + b. From old BMS code, not sure if it's good.
    };

    void setMode(Mode const mode) noexcept {
        this->mode = mode;
    }

    [[nodiscard]] Mode getMode() const noexcept {
        return mode;
    }

private:
    Mode volatile mode{ Automatic };

    static constexpr uint8_t kMinDutyCycle{ 0 };
    static constexpr uint8_t kLowDutyCycle{ 10 };
    static constexpr uint8_t kMaxDutyCycle{ 100 };
    static constexpr uint16_t kLowTemp{ 2000 };
    static constexpr uint16_t kHighTemp{ 6000 };
    static constexpr float kM{ static_cast<float>(kMaxDutyCycle - kLowDutyCycle) / static_cast<float>(kHighTemp - kLowTemp) };
    static constexpr auto kB{ kLowDutyCycle - kM * kLowTemp };
};

#endif /* PWMFAN_H_ */

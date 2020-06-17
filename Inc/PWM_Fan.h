#include "stm32f4xx_hal.h"

class PWM_Fan {
public:
    PWM_Fan(uint8_t duty_cycle = kFanLowDutyCycle) {
        SetFanDutyCycle(duty_cycle);
    }

    void SetFanDutyCycle(uint8_t duty_cycle) const {
        if (!manual_mode) {
            if (duty_cycle > kFanDCMax)
                duty_cycle = kFanDCMax;
            else if (duty_cycle < kFanDCMin)
                duty_cycle = kFanDCMin;

            /* PWM period is 20000 cycles, so the duty cycle is:
             * (duty_cycle / 100) * 20000 or, duty_cycle * 200 */
            TIM2->CCR4 = duty_cycle * 200;
        }
    }

    uint8_t CalcDutyCycle(int16_t const max_temp) const {
        if (max_temp > kT2DCHighTemp)
            return kFanDCMax;
        else if (max_temp < kT2DCLowTemp)
            return kFanLowDutyCycle;
        else
            return (max_temp * kT2DC_M) + (kFanLowDutyCycle - kT2DC_M * kT2DCLowTemp);
    };

    bool manual_mode{ false };

private:
    static constexpr uint8_t kFanLowDutyCycle{ 10 };
    static constexpr uint8_t kFanHighDutyCycle{ 100 };
    static constexpr uint8_t kFanDCMax{ 100 };
    static constexpr uint8_t kFanDCMin{ 0 };
    static constexpr uint16_t kT2DCLowTemp{ 2000 };
    static constexpr uint16_t kT2DCHighTemp{ 6000 };
    static constexpr float kT2DC_M{ static_cast<float>(kFanHighDutyCycle - kFanLowDutyCycle) / (kT2DCHighTemp - kT2DCLowTemp) };
};

#include "IVT.h"

[[nodiscard]] IVT::State IVT::compare_precharge(uint32_t const sum_of_cells) noexcept {
    static constexpr float kPrechargeMinStartVoltage{ 470.0f };
    static constexpr float kPrechargeMaxEndVoltage{ 450.0f };
    static constexpr uint8_t kHysteresis{ 10 };

    if (is_lost())
        return Lost;

    float const percentage = voltage1 * 100 / voltage2;
    float const match_percentage = voltage2 * 100 / sum_of_cells - 100;
    bool const is_voltage_match = match_percentage < kHysteresis && match_percentage > -kHysteresis;

    if (percentage >= 95 && is_voltage_match && voltage1 > kPrechargeMinStartVoltage && voltage2 > kPrechargeMinStartVoltage)
        return Charged;
    else if (voltage1 < kPrechargeMaxEndVoltage || voltage2 < kPrechargeMaxEndVoltage)
        return NotCharged;
    else
        return Hysteresis;
}

void IVT::set_current(float const new_current) noexcept {
    current = new_current;
    last_update = uwTick;
}

[[nodiscard]] float IVT::get_current() noexcept {
    return current;
}

void IVT::set_voltage1(float const new_voltage1) noexcept {
    voltage1 = new_voltage1;
    last_update = uwTick;
}

[[nodiscard]] float IVT::get_voltage1() noexcept {
    return voltage1;
}

void IVT::set_voltage2(float const new_voltage2) noexcept {
    voltage2 = new_voltage2;
    last_update = uwTick;
}

[[nodiscard]] float IVT::get_voltage2() noexcept {
    return voltage2;
}

[[nodiscard]] bool IVT::is_lost() noexcept {
    static constexpr uint32_t kMaxDelay{ 500 }; // time in ms / ticks

    return uwTick - last_update > kMaxDelay;
}

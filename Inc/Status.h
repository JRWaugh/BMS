/*
 * Status.h
 *
 *  Created on: 13 Jan 2021
 *      Author: Joshua
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "main.h"
#include <type_traits>

enum Error {
    NoError, OverVoltage, UnderVoltage, Limping, OverTemp, UnderTemp, OverCurrent,
    OverPower, Extern, PECError, AccuUnderVoltage, IVTLost, OverTempCharging,
    NumberOfErrors
};

enum Op_Mode {
    Core = 1 << 0, Balance = 1 << 1, Charging = 1 << 2, Debug = 1 << 3, Logging = 1 << 4
};

static constexpr  int32_t kMaxPower{ 8000000 };
static constexpr uint16_t kMinVoltage{ 31000 }, kMaxVoltage{ 42000 }, kLimpMinVoltage{ 34000 };
static constexpr  int16_t kMinTemp{ -1500 }, kMaxTemp{ 5900 }, kMaxChargeTemp{ 4400 };
static constexpr    float kMaxCurrent{ 180.0f }, kAccuMinVoltage{ 490.0f };

class Status {
public:
    Status(uint8_t op_mode);

    void set_precharge_state(GPIO_PinState state) const noexcept;

    [[nodiscard]] GPIO_PinState get_precharge_state() const noexcept;

    void set_AIR_state(GPIO_PinState state) const noexcept;

    [[nodiscard]] GPIO_PinState get_AIR_state() const noexcept;

    bool is_error(Error const e, bool const error) noexcept;

    uint8_t get_op_mode() const noexcept {
        return op_mode;
    }

    void set_op_mode(Op_Mode op_mode) noexcept {
        this->op_mode = op_mode;
    }

    Error get_last_error() const noexcept {
        return last_error;
    }

    uint8_t get_error_count(Error error) {
        return error_counters[error];
    }

    bool get_error_over_limit(Error error) {
        return error_counters[error] > error_limits[error];
    }
private:
    uint8_t volatile op_mode;
    Error last_error{ NoError };
    uint8_t error_counters[NumberOfErrors];
    static constexpr uint8_t error_limits[NumberOfErrors] {
        [NoError] = 0, [OverVoltage] = 2, [UnderVoltage] = 2, [Limping] = 2,
                [OverTemp] = 2,
                [UnderTemp] = 2,
                [OverCurrent] = 2,
                [OverPower] = 2,
                [Extern] = 2,
                [PECError] = 2,
                [AccuUnderVoltage] = 2,
                [IVTLost] = 1,
                [OverTempCharging] = 2
    };
};

#endif /* STATUS_H_ */

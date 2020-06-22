/*
 * Struct.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua Waugh
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include <ctime>
#include <atomic>

/*** Debug functionality enable/disable ***/
#define BMS_RELAY_CTRL_BYPASS		0
#define STOP_CORE_ON_SAFE_STATE		1
#define START_DEBUG_ON_SAFE_STATE	1
#define BYPASS_INITIAL_CHECK		0
#define SKIP_PEC_ERROR_ACTIONS		1

enum State : bool {
    Open, Closed
};

class Counter {
public:
    Counter(int8_t const limit) : count{ 0 }, limit{ limit } {}

    Counter& incrementBy(uint8_t const amount = 1) noexcept {
        count += amount;

        return *this;
    }

    Counter& decrementBy(uint8_t const amount = 1) noexcept {
        count -= amount;

        if (count < 0)
            count = 0;

        return *this;
    }

    [[nodiscard]] bool isOverLimit() const noexcept {
        return count > limit;
    }

    [[nodiscard]] int8_t getCount() const noexcept {
        return count;
    }

private:
    int8_t count;
    int8_t const limit;
};

struct Status {
public:
    enum Error : uint8_t {
        NoError, OverVoltage, UnderVoltage, Limping, OverTemp, UnderTemp, OverCurrent,
        OverPower, Extern, PECError, AccuUnderVoltage, IVTLost, OverTempCharging,
        NumberOfErrors
    };

    enum OpMode {
        Core = 1 << 0, Balance = 1 << 1, Charging = 1 << 2, Debug = 1 << 3, Logging = 1 << 4
    };

    static constexpr int32_t kMaxPower{ 8000000 };
    static constexpr uint16_t kMaxVoltage{ 42000 };
    static constexpr uint16_t kMinVoltage{ 31000 };
    static constexpr int16_t kMaxTemp{ 5900 };
    static constexpr int16_t kMinTemp{ -1500 };
    static constexpr int16_t kMaxChargeTemp{ 4400 };
    static constexpr uint16_t kLimpMinVoltage{ 34000 };
    static constexpr float kMaxCurrent{ 180.0f };
    static constexpr float kAccuMinVoltage{ 490.0f };

    Status(uint8_t const op_mode) : op_mode { op_mode } {
        setPrechargeState(Open);
        setAIRState(Open);
    };

    void setOpMode(uint8_t const op_mode) noexcept {
        this->op_mode = op_mode;
    }

    [[nodiscard]] uint8_t getOpMode(void) const noexcept {
        return op_mode;
    }

    /* Energize / De-energize Pre-charge Relay. */
    void setPrechargeState(State const state) noexcept {
        HAL_GPIO_WritePin(PreCharge_GPIO_Port, PreCharge_Pin, static_cast<GPIO_PinState>(state));
        HAL_GPIO_WritePin(Led1_GPIO_Port, Led1_Pin, static_cast<GPIO_PinState>(state));
        precharge_state = state;
    }

    [[nodiscard]] State getPrechargeState(void) const noexcept {
        return precharge_state;
    }

    /* Energize / De-energize AIR (Accumulator Indicator Relay). */
    void setAIRState(State const state) noexcept {
        HAL_GPIO_WritePin(BMSrelay_GPIO_Port, BMSrelay_Pin, static_cast<GPIO_PinState>(state));
        HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin, static_cast<GPIO_PinState>(state));
        AIR_state = state;
    }

    [[nodiscard]] State getAIRState(void) const noexcept {
        return AIR_state;
    }

    bool isError(Error const e, bool const is_error) noexcept {
        if (is_error) {
            if (error_counters[e].incrementBy(1).isOverLimit()) {
                if (e == Limping)
                    error_counters[e].incrementBy(9); // Add some amount to the counter when limping so that it takes some time to return to non-limping
                else
                    goToSafeState(e); // This function call is the most glaring, ugly side-effect in the entire BMS. Should not be hidden away like this.
            }
        } else {
            error_counters[e].decrementBy(1);
        }

        return is_error;
    }

    [[nodiscard]] uint32_t getErrorCount(Error const e) const noexcept {
        return error_counters[e].getCount();
    }

    [[nodiscard]] bool isErrorOverLimit(Error const e) const noexcept {
        return error_counters[e].isOverLimit();
    }

    [[nodiscard]] uint8_t getLastError(void) const noexcept {
        return last_error;
    }

    std::atomic<uint32_t> tick{ 0 }; // time in ms

    struct tm rtc{ 0 };

private:
    State precharge_state;
    State AIR_state;
    uint8_t op_mode;
    Error last_error{ NoError };
    Counter error_counters[NumberOfErrors] {
        [NoError] = Counter{ 0 },
        [OverVoltage] = Counter{ 2 },
        [UnderVoltage] = Counter{ 2 },
        [Limping] = Counter{ 2 },
        [OverTemp] = Counter{ 2 },
        [UnderTemp] = Counter{ 2 },
        [OverCurrent] = Counter{ 2 },
        [OverPower] = Counter{ 2 },
        [Extern] = Counter{ 2 },
        [PECError] = Counter{ 2 },
        [AccuUnderVoltage] = Counter{ 2 },
        [IVTLost] = Counter{ 1 },
        [OverTempCharging] = Counter{ 2 }
    };

    void goToSafeState(Error const e) noexcept {
#if BMS_RELAY_CTRL_BYPASS
        // Do nothing.
#elif SKIP_PEC_ERROR_ACTIONS
        if (e != PECError) {
            setAIRState(Open);
            setPrechargeState(Open);
        }
#else
        setAIRState(Open);
        setPrechargeState(Open);
#endif
#if STOP_CORE_ON_SAFE_STATE
        op_mode &= ~Core;
#endif
#if START_DEBUG_ON_SAFE_STATE
        op_mode |= Debug;
#endif
        last_error = e;
    }
};

#endif /* STATUS_H_ */

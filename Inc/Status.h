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

    [[nodiscard]] bool getPrechargeState(void) const noexcept {
        return precharge_state;
    }

    /* Energize / De-energize AIR (Accumulator Indicator Relay). */
    void setAIRState(State const state) noexcept {
        HAL_GPIO_WritePin(BMSrelay_GPIO_Port, BMSrelay_Pin, static_cast<GPIO_PinState>(state));
        HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin, static_cast<GPIO_PinState>(state));
        AIR_state = state;
    }

    [[nodiscard]] bool getAIRState(void) const noexcept {
        return AIR_state;
    }

    bool isError(Error const e, bool const is_error) noexcept {
        if (is_error && ++error_counters[e] > error_limits[e]) {
            if (e == Limping)
                error_counters[e] += 9; // Add some amount to the counter when limping so that it takes some time to return to non-limping
            else {
                GoToSafeState(e); // This function call is the most glaring, ugly side-effect in the entire BMS. Should not be hidden away like this.
                error_counters[e] = error_limits[e];
            }
        } else if (error_counters[e] > 0) {
            --error_counters[e];
        }

        return is_error;
    }

    [[nodiscard]] uint32_t getErrorCount(Error const e) const noexcept {
        return error_counters[e];
    }

    [[nodiscard]] uint8_t getLastError(void) const noexcept {
        return last_error;
    }

    [[nodiscard]] bool isLimping() const noexcept {
        return error_counters[Limping] > error_limits[Limping];
    }

    std::atomic<uint32_t> tick{ 0 }; // time in ms

    struct tm rtc{ 0 };

private:
    bool precharge_state{ false };
    bool AIR_state{ false };
    uint8_t op_mode;
    Error last_error{ NoError };
    uint8_t error_counters[NumberOfErrors] { 0 };
    uint8_t error_limits[NumberOfErrors] {
        [NoError] = 0, [OverVoltage] = 2, [UnderVoltage] = 2, [Limping] = 2,
                [OverTemp] = 2, [UnderTemp] = 2, [OverCurrent] = 2,
                [OverPower] = 2, [Extern] = 2, [PECError] = 2,
                [AccuUnderVoltage] = 2, [IVTLost] = 1, [OverTempCharging] = 2
    };

    void GoToSafeState(Error const e) noexcept {
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

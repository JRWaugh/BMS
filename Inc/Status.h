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
#include "Counter.h"

/* * * Debug functionality enable/disable * * */
#define BMS_RELAY_CTRL_BYPASS		0
#define STOP_CORE_ON_SAFE_STATE		1
#define START_DEBUG_ON_SAFE_STATE	1
#define BYPASS_INITIAL_CHECK		0
#define SKIP_PEC_ERROR_ACTIONS		1

enum State : bool {
    Open, Closed
};

class Status {
public:
    enum Error : uint8_t {
        NoError, OverVoltage, UnderVoltage, Limping, OverTemp, UnderTemp, OverCurrent,
        OverPower, Extern, PECError, AccuUnderVoltage, IVTLost, OverTempCharging,
        NumberOfErrors
    };

    enum OpMode {
        Core = 1 << 0, Balance = 1 << 1, Charging = 1 << 2, Debug = 1 << 3, Logging = 1 << 4
    };

    /* * * Error Limits * * */
    static constexpr  int32_t kMaxPower{ 8000000 };
    static constexpr uint16_t kMinVoltage{ 31000 }, kMaxVoltage{ 42000 }, kLimpMinVoltage{ 34000 };
    static constexpr  int16_t kMinTemp{ -1500 }, kMaxTemp{ 5900 }, kMaxChargeTemp{ 4400 };
    static constexpr    float kMaxCurrent{ 180.0f }, kAccuMinVoltage{ 490.0f };

    Status(uint8_t const opMode) : mOpMode { opMode } {
        setPrechargeState(Open);
        setAIRState(Open);
    };

    void setOpMode(uint8_t const opMode) noexcept {
        mOpMode = opMode;
    }

    [[nodiscard]] uint8_t getOpMode() const noexcept {
        return mOpMode;
    }

    /* Energize / De-energize Pre-charge Relay. */
    void setPrechargeState(State const preState) noexcept {
        HAL_GPIO_WritePin(PreCharge_GPIO_Port, PreCharge_Pin, static_cast<GPIO_PinState>(preState));
        HAL_GPIO_WritePin(Led1_GPIO_Port, Led1_Pin, static_cast<GPIO_PinState>(preState));
        mPreState = preState;
    }

    [[nodiscard]] State getPrechargeState() const noexcept {
        return mPreState;
    }

    /* Energize / De-energize AIR (Accumulator Indicator Relay). */
    void setAIRState(State const AIRState) noexcept {
        HAL_GPIO_WritePin(BMSrelay_GPIO_Port, BMSrelay_Pin, static_cast<GPIO_PinState>(AIRState));
        HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin, static_cast<GPIO_PinState>(AIRState));
        mAIRState = AIRState;
    }

    [[nodiscard]] State getAIRState() const noexcept {
        return mAIRState;
    }

    bool isError(Error const e, bool const error) noexcept {
        if (error) {
            ++mErrorCounters[e];
            if (mErrorCounters[e].isOverLimit()) {
                if (e == Limping)
                    mErrorCounters[e] += 9; // Add some amount to the counter when limping so that it takes some time to return to non-limping
                else
                    goToSafeState(e); // This function call is the most glaring, ugly side-effect in the entire BMS. Should not be hidden away like this.
            }
        } else {
            --mErrorCounters[e];
        }

        return error;
    }

    [[nodiscard]] uint32_t getErrorCount(Error const e) const noexcept {
        return mErrorCounters[e].getCount();
    }

    [[nodiscard]] bool isErrorOverLimit(Error const e) const noexcept {
        return mErrorCounters[e].isOverLimit();
    }

    [[nodiscard]] uint8_t getLastError() const noexcept {
        return mLastError;
    }

    [[nodiscard]] uint32_t getUptime() const noexcept {
        /* TODO:
         * This function is returning time in deciseconds, because that's what it seemed like it was doing on the old BMS.
         * If this is wrong, remove the divisor to return milliseconds, or divide by 1000 to return seconds. */
        return mCounter / 10;
    }

    void tick() noexcept {
        ++mCounter;

        /* Put anything else you want to happen inside this class each systick.
         * Remember, anything changed inside this function MUST be either:
         * volatile, if it is being either read from or written to separately, or
         * atomic, if it is being read from and written to at the same time (such as with the increment operator). */
    }

    struct tm volatile rtc{ 0 };

private:
    uint8_t volatile mOpMode;
    Error mLastError{ NoError };
    State mPreState;
    State mAIRState;
    std::atomic<uint32_t> mCounter{ 0 }; // time in ms

    Counter mErrorCounters[NumberOfErrors] {
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
        mOpMode &= ~Core;
#endif
#if START_DEBUG_ON_SAFE_STATE
        mOpMode |= Debug;
#endif
        mLastError = e;
    }
};

#endif /* STATUS_H_ */

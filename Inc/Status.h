/*
 * Struct.h
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "NLG5.h"
#include "LTC6811.h"
#include <ctime>
#include <atomic>

/*** Debug functionality enable/disable ***/
#define CAN_DEBUG					1
#define CAN_ENABLED					1
#define SD_CARD_DEBUG				1
#define FAN_DEBUG					1
#define BMS_RELAY_CTRL_BYPASS		0
#define STOP_CORE_ON_SAFE_STATE		1
#define START_DEBUG_ON_SAFE_STATE	1
#define BYPASS_INITIAL_CHECK		0
#define SKIP_PEC_ERROR_ACTIONS		1

struct Status {
public:
    enum Error {
        NoError, OverVoltage, UnderVoltage, Limping, OverTemp, UnderTemp, OverCurrent,
        OverPower, Extern, PECError, AccuUnderVoltage, IVTLost, OverTempCharging,
        NumberOfErrors
    };

    enum OpMode {
        Core = 1 << 0, Balance = 1 << 1, Charging = 1 << 2, Debug = 1 << 3, Logging = 1 << 4
    };

    Status(uint8_t op_mode) : op_mode { op_mode } {
        OpenPre();
        OpenAIR();
    };

    uint8_t op_mode;

    static constexpr int32_t kMaxPower{ 8000000 };
    static constexpr uint16_t kMaxVoltage{ 42000 };
    static constexpr uint16_t kMinVoltage{ 31000 };
    static constexpr int16_t kMaxTemp{ 5900 };
    static constexpr int16_t kMinTemp{ -1500 };
    static constexpr int16_t kMaxChargeTemp{ 4400 };
    static constexpr uint16_t kLimpMinVoltage{ 34000 };
    static constexpr float kMaxCurrent{ 180.0f };
    static constexpr float kAccuMinVoltage{ 490.0f };


    /* Energize Pre-charge Relay. */
    void ClosePre(void) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // PRECHARGE
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); // LED1
        precharge_flag = true;
    }

    /* De-energize Pre-charge Relay. */
    void OpenPre(void) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // PRECHARGE
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); // LED1
        precharge_flag = false;
    }

    /* Energize AIR (Accumulator Indicator Relay). */
    void CloseAIR(void) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // BMSRelay
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET); // LED2
        AIR_flag = true;
    }

    /* De-energize AIR (Accumulator Indicator Relay). */
    void OpenAIR(void) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // BMSRelay
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET); // LED2
        AIR_flag = false;
    }

    uint32_t getPECError() {
        return error_counters[PECError];
    }

    bool isError(Error const e, bool const error) {
        if (error) {
            if (++error_counters[e] > error_limits[e]) {
                if (e == Limping)
                    error_counters[e] += 9; // Add some amount to the counter when limping so that it takes some time to return to non-limping
                else {
                    GoToSafeState(e); // This function call is the most glaring, ugly side-effect in the entire BMS. Should not be hidden away like this.
                    error_counters[e] = error_limits[e];
                }
            }
        } else if (error_counters[e] > 0) {
            --error_counters[e];
        }

        return error;
    }


    bool isLimping() const {
        return error_counters[Limping] > error_limits[Limping];
    }

    std::atomic<uint32_t> tick{ 0 }; // time in ms
    bool precharge_flag{ false };
    bool AIR_flag { false };
    Error last_error{ NoError };
    struct tm rtc{ 0 };

private:
    uint8_t error_counters[NumberOfErrors] { 0 };
    uint8_t error_limits[NumberOfErrors] {
        [NoError] = 0, [OverVoltage] = 2, [UnderVoltage] = 2, [Limping] = 2,
        [OverTemp] = 2, [UnderTemp] = 2, [OverCurrent] = 2,
        [OverPower] = 2, [Extern] = 2, [PECError] = 2,
        [AccuUnderVoltage] = 2, [IVTLost] = 1, [OverTempCharging] = 2
    };
    uint32_t PEC_counter_last{ 0 };

    void GoToSafeState(Error const  e) {
#if BMS_RELAY_CTRL_BYPASS
// Do nothing.
#elif SKIP_PEC_ERROR_ACTIONS
        if (e != PECError) {
            OpenAIR();
            OpenPre();
        }
#else
        OpenAIR();
        OpenPRE();
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

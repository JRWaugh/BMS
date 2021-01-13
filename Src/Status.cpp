/*
 * Status.cpp
 *
 *  Created on: 13 Jan 2021
 *      Author: Joshua
 */

#include "Status.h"

Status::Status(uint8_t op_mode) : op_mode{ op_mode } {
    set_precharge_state(GPIO_PIN_RESET);
    set_AIR_state(GPIO_PIN_RESET);
};

void Status::set_precharge_state(GPIO_PinState state) const noexcept {
    HAL_GPIO_WritePin(PreCharge_GPIO_Port, PreCharge_Pin, state);
    HAL_GPIO_WritePin(Led1_GPIO_Port, Led1_Pin, state);
}

[[nodiscard]] GPIO_PinState Status::get_precharge_state() const noexcept {
    return HAL_GPIO_ReadPin(PreCharge_GPIO_Port, PreCharge_Pin);
}

void Status::set_AIR_state(GPIO_PinState state) const noexcept {
    HAL_GPIO_WritePin(BMSrelay_GPIO_Port, BMSrelay_Pin, state);
    HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin, state);
}

[[nodiscard]] GPIO_PinState Status::get_AIR_state() const noexcept {
    return HAL_GPIO_ReadPin(BMSrelay_GPIO_Port, BMSrelay_Pin);
}

bool Status::is_error(Error const e, bool const error) noexcept {
    if (error) {
        ++error_counters[e];
        if (error_counters[e] > error_limits[e]) {
            if (e == Limping)
                error_counters[e] += 9; // Add some amount to the counter when limping so that it takes some time to return to non-limping
            else {
#if BMS_RELAY_CTRL_BYPASS
                // Do nothing.
#elif SKIP_PEC_ERROR_ACTIONS
                if (e != PECError) {
                    set_AIR_state(GPIO_PIN_RESET);
                    set_precharge_state(GPIO_PIN_RESET);
                }
#else
                set_AIR_state(GPIO_PIN_RESET);
                setPrechargeState(GPIO_PIN_RESET);
#endif
#if STOP_CORE_ON_SAFE_STATE
                op_mode &= ~Core;
#endif
#if START_DEBUG_ON_SAFE_STATE
                op_mode |= Debug;
#endif
            }
            last_error = e;
        }
    } else if (error_counters[e] > 0) { // If no error, decrement counter if it is greater than 0.
        --error_counters[e];
    }

    return error;
}

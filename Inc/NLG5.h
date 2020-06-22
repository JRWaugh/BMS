/*
 * NLG5.h
 *
 *  Created on: 14 Mar 2020
 *      Author: Joshua Waugh
 */

#ifndef NLG5_H_
#define NLG5_H_

#include "stm32f4xx_hal.h"
#include <atomic>

struct NLG5 {
    /*** Bit definitions in NLG5 Control Bitmap (NLG5_CTLB) ***/
    enum { C_CP_V = 1 << 5, C_C_EL = 1 << 6, C_C_EN = 1 << 7 };

    NLG5(uint16_t const mc_limit = 160, uint16_t const oc_limit = 60, uint16_t const ov_limit = 2990) : mc_limit { mc_limit }, oc_limit { oc_limit }, ov_limit { ov_limit } {};

    void setChargeCurrent(uint16_t const max_voltage) noexcept {
        if (max_voltage > kChargerDis)
            ctrl = 0;
        else if (max_voltage < kChargerEn)
            ctrl = C_C_EN;
    }

    void tick() noexcept {
        /* Every fifth time the timeout occurs, ctrl is set to a reset command if charger is in fault state. Otherwise it is set to a charge command.
         * NOTE: It would be nicer if the NLG5 class had a reference to the CAN struct and sent this stuff itself when it was ready. */
        if (++counter >= kChargerEventTimeout) {
            /* This code used to check the below condition and put the charger event stuff in an else if
             * I'm not sure if I inverted the condition properly, which is why this comment is here! */
#if OLDCODE
            if ((a_buffer[0] == 136 || a_buffer[0] == 152) && (b_buffer[0] == 136 || b_buffer[0] == 152))
#endif
                if ((a_buffer[0] != 136 && a_buffer[0] != 152) || (b_buffer[0] != 136 && b_buffer[0] != 152)) {
                    if (++event_counter >= 5) {
                        ctrl = C_C_EL;
                        event_counter = 0;
                    } else {
                        ctrl = C_C_EN;
                    }
                }

            counter = 0;
            charger_flag = true;
        }
    }

    [[nodiscard]] bool isChargerEvent() noexcept {
        // Basically working like a 1-item queue.
        bool previous = charger_flag.load();

        charger_flag = false;

        return previous;
    }

    uint8_t ctrl;
    uint16_t mc_limit;
    uint16_t oc_limit;
    uint16_t ov_limit;
    uint8_t a_buffer[4];
    uint8_t b_buffer[4];


private:
    uint8_t event_counter{ 0 };
    std::atomic<uint32_t> counter{ 0 };
    std::atomic<bool> charger_flag{ false };

    static constexpr uint16_t kChargerDis{ 41800 };
    static constexpr uint16_t kChargerEn{ 41500 };
    static constexpr uint8_t kChargerEventTimeout{ 100 }; // time in ms
};

#endif /* NLG5_H_ */

/*
 * NLG5.h
 *
 *  Created on: 14 Mar 2020
 *      Author: Joshua
 */

#ifndef NLG5_H_
#define NLG5_H_

#include "stm32f4xx_hal.h"
#include <atomic>

struct NLG5 {
    /*** Bit definitions in NLG5 Control Bitmap (NLG5_CTLB) ***/
    enum { C_CP_V = 1 << 5, C_C_EL = 1 << 6, C_C_EN = 1 << 7 };

    NLG5(uint16_t const mc_limit = 160, uint16_t const oc_limit = 60, uint16_t const ov_limit = 2990) : mc_limit { mc_limit }, oc_limit { oc_limit }, ov_limit { ov_limit } {};

    void SetChargeCurrent(uint16_t const max_voltage) {
        if (max_voltage > kChargerDis)
            ctrl = 0;
        else if (max_voltage < kChargerEn)
            ctrl = C_C_EN;
    }

    bool isChargerEvent() const {
        return tick - previous_tick >= kChargerEventTimeout;
    }

    uint8_t ctrl;
    uint16_t mc_limit;
    uint16_t oc_limit;
    uint16_t ov_limit;
    uint8_t a_buffer[4];
    uint8_t b_buffer[4];
    uint8_t event_counter{ 0 };
    std::atomic<uint32_t> tick{ 0 };
    std::atomic<uint32_t> previous_tick{ 0 };

    static constexpr uint16_t kChargerDis{ 41800 };
    static constexpr uint16_t kChargerEn{ 41500 };
    static constexpr uint8_t kChargerEventTimeout{ 100 }; // time in ms




};

#endif /* NLG5_H_ */

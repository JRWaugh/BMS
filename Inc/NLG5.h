/*
 * NLG5.h
 *
 *  Created on: 14 Mar 2020
 *      Author: Joshua Waugh
 */

#ifndef NLG5_H_
#define NLG5_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include <atomic>

class NLG5 {
private:
    CAN_HandleTypeDef& hcan;
    CAN_TxHeaderTypeDef& TxHeader;
    std::atomic<uint32_t> mCounter{ 0 };

    static constexpr uint16_t kChargerDis{ 41800 };
    static constexpr uint16_t kChargerEn{ 41500 };
    static constexpr uint8_t kChargerEventTimeout{ 100 }; // time in ms

public:
    /*** Bit definitions in NLG5 Control Bitmap (NLG5_CTLB) ***/
    enum { C_CP_V = 1 << 5, C_C_EL = 1 << 6, C_C_EN = 1 << 7 }; // From http://media3.ev-tv.me/BrusaCANbusspec201.pdf

    NLG5(CAN_HandleTypeDef& hcan, CAN_TxHeaderTypeDef& TxHeader, uint16_t const mc_limit = 160, uint16_t const oc_limit = 60, uint16_t const ov_limit = 2990)
    : hcan{ hcan }, TxHeader{ TxHeader}, ctrl{ C_C_EN }, mc_limit { mc_limit }, oc_limit { oc_limit }, ov_limit { ov_limit }, a_buffer{ 0 }, b_buffer{ 0 } {};

    void setChargeCurrent(uint16_t const max_voltage) noexcept {
        if (max_voltage > kChargerDis)
            ctrl = 0;
        else if (max_voltage < kChargerEn)
            ctrl = C_C_EN;
    }

    void tick() noexcept {
        /* Every fifth time the timeout occurs, ctrl is set to a reset command if charger is in fault state. Otherwise it is set to a charge command.
         * NOTE: It would be nicer if the NLG5 class had a reference to the CAN struct and sent this stuff itself when it was ready. */

#if CAN_ENABLED
        static std::atomic<uint8_t> event_counter{ 0 };

        if (++mCounter >= kChargerEventTimeout) {
            mCounter = 0;

            /* Checks specified chargers MOB status */
            if ((a_buffer[0] != 136 && a_buffer[0] != 152) || (b_buffer[0] != 136 && b_buffer[0] != 152)) {
                if (++event_counter >= 5) {
                    ctrl = C_C_EL;
                    event_counter = 0;
                } else {
                    ctrl = C_C_EN;
                }
            }

            TxHeader.StdId = NLGACtrl;
            TxHeader.DLC = 7;
            uint32_t mailbox{ 0 };
            uint8_t data[7] {
                ctrl,
                static_cast<uint8_t>(mc_limit >> 8),
                static_cast<uint8_t>(mc_limit),
                static_cast<uint8_t>(ov_limit >> 8),
                static_cast<uint8_t>(ov_limit),
                static_cast<uint8_t>(oc_limit >> 8),
                static_cast<uint8_t>(oc_limit)
            };

            HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &mailbox);
        }
#endif

        /* Put anything else you want to happen inside this class each systick.
         * Remember, anything changed inside this function MUST be either:
         * volatile, if it is being either read from or written to separately, or
         * atomic, if it is being read from and written to at the same time (such as with the increment operator). */
    }


    uint8_t volatile ctrl;
    uint16_t const mc_limit;
    uint16_t volatile oc_limit;
    uint16_t const ov_limit;
    uint8_t volatile a_buffer[4];
    uint8_t volatile b_buffer[4];
};

#endif /* NLG5_H_ */

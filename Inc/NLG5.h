/*
 * NLG5.h
 *
 *  Created on: 14 Mar 2020
 *      Author: Joshua
 */

#ifndef NLG5_H_
#define NLG5_H_

#include "stm32f4xx_hal.h"

struct NLG5 {
    /*** Bit definitions in NLG5 Control Bitmap (NLG5_CTLB) ***/
    enum { C_CP_V = 1 << 5, C_C_EL = 1 << 6, C_C_EN = 1 << 7 };
    uint8_t ctrl;
    uint16_t mc_limit;
    uint16_t oc_limit;
    uint16_t ov_limit;
    uint8_t a_buffer[4];
    uint8_t b_buffer[4];

    NLG5(uint16_t mc_limit = 160, uint16_t oc_limit = 60, uint16_t ov_limit = 2990) :
        mc_limit { mc_limit }, oc_limit { oc_limit }, ov_limit { ov_limit } {};
};

#endif /* NLG5_H_ */

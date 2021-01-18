/*
 * DWTWrapper.h
 *
 *  Created on: 24 Jun 2020
 *      Author: Joshua Waugh
 */

#include "stm32f4xx_hal.h"

#ifndef DWTWRAPPER_H_
#define DWTWRAPPER_H_

class DWTWrapper {
public:
    static void delay_us(uint32_t const microseconds) noexcept {
        static const DWTWrapper dwtSingleton;
        uint32_t const startTicks = DWT->CYCCNT;
        uint32_t const delayTicks = (SystemCoreClock / 1'000'000) * microseconds;
        while (DWT->CYCCNT - startTicks < delayTicks);
    }

    DWTWrapper(DWTWrapper const&)       = delete;
    void operator=(DWTWrapper const&)   = delete;

private:
    constexpr DWTWrapper() {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    };
};
#endif /* DWTWRAPPER_H_ */

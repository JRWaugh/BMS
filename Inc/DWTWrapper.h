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
    DWTWrapper(DWTWrapper const&)       = delete;
    void operator=(DWTWrapper const&)   = delete;

    [[nodiscard]] static const DWTWrapper& getInstance() noexcept {
        static const DWTWrapper dwtSingleton;

        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        return dwtSingleton;
    }

    void delay(uint32_t const microseconds) const noexcept {
        uint32_t const startTicks = DWT->CYCCNT;
        uint32_t const delayTicks = (SystemCoreClock / 1'000'000) * microseconds;

        while (DWT->CYCCNT - startTicks < delayTicks);
    }

    void setTicks(uint32_t const ticks) const noexcept {
        DWT->CYCCNT = ticks;
    }

    [[nodiscard]] uint32_t getTicks() const noexcept {
        return DWT->CYCCNT;
    }

private:
    constexpr DWTWrapper() {};
};
#endif /* DWTWRAPPER_H_ */

/*
 * Simple microseconds delay routine, utilizing ARM's DWT
 * (Data Watchpoint and Trace Unit) and HAL library.
 * Intended to use with gcc compiler, but I hope it can be used
 * with any other C compiler across the Universe (provided that
 * ARM and CMSIS already invented) :)
 * Max K
 *
 *
 * This file is part of DWT_Delay package.
 * DWT_Delay is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * us_delay is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * http://www.gnu.org/licenses/.
 */

#include "stm32f4xx_hal.h"

#ifndef DWTWRAPPER_H_
#define DWTWRAPPER_H_

class DWTWrapper {
public:
    DWTWrapper(DWTWrapper const&)       = delete;
    void operator=(DWTWrapper const&)   = delete;

    [[nodiscard]] static DWTWrapper& getInstance() noexcept {
        static DWTWrapper dwtWrapper;

        return dwtWrapper;
    }

    void init() const noexcept {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    void delay(uint32_t const microseconds) const noexcept {
        uint32_t startTicks = getTicks();
        uint32_t delayTicks = microseconds * SystemCoreClock / 1000000;

        while (DWT->CYCCNT - startTicks < delayTicks);
    }

    void setTicks(uint32_t const ticks) const noexcept {
        // Not sure why you would want to do this, but why not?
        DWT->CYCCNT = ticks;
    }

    [[nodiscard]] uint32_t getTicks() const noexcept {
        return DWT->CYCCNT;
    }

private:
    constexpr DWTWrapper() {};
};
#endif /* DWTWRAPPER_H_ */

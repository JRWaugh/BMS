/*
 * NVICMutex.h
 *
 *  Created on: 24 Jun 2020
 *      Author: Joshua
 */

#ifndef NVICMUTEX_H_
#define NVICMUTEX_H_

class NVICMutex {
public:
    NVICMutex() : disabled { static_cast<bool>(__get_PRIMASK()) } {
        __disable_irq();
    };

    ~NVICMutex() {
        if (disabled)
            __enable_irq();
    };

private:
    bool const disabled;
};

#endif /* NVICMUTEX_H_ */

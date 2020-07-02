/*
 * Counter.h
 *
 *  Created on: 25 Jun 2020
 *      Author: Joshua
 */

#ifndef COUNTER_H_
#define COUNTER_H_

class Counter {
public:
    Counter(uint16_t const limit) : limit{ limit } {}

    Counter operator++(int) noexcept {
        // Post-increment operator
        Counter old = *this;
        ++count;

        return old;
    }

    constexpr Counter& operator++() noexcept {
        // Pre-increment operator
        ++count;
        return *this;
    }

    Counter operator--(int) noexcept {
        // Post-decrement operator
        Counter old{ *this };
        --count;

        return old;
    }

    constexpr Counter& operator--() noexcept {
        // Pre-decrement operator
        --count;

        return *this;
    }

    template <typename Integer, std::enable_if_t<std::is_integral<Integer>::value, int> = 0>
    friend constexpr Counter operator+(Counter const & lhs, Integer const rhs) noexcept {
        Counter copy{ lhs };
        copy.count += rhs;
        return copy;
    }

    template <typename Integer, std::enable_if_t<std::is_integral<Integer>::value, int> = 0>
    friend constexpr Counter& operator+=(Counter& lhs, Integer const rhs) noexcept {
        lhs.count += rhs;
        return lhs;
    }

    template <typename Integer, std::enable_if_t<std::is_integral<Integer>::value, int> = 0>
    friend constexpr Counter operator-(Counter const & lhs, Integer const rhs) noexcept {
        Counter copy{ lhs };

        if (rhs >= copy.count)
            copy.count = 0;
        else
            copy.count -= rhs;

        return copy;
    }

    template <typename Integer, std::enable_if_t<std::is_integral<Integer>::value, int> = 0>
    friend constexpr Counter& operator-=(Counter& lhs, Integer const rhs) noexcept {
        if (rhs >= lhs.count)
            lhs.count = 0;
        else
            lhs.count -= rhs;

        return lhs;
    }

    [[nodiscard]] bool isOverLimit() const noexcept {
        return count > limit;
    }

    [[nodiscard]] auto getCount() const noexcept {
        return count;
    }

private:
    uint16_t count{ 0 };
    uint16_t const limit;
};

#endif /* COUNTER_H_ */

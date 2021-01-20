/*
 * LTC6811.cpp
 *
 *  Created on: 12 Mar 2020
 *      Author: Joshua Waugh
 */

#include "LTC6811.h"
#include "DWTWrapper.h"

LTC6811::LTC6811(
        void (*chip_select)(bool level),
        void (*spi_transmit)(uint8_t const * tx_buffer, std::size_t size),
        void (*spi_receive)(uint8_t* rx_buffer, std::size_t size)
) : chip_select{ chip_select }, spi_transmit{ spi_transmit }, spi_receive{ spi_receive } {}

void LTC6811::WRCFG(Group group, RegisterGroup<uint8_t>& register_group) {
    static constexpr Command commands[]{ { 0b00000000001 }, { 0b00000100100 } };
    write(commands[group], register_group);
}

bool LTC6811::RDCFG(Group group, RegisterGroup<uint8_t>& register_group) {
    static constexpr Command commands[]{ { 0b00000000010 }, { 0b00000100110 } };
    return read(commands[group], register_group);
}

bool LTC6811::RDCV(Group group, RegisterGroup<uint16_t>& register_group) {
    static constexpr Command commands[]{ { 0b00000000100 }, { 0b00000000110 },
            { 0b00000001000 }, { 0b00000001010 }, { 0b00000001001 }, { 0b00000001011 } };
    return read(commands[group], register_group);
}

bool LTC6811::RDAUX(Group group, RegisterGroup<int16_t>& register_group) {
    static constexpr Command commands[]{ { 0b00000001100 }, { 0b00000001110 },
            { 0b00000001101 }, { 0b00000001111 } };
    return read(commands[group], register_group);
}

bool LTC6811::RDSTAT(Group group, RegisterGroup<uint16_t>& register_group) {
    static constexpr Command commands[]{ { 0b00000010000 }, { 0b00000010010 },
            { 0b00000001101 }, { 0b00000001111 } };
    return read(commands[group], register_group);
}

void LTC6811::ADCV() {
    static constexpr Command command{ 0b01001100000 | md << 7 | dcp << 4 | ch };
    write(command);
}

void LTC6811::ADAX() {
    static constexpr Command command{ 0b10001100000 | md << 7 | chg };
    write(command);
}

void LTC6811::ADSTAT() {
    static constexpr Command command{ 0b10001101000 | md << 7 | chst };
    write(command);
}

void LTC6811::CLRCELL() {
    static constexpr Command command{ 0b11100010001 };
    write(command);
}

void LTC6811::CLRAUX() {
    static constexpr Command command{ 0b11100010010 };
    write(command);
}

void LTC6811::CLRSTAT() {
    static constexpr Command command{ 0b11100010011 };
    write(command);
}

void LTC6811::wake_up() noexcept {
    static constexpr uint16_t tREADY{ 10 }, tWAKE{ 400 }; // Time in us
    static constexpr uint16_t tIDLE{ 5 }, tSLEEP{ 2000 }; // Time in ms. tIDLE is rounded down from 5.5, just to be safe.

    static auto toggle_chip_select = [this](uint16_t pulse_length) {
        for (std::size_t i = 0; i < kDaisyChainLength; ++i) {
            chip_select(false);
            DWTWrapper::delay_us(pulse_length);
            chip_select(true);
        }
    };

    if (HAL_GetTick() - last_tick > tSLEEP || last_tick == 0)
        toggle_chip_select(tWAKE);
    else if (HAL_GetTick() - last_tick > tIDLE)
        toggle_chip_select(tREADY);

    last_tick = HAL_GetTick();
}

void LTC6811::write(Command const & command) {
    wake_up();
    chip_select(false);
    spi_transmit(reinterpret_cast<uint8_t const *>(&command), sizeof(command));
    for (uint8_t value = 0; value == 0; ) // Poll for ADC complete
        spi_receive(&value, 1);
    chip_select(true);
}

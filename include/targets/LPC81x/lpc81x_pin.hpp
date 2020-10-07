// ----------------------------------------------------------------------------
// @file    lpc81x_pin.hpp
// @brief   NXP LPC81x pin class.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_TARGETS_LPC81X_PIN_HPP
#define XARMLIB_TARGETS_LPC81X_PIN_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_specs.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"

#include <array>
#include <cassert>




namespace xarmlib::targets::lpc81x
{

class Pin
{
public:

    using Name = xarmlib::targets::PinName;

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // Function modes (defined to map the PIO register directly)
    enum class FunctionMode
    {
        hiz       = (0 << 3),
        pull_down = (1 << 3),
        pull_up   = (2 << 3),
        repeater  = (3 << 3)
    };

    // Input hysteresis (defined to map the PIO register directly)
    enum class InputHysteresis
    {
        disable = (0 << 5),
        enable  = (1 << 5)
    };

    // Input invert (defined to map the PIO register directly)
    enum class InputInvert
    {
        normal   = (0 << 6),
        inverted = (1 << 6)
    };

    // I2C mode (defined to map the PIO register directly)
    enum class I2cMode
    {
        standard_fast_i2c = (0 << 8),
        standard_gpio     = (1 << 8),
        fast_plus_i2c     = (2 << 8)
    };

    // Open-drain mode (defined to map the PIO register directly)
    enum class OpenDrain
    {
        disable = (0 << 10),
        enable  = (1 << 10)
    };

    // Input filter samples (defined to map the PIO register directly)
    enum class InputFilter
    {
        bypass           = (0 << 11),
        clocks_1_clkdiv0 = (1 << 11) | (0 << 13),
        clocks_1_clkdiv1 = (1 << 11) | (1 << 13),
        clocks_1_clkdiv2 = (1 << 11) | (2 << 13),
        clocks_1_clkdiv3 = (1 << 11) | (3 << 13),
        clocks_1_clkdiv4 = (1 << 11) | (4 << 13),
        clocks_1_clkdiv5 = (1 << 11) | (5 << 13),
        clocks_1_clkdiv6 = (1 << 11) | (6 << 13),
        clocks_2_clkdiv0 = (2 << 11) | (0 << 13),
        clocks_2_clkdiv1 = (2 << 11) | (1 << 13),
        clocks_2_clkdiv2 = (2 << 11) | (2 << 13),
        clocks_2_clkdiv3 = (2 << 11) | (3 << 13),
        clocks_2_clkdiv4 = (2 << 11) | (4 << 13),
        clocks_2_clkdiv5 = (2 << 11) | (5 << 13),
        clocks_2_clkdiv6 = (2 << 11) | (6 << 13),
        clocks_3_clkdiv0 = (3 << 11) | (0 << 13),
        clocks_3_clkdiv1 = (3 << 11) | (1 << 13),
        clocks_3_clkdiv2 = (3 << 11) | (2 << 13),
        clocks_3_clkdiv3 = (3 << 11) | (3 << 13),
        clocks_3_clkdiv4 = (3 << 11) | (4 << 13),
        clocks_3_clkdiv5 = (3 << 11) | (5 << 13),
        clocks_3_clkdiv6 = (3 << 11) | (6 << 13)
    };

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Set mode of normal pins
    static void set_mode(const Name pin_name, const FunctionMode    function_mode,
                                              const OpenDrain       open_drain       = OpenDrain::disable,
                                              const InputFilter     input_filter     = InputFilter::bypass,
                                              const InputInvert     input_invert     = InputInvert::normal,
                                              const InputHysteresis input_hysteresis = InputHysteresis::enable)
    {
        // Exclude NC
        assert(pin_name != Name::nc);

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
        // Exclude true open-drain pins
        assert(pin_name != Name::p0_10 && pin_name != Name::p0_11);
#endif

        const int32_t pio_index = s_pin_number_to_iocon[static_cast<int32_t>(pin_name)];

        LPC_IOCON->PIO[pio_index] = static_cast<uint32_t>(function_mode)
                                  | static_cast<uint32_t>(input_hysteresis)
                                  | static_cast<uint32_t>(input_invert)
                                  | static_cast<uint32_t>(open_drain)
                                  | (1 << 7) // RESERVED
                                  | static_cast<uint32_t>(input_filter);
    }

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    // Set mode of true open-drain pins (only available on P0_10 and P0_11)
    static void set_mode(const Name pin_name, const I2cMode     i2c_mode,
                                              const InputFilter input_filter,
                                              const InputInvert input_invert)
    {
        // Available only on open-drain pins
        assert(pin_name == Name::p0_10 || pin_name == Name::p0_11);

        const int32_t pio_index = s_pin_number_to_iocon[static_cast<int32_t>(pin_name)];

        LPC_IOCON->PIO[pio_index] = static_cast<uint32_t>(input_invert)
                                  | (1 << 7) // RESERVED
                                  | static_cast<uint32_t>(i2c_mode)
                                  | static_cast<uint32_t>(input_filter);
    }
#endif

private:

    // -------------------------------------------------------------------------
    // PRIVATE DEFINITIONS
    // -------------------------------------------------------------------------

    // IOCON pin values
    static constexpr std::array<uint8_t, TARGET_GPIO_COUNT> s_pin_number_to_iocon
    {
        // The following pins are present in all packages

        // PORT0
        0x11,   // p0.0
        0x0b,   // p0.1
        0x06,   // p0.2
        0x05,   // p0.3
        0x04,   // p0.4
        0x03,   // p0.5

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
        // The following pins are only present in
        // TSSOP16 / XSON16 / SO20 / TSSOP20 packages
        0x10,   // p0.6
        0x0f,   // p0.7
        0x0e,   // p0.8
        0x0d,   // p0.9
        0x08,   // p0.10
        0x07,   // p0.11
        0x02,   // p0.12
        0x01,   // p0.13
#endif // (TARGET_PACKAGE_PIN_COUNT >= 16)

#if (TARGET_PACKAGE_PIN_COUNT == 20)
        // The following pins are only present in
        // SO20 / TSSOP20 packages
        0x12,   // p0.14
        0x0a,   // p0.15
        0x09,   // p0.16
        0x00    // p0.17
#endif // (TARGET_PACKAGE_PIN_COUNT == 20)
    };
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_PIN_HPP

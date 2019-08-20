// ----------------------------------------------------------------------------
// @file    lpc84x_pin.hpp
// @brief   NXP LPC84x pin class.
// @date    9 April 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
//
// See README.md file for additional credits and acknowledgments.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// ----------------------------------------------------------------------------

#ifndef __XARMLIB_TARGETS_LPC84X_PIN_HPP
#define __XARMLIB_TARGETS_LPC84X_PIN_HPP

#include "targets/LPC84x/lpc84x_cmsis.hpp"
#include "core/target_specs.hpp"

#include <array>
#include <cassert>

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




class PinDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Pin names according to the target package
        enum class Name
        {
            // The following pins are present in all packages
            p0_0 = 0,
            p0_1,
            p0_2,
            p0_3,
            p0_4,
            p0_5,
            p0_6,
            p0_7,
            p0_8,
            p0_9,
            p0_10,
            p0_11,
            p0_12,
            p0_13,
            p0_14,
            p0_15,
            p0_16,
            p0_17,
            p0_18,
            p0_19,
            p0_20,
            p0_21,
            p0_22,
            p0_23,
            p0_24,
            p0_25,
            p0_26,
            p0_27,
            p0_28,

#if (TARGET_PACKAGE_PIN_COUNT >= 48)
            // The following pins are only present in
            // HVQFN48 / LQFP48 / LQFP64 packages
            p0_29,
            p0_30,
            p0_31,
            p1_0,
            p1_1,
            p1_2,
            p1_3,
            p1_4,
            p1_5,
            p1_6,
            p1_7,
            p1_8,
            p1_9,
#endif // (TARGET_PACKAGE_PIN_COUNT >= 48)

#if (TARGET_PACKAGE_PIN_COUNT == 64)
            // The following pins are only present in
            // LQFP64 packages
            p1_10,
            p1_11,
            p1_12,
            p1_13,
            p1_14,
            p1_15,
            p1_16,
            p1_17,
            p1_18,
            p1_19,
            p1_20,
            p1_21,
#endif // (TARGET_PACKAGE_PIN_COUNT == 64)

            // Not connected
            nc
        };

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

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Set mode of normal pins
        static void set_mode(const Name pin_name, const FunctionMode    function_mode,
                                                  const OpenDrain       open_drain       = OpenDrain::disable,
                                                  const InputFilter     input_filter     = InputFilter::bypass,
                                                  const InputInvert     input_invert     = InputInvert::normal,
                                                  const InputHysteresis input_hysteresis = InputHysteresis::enable)
        {
            // Exclude NC and true open-drain pins
            assert(pin_name != Name::nc && pin_name != Name::p0_10 && pin_name != Name::p0_11);

            const int32_t pin_index = m_pin_number_to_iocon[static_cast<int32_t>(pin_name)];

            LPC_IOCON->PIO[pin_index] = static_cast<uint32_t>(function_mode)
                                      | static_cast<uint32_t>(input_hysteresis)
                                      | static_cast<uint32_t>(input_invert)
                                      | static_cast<uint32_t>(open_drain)
                                      | (1 << 7) // RESERVED
                                      | static_cast<uint32_t>(input_filter);
        }

        // Set mode of true open-drain pins (only available on P0_10 and P0_11)
        static void set_mode(const Name pin_name, const I2cMode     i2c_mode,
                                                  const InputFilter input_filter,
                                                  const InputInvert input_invert)
        {
            // Available only on true open-drain pins
            assert(pin_name == Name::p0_10 || pin_name == Name::p0_11);

            const int32_t pin_index = m_pin_number_to_iocon[static_cast<int32_t>(pin_name)];

            LPC_IOCON->PIO[pin_index] = static_cast<uint32_t>(input_invert)
                                      | (1 << 7) // RESERVED
                                      | static_cast<uint32_t>(i2c_mode)
                                      | static_cast<uint32_t>(input_filter);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // IOCON pin values
        static constexpr std::array<uint8_t, TARGET_GPIO_COUNT> m_pin_number_to_iocon
        {
            // The following pins are present in all packages

            // PORT0
            0x11,   // P0.0
            0x0B,   // P0.1
            0x06,   // P0.2
            0x05,   // P0.3
            0x04,   // P0.4
            0x03,   // P0.5
            0x10,   // P0.6
            0x0F,   // P0.7
            0x0E,   // P0.8
            0x0D,   // P0.9
            0x08,   // P0.10
            0x07,   // P0.11
            0x02,   // P0.12
            0x01,   // P0.13
            0x12,   // P0.14
            0x0A,   // P0.15
            0x09,   // P0.16
            0x00,   // P0.17
            0x1E,   // P0.18
            0x1D,   // P0.19
            0x1C,   // P0.20
            0x1B,   // P0.21
            0x1A,   // P0.22
            0x19,   // P0.23
            0x18,   // P0.24
            0x17,   // P0.25
            0x16,   // P0.26
            0x15,   // P0.27
            0x14,   // P0.28

#if (TARGET_PACKAGE_PIN_COUNT >= 48)
            // The following pins are only present in
            // HVQFN48 / LQFP48 / LQFP64 packages
            0x32,   // P0.29
            0x33,   // P0.30
            0x23,   // P0.31

            // PORT1
            0x24,   // P1.0
            0x25,   // P1.1
            0x26,   // P1.2
            0x29,   // P1.3
            0x2A,   // P1.4
            0x2B,   // P1.5
            0x2E,   // P1.6
            0x31,   // P1.7
            0x1F,   // P1.8
            0x20,   // P1.9
#endif // (TARGET_PACKAGE_PIN_COUNT >= 48)

#if (TARGET_PACKAGE_PIN_COUNT == 64)
            // The following pins are only present in
            // LQFP64 packages
            0x37,   // P1.10
            0x36,   // P1.11
            0x21,   // P1.12
            0x22,   // P1.13
            0x27,   // P1.14
            0x28,   // P1.15
            0x2C,   // P1.16
            0x2D,   // P1.17
            0x2F,   // P1.18
            0x30,   // P1.19
            0x34,   // P1.20
            0x35    // P1.21
#endif // (TARGET_PACKAGE_PIN_COUNT == 64)
        };
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_PIN_HPP

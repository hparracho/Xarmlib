// ----------------------------------------------------------------------------
// @file    lpc81x_pin.hpp
// @brief   NXP LPC81x pin class.
// @date    29 November 2018
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

#ifndef __XARMLIB_TARGETS_LPC81X_PIN_HPP
#define __XARMLIB_TARGETS_LPC81X_PIN_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "core/target_specs.hpp"

#include <array>

namespace xarmlib
{
namespace targets
{
namespace lpc81x
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
            P0_0 = 0,
            P0_1,
            P0_2,
            P0_3,
            P0_4,
            P0_5,

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
            // The following pins are only present in
            // TSSOP16 / XSON16 / SO20 / TSSOP20 packages
            P0_6,
            P0_7,
            P0_8,
            P0_9,
            P0_10,
            P0_11,
            P0_12,
            P0_13,
#endif // (TARGET_PACKAGE_PIN_COUNT >= 16)

#if (TARGET_PACKAGE_PIN_COUNT == 20)
            // The following pins are only present in
            // SO20 / TSSOP20 packages
            P0_14,
            P0_15,
            P0_16,
            P0_17,
#endif // (TARGET_PACKAGE_PIN_COUNT == 20)


            // Not connected
            NC
        };

        // Function modes (defined to map the PIO register directly)
        enum class FunctionMode
        {
            HIZ       = (0 << 3),
            PULL_DOWN = (1 << 3),
            PULL_UP   = (2 << 3),
            REPEATER  = (3 << 3)
        };

        // Input hysteresis (defined to map the PIO register directly)
        enum class InputHysteresis
        {
            DISABLE = (0 << 5),
            ENABLE  = (1 << 5)
        };

        // Input invert (defined to map the PIO register directly)
        enum class InputInvert
        {
            NORMAL   = (0 << 6),
            INVERTED = (1 << 6)
        };

        // I2C mode (defined to map the PIO register directly)
        enum class I2cMode
        {
            STANDARD_FAST_I2C = (0 << 8),
            STANDARD_GPIO     = (1 << 8),
            FAST_PLUS_I2C     = (2 << 8)
        };

        // Open-drain mode (defined to map the PIO register directly)
        enum class OpenDrain
        {
            DISABLE = (0 << 10),
            ENABLE  = (1 << 10)
        };

        // Input filter samples (defined to map the PIO register directly)
        enum class InputFilter
        {
            BYPASS           = (0 << 11),
            CLOCKS_1_CLKDIV0 = (1 << 11) | (0 << 13),
            CLOCKS_1_CLKDIV1 = (1 << 11) | (1 << 13),
            CLOCKS_1_CLKDIV2 = (1 << 11) | (2 << 13),
            CLOCKS_1_CLKDIV3 = (1 << 11) | (3 << 13),
            CLOCKS_1_CLKDIV4 = (1 << 11) | (4 << 13),
            CLOCKS_1_CLKDIV5 = (1 << 11) | (5 << 13),
            CLOCKS_1_CLKDIV6 = (1 << 11) | (6 << 13),
            CLOCKS_2_CLKDIV0 = (2 << 11) | (0 << 13),
            CLOCKS_2_CLKDIV1 = (2 << 11) | (1 << 13),
            CLOCKS_2_CLKDIV2 = (2 << 11) | (2 << 13),
            CLOCKS_2_CLKDIV3 = (2 << 11) | (3 << 13),
            CLOCKS_2_CLKDIV4 = (2 << 11) | (4 << 13),
            CLOCKS_2_CLKDIV5 = (2 << 11) | (5 << 13),
            CLOCKS_2_CLKDIV6 = (2 << 11) | (6 << 13),
            CLOCKS_3_CLKDIV0 = (3 << 11) | (0 << 13),
            CLOCKS_3_CLKDIV1 = (3 << 11) | (1 << 13),
            CLOCKS_3_CLKDIV2 = (3 << 11) | (2 << 13),
            CLOCKS_3_CLKDIV3 = (3 << 11) | (3 << 13),
            CLOCKS_3_CLKDIV4 = (3 << 11) | (4 << 13),
            CLOCKS_3_CLKDIV5 = (3 << 11) | (5 << 13),
            CLOCKS_3_CLKDIV6 = (3 << 11) | (6 << 13)
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Set mode of normal pins
        static void set_mode(const Name pin_name, const FunctionMode    function_mode,
                                                  const OpenDrain       open_drain       = OpenDrain::DISABLE,
                                                  const InputFilter     input_filter     = InputFilter::BYPASS,
                                                  const InputInvert     input_invert     = InputInvert::NORMAL,
                                                  const InputHysteresis input_hysteresis = InputHysteresis::ENABLE)
        {
            // Exclude NC
            assert(pin_name != Name::NC);
#if (TARGET_PACKAGE_PIN_COUNT >= 16)
            // Exclude true open-drain pins
            assert(pin_name != Name::P0_10 && pin_name != Name::P0_11);
#endif
            const int32_t pin_index = m_pin_number_to_iocon[static_cast<int32_t>(pin_name)];

            LPC_IOCON->PIO[pin_index] = static_cast<uint32_t>(function_mode)
                                      | static_cast<uint32_t>(input_hysteresis)
                                      | static_cast<uint32_t>(input_invert)
                                      | static_cast<uint32_t>(open_drain)
                                      | (1 << 7) // RESERVED
                                      | static_cast<uint32_t>(input_filter);
        }

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
        // Set mode of true open-drain pins (only available on P0_10 and P0_11)
        static void set_mode(const Name pin_name, const I2cMode     i2c_mode,
                                                  const InputFilter input_filter,
                                                  const InputInvert input_invert)
        {
            // Available only on true open-drain pins
            assert(pin_name == Name::P0_10 || pin_name == Name::P0_11);

            const int32_t pin_index = m_pin_number_to_iocon[static_cast<int32_t>(pin_name)];

            LPC_IOCON->PIO[pin_index] = static_cast<uint32_t>(input_invert)
                                      | (1 << 7) // RESERVED
                                      | static_cast<uint32_t>(i2c_mode)
                                      | static_cast<uint32_t>(input_filter);
        }
#endif

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

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
            // The following pins are only present in
            // TSSOP16 / XSON16 / SO20 / TSSOP20 packages
            0x10,   // P0.6
            0x0F,   // P0.7
            0x0E,   // P0.8
            0x0D,   // P0.9
            0x08,   // P0.10
            0x07,   // P0.11
            0x02,   // P0.12
            0x01,   // P0.13
#endif // (TARGET_PACKAGE_PIN_COUNT >= 16)

#if (TARGET_PACKAGE_PIN_COUNT == 20)
            // The following pins are only present in
            // SO20 / TSSOP20 packages
            0x12,   // P0.14
            0x0A,   // P0.15
            0x09,   // P0.16
            0x00    // P0.17
#endif // (TARGET_PACKAGE_PIN_COUNT == 20)
        };
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_PIN_HPP

// ----------------------------------------------------------------------------
// @file    lpc81x_syscon_power.hpp
// @brief   NXP LPC81x SYSCON power control / brown-out classes.
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

#ifndef __XARMLIB_TARGETS_LPC81X_SYSCON_POWER_HPP
#define __XARMLIB_TARGETS_LPC81X_SYSCON_POWER_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class PowerDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Power configuration
        enum class Peripheral
        {
            // PDRUNCFG, Power configuration register
            IRCOUT = 0,
            IRC    = 1,
            FLASH  = 2,
            BOD    = 3,
            SYSOSC = 5,
            WDTOSC = 6,
            SYSPLL = 7,
            ACMP   = 15
        };

        // Peripheral reset
        enum class ResetPeripheral
        {
            SPI0 = 0,
            SPI1,
            USARTFRG,
            USART0,
            USART1,
            USART2,
            I2C,
            MRT,
            SCT,
            WKT,
            GPIO,
            FLASH,
            ACMP
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Powers up a block
        static void power_up(const Peripheral peripheral)
        {
            // Apply power to the block by setting low
            LPC_SYSCON->PDRUNCFG &= ~(1 << static_cast<uint32_t>(peripheral));
        }

        // Powers down a block
        static void power_down(const Peripheral peripheral)
        {
            // Disable the block by setting high
            LPC_SYSCON->PDRUNCFG |= (1 << static_cast<uint32_t>(peripheral));
        }

        // Resets a peripheral
        static void reset(const ResetPeripheral peripheral)
        {
            // Assert reset for a peripheral (the peripheral will stay in reset until reset is de-asserted)
            LPC_SYSCON->PRESETCTRL &= ~(1 << static_cast<uint32_t>(peripheral));
            // De-assert reset for a peripheral
            LPC_SYSCON->PRESETCTRL |=  (1 << static_cast<uint32_t>(peripheral));
        }
};




class BrownOutDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Brown-out detector reset or interrupt level
        enum class Level
        {
         // RESERVED        // Brown-out reset at 1.46V ~ 1.63V / interrupt at 1.65V ~ 1.80V (RESERVED?)
            LEVEL_1 = 1,    // Brown-out reset at 2.06V ~ 2.15V / interrupt at 2.22V ~ 2.35V
            LEVEL_2,        // Brown-out reset at 2.35V ~ 2.43V / interrupt at 2.52V ~ 2.66V
            LEVEL_3,        // Brown-out reset at 2.63V ~ 2.71V / interrupt at 2.80V ~ 2.90V
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Sets brown-out detection reset level and enables it
        static void enable_reset(const Level level)
        {
            // BOD reset enable (BODRSTENA) = (1 << 4)
            LPC_SYSCON->BODCTRL |= static_cast<uint32_t>(level) | (1 << 4);
        }

        // Disables brown-out detection reset
        static void disable_reset()
        {
            // BOD reset enable (BODRSTENA) = (1 << 4)
            LPC_SYSCON->BODCTRL &= ~(1 << 4);
        }
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_SYSCON_POWER_HPP

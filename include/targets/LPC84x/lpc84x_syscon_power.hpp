// ----------------------------------------------------------------------------
// @file    lpc84x_syscon_power.hpp
// @brief   NXP LPC84x SYSCON power control / brown-out classes.
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

#ifndef __XARMLIB_TARGETS_LPC84X_SYSCON_POWER_HPP
#define __XARMLIB_TARGETS_LPC84X_SYSCON_POWER_HPP

#include "targets/LPC84x/lpc84x_cmsis.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
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
            froout = 0,
            fro,
            flash,
            bod,
            adc,
            sysosc,
            wdtosc,
            syspll,
            dac0 = 13,
            dac1,
            acmp
        };

        // Peripheral reset
        enum class ResetPeripheral
        {
            // PRESETCTRL0, Peripheral reset control 0 register
            flash = 4,
            i2c0,
            gpio0,
            swm,
            sct,
            wkt,
            mrt,
            spi0,
            spi1,
            crc,
            usart0,
            usart1,
            usart2,
         // RESERVED
            iocon = 18,
            acomp,
            gpio1,
            i2c1,
            i2c2,
            i2c3,
            adc,
            ctimer0,
         // RESERVED
            dac0 = 27,
            gpioint,
            dma,
            usart3,
            usart4,

            // PRESETCTRL1, Peripheral reset control 1 register
            capt = 32,
            dac1,
            frg0,
            frg1
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
            if(static_cast<const uint32_t>(peripheral) < 32)
            {
                // Assert reset for a peripheral (the peripheral will stay in reset until reset is de-asserted)
                LPC_SYSCON->PRESETCTRL0 &= ~(1 << static_cast<uint32_t>(peripheral));
                // De-assert reset for a peripheral
                LPC_SYSCON->PRESETCTRL0 |=  (1 << static_cast<uint32_t>(peripheral));
            }
            else
            {
                // Assert reset for a peripheral (the peripheral will stay in reset until reset is de-asserted)
                LPC_SYSCON->PRESETCTRL1 &= ~(1 << (static_cast<uint32_t>(peripheral) - 32));
                // De-assert reset for a peripheral
                LPC_SYSCON->PRESETCTRL1 |=  (1 << (static_cast<uint32_t>(peripheral) - 32));
            }
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
         // RESERVED        // Brown-out reset at 1.84V ~ 1.97V (RESERVED?)
            level_1 = 1,    // Brown-out reset at 2.05V ~ 2.18V / interrupt at 2.25V ~ 2.38V
            level_2,        // Brown-out reset at 2.35V ~ 2.47V / interrupt at 2.55V ~ 2.66V
            level_3,        // Brown-out reset at 2.63V ~ 2.76V / interrupt at 2.84V ~ 2.92V
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




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_SYSCON_POWER_HPP

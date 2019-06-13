// ----------------------------------------------------------------------------
// @file    kv4x_system.hpp
// @brief   Kinetis KV4x system level configuration class.
// @date    13 June 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_TARGETS_KV4X_SYSTEM_HPP
#define __XARMLIB_TARGETS_KV4X_SYSTEM_HPP

#include "targets/KV4x/kv4x_fsl_clock_config.h"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class SystemDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Possible clock frequencies selection
        enum class Clock
        {
            irc_4mhz_vlpr,      // @ 4 MHz - Very Low Power Run (VLPR) using Fast Internal Reference Clock (IRC @ 4 MHz)
                                // Outputs:
                                // MCGIRCLK: 4 MHz
                                // MCGFFCLK: Inactive
                                // Core clock: 4 MHz
                                // System clock: 4 MHz
                                // Fast peripheral clock: 4 MHz
                                // Bus/Flash clock: 800 kHz
                                // LPO clock: 1 kHz
                                // WDOGCLK: 4 MHz (MCGIRCLK)

            irc_96mhz_run,      // @ 95.977472 MHz - Normal Run (RUN) using Slow Internal Reference Clock (IRC @ 32.768 kHz)
                                // Outputs:
                                // MCGIRCLK: 4 MHz
                                // MCGFFCLK: Inactive
                                // Core clock: 95.977472 MHz
                                // System clock: 95.977472 MHz
                                // Fast peripheral clock: 95.977472 MHz
                                // Bus/Flash clock: 23.994368 MHz
                                // LPO clock: 1 kHz
                                // WDOGCLK: 4 MHz (MCGIRCLK)

            xtal_80mhz_run,     // NOTE: recommended for CAN use
                                // @ 80 MHz - Normal Run (RUN) using external crystal
                                // Outputs:
                                // MCGIRCLK: 4 MHz
                                // MCGFFCLK: Inactive
                                // Core clock: 80 MHz
                                // System clock: 80 MHz
                                // Fast peripheral clock: 80 MHz
                                // Bus/Flash clock: 20 MHz
                                // LPO clock: 1 kHz
                                // WDOGCLK: 4 MHz (MCGIRCLK)

            xtal_160mhz_hsrun   // @ 160 MHz - High Speed Run (HSRUN) using external crystal
                                // Outputs:
                                // MCGIRCLK: 4 MHz
                                // MCGFFCLK: Inactive
                                // Core clock: 160 MHz
                                // System clock: 160 MHz
                                // Fast peripheral clock: 80 MHz
                                // Bus/Flash clock: 20 MHz
                                // LPO clock: 1 kHz
                                // WDOGCLK: 4 MHz (MCGIRCLK)
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // 4 MHz Internal Reference Clock (MCGIRCLK)
        static constexpr int32_t get_internal_reference_clock_frequency()
        {
            return IRC_FAST_4MHZ_FREQ;
        }

        static constexpr int32_t get_core_clock_frequency(const Clock clock)
        {
            switch(clock)
            {
                case Clock::irc_4mhz_vlpr:     return CLOCK_CONFIG_IRC_4MHZ_VLPR_CORE_CLOCK;     break;
                case Clock::irc_96mhz_run:     return CLOCK_CONFIG_IRC_96MHZ_RUN_CORE_CLOCK;     break;
                case Clock::xtal_80mhz_run:    return CLOCK_CONFIG_XTAL_80MHZ_RUN_CORE_CLOCK;    break;
                case Clock::xtal_160mhz_hsrun: return CLOCK_CONFIG_XTAL_160MHZ_HSRUN_CORE_CLOCK; break;
                default:                       return 0;                                         break;
            }
        }

        static constexpr int32_t get_fast_peripheral_clock_frequency(const Clock clock)
        {
            switch(clock)
            {
                case Clock::irc_4mhz_vlpr:     return CLOCK_CONFIG_IRC_4MHZ_VLPR_CORE_CLOCK;         break;
                case Clock::irc_96mhz_run:     return CLOCK_CONFIG_IRC_96MHZ_RUN_CORE_CLOCK;         break;
                case Clock::xtal_80mhz_run:    return CLOCK_CONFIG_XTAL_80MHZ_RUN_CORE_CLOCK;        break;
                case Clock::xtal_160mhz_hsrun: return CLOCK_CONFIG_XTAL_160MHZ_HSRUN_CORE_CLOCK / 2; break;
                default:                       return 0;                                             break;
            }
        }

        static constexpr int32_t get_bus_clock_frequency(const Clock clock)
        {
            switch(clock)
            {
                case Clock::irc_4mhz_vlpr:     return CLOCK_CONFIG_IRC_4MHZ_VLPR_CORE_CLOCK / 5;     break;
                case Clock::irc_96mhz_run:     return CLOCK_CONFIG_IRC_96MHZ_RUN_CORE_CLOCK / 4;     break;
                case Clock::xtal_80mhz_run:    return CLOCK_CONFIG_XTAL_80MHZ_RUN_CORE_CLOCK / 4;    break;
                case Clock::xtal_160mhz_hsrun: return CLOCK_CONFIG_XTAL_160MHZ_HSRUN_CORE_CLOCK / 8; break;
                default:                       return 0;                                             break;
            }
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // 4 MHz fast Internal Reference Clock (IRC)
        static constexpr int32_t IRC_FAST_4MHZ_FREQ = 4000000;
        /*
        // 32 kHz slow Internal Reference Clock (IRC)
        static constexpr int32_t IRC_SLOW_32KHZ_FREQ = 32768;
        // Crystal frequency (8 MHz fixed)
        static constexpr int32_t CRYSTAL_8MHZ_FREQ = BOARD_XTAL0_CLK_HZ;
        // External clock pin input frequency (EXTAL) (currently not implemented)
        static constexpr int32_t CLK_INPUT_PIN_FREQ = 0;
        */
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_SYSTEM_HPP

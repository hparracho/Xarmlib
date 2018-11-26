// ----------------------------------------------------------------------------
// @file    kv4x_system.hpp
// @brief   Kinetis KV4x system level configuration class.
// @date    16 November 2018
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

#ifndef __XARMLIB_TARGETS_KV4X_SYSTEM_HPP
#define __XARMLIB_TARGETS_KV4X_SYSTEM_HPP

#include "targets/KV4x/kv4x_fsl_clock_config.h"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class System
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Possible clock frequencies selection
        enum class Clock
        {
            IRC_4MHZ,           // Very Low Power Run (VLPR) using Internal Reference Clock (IRC) @ 4MHz
                                // Outputs:
                                // MCGFFCLK: Inactive
                                // Core clock: 4 MHz
                                // System clock: 4 MHz
                                // Fast peripheral clock: 4 MHz
                                // Bus/Flash clock: 800 kHz
                                // LPO clock: 1 kHz

            XTAL_94MHZ,         // Normal Run (RUN) using external crystal @ 94MHz
                                // Outputs:
                                // MCGFFCLK: Inactive
                                // Core clock: 94 MHz
                                // System clock: 94 MHz
                                // Fast peripheral clock: 94 MHz
                                // Bus/Flash clock: 23.5 MHz
                                // LPO clock: 1 kHz

            XTAL_168MHZ         // High Speed Run (HSRUN) using external crystal @ 168MHz
                                // Outputs:
                                // MCGFFCLK: Inactive
                                // Core clock: 168 MHz
                                // System clock: 168 MHz
                                // Fast peripheral clock: 84 MHz
                                // Bus/Flash clock: 21 MHz
                                // LPO clock: 1 kHz
        };

        /*
        // 4 MHz fast Internal Reference Clock (IRC)
        static constexpr int32_t IRC_FAST_4MHZ_FREQ = 4000000;
        // 32 kHz slow Internal Reference Clock (IRC)
        static constexpr int32_t IRC_SLOW_32KHZ_FREQ = 32768;
        // Crystal frequency (8 MHz fixed)
        static constexpr int32_t CRYSTAL_8MHZ_FREQ = BOARD_XTAL0_CLK_HZ;
        // External clock pin input frequency (EXTAL) (currently not implemented)
        static constexpr int32_t CLK_INPUT_PIN_FREQ = 0;
        */

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr int32_t get_core_clock_frequency(const Clock clock)
        {
            switch(clock)
            {
                case Clock::IRC_4MHZ:    return CLOCK_CONFIG_IRC_4MHZ_CORE_CLOCK;    break;
                case Clock::XTAL_94MHZ:  return CLOCK_CONFIG_XTAL_94MHZ_CORE_CLOCK;  break;
                case Clock::XTAL_168MHZ: return CLOCK_CONFIG_XTAL_168MHZ_CORE_CLOCK; break;
                default:                 return 0;                                   break;
            }
        }
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_SYSTEM_HPP

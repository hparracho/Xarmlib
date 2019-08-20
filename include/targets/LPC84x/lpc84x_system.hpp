// ----------------------------------------------------------------------------
// @file    lpc84x_system.hpp
// @brief   NXP LPC84x system level configuration class.
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

#ifndef __XARMLIB_TARGETS_LPC84X_SYSTEM_HPP
#define __XARMLIB_TARGETS_LPC84X_SYSTEM_HPP

#include <cstdint>

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




class SystemDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // SWD interface availability selection
        enum class Swd
        {
            enabled = 0,
            disabled
        };

        // Possible clock frequencies selection
        enum class Clock
        {
            osc_low_power_1125khz = 0,  // Low power boot using internal oscillator set @ 1.125MHz
            osc_low_power_1500khz,      // Low power boot using internal oscillator set @ 1.500MHz
            osc_low_power_1875khz,      // Low power boot using internal oscillator set @ 1.875MHz

            osc_9mhz,                   // Normal boot using internal oscillator set @  9MHz
            osc_12mhz,                  // Normal boot using internal oscillator set @ 12MHz
            osc_15mhz,                  // Normal boot using internal oscillator set @ 15MHz
            osc_18mhz,                  // Normal boot using internal oscillator set @ 18MHz
            osc_24mhz,                  // Normal boot using internal oscillator set @ 24MHz
            osc_30mhz,                  // Normal boot using internal oscillator set @ 30MHz

            xtal_9mhz,                  // Normal boot using external crystal and PPL set @  9MHz
            xtal_12mhz,                 // Normal boot using external crystal and PPL set @ 12MHz
            xtal_15mhz,                 // Normal boot using external crystal and PPL set @ 15MHz
            xtal_18mhz,                 // Normal boot using external crystal and PPL set @ 18MHz
            xtal_24mhz,                 // Normal boot using external crystal and PPL set @ 24MHz
            xtal_30mhz                  // Normal boot using external crystal and PPL set @ 30MHz
        };

        // Crystal frequency (12MHz fixed)
        static constexpr int32_t CRYSTAL_12MHZ_FREQ = 12000000;
        // External clock pin input frequency (currently not implemented)
        static constexpr int32_t CLK_INPUT_PIN_FREQ = 0;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr int32_t get_core_clock_frequency(const Clock clock)
        {
            switch(clock)
            {
                case Clock::osc_low_power_1125khz: return  1125000; break;
                case Clock::osc_low_power_1500khz: return  1500000; break;
                case Clock::osc_low_power_1875khz: return  1875000; break;

                case Clock::osc_9mhz:              return  9000000; break;
                case Clock::osc_12mhz:             return 12000000; break;
                case Clock::osc_15mhz:             return 15000000; break;
                case Clock::osc_18mhz:             return 18000000; break;
                case Clock::osc_24mhz:             return 24000000; break;
                case Clock::osc_30mhz:             return 30000000; break;

                case Clock::xtal_9mhz:             return  9000000; break;
                case Clock::xtal_12mhz:            return 12000000; break;
                case Clock::xtal_15mhz:            return 15000000; break;
                case Clock::xtal_18mhz:            return 18000000; break;
                case Clock::xtal_24mhz:            return 24000000; break;
                case Clock::xtal_30mhz:            return 30000000; break;
                default:                           return        0; break;
            }
        }

        static constexpr int32_t get_main_clock_frequency(const Clock clock)
        {
            switch(clock)
            {
                case Clock::osc_low_power_1125khz: return  1125000; break;
                case Clock::osc_low_power_1500khz: return  1500000; break;
                case Clock::osc_low_power_1875khz: return  1875000; break;

                case Clock::osc_9mhz:              return  9000000; break;
                case Clock::osc_12mhz:             return 12000000; break;
                case Clock::osc_15mhz:             return 15000000; break;
                case Clock::osc_18mhz:             return 18000000; break;
                case Clock::osc_24mhz:             return 24000000; break;
                case Clock::osc_30mhz:             return 30000000; break;

                case Clock::xtal_9mhz:             return 36000000; break;
                case Clock::xtal_12mhz:            return 24000000; break;
                case Clock::xtal_15mhz:            return 60000000; break;
                case Clock::xtal_18mhz:            return 36000000; break;
                case Clock::xtal_24mhz:            return 24000000; break;
                case Clock::xtal_30mhz:            return 60000000; break;
                default:                           return        0; break;
            }
        }
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_SYSTEM_HPP

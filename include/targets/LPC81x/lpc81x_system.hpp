// ----------------------------------------------------------------------------
// @file    lpc81x_system.hpp
// @brief   NXP LPC81x system level configuration class.
// @date    14 July 2018
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

#ifndef __XARMLIB_TARGETS_LPC81X_SYSTEM_HPP
#define __XARMLIB_TARGETS_LPC81X_SYSTEM_HPP

#include "targets/LPC81x/lpc81x_specs.hpp"

#include <cstdint>

namespace xarmlib
{
namespace targets
{
namespace lpc81x
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
            OSC_12MHZ,                  // Using direct 12 MHz internal RC oscillator
            OSC_24MHZ,                  // Using 12 MHz internal RC oscillator and PLL
            OSC_30MHZ,                  // Using 12 MHz internal RC oscillator and PLL

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
			// The use of an external crystal is not possible in DIP8 packages
            XTAL_12MHZ,                 // Using direct external crystal
            XTAL_24MHZ,                 // Using external crystal and PPL
            XTAL_30MHZ                  // Using external crystal and PPL
#endif
        };

        // 12 MHz internal RC oscillator
        static constexpr int32_t IRC_12MHZ_FREQ = 12000000;
        // Crystal frequency (12 MHz fixed)
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
                case Clock::OSC_12MHZ:  return 12000000; break;
                case Clock::OSC_24MHZ:  return 24000000; break;
                case Clock::OSC_30MHZ:  return 30000000; break;

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
                case Clock::XTAL_12MHZ: return 12000000; break;
                case Clock::XTAL_24MHZ: return 24000000; break;
                case Clock::XTAL_30MHZ: return 30000000; break;
#endif
                default:                return        0; break;
            }
        }

        static constexpr int32_t get_main_clock_frequency(const Clock clock)
        {
            switch(clock)
            {
                case Clock::OSC_12MHZ:  return 12000000; break;
                case Clock::OSC_24MHZ:  return 24000000; break;
                case Clock::OSC_30MHZ:  return 60000000; break;

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
                case Clock::XTAL_12MHZ: return 12000000; break;
                case Clock::XTAL_24MHZ: return 24000000; break;
                case Clock::XTAL_30MHZ: return 60000000; break;
#endif
                default:                return        0; break;
            }
        }
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_SYSTEM_HPP

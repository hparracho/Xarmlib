// ----------------------------------------------------------------------------
// @file    lpc84x_system.hpp
// @brief   NXP LPC84x system level configuration class.
// @date    29 March 2018
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
namespace lpc84x
{




class System
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // SWD interface availability selection
        enum class Swd
        {
            ENABLED = 0,
            DISABLED
        };

        // Possible clock frequencies selection
        enum class Clock
        {
            OSC_LOW_POWER_1125KHZ = 0,  // Low power boot using internal oscillator set @ 1.125MHz
            OSC_LOW_POWER_1500KHZ,      // Low power boot using internal oscillator set @ 1.500MHz
            OSC_LOW_POWER_1875KHZ,      // Low power boot using internal oscillator set @ 1.875MHz

            OSC_9MHZ,                   // Normal boot using internal oscillator set @  9MHz
            OSC_12MHZ,                  // Normal boot using internal oscillator set @ 12MHz
            OSC_15MHZ,                  // Normal boot using internal oscillator set @ 15MHz
            OSC_18MHZ,                  // Normal boot using internal oscillator set @ 18MHz
            OSC_24MHZ,                  // Normal boot using internal oscillator set @ 24MHz
            OSC_30MHZ,                  // Normal boot using internal oscillator set @ 30MHz

            XTAL_9MHZ,                  // Normal boot using external crystal and PPL set @  9MHz
            XTAL_12MHZ,                 // Normal boot using external crystal and PPL set @ 12MHz
            XTAL_15MHZ,                 // Normal boot using external crystal and PPL set @ 15MHz
            XTAL_18MHZ,                 // Normal boot using external crystal and PPL set @ 18MHz
            XTAL_24MHZ,                 // Normal boot using external crystal and PPL set @ 24MHz
            XTAL_30MHZ                  // Normal boot using external crystal and PPL set @ 30MHz
        };

        // Crystal frequency (12MHz fixed)
        static constexpr int32_t CRYSTAL_12MHZ_FREQ = 12000000;
        // External clock pin input frequency (currently not implemented)
        static constexpr int32_t CLK_INPUT_PIN_FREQ = 0;
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_SYSTEM_HPP

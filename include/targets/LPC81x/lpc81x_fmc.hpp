// ----------------------------------------------------------------------------
// @file    lpc81x_fmc.hpp
// @brief   NXP LPC81x Flash Memory Controller (FMC) class.
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

#ifndef __XARMLIB_TARGETS_LPC81X_FMC_HPP
#define __XARMLIB_TARGETS_LPC81X_FMC_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class FmcDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Flash access time
        enum class AccessTime
        {
            TIME_1_SYSCLK = 0,      // Flash accesses use 1 system clock (valid for up to 20 MHz CPU clock)
            TIME_2_SYSCLK,          // Flash accesses use 2 system clocks (valid for up to 30 MHz CPU clock)
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Set flash memory access time in clocks
        static void set_access_time(const AccessTime clocks)
        {
            LPC_FMC->FLASHCFG = (LPC_FMC->FLASHCFG & (~0x03))
                              | static_cast<uint32_t>(clocks);
        }
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_FMC_HPP

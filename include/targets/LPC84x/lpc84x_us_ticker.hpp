// ----------------------------------------------------------------------------
// @file    lpc84x_us_ticker.hpp
// @brief   NXP LPC84x SysTick timer class (microsecond resolution).
// @date    7 May 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_US_TICKER_HPP
#define __XARMLIB_TARGETS_LPC84X_US_TICKER_HPP

#include "system/chrono"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace lpc84x
{




class UsTicker
{
    protected:

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static std::chrono::microseconds now()
        {
            if(m_initialized == false)
            {
                initialize();
            }

            return std::chrono::microseconds(LPC_SCT->COUNT);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void initialize()
        {
            m_initialized = true;

            // Enable and reset the SCT clock
            Clock::enable(Clock::Peripheral::SCT);
            Power::reset(Power::ResetPeripheral::SCT);

            // Unified counter (32 bits)
            LPC_SCT->CONFIG |= 1;

            // Halt and clear the counter
            LPC_SCT->CTRL |= (1 << 2) | (1 << 3);

            // System Clock -> us_ticker 1MHz
            LPC_SCT->CTRL |= ((SystemCoreClock / 1000000 - 1) << 5);

            // Unhalt the counter - clearing bit 2 of the CTRL register
            LPC_SCT->CTRL &= ~(1 << 2);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        static bool m_initialized;
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_US_TICKER_HPP

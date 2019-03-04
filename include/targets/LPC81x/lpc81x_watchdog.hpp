// ----------------------------------------------------------------------------
// @file    lpc81x_watchdog.hpp
// @brief   NXP LPC81x Watchdog class.
// @date    4 March 2019
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

#ifndef __XARMLIB_TARGETS_LPC81X_WATCHDOG_HPP
#define __XARMLIB_TARGETS_LPC81X_WATCHDOG_HPP

#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"

#include <chrono>

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class WatchdogDriver
{
    protected:

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template<int64_t duration_value, typename Duration = std::chrono::microseconds>
        static void start()
        {
            static_assert(std::chrono::is_duration<Duration>::value, "Duration must be a std::chrono::duration type.");

            assert(m_initialized == false);
            m_initialized = true;

            PowerDriver::power_up(PowerDriver::Peripheral::wdtosc);
            ClockDriver::enable(ClockDriver::Peripheral::wwdt);

            // Disable watchdog
            LPC_WWDT->MOD     = 0;
            LPC_WWDT->WARNINT = 0x00;
            LPC_WWDT->WINDOW  = 0xFFFFFF;

            constexpr ClockDriver::WatchdogConfig wdt_config = ClockDriver::get_watchdog_osc_config<duration_value, Duration>();

            ClockDriver::set_watchdog_osc_frequency(wdt_config.frequency, wdt_config.divider);

            LPC_WWDT->TC  = wdt_config.counter;
            LPC_WWDT->MOD = mod_wden | mod_wdreset | mod_wdprotect | mod_lock;

            reset();
        }

        // Feed the watchdog timer
        static void reset()
        {
            assert(m_initialized == true);

            // Don't want anything else to happen between feeds (like another feed)

            const uint32_t primask = __get_PRIMASK();
            __disable_irq();

            LPC_WWDT->FEED = 0xAA;
            LPC_WWDT->FEED = 0x55;

            __set_PRIMASK(primask);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Windowed Watchdog MOD Register bits
        enum MOD
        {
            mod_wden      = (1 << 0),       // WWDT enable bit
            mod_wdreset   = (1 << 1),       // WWDT reset enable bit
            mod_wdtof     = (1 << 2),       // WWDT time-out flag bit
            mod_wdint     = (1 << 3),       // WWDT warning interrupt flag bit
            mod_wdprotect = (1 << 4),       // WWDT update mode bit
            mod_lock      = (1 << 5)        // WWDT osc lock mode bit
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        inline static bool m_initialized { false };
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif  // __XARMLIB_TARGETS_LPC81X_WATCHDOG_HPP

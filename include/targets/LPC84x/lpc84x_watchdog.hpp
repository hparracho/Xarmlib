// ----------------------------------------------------------------------------
// @file    lpc84x_watchdog.hpp
// @brief   NXP LPC84x Watchdog class.
// @date    21 March 2018
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




#ifndef __XARMLIB_TARGETS_LPC84X_WATCHDOG_HPP
#define __XARMLIB_TARGETS_LPC84X_WATCHDOG_HPP

#ifdef __LPC84X__

#include <assert.h>

#include "xarmlib_chrono.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace lpc84x
{




class Watchdog
{
    public:

        template<int64_t duration_value, typename Duration = std::chrono::microseconds>
        static void start()
        {
            static_assert(is_chrono_duration<Duration>::value, "Duration must be a std::chrono::duration type.");

            assert(m_initialized == false);
            m_initialized = true;

            Power::power_up(Power::Peripheral::WDTOSC);
            Clock::enable(Clock::Peripheral::WWDT);

            // Disable watchdog
            LPC_WWDT->MOD     = 0;
            LPC_WWDT->WARNINT = 0x00;
            LPC_WWDT->WINDOW  = 0xFFFFFF;

            constexpr Clock::WatchdogConfig wdt_config = Clock::get_watchdog_osc_config<Duration, duration_value>();

            Clock::set_watchdog_osc_frequency(wdt_config.frequency, wdt_config.divider);

            LPC_WWDT->TC  = wdt_config.counter;
            LPC_WWDT->MOD = MOD_WDEN | MOD_WDRESET | MOD_WDPROTECT | MOD_LOCK;

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

        static bool m_initialized;

        // Windowed Watchdog MOD Register bits
        enum MOD
        {
            MOD_WDEN      = (1 << 0),       // WWDT enable bit
            MOD_WDRESET   = (1 << 1),       // WWDT reset enable bit
            MOD_WDTOF     = (1 << 2),       // WWDT time-out flag bit
            MOD_WDINT     = (1 << 3),       // WWDT warning interrupt flag bit
            MOD_WDPROTECT = (1 << 4),       // WWDT update mode bit
            MOD_LOCK      = (1 << 5)        // WWDT osc lock mode bit
        };
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __LPC84X__

#endif  // __XARMLIB_TARGETS_LPC84X_WATCHDOG_HPP

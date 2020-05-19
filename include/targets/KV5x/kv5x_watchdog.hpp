// ----------------------------------------------------------------------------
// @file    kv5x_watchdog.hpp
// @brief   Kinetis KV5x Watchdog class.
// @date    19 May 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_TARGETS_KV5X_WATCHDOG_HPP
#define __XARMLIB_TARGETS_KV5X_WATCHDOG_HPP

#include "xarmlib_config.hpp"
#include "fsl_wdog.h"

#include <chrono>

namespace xarmlib
{
namespace targets
{
namespace kv5x
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

            const int64_t duration_us = std::chrono::duration_cast<std::chrono::microseconds>(Duration(duration_value)).count();

            assert(duration_us >= get_min_timeout_us());
            assert(duration_us <= get_max_timeout_us());

            assert(m_initialized == false);
            m_initialized = true;

            const uint32_t timeout_value = convert_us_to_timeout(duration_us);

            const wdog_config_t wdog_config =
            {
                true,                       // Enables WDOG
                kWDOG_LpoClockSource,       // MCGIRCLK (4 MHz) as clock source
                kWDOG_ClockPrescalerDivide4,// Clock source divided by 4
                {                           // wdog_work_mode_t
                    true,                   // Enables WDOG in wait mode
                    false,                  // Disables WDOG in stop mode
                    false                   // Disables WDOG in debug mode
                },
                false,                      // Update write-once register disable
                false,                      // Disables WDOG interrupt
                false,                      // Disables WDOG window mode
                0U,                         // Window value
                timeout_value               // Timeout value
            };

            WDOG_Init(WDOG, &wdog_config);

            reset();
        }

        // Feed the watchdog timer
        static void reset()
        {
            assert(m_initialized == true);

            WDOG_Refresh(WDOG);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Get timeout value (ready to load into TOVAL register) based on supplied duration in microseconds
        static uint32_t convert_us_to_timeout(const int64_t duration_us)
        {
            constexpr int32_t prescaler_output = SystemDriver::get_internal_reference_clock_frequency() / 4;

            return static_cast<uint32_t>(prescaler_output * duration_us / 1000000UL);
        }

        // Get the minimum allowed watchdog timeout in microseconds
        static int64_t get_min_timeout_us()
        {
            // NOTE: timeout value must be always greater than 2xWCT time + 20 bus clock cycles.
            //       (WCT = 256 bus clock cycles)
            //       For further details see the section 28.4.2 - Watchdog configuration time (WCT)
            //       from the reference manual (KV5XP144M240RM)
            constexpr int64_t min_timeout = 532;

            return ((min_timeout * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK))
                  +((min_timeout * 1000000UL % SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK)) != 0));
        }

        // Get the maximum allowed watchdog timeout in microseconds
        static int64_t get_max_timeout_us()
        {
            constexpr int64_t max_timeout = 0xFFFFFFFF;

            constexpr int32_t prescaler_output = SystemDriver::get_internal_reference_clock_frequency() / 4;

            return (max_timeout * 1000000UL / prescaler_output);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        inline static bool m_initialized { false };
};




} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif  // __XARMLIB_TARGETS_KV5X_WATCHDOG_HPP

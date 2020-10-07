// ----------------------------------------------------------------------------
// @file    hal_watchdog_base.hpp
// @brief   Watchdog HAL interface class.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_HAL_WATCHDOG_BASE_HPP
#define XARMLIB_HAL_WATCHDOG_BASE_HPP

#include "core/chrono.hpp"




namespace xarmlib::hal
{

template <typename Driver>
class WatchdogBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Start the watchdog timer with the supplied timeout duration
    template<int64_t duration_value, typename Duration = std::chrono::microseconds>
    static void start()
    {
        static_assert(chrono::is_duration<Duration>::value, "Duration must be a std::chrono::duration type.");

        Driver::template start<duration_value, Duration>();
    }

    // Reset the watchdog timeout duration
    static void reset()
    {
        Driver::reset();
    }
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_WATCHDOG_BASE_HPP

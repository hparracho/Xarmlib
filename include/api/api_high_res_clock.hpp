// ----------------------------------------------------------------------------
// @file    api_high_res_clock.hpp
// @brief   A C++11 Clock representing the hal::UsTicker with a duration of U32.
// @date    1 Obtober 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_API_HIGH_RES_CLOCK_HPP
#define XARMLIB_API_HIGH_RES_CLOCK_HPP

#include "hal/hal_us_ticker.hpp"
#include "core/chrono.hpp"




namespace xarmlib
{

class HighResClock
{
public:

    using duration   = chrono::microseconds_u32;
    using rep        = duration::rep;
    using period     = duration::period;
    using time_point = std::chrono::time_point<HighResClock>;

    static constexpr bool is_steady = false;

    // Read the current time
    static time_point now()
    {
        return time_point{duration{hal::UsTicker::read()}};
    }

    // Wait for (at least) the supplied duration time
    static void wait(const duration dur)
    {
        const auto start = now();

        while(!is_timeout(start, dur))
        {}
    }

    // Check if the supplied duration since the starting point has already passed
    static bool is_timeout(const time_point start, const duration dur)
    {
        return static_cast<rep>((now() - start).count()) >= static_cast<rep>(dur.count());
    }
};

} // namespace xarmlib




#endif // XARMLIB_API_HIGH_RES_CLOCK_HPP

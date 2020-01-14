// ----------------------------------------------------------------------------
// @file    kv5x_watchdog.cpp
// @brief   Kinetis KV5x Watchdog class.
// @date    14 January 2020
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

#include "core/target_specs.hpp"

#ifdef __KV5X__

#include "xarmlib_config.hpp"
#include "targets/KV5x/kv5x_watchdog.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv5x
{




// ----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// ----------------------------------------------------------------------------

int64_t WatchdogDriver::get_min_timeout_us()
{
    // NOTE: timeout value must be always greater than 2xWCT time + 20 bus clock cycles.
    //       (WCT = 256 bus clock cycles)
    //       For further details see the section 28.4.2 - Watchdog configuration time (WCT)
    //       from the reference manual (KV5XP144M240RM)
    constexpr int64_t min_timeout = 532;

    return ((min_timeout * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK))
          +((min_timeout * 1000000UL % SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK)) != 0));
}




} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif // __KV5X__

// ----------------------------------------------------------------------------
// @file    kv4x_watchdog.cpp
// @brief   Kinetis KV4x Watchdog class.
// @date    7 December 2018
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

#include "core/target_specs.hpp"

#ifdef __KV4X__

#include "xarmlib_config.hpp"
#include "targets/KV4x/kv4x_watchdog.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




// ----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// ----------------------------------------------------------------------------

int64_t WatchdogDriver::get_min_timeout_us()
{
    // NOTE: timeout value must be always greater than 2xWCT time + 20 bus clock cycles.
    //       (WCT = 256 bus clock cycles)
    //       For further details see the section 26.4.2 - Watchdog configuration time (WCT)
    //       from the reference manual (KV4XP100M168RM)
    constexpr int64_t min_timeout = 532;

    return ((min_timeout * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK))
          +((min_timeout * 1000000UL % SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK)) != 0));
}




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __KV4X__

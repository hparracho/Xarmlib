// ----------------------------------------------------------------------------
// @file    hal_us_ticker.hpp
// @brief   Microsecond ticker HAL interface class.
// @note    Target KV5x and KV4x takes control of one available Timer (PIT)
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

#ifndef __XARMLIB_HAL_US_TICKER_HPP
#define __XARMLIB_HAL_US_TICKER_HPP

#include <chrono>

namespace xarmlib
{
namespace hal
{




template <typename UsTickerDriver>
class UsTickerBase : protected UsTickerDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Read the current ticker value
        static std::chrono::microseconds now() { return UsTickerDriver::now(); }

        // Wait for the supplied duration time
        static void wait(const std::chrono::microseconds& duration)
        {
            const auto start = now();

            while(static_cast<uint32_t>((now() - start).count()) <
                  static_cast<uint32_t>(duration.count()))
            {}
        }

        static bool is_timeout(const std::chrono::microseconds& start,
                               const std::chrono::microseconds& duration)
        {
            return static_cast<uint32_t>((now() - start).count()) >=
                   static_cast<uint32_t>(duration.count());
        }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{

using UsTicker = UsTickerBase<targets::kv5x::UsTickerDriver>;

} // namespace hal

using UsTicker = hal::UsTicker;

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{

using UsTicker = UsTickerBase<targets::kv4x::UsTickerDriver>;

} // namespace hal

using UsTicker = hal::UsTicker;

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{

using UsTicker = UsTickerBase<targets::lpc84x::UsTickerDriver>;

} // namespace hal

using UsTicker = hal::UsTicker;

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{

using UsTicker = UsTickerBase<targets::lpc81x::UsTickerDriver>;

} // namespace hal

using UsTicker = hal::UsTicker;

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using UsTicker = UsTickerBase<targets::other_target::UsTickerDriver>;

} // namespace hal

using UsTicker = hal::UsTicker;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_US_TICKER_HPP

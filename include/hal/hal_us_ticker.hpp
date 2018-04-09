// ----------------------------------------------------------------------------
// @file    hal_us_ticker.hpp
// @brief   Microsecond ticker HAL interface class.
// @date    9 April 2018
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

#ifndef __XARMLIB_HAL_US_TICKER_HPP
#define __XARMLIB_HAL_US_TICKER_HPP

#include "system/chrono"

namespace xarmlib
{
namespace hal
{




template <class TargetUsTicker>
class UsTicker
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Read the current ticker value
        static std::chrono::microseconds now()
        {
            return TargetUsTicker::now();
        }
        
        // Wait for the supplied duration time
        static void wait(const std::chrono::microseconds duration)
        {
            const auto start = TargetUsTicker::now();

            while((TargetUsTicker::now() - start) < duration)
            {}
        }
};




} // namespace hal
} // namespace xarmlib




#include "system/target.h"




#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_us_ticker.hpp"

namespace xarmlib
{
using UsTicker = hal::UsTicker<lpc84x::UsTicker>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using UsTicker = hal::UsTicker<other_target::UsTicker>;
}

#endif




#endif // __XARMLIB_HAL_US_TICKER_HPP

// ----------------------------------------------------------------------------
// @file    hal_timer.hpp
// @brief   Timer HAL interface class.
// @date    4 April 2018
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

#ifndef __XARMLIB_HAL_TIMER_HPP
#define __XARMLIB_HAL_TIMER_HPP

#include "system/chrono"
#include "system/delegate"
#include "system/target.h"

namespace xarmlib
{
namespace hal
{




template <class TargetTimer>
class Timer : private TargetTimer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        using Callback = typename TargetTimer::Callback;
        using Mode     = typename TargetTimer::Mode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Timer() : TargetTimer()
        {}

        ~Timer()
        {
            TargetTimer::~Timer();
        }

        void start(const std::chrono::microseconds rate_us, const Mode mode)
        {
            TargetTimer::start(rate_us, mode);
        }

        void reload()
        {
            TargetTimer::reload();
        }

        void stop()
        {
            TargetTimer::stop();
        }

        bool is_running() const
        {
            return TargetTimer::is_running();
        }

        bool is_irq_pending() const
        {
            return TargetTimer::is_irq_pending();
        }

        void clear_irq_pending()
        {
            TargetTimer::clear_irq_pending();
        }

        bool is_irq_enabled() const
        {
            return TargetTimer::ir_irq_enabled();
        }

#ifdef __TARGET_TIMER_TYPE_IS_MRT__
        // NOTE: Timer type is a multi-rate timer (single timer with multiple channels).
        //       Only one IRQ and one priority available for all channels.
        static void set_mrt_priority(const int32_t priority)
        {
            TargetTimer::set_mrt_priority(priority);
        }

        void assign_callback(Callback callback)
        {
            TargetTimer::assign_callback(callback);
        }
#else
        // NOTE: Timer type is independent timer (multiple individual timers).
        //       Each timer have their own IRQ with different priorities.
        void assign_callback(Callback callback, const int32_t priority)
        {
            TargetTimer::assign_callback(callback, priority);
        }
#endif

        void remove_callback()
        {
            TargetTimer::remove_callback();
        }
};




} // namespace hal
} // namespace xarmlib




#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_timer.hpp"

namespace xarmlib
{
using Timer = hal::Timer<lpc84x::Timer>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Timer = hal::Timer<other_target::Timer>;
}

#endif




#endif // __XARMLIB_HAL_TIMER_HPP

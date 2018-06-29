// ----------------------------------------------------------------------------
// @file    hal_timer.hpp
// @brief   Timer HAL interface class.
// @date    29 June 2018
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

#include "system/target"

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

        using Mode       = typename TargetTimer::Mode;
        using IrqHandler = typename TargetTimer::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- START / STOP ----------------------------------------------

        using TargetTimer::start;
        using TargetTimer::reload;
        using TargetTimer::stop;
        using TargetTimer::is_running;

        // -------- INTERRUPTS ------------------------------------------------

        using TargetTimer::enable_irq;
        using TargetTimer::disable_irq;

        using TargetTimer::is_enabled_irq;
        using TargetTimer::is_pending_irq;
        using TargetTimer::clear_pending_irq;

        #ifdef __TARGET_TIMER_TYPE_IS_MRT__
        // NOTE: Timer type is a multi-rate timer (single timer with multiple channels).
        //       Only one IRQ and one priority available for all channels.
        using TargetTimer::set_mrt_irq_priority;
#else
        // NOTE: Timer type is independent timer (multiple individual timers).
        //       Each timer have their own IRQ with different priorities.

        using TargetTimer::set_irq_priority;
#endif

        using TargetTimer::assign_irq_handler;
        using TargetTimer::remove_irq_handler;
};




} // namespace hal
} // namespace xarmlib




#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_timer.hpp"

namespace xarmlib
{
using Timer = hal::Timer<targets::lpc84x::Timer>;
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_timer.hpp"

namespace xarmlib
{
using Timer = hal::Timer<targets::lpc81x::Timer>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Timer = hal::Timer<targets::other_target::Timer>;
}

#endif




#endif // __XARMLIB_HAL_TIMER_HPP

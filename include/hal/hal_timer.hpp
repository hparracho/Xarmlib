// ----------------------------------------------------------------------------
// @file    hal_timer.hpp
// @brief   Timer 32-bit HAL interface class.
// @date    4 March 2019
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

#include <chrono>

namespace xarmlib
{
namespace hal
{




template <typename TargetTimerDriver>
class TimerHal : protected TargetTimerDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using IrqHandler = typename TargetTimerDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- START / STOP ----------------------------------------------

        inline void start(const std::chrono::microseconds& rate_us) { TargetTimerDriver::start(rate_us); }
        inline void reload()                                        { TargetTimerDriver::reload(); }
        inline void stop()                                          { TargetTimerDriver::stop(); }
        inline bool is_running() const                              { return TargetTimerDriver::is_running(); }

        // -------- INTERRUPTS ------------------------------------------------

        inline void enable_irq() { TargetTimerDriver::enable_irq(); }
        inline void disable_irq() { TargetTimerDriver::disable_irq(); }
        inline bool is_irq_enabled() const { return TargetTimerDriver::is_irq_enabled(); }

        inline bool is_irq_pending() const { return TargetTimerDriver::is_irq_pending(); }
        inline void clear_irq_pending() { TargetTimerDriver::clear_irq_pending(); }

#ifdef TARGET_TIMER_TYPE_MRT
        // NOTE: Timer type is a multi-rate timer (single timer with multiple channels).
        //       Only one IRQ and one priority available for all channels.
        static inline void set_mrt_irq_priority(const uint32_t irq_priority) { TargetTimerDriver::set_mrt_irq_priority(irq_priority); }
#else
        // NOTE: Timer type is independent timer (multiple individual timers).
        //       Each timer have their own IRQ with different priorities.
        inline void set_irq_priority(const uint32_t irq_priority) { TargetTimerDriver::set_irq_priority(irq_priority); }
#endif

        inline void assign_irq_handler(const IrqHandler& irq_handler) { TargetTimerDriver::assign_irq_handler(irq_handler); }
        inline void remove_irq_handler() { TargetTimerDriver::remove_irq_handler(); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_timer.hpp"

namespace xarmlib
{
using TimerHal = hal::TimerHal<targets::kv4x::TimerDriver>;
//using Timer = TimerHal;
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_timer.hpp"

namespace xarmlib
{
using TimerHal = hal::TimerHal<targets::lpc84x::TimerDriver>;

//class Timer : public TimerHal
//{
//    public:
//
//        // --------------------------------------------------------------------
//        // PUBLIC TYPE ALIASES
//        // --------------------------------------------------------------------
//
//        using Mode = typename TimerHal::Mode;
//
//        // --------------------------------------------------------------------
//        // PUBLIC MEMBER FUNCTIONS
//        // --------------------------------------------------------------------
//
//        inline void start(const std::chrono::microseconds& rate_us, const Mode mode = Mode::FREE_RUNNING) { TimerHal::TimerDriver::start(rate_us, mode); }
//};
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_timer.hpp"

namespace xarmlib
{
using TimerHal = hal::TimerHal<targets::lpc81x::TimerDriver>;

class Timer : public TimerHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Mode = typename TimerHal::Mode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        inline void start(const std::chrono::microseconds& rate_us, const Mode mode = Mode::free_running) { TimerHal::TimerDriver::start(rate_us, mode); }
};
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using TimerHal = hal::TimerHal<targets::other_target::TimerDriver>;
using Timer = TimerHal;
}

#endif




#endif // __XARMLIB_HAL_TIMER_HPP

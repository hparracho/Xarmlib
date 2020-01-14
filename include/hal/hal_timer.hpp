// ----------------------------------------------------------------------------
// @file    hal_timer.hpp
// @brief   Timer 32-bit HAL interface class.
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

#ifndef __XARMLIB_HAL_TIMER_HPP
#define __XARMLIB_HAL_TIMER_HPP

#include <chrono>

namespace xarmlib
{
namespace hal
{




template <typename TimerDriver>
class TimerBase : protected TimerDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using IrqHandler = typename TimerDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- START / STOP ----------------------------------------------

        void start(const std::chrono::microseconds& rate_us) { TimerDriver::start(rate_us); }
        void reload()                                        { TimerDriver::reload(); }
        void stop()                                          { TimerDriver::stop(); }
        bool is_running() const                              { return TimerDriver::is_running(); }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_irq() { TimerDriver::enable_irq(); }
        void disable_irq() { TimerDriver::disable_irq(); }
        bool is_irq_enabled() const { return TimerDriver::is_irq_enabled(); }

        bool is_irq_pending() const { return TimerDriver::is_irq_pending(); }
        void clear_irq_pending() { TimerDriver::clear_irq_pending(); }

#ifdef TARGET_TIMER_TYPE_MRT
        // NOTE: Timer type is a multi-rate timer (single timer with multiple channels).
        //       Only one IRQ and one priority available for all channels.
        static void set_mrt_irq_priority(const uint32_t irq_priority) { TimerDriver::set_mrt_irq_priority(irq_priority); }
#else
        // NOTE: Timer type is independent timer (multiple individual timers).
        //       Each timer have their own IRQ with different priorities.
        void set_irq_priority(const uint32_t irq_priority) { TimerDriver::set_irq_priority(irq_priority); }
#endif

        void assign_irq_handler(const IrqHandler& irq_handler) { TimerDriver::assign_irq_handler(irq_handler); }
        void remove_irq_handler() { TimerDriver::remove_irq_handler(); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_timer.hpp"

namespace xarmlib
{
namespace hal
{

using Timer = TimerBase<targets::kv5x::TimerDriver>;

} // namespace hal

using Timer = hal::Timer;

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_timer.hpp"

namespace xarmlib
{
namespace hal
{

using Timer = TimerBase<targets::kv4x::TimerDriver>;

} // namespace hal

using Timer = hal::Timer;

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_timer.hpp"

namespace xarmlib
{
namespace hal
{

using Timer = TimerBase<targets::lpc84x::TimerDriver>;

} // namespace hal

class Timer : public hal::Timer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Timer;

        using Mode = typename Hal::Mode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void start(const std::chrono::microseconds& rate_us, const Mode mode = Mode::free_running) { Hal::TimerDriver::start(rate_us, mode); }
};

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_timer.hpp"

namespace xarmlib
{
namespace hal
{

using Timer = TimerBase<targets::lpc81x::TimerDriver>;

} // namespace hal

class Timer : public hal::Timer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Timer;

        using Mode = typename Hal::Mode;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void start(const std::chrono::microseconds& rate_us, const Mode mode = Mode::free_running) { Hal::TimerDriver::start(rate_us, mode); }
};

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Timer = TimerBase<targets::other_target::TimerDriver>;

} // namespace hal

using Timer = hal::Timer;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_TIMER_HPP

// ----------------------------------------------------------------------------
// @file    hal_timer16.hpp
// @brief   Timer 16-bit HAL interface class.
// @date    4 December 2018
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

#ifndef __XARMLIB_HAL_TIMER16_HPP
#define __XARMLIB_HAL_TIMER16_HPP

#include <chrono>

namespace xarmlib
{
namespace hal
{




template <typename TargetTimer16Driver>
class Timer16Hal : protected TargetTimer16Driver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using IrqHandler = typename TargetTimer16Driver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- START / STOP ----------------------------------------------

        inline void start(const std::chrono::microseconds& rate_us) { TargetTimer16Driver::start(rate_us); }
        inline void reload()                                        { TargetTimer16Driver::reload(); }
        inline void stop()                                          { TargetTimer16Driver::stop(); }
        inline bool is_running() const                              { return TargetTimer16Driver::is_running(); }

        // -------- INTERRUPTS ------------------------------------------------

        inline void enable_irq() { TargetTimer16Driver::enable_irq(); }
        inline void disable_irq() { TargetTimer16Driver::disable_irq(); }
        inline bool is_irq_enabled() const { return TargetTimer16Driver::is_irq_enabled(); }

        inline bool is_irq_pending() const { return TargetTimer16Driver::is_irq_pending(); }
        inline void clear_irq_pending() { TargetTimer16Driver::clear_irq_pending(); }

        inline void set_irq_priority(const uint32_t irq_priority) { TargetTimer16Driver::set_irq_priority(irq_priority); }

        inline void assign_irq_handler(const IrqHandler& irq_handler) { TargetTimer16Driver::assign_irq_handler(irq_handler); }
        inline void remove_irq_handler() { TargetTimer16Driver::remove_irq_handler(); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_timer16.hpp"

namespace xarmlib
{
using Timer16Hal = hal::Timer16Hal<targets::kv4x::Timer16Driver>;
//using Timer16 = Timer16Hal;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Timer16Hal = hal::Timer16Hal<targets::other_target::Timer16Driver>;
using Timer16 = Timer16Hal;
}

#endif




#endif // __XARMLIB_HAL_TIMER16_HPP

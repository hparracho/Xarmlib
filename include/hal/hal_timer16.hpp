// ----------------------------------------------------------------------------
// @file    hal_timer16.hpp
// @brief   Timer 16-bit HAL interface class.
// @date    9 May 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2019 Helder Parracho (hparracho@gmail.com)
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




template <typename Timer16Driver>
class Timer16Base : protected Timer16Driver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using IrqHandler = typename Timer16Driver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- START / STOP ----------------------------------------------

        void start(const std::chrono::microseconds& rate_us) { Timer16Driver::start(rate_us); }
        void reload()                                        { Timer16Driver::reload(); }
        void stop()                                          { Timer16Driver::stop(); }
        bool is_running() const                              { return Timer16Driver::is_running(); }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_irq() { Timer16Driver::enable_irq(); }
        void disable_irq() { Timer16Driver::disable_irq(); }
        bool is_irq_enabled() const { return Timer16Driver::is_irq_enabled(); }

        bool is_irq_pending() const { return Timer16Driver::is_irq_pending(); }
        void clear_irq_pending() { Timer16Driver::clear_irq_pending(); }

        void set_irq_priority(const uint32_t irq_priority) { Timer16Driver::set_irq_priority(irq_priority); }

        void assign_irq_handler(const IrqHandler& irq_handler) { Timer16Driver::assign_irq_handler(irq_handler); }
        void remove_irq_handler() { Timer16Driver::remove_irq_handler(); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_timer16.hpp"

namespace xarmlib
{
namespace hal
{

using Timer16 = Timer16Base<targets::kv4x::Timer16Driver>;

} // namespace hal

using Timer16 = hal::Timer16;

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Timer16 = Timer16Base<targets::other_target::Timer16Driver>;

} // namespace hal

using Timer16 = hal::Timer16;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_TIMER16_HPP

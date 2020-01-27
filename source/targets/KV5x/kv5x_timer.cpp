// ----------------------------------------------------------------------------
// @file    kv5x_timer.cpp
// @brief   Kinetis KV5x Timer (PIT) class.
// @note    Timers stop in debug mode.
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
#include "targets/KV5x/kv5x_timer.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv5x
{




// ----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// ----------------------------------------------------------------------------

uint32_t TimerDriver::convert_us_to_period(const int64_t rate_us)
{
    return static_cast<uint32_t>(SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK) * rate_us / 1000000UL);
}




int64_t TimerDriver::convert_period_to_us(const uint32_t period)
{
    return (static_cast<int64_t>(period) * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
}




int64_t TimerDriver::get_min_rate_us()
{
    constexpr int64_t min_period = 0x01;

    return (min_period * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
}




int64_t TimerDriver::get_max_rate_us()
{
    constexpr int64_t max_period = 0xFFFFFFFF;

    return (max_period * 1000000UL / SystemDriver::get_bus_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));
}




} // namespace kv5x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::kv5x;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void PIT0_IRQHandler(void)
{
    const int32_t yield = TimerDriver::irq_handler(TimerDriver::Channel::channel0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void PIT1_IRQHandler(void)
{
    const int32_t yield = TimerDriver::irq_handler(TimerDriver::Channel::channel1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void PIT2_IRQHandler(void)
{
    const int32_t yield = TimerDriver::irq_handler(TimerDriver::Channel::channel2);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void PIT3_IRQHandler(void)
{
    const int32_t yield = TimerDriver::irq_handler(TimerDriver::Channel::channel3);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#endif // __KV5X__

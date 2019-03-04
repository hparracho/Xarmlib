// ----------------------------------------------------------------------------
// @file    lpc81x_usart.cpp
// @brief   NXP LPC81x USART class.
// @notes   Synchronous mode not implemented.
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

#include "core/target_specs.hpp"

#ifdef __LPC81X__

#include "xarmlib_config.hpp"
#include "targets/LPC81x/lpc81x_usart.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




// --------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// --------------------------------------------------------------------

void UsartDriver::initialize_usart_frg()
{
    constexpr int32_t main_clk_freq = SystemDriver::get_main_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK);
    constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

    constexpr uint8_t mul = get_frg_mul(usart_freq, main_clk_freq);
    constexpr uint8_t div = 0xFF; // Fixed value to use with the fractional baudrate generator

    // Set the USART FRG fractional divider
    ClockDriver::set_usart_frg_clock_divider(mul, div);
}




int32_t UsartDriver::get_baudrate_generator_div(const int32_t baudrate)
{
    constexpr int32_t main_clk_freq = SystemDriver::get_main_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK);
    constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

    return usart_freq / 16 / baudrate;
}




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::lpc81x;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void USART0_IRQHandler(void)
{
    const int32_t yield = UsartDriver::irq_handler(UsartDriver::Name::usart0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void USART1_IRQHandler(void)
{
    const int32_t yield = UsartDriver::irq_handler(UsartDriver::Name::usart1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#if (TARGET_USART_COUNT == 3)

extern "C" void USART2_IRQHandler(void)
{
    const int32_t yield = UsartDriver::irq_handler(UsartDriver::Name::usart2);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

#endif // (TARGET_USART_COUNT == 3)




#endif // __LPC81X__

// ----------------------------------------------------------------------------
// @file    lpc84x_usart.cpp
// @brief   NXP LPC84x USART class (takes control of FRG0).
// @notes   Synchronous mode not implemented.
// @date    28 June 2018
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

#include "system/target"

#ifdef __LPC84X__

#include "xarmlib_config.hpp"
#include "targets/LPC84x/lpc84x_usart.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




// ----------------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// ----------------------------------------------------------------------------

void Usart::initialize_frg0()
{
    constexpr int32_t main_clk_freq = System::get_main_clock_frequency(XARMLIB_SYSTEM_CLOCK);
    constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

    constexpr uint8_t mul = get_frg_mul(usart_freq, main_clk_freq);
    constexpr uint8_t div = 0xFF; // Fixed value to use with the fractional baudrate generator

    // Select main clock as the source for FRG0
    Clock::set_frg_clock_source(Clock::FrgClockSelect::FRG0, Clock::FrgClockSource::MAIN_CLK);

    // Set the FRG0 fractional divider
    Clock::set_frg_clock_divider(Clock::FrgClockSelect::FRG0, mul, div);
}




int32_t Usart::get_baudrate_generator_div(const int32_t baudrate)
{
    constexpr int32_t main_clk_freq = System::get_main_clock_frequency(XARMLIB_SYSTEM_CLOCK);
    constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

    return usart_freq / 16 / baudrate;
}




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::lpc84x;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void USART0_IRQHandler(void)
{
    const int32_t yield = Usart::irq_handler(Usart::Name::USART0);

#ifdef XARMLIB_USE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void USART1_IRQHandler(void)
{
    const int32_t yield = Usart::irq_handler(Usart::Name::USART1);

#ifdef XARMLIB_USE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#ifdef __LPC845__

extern "C" void USART2_IRQHandler(void)
{
    const int32_t yield = Usart::irq_handler(Usart::Name::USART2);

#ifdef XARMLIB_USE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




// NOTE: USART3 and USART4 interrupts that are shared with PININT6 and
//       PININT7 are implemented in 'lpc84x_shared_interrupts.cpp' file.

#endif // __LPC845__

#endif // __LPC84X__

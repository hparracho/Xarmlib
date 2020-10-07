// ----------------------------------------------------------------------------
// @file    lpc84x_shared_interrupts.cpp
// @brief   NXP LPC84x IRQ handlers that are shared by different peripherals.
// @date    20 September 2020
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

#ifdef __LPC84X__

#include "core/os_support.hpp"
#include "targets/LPC84x/lpc84x_usart.hpp"




using namespace xarmlib;
using namespace xarmlib::targets::lpc84x;

// ----------------------------------------------------------------------------
// SHARED IRQ HANDLERS
// ----------------------------------------------------------------------------

// Analog Comparator / Cap Touch shared handler
extern "C" void ACMP_CAPT_IRQHandler(void)
{}




// PININT5 / DAC1 shared handler
extern "C" void PININT5_DAC1_IRQHandler(void)
{}




// PININT6 / USART3 shared handler
extern "C" void PININT6_USART3_IRQHandler(void)
{
    int32_t yield = 0;

#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
    yield = UsartDriver::irq_handler(UsartDriver::Name::usart3);
#endif

    Os::yield_from_isr(yield);
}




// PININT7 / USART4 shared handler
extern "C" void PININT7_USART4_IRQHandler(void)
{
    int32_t yield = 0;

#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
    yield = UsartDriver::irq_handler(UsartDriver::Name::usart4);
#endif

    Os::yield_from_isr(yield);
}




#endif // __LPC84X__

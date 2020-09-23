// ----------------------------------------------------------------------------
// @file    lpc81x_pin_int.cpp
// @brief   NXP LPC81x pin interrupt class.
// @date    15 September 2020
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

#ifdef __LPC81X__

#include "core/os_support.hpp"
#include "targets/LPC81x/lpc81x_pin_int.hpp"




using namespace xarmlib;
using namespace xarmlib::targets::lpc81x;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void PININT0_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int0);

    Os::yield_from_isr(yield);
}




extern "C" void PININT1_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int1);

    Os::yield_from_isr(yield);
}




extern "C" void PININT2_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int2);

    Os::yield_from_isr(yield);
}




extern "C" void PININT3_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int3);

    Os::yield_from_isr(yield);
}




extern "C" void PININT4_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int4);

    Os::yield_from_isr(yield);
}




extern "C" void PININT5_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int5);

    Os::yield_from_isr(yield);
}




extern "C" void PININT6_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int6);

    Os::yield_from_isr(yield);
}




extern "C" void PININT7_IRQHandler(void)
{
    const int32_t yield = PinIntDriver::irq_handler(PinIntDriver::Name::pin_int7);

    Os::yield_from_isr(yield);
}




#endif // __LPC81X__

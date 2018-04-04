// ----------------------------------------------------------------------------
// @file    lpc84x_timer.cpp
// @brief   NXP LPC84x Timer (MRT) class.
// @date    4 April 2018
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

#include "system/target.h"

#ifdef __LPC84X__

#include <targets/LPC84x/lpc84x_timer.hpp>

namespace xarmlib
{
namespace lpc84x
{




// Static definition
constexpr int32_t                       Timer::NUM_CHANNELS;

// Static initialization
std::array<Timer*, Timer::NUM_CHANNELS> Timer::m_timers_array { nullptr };
uint32_t                                Timer::m_timers_used  { 0 };




// --------------------------------------------------------------------
// IRQ HANDLER
// --------------------------------------------------------------------

extern "C" void MRT_IRQHandler(void)
{
    const int32_t yield = Timer::irq_handler();

#ifdef USE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




} // namespace lpc84x
} // namespace xarmlib

#endif // __LPC84X__

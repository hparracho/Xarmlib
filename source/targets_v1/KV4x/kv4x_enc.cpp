// ----------------------------------------------------------------------------
// @file    kv4x_enc.cpp
// @brief   Kinetis KV4x Quadrature Encoder/Decoder (ENC) class.
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

#ifdef __KV4X__

#include "core/os_support.hpp"
#include "targets/KV4x/kv4x_enc.hpp"




using namespace xarmlib;
using namespace xarmlib::targets::kv4x;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void ENC_HOME_IRQHandler(void)
{
    const int32_t yield = EncDriver::home_irq_handler();

    Os::yield_from_isr(yield);
}

extern "C" void ENC_INDEX_IRQHandler(void)
{
    const int32_t yield = EncDriver::index_irq_handler();

    Os::yield_from_isr(yield);
}

extern "C" void ENC_WDOG_SAB_IRQHandler(void)
{
    const int32_t yield = EncDriver::watchdog_and_sab_irq_handler();

    Os::yield_from_isr(yield);
}

extern "C" void ENC_COMPARE_IRQHandler(void)
{
    const int32_t yield = EncDriver::compare_irq_handler();

    Os::yield_from_isr(yield);
}




#endif // __KV4X__

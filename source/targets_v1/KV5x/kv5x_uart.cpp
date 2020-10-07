// ----------------------------------------------------------------------------
// @file    kv5x_uart.cpp
// @brief   Kinetis KV5x UART class.
// @notes   TX and RX FIFOs are always used due to FSL driver implementation.
//          TX FIFO watermark = 0 and RX FIFO watermark = 1.
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

#ifdef __KV5X__

#include "core/os_support.hpp"
#include "targets/KV5x/kv5x_uart.hpp"




using namespace xarmlib;
using namespace xarmlib::targets::kv5x;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void UART0_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart0);

    Os::yield_from_isr(yield);
}

extern "C" void UART0_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart0);

    Os::yield_from_isr(yield);
}




extern "C" void UART1_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart1);

    Os::yield_from_isr(yield);
}

extern "C" void UART1_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart1);

    Os::yield_from_isr(yield);
}




extern "C" void UART2_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart2);

    Os::yield_from_isr(yield);
}

extern "C" void UART2_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart2);

    Os::yield_from_isr(yield);
}




extern "C" void UART3_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart3);

    Os::yield_from_isr(yield);
}

extern "C" void UART3_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart3);

    Os::yield_from_isr(yield);
}




extern "C" void UART4_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart4);

    Os::yield_from_isr(yield);
}

extern "C" void UART4_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart4);

    Os::yield_from_isr(yield);
}




extern "C" void UART5_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart5);

    Os::yield_from_isr(yield);
}

extern "C" void UART5_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart5);

    Os::yield_from_isr(yield);
}




#endif // __KV5X__

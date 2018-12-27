// ----------------------------------------------------------------------------
// @file    kv4x_uart.cpp
// @brief   Kinetis KV4x UART class.
// @notes   TX and RX FIFOs are always used due to FSL driver implementation.
//          TX FIFO watermark = 0 and RX FIFO watermark = 1.
// @date    27 December 2018
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

#ifdef __KV4X__

#include "xarmlib_config.hpp"
#include "targets/KV4x/kv4x_uart.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




// --------------------------------------------------------------------
// PRIVATE MEMBER FUNCTIONS
// --------------------------------------------------------------------

void UartDriver::initialize(const Config& config)
{
    assert(config.baudrate > 0);

    const uart_config_t uart_config =
    {
        static_cast<uint32_t>(config.baudrate),
        static_cast<uart_parity_mode_t>(config.parity),
        static_cast<uart_stop_bit_count_t>(config.stop_bits),
        0,      // TX FIFO watermark
        1,      // RX FIFO watermark
        false,  // RX RTS disable
        false,  // TX CTS disable
        static_cast<uart_idle_type_select_t>(config.idle_type),
        false,  // Disable TX
        false   // Disable RX
    };

    const int32_t result = UART_Init(m_uart_base, &uart_config, SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

    // Assert baudrate less than 3%
    assert(result == 0);

    if(config.data_bits == DataBits::bits_9)
    {
        m_uart_base->C1 |= UART_C1_M_MASK;

        if(config.parity != Parity::none)
        {
            m_uart_base->C4 |= UART_C4_M10_MASK;
        }
    }
}




void UartDriver::set_baudrate(const int32_t baudrate)
{
    assert(baudrate > 0);

    const int32_t result = UART_SetBaudRate(m_uart_base, static_cast<uint32_t>(baudrate), SystemDriver::get_fast_peripheral_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK));

    // Assert baudrate less than 3%
    assert(result == 0);
}




} // namespace kv4x
} // namespace targets
} // namespace xarmlib




using namespace xarmlib::targets::kv4x;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void UART0_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void UART0_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart0);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




extern "C" void UART1_RX_TX_IRQHandler(void)
{
    const int32_t yield = UartDriver::status_irq_handler(UartDriver::Name::uart1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}

extern "C" void UART1_ERR_IRQHandler(void)
{
    const int32_t yield = UartDriver::error_irq_handler(UartDriver::Name::uart1);

#ifdef XARMLIB_ENABLE_FREERTOS
    portEND_SWITCHING_ISR(yield);
#else
    (void)yield;
#endif
}




#endif // __KV4X__

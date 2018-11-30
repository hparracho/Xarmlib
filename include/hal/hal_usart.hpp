// ----------------------------------------------------------------------------
// @file    hal_usart.hpp
// @brief   USART HAL interface class.
// @notes   Synchronous mode not implemented.
// @date    30 November 2018
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

#ifndef __XARMLIB_HAL_USART_HPP
#define __XARMLIB_HAL_USART_HPP

#include "external/gsl.hpp"
#include "hal/hal_pin.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{




template <class TargetUsart>
class Usart : private TargetUsart
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using DataBits         = typename TargetUsart::DataBits;
        using StopBits         = typename TargetUsart::StopBits;
        using Parity           = typename TargetUsart::Parity;

        using Status           = typename TargetUsart::Status;
        using StatusBitmask    = typename TargetUsart::StatusBitmask;

        using Interrupt        = typename TargetUsart::Interrupt;
        using InterruptBitmask = typename TargetUsart::InterruptBitmask;

        using IrqHandler       = typename TargetUsart::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Usart(const xarmlib::PinHal::Name txd,
              const xarmlib::PinHal::Name rxd,
              const int32_t               baudrate,
              const DataBits              data_bits = DataBits::BITS_8,
              const StopBits              stop_bits = StopBits::BITS_1,
              const Parity                parity    = Parity::NONE)
            : TargetUsart(txd, rxd, baudrate, data_bits, stop_bits, parity)
        {}

        // -------- FORMAT / BAUDRATE -----------------------------------------

        using TargetUsart::set_format;
        using TargetUsart::set_data_bits;
        using TargetUsart::set_stop_bits;
        using TargetUsart::set_parity;
        using TargetUsart::set_baudrate;

        // -------- ENABLE / DISABLE ------------------------------------------

        using TargetUsart::enable;
        using TargetUsart::disable;
        using TargetUsart::is_enabled;

        // -------- STATUS FLAGS ----------------------------------------------

        using TargetUsart::is_rx_ready;
        using TargetUsart::is_rx_idle;
        using TargetUsart::is_tx_ready;
        using TargetUsart::is_tx_idle;

        using TargetUsart::get_status;
        using TargetUsart::clear_status;

        // -------- INTERRUPT FLAGS -------------------------------------------

        using TargetUsart::enable_interrupts;
        using TargetUsart::disable_interrupts;
        using TargetUsart::get_interrupts_enabled;

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        using TargetUsart::enable_irq;
        using TargetUsart::disable_irq;
        using TargetUsart::is_irq_enabled;

        using TargetUsart::set_irq_priority;

        using TargetUsart::assign_irq_handler;
        using TargetUsart::remove_irq_handler;

        // -------- READ / WRITE ----------------------------------------------

        // Read data as soon as possible (with infinite timeout)
        uint32_t read() const
        {
            while(is_rx_ready() == false);

            return TargetUsart::read_data();
        }

        // Read data as soon as possible (with timeout)
        uint32_t read(const std::chrono::microseconds timeout_us) const
        {
            const auto start = UsTickerHal::now();

            while(is_rx_ready() == false && UsTickerHal::is_timeout(start, timeout_us) == false);

            return TargetUsart::read_data();
        }

        // Read buffer with timeout, returning the number of actual read bytes.
        int32_t read_buffer(const gsl::span<uint8_t> buffer, const std::chrono::microseconds timeout_us) const
        {
            const auto start = UsTickerHal::now();
            int32_t count = 0;

            while(count < buffer.size() && UsTickerHal::is_timeout(start, timeout_us) == false)
            {
                if(is_rx_ready() == true)
                {
                    buffer[count] = TargetUsart::read_data();

                    count++;
                }
            }

            return count;
        }

        // Write data as soon as possible (with infinite timeout)
        void write(const uint32_t value)
        {
            while(is_tx_ready() == false);

            TargetUsart::write_data(value);
        }

        // Write data as soon as possible (with timeout)
        void write(const uint32_t value, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTickerHal::now();

            while(is_tx_ready() == false && UsTickerHal::is_timeout(start, timeout_us) == false);

            TargetUsart::write_data(value);
        }

        // Write buffer with timeout, returning the number of actual written bytes.
        int32_t write_buffer(const gsl::span<const uint8_t> buffer, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTickerHal::now();
            int32_t count = 0;

            while(count < buffer.size() && UsTickerHal::is_timeout(start, timeout_us) == false)
            {
                if(is_tx_ready() == true)
                {
                    TargetUsart::write_data(buffer[count]);

                    count++;
                }
            }

            return count;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE TYPE ALIASES
        // --------------------------------------------------------------------

        using UsTickerHal = xarmlib::UsTickerHal;
};



} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_usart.hpp"

namespace xarmlib
{
using Usart = hal::Usart<targets::lpc84x::Usart>;
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_usart.hpp"

namespace xarmlib
{
using UsartHal = hal::Usart<targets::lpc81x::Usart>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Usart = hal::Usart<targets::other_target::Usart>;
}

#endif




#endif // __XARMLIB_HAL_USART_HPP

// ----------------------------------------------------------------------------
// @file    hal_usart.hpp
// @brief   USART HAL interface class.
// @notes   Synchronous mode not implemented.
// @date    6 February 2019
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

#include "external/span.hpp"
#include "hal/hal_pin.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{




template <typename TargetUartDriver>
class UartHal : protected TargetUartDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Config        = typename TargetUartDriver::Config;

        using Status        = typename TargetUartDriver::Status;
        using StatusBitmask = typename TargetUartDriver::StatusBitmask;

        using IrqHandler    = typename TargetUartDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        UartHal(const xarmlib::PinHal::Name txd, const xarmlib::PinHal::Name rxd, const Config& config) : TargetUartDriver(txd, rxd, config)
        {}

        // -------- BAUDRATE --------------------------------------------------

        inline void set_baudrate(const int32_t baudrate) { TargetUartDriver::set_baudrate(baudrate); }

        // -------- ENABLE / DISABLE ------------------------------------------

        inline void enable()           { TargetUartDriver::enable(); }
        inline void disable()          { TargetUartDriver::disable(); }
        inline bool is_enabled() const { return TargetUartDriver::is_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        inline bool is_rx_ready() const { return TargetUartDriver::is_rx_ready(); }
        inline bool is_rx_idle () const { return TargetUartDriver::is_rx_idle(); }
        inline bool is_tx_ready() const { return TargetUartDriver::is_tx_ready(); }
        inline bool is_tx_idle () const { return TargetUartDriver::is_tx_idle(); }

        inline StatusBitmask get_status() const                        { return TargetUartDriver::get_status(); }
        inline void          clear_status(const StatusBitmask bitmask) { TargetUartDriver::clear_status(bitmask); }

        // -------- READ / WRITE ----------------------------------------------

        // Read data as soon as possible (with infinite timeout)
        inline uint32_t read() const
        {
            while(is_rx_ready() == false);

            return TargetUartDriver::read_data();
        }

        // Read data as soon as possible (with timeout)
        inline uint32_t read(const std::chrono::microseconds timeout_us) const
        {
            const auto start = UsTickerHal::now();

            while(is_rx_ready() == false && UsTickerHal::is_timeout(start, timeout_us) == false);

            return TargetUartDriver::read_data();
        }

        // Read buffer with timeout, returning the number of actual read bytes
        inline int32_t read_buffer(const std::span<uint8_t> buffer, const std::chrono::microseconds timeout_us) const
        {
            const auto start = UsTickerHal::now();
            int32_t count = 0;

            while(count < buffer.size() && UsTickerHal::is_timeout(start, timeout_us) == false)
            {
                if(is_rx_ready() == true)
                {
                    buffer[count] = TargetUartDriver::read_data();

                    count++;
                }
            }

            return count;
        }

        // Write data as soon as possible (with infinite timeout)
        inline void write(const uint32_t value)
        {
            while(is_tx_ready() == false);

            TargetUartDriver::write_data(value);
        }

        // Write data as soon as possible (with timeout)
        inline void write(const uint32_t value, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTickerHal::now();

            while(is_tx_ready() == false && UsTickerHal::is_timeout(start, timeout_us) == false);

            TargetUartDriver::write_data(value);
        }

        // Write buffer with timeout, returning the number of actual written bytes
        inline int32_t write_buffer(const std::span<const uint8_t> buffer, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTickerHal::now();
            int32_t count = 0;

            while(count < buffer.size() && UsTickerHal::is_timeout(start, timeout_us) == false)
            {
                if(is_tx_ready() == true)
                {
                    TargetUartDriver::write_data(buffer[count]);

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

#if defined __KV4X__

#include "targets/KV4x/kv4x_uart.hpp"

namespace xarmlib
{
using UartHal = hal::UartHal<targets::kv4x::UartDriver>;

class Uart : public UartHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using DataBits = typename UartHal::DataBits;
        using StopBits = typename UartHal::StopBits;
        using Parity   = typename UartHal::Parity;
        using IdleType = typename UartHal::IdleType;

        using StatusInterrupt        = typename UartHal::StatusInterrupt;
        using StatusInterruptBitmask = typename UartHal::StatusInterruptBitmask;
        using ErrorInterrupt         = typename UartHal::ErrorInterrupt;
        using ErrorInterruptBitmask  = typename UartHal::ErrorInterruptBitmask;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Uart(const PinHal::Name txd, const PinHal::Name rxd, const Config& config) : UartHal(txd, rxd, config)
        {}

        // -------- FLUSH FIFOS -----------------------------------------------

        // NOTE: Reading an empty data register to clear one of the flags of
        //       the S1 register causes the FIFO pointers to become misaligned.
        //       A receive FIFO flush reinitializes the pointers.

        inline void flush_rx_fifo() { UartHal::flush_rx_fifo(); }
        inline void flush_tx_fifo() { UartHal::flush_tx_fifo(); }

        // -------- STATUS INTERRUPTS -----------------------------------------

        inline void                   enable_status_interrupts (const StatusInterruptBitmask bitmask) { UartHal::enable_status_interrupts(bitmask); }
        inline void                   disable_status_interrupts(const StatusInterruptBitmask bitmask) { UartHal::disable_status_interrupts(bitmask); }
        inline StatusInterruptBitmask get_status_interrupts_enabled() const                           { return UartHal::get_status_interrupts_enabled(); }

        // -------- ERROR INTERRUPTS ------------------------------------------

        inline void                  enable_error_interrupts (const ErrorInterruptBitmask bitmask) { UartHal::enable_error_interrupts(bitmask); }
        inline void                  disable_error_interrupts(const ErrorInterruptBitmask bitmask) { UartHal::disable_error_interrupts(bitmask); }
        inline ErrorInterruptBitmask get_error_interrupts_enabled() const                          { return UartHal::get_error_interrupts_enabled(); }

        // -------- STATUS IRQ / IRQ HANDLER ----------------------------------

        inline void enable_status_irq()     { UartHal::enable_status_irq(); }
        inline void disable_status_irq()    { UartHal::disable_status_irq(); }
        inline bool is_status_irq_enabled() { return UartHal::is_status_irq_enabled(); }

        inline void set_status_irq_priority(const int32_t irq_priority) { UartHal::set_status_irq_priority(irq_priority); }

        inline void assign_status_irq_handler(const IrqHandler& irq_handler) { UartHal::assign_status_irq_handler(irq_handler); }
        inline void remove_status_irq_handler()                              { UartHal::remove_status_irq_handler(); }

        // -------- ERROR IRQ / IRQ HANDLER -----------------------------------

        inline void enable_error_irq()     { UartHal::enable_error_irq(); }
        inline void disable_error_irq()    { UartHal::disable_error_irq(); }
        inline bool is_error_irq_enabled() { return UartHal::is_error_irq_enabled(); }

        inline void set_error_irq_priority(const int32_t irq_priority) { UartHal::set_error_irq_priority(irq_priority); }

        inline void assign_error_irq_handler(const IrqHandler& irq_handler) { UartHal::assign_error_irq_handler(irq_handler); }
        inline void remove_error_irq_handler()                              { UartHal::remove_error_irq_handler(); }
};
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_usart.hpp"

namespace xarmlib
{
using UartHal = hal::UartHal<targets::lpc84x::UsartDriver>;

class Uart : public UartHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using DataBits = typename UartHal::DataBits;
        using StopBits = typename UartHal::StopBits;
        using Parity   = typename UartHal::Parity;

        using Interrupt        = typename UartHal::Interrupt;
        using InterruptBitmask = typename UartHal::InterruptBitmask;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Uart(const PinHal::Name txd, const PinHal::Name rxd, const Config& config) : UartHal(txd, rxd, config)
        {}

        // -------- FORMAT ----------------------------------------------------

        inline void set_format   (const DataBits data_bits, const StopBits stop_bits, const Parity parity) { UartHal::set_format(data_bits, stop_bits, parity); }
        inline void set_data_bits(const DataBits data_bits)                                                { UartHal::set_data_bits(data_bits); }
        inline void set_stop_bits(const StopBits stop_bits)                                                { UartHal::set_stop_bits(stop_bits); }
        inline void set_parity   (const Parity   parity)                                                   { UartHal::set_parity(parity); }

        // -------- INTERRUPT FLAGS -------------------------------------------

        inline void             enable_interrupts (const InterruptBitmask bitmask) { UartHal::enable_interrupts(bitmask); }
        inline void             disable_interrupts(const InterruptBitmask bitmask) { UartHal::disable_interrupts(bitmask); }
        inline InterruptBitmask get_interrupts_enabled() const                     { return UartHal::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        inline void enable_irq()     { UartHal::enable_irq(); }
        inline void disable_irq()    { UartHal::disable_irq(); }
        inline bool is_irq_enabled() { return UartHal::is_irq_enabled(); }

        inline void set_irq_priority(const int32_t irq_priority) { UartHal::set_irq_priority(irq_priority); }

        inline void assign_irq_handler(const IrqHandler& irq_handler) { UartHal::assign_irq_handler(irq_handler); }
        inline void remove_irq_handler()                              { UartHal::remove_irq_handler(); }
};
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_usart.hpp"

namespace xarmlib
{
using UartHal = hal::UartHal<targets::lpc81x::UsartDriver>;

class Uart : public UartHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using DataBits = typename UartHal::DataBits;
        using StopBits = typename UartHal::StopBits;
        using Parity   = typename UartHal::Parity;

        using Interrupt        = typename UartHal::Interrupt;
        using InterruptBitmask = typename UartHal::InterruptBitmask;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Uart(const PinHal::Name txd, const PinHal::Name rxd, const Config& config) : UartHal(txd, rxd, config)
        {}

        // -------- FORMAT ----------------------------------------------------

        inline void set_format   (const DataBits data_bits, const StopBits stop_bits, const Parity parity) { UartHal::set_format(data_bits, stop_bits, parity); }
        inline void set_data_bits(const DataBits data_bits)                                                { UartHal::set_data_bits(data_bits); }
        inline void set_stop_bits(const StopBits stop_bits)                                                { UartHal::set_stop_bits(stop_bits); }
        inline void set_parity   (const Parity   parity)                                                   { UartHal::set_parity(parity); }

        // -------- INTERRUPT FLAGS -------------------------------------------

        inline void             enable_interrupts (const InterruptBitmask bitmask) { UartHal::enable_interrupts(bitmask); }
        inline void             disable_interrupts(const InterruptBitmask bitmask) { UartHal::disable_interrupts(bitmask); }
        inline InterruptBitmask get_interrupts_enabled() const                     { return UartHal::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        inline void enable_irq()     { UartHal::enable_irq(); }
        inline void disable_irq()    { UartHal::disable_irq(); }
        inline bool is_irq_enabled() { return UartHal::is_irq_enabled(); }

        inline void set_irq_priority(const int32_t irq_priority) { UartHal::set_irq_priority(irq_priority); }

        inline void assign_irq_handler(const IrqHandler& irq_handler) { UartHal::assign_irq_handler(irq_handler); }
        inline void remove_irq_handler()                              { UartHal::remove_irq_handler(); }
};
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using UartHal = hal::UartHal<targets::other_target::UartDriver>;
using Uart = UartHal;
}

#endif




#endif // __XARMLIB_HAL_USART_HPP

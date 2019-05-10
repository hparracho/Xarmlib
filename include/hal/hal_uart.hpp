// ----------------------------------------------------------------------------
// @file    hal_uart.hpp
// @brief   UART HAL interface class.
// @date    10 May 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_HAL_UART_HPP
#define __XARMLIB_HAL_UART_HPP

#include "external/span.hpp"
#include "hal/hal_pin.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{




template <typename UartDriver>
class UartBase : protected UartDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Config        = typename UartDriver::Config;

        using Status        = typename UartDriver::Status;
        using StatusBitmask = typename UartDriver::StatusBitmask;

        using IrqHandler    = typename UartDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        UartBase(const hal::Pin::Name txd, const hal::Pin::Name rxd, const Config& config) : UartDriver(txd, rxd, config)
        {}

        // -------- BAUDRATE --------------------------------------------------

        void set_baudrate(const int32_t baudrate) { UartDriver::set_baudrate(baudrate); }

        // -------- ENABLE / DISABLE ------------------------------------------

        void enable()           { UartDriver::enable(); }
        void disable()          { UartDriver::disable(); }
        bool is_enabled() const { return UartDriver::is_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_rx_ready() const { return UartDriver::is_rx_ready(); }
        bool is_rx_idle () const { return UartDriver::is_rx_idle(); }
        bool is_tx_ready() const { return UartDriver::is_tx_ready(); }
        bool is_tx_idle () const { return UartDriver::is_tx_idle(); }

        StatusBitmask get_status() const                        { return UartDriver::get_status(); }
        void          clear_status(const StatusBitmask bitmask) { UartDriver::clear_status(bitmask); }

        // -------- READ / WRITE ----------------------------------------------

        // Read data as soon as possible (with infinite timeout)
        uint32_t read() const
        {
            while(is_rx_ready() == false);

            return UartDriver::read_data();
        }

        // Read data as soon as possible (with timeout)
        uint32_t read(const std::chrono::microseconds timeout_us) const
        {
            const auto start = UsTicker::now();

            while(is_rx_ready() == false && UsTicker::is_timeout(start, timeout_us) == false);

            return UartDriver::read_data();
        }

        // Read buffer with timeout, returning the number of actual read bytes
        int32_t read_buffer(const std::span<uint8_t> buffer, const std::chrono::microseconds timeout_us) const
        {
            const auto start = UsTicker::now();
            int32_t count = 0;

            while(count < buffer.size() && UsTicker::is_timeout(start, timeout_us) == false)
            {
                if(is_rx_ready() == true)
                {
                    buffer[count] = UartDriver::read_data();

                    count++;
                }
            }

            return count;
        }

        // Write data as soon as possible (with infinite timeout)
        void write(const uint32_t value)
        {
            while(is_tx_ready() == false);

            UartDriver::write_data(value);
        }

        // Write data as soon as possible (with timeout)
        void write(const uint32_t value, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTicker::now();

            while(is_tx_ready() == false && UsTicker::is_timeout(start, timeout_us) == false);

            UartDriver::write_data(value);
        }

        // Write buffer with timeout, returning the number of actual written bytes
        int32_t write_buffer(const std::span<const uint8_t> buffer, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTicker::now();
            int32_t count = 0;

            while(count < buffer.size() && UsTicker::is_timeout(start, timeout_us) == false)
            {
                if(is_tx_ready() == true)
                {
                    UartDriver::write_data(buffer[count]);

                    count++;
                }
            }

            return count;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE TYPE ALIASES
        // --------------------------------------------------------------------

        using UsTicker = hal::UsTicker;
};



} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_uart.hpp"

namespace xarmlib
{
namespace hal
{

using Uart = UartBase<targets::kv4x::UartDriver>;

} // namespace hal

class Uart : public hal::Uart
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Uart;

        using DataBits = typename Hal::DataBits;
        using StopBits = typename Hal::StopBits;
        using Parity   = typename Hal::Parity;
        using IdleType = typename Hal::IdleType;

        using StatusInterrupt        = typename Hal::StatusInterrupt;
        using StatusInterruptBitmask = typename Hal::StatusInterruptBitmask;
        using ErrorInterrupt         = typename Hal::ErrorInterrupt;
        using ErrorInterruptBitmask  = typename Hal::ErrorInterruptBitmask;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        // -------- FLUSH FIFOS -----------------------------------------------

        // NOTE: Reading an empty data register to clear one of the flags of
        //       the S1 register causes the FIFO pointers to become misaligned.
        //       A receive FIFO flush reinitializes the pointers.

        void flush_rx_fifo() { Hal::flush_rx_fifo(); }
        void flush_tx_fifo() { Hal::flush_tx_fifo(); }

        // -------- STATUS INTERRUPTS -----------------------------------------

        void                   enable_status_interrupts (const StatusInterruptBitmask bitmask) { Hal::enable_status_interrupts(bitmask); }
        void                   disable_status_interrupts(const StatusInterruptBitmask bitmask) { Hal::disable_status_interrupts(bitmask); }
        StatusInterruptBitmask get_status_interrupts_enabled() const                           { return Hal::get_status_interrupts_enabled(); }

        // -------- ERROR INTERRUPTS ------------------------------------------

        void                  enable_error_interrupts (const ErrorInterruptBitmask bitmask) { Hal::enable_error_interrupts(bitmask); }
        void                  disable_error_interrupts(const ErrorInterruptBitmask bitmask) { Hal::disable_error_interrupts(bitmask); }
        ErrorInterruptBitmask get_error_interrupts_enabled() const                          { return Hal::get_error_interrupts_enabled(); }

        // -------- STATUS IRQ / IRQ HANDLER ----------------------------------

        void enable_status_irq()     { Hal::enable_status_irq(); }
        void disable_status_irq()    { Hal::disable_status_irq(); }
        bool is_status_irq_enabled() { return Hal::is_status_irq_enabled(); }

        void set_status_irq_priority(const int32_t irq_priority) { Hal::set_status_irq_priority(irq_priority); }

        void assign_status_irq_handler(const IrqHandler& irq_handler) { Hal::assign_status_irq_handler(irq_handler); }
        void remove_status_irq_handler()                              { Hal::remove_status_irq_handler(); }

        // -------- ERROR IRQ / IRQ HANDLER -----------------------------------

        void enable_error_irq()     { Hal::enable_error_irq(); }
        void disable_error_irq()    { Hal::disable_error_irq(); }
        bool is_error_irq_enabled() { return Hal::is_error_irq_enabled(); }

        void set_error_irq_priority(const int32_t irq_priority) { Hal::set_error_irq_priority(irq_priority); }

        void assign_error_irq_handler(const IrqHandler& irq_handler) { Hal::assign_error_irq_handler(irq_handler); }
        void remove_error_irq_handler()                              { Hal::remove_error_irq_handler(); }
};

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_usart.hpp"

namespace xarmlib
{
namespace hal
{

using Uart = UartBase<targets::lpc84x::UsartDriver>;

} // namespace hal

class Uart : public hal::Uart
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Uart;

        using DataBits = typename Hal::DataBits;
        using StopBits = typename Hal::StopBits;
        using Parity   = typename Hal::Parity;

        using Interrupt        = typename Hal::Interrupt;
        using InterruptBitmask = typename Hal::InterruptBitmask;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        // -------- FORMAT ----------------------------------------------------

        void set_format   (const DataBits data_bits, const StopBits stop_bits, const Parity parity) { Hal::set_format(data_bits, stop_bits, parity); }
        void set_data_bits(const DataBits data_bits)                                                { Hal::set_data_bits(data_bits); }
        void set_stop_bits(const StopBits stop_bits)                                                { Hal::set_stop_bits(stop_bits); }
        void set_parity   (const Parity   parity)                                                   { Hal::set_parity(parity); }

        // -------- INTERRUPT FLAGS -------------------------------------------

        void             enable_interrupts (const InterruptBitmask bitmask) { Hal::enable_interrupts(bitmask); }
        void             disable_interrupts(const InterruptBitmask bitmask) { Hal::disable_interrupts(bitmask); }
        InterruptBitmask get_interrupts_enabled() const                     { return Hal::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()     { Hal::enable_irq(); }
        void disable_irq()    { Hal::disable_irq(); }
        bool is_irq_enabled() { return Hal::is_irq_enabled(); }

        void set_irq_priority(const int32_t irq_priority) { Hal::set_irq_priority(irq_priority); }

        void assign_irq_handler(const IrqHandler& irq_handler) { Hal::assign_irq_handler(irq_handler); }
        void remove_irq_handler()                              { Hal::remove_irq_handler(); }
};

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_usart.hpp"

namespace xarmlib
{
namespace hal
{

using Uart = UartBase<targets::lpc81x::UsartDriver>;

} // namespace hal

class Uart : public hal::Uart
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Uart;

        using DataBits = typename Hal::DataBits;
        using StopBits = typename Hal::StopBits;
        using Parity   = typename Hal::Parity;

        using Interrupt        = typename Hal::Interrupt;
        using InterruptBitmask = typename Hal::InterruptBitmask;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        // -------- FORMAT ----------------------------------------------------

        void set_format   (const DataBits data_bits, const StopBits stop_bits, const Parity parity) { Hal::set_format(data_bits, stop_bits, parity); }
        void set_data_bits(const DataBits data_bits)                                                { Hal::set_data_bits(data_bits); }
        void set_stop_bits(const StopBits stop_bits)                                                { Hal::set_stop_bits(stop_bits); }
        void set_parity   (const Parity   parity)                                                   { Hal::set_parity(parity); }

        // -------- INTERRUPT FLAGS -------------------------------------------

        void             enable_interrupts (const InterruptBitmask bitmask) { Hal::enable_interrupts(bitmask); }
        void             disable_interrupts(const InterruptBitmask bitmask) { Hal::disable_interrupts(bitmask); }
        InterruptBitmask get_interrupts_enabled() const                     { return Hal::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()     { Hal::enable_irq(); }
        void disable_irq()    { Hal::disable_irq(); }
        bool is_irq_enabled() { return Hal::is_irq_enabled(); }

        void set_irq_priority(const int32_t irq_priority) { Hal::set_irq_priority(irq_priority); }

        void assign_irq_handler(const IrqHandler& irq_handler) { Hal::assign_irq_handler(irq_handler); }
        void remove_irq_handler()                              { Hal::remove_irq_handler(); }
};

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Uart = UartBase<targets::other_target::UartDriver>;

} // namespace hal

using Uart = hal::Uart;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_UART_HPP

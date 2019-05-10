// ----------------------------------------------------------------------------
// @file    hal_can.hpp
// @brief   CAN HAL interface class.
// @notes   Rx FIFO is always used (up to 6 Message Buffers).
//          6 Message Buffers are defined as Tx MB.
//          16 Rx FIFO ID filter table elements are available as Type A
//          (one full ID (standard and extended) per ID Filter element).
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

#ifndef __XARMLIB_HAL_CAN_HPP
#define __XARMLIB_HAL_CAN_HPP

#include "hal/hal_pin.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{




template <typename CanDriver>
class CanBase : protected CanDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Baudrate                          = typename CanDriver::Baudrate;
        using LoopBackMode                      = typename CanDriver::LoopBackMode;
        using Config                            = typename CanDriver::Config;

        using Frame                             = typename CanDriver::Frame;

        using RxFifoFilterElement               = typename CanDriver::RxFifoFilterElement;

        using TxMessageBuffer                   = typename CanDriver::TxMessageBuffer;

        using RxFifoStatus                      = typename CanDriver::RxFifoStatus;
        using RxFifoStatusBitmask               = typename CanDriver::RxFifoStatusBitmask;

        using ErrorAndStatus                    = typename CanDriver::ErrorAndStatus;
        using ErrorAndStatusBitmask             = typename CanDriver::ErrorAndStatusBitmask;

        using OredMessageBufferInterrupt        = typename CanDriver::OredMessageBufferInterrupt;
        using OredMessageBufferInterruptBitmask = typename CanDriver::OredMessageBufferInterruptBitmask;

        using Interrupt                         = typename CanDriver::Interrupt;
        using InterruptBitmask                  = typename CanDriver::InterruptBitmask;

        using IrqHandler                        = typename CanDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        CanBase(const hal::Pin::Name txd, const hal::Pin::Name rxd, const Config& config) : CanDriver(txd, rxd, config)
        {}

        // -------- RX FIFO CONFIGURATION -------------------------------------

        // NOTE: only 16 filter table elements are available
        //       (all as Type A - one full ID (standard and extended) per ID filter element)
        template<std::size_t Size>
        void config_rx_fifo_id_filter_table(const std::array<const RxFifoFilterElement, Size> element_array) { CanDriver::config_rx_fifo_id_filter_table(element_array); }

        // -------- FREE TX MESSAGE BUFFER ------------------------------------

        TxMessageBuffer get_free_tx_message_buffer() const { return CanDriver::get_free_tx_message_buffer(); }

        // -------- READ / WRITE FRAME ----------------------------------------

        // NOTE: clear the Rx FIFO frame available flag after reading the frame to update
        //       the output of the FIFO with the next frame, reissuing the interrupt
        void read_rx_fifo_frame(Frame& frame) const { CanDriver::read_rx_fifo_frame(frame); }

        // NOTE: tx_message_buffer must be a free Message Buffer (use get_free_tx_message_buffer method)
        void write_frame(const TxMessageBuffer tx_message_buffer, const Frame& frame) { CanDriver::write_frame(tx_message_buffer, frame); }

        // Read frame as soon as possible (with infinite timeout)
        void read(Frame& frame)
        {
            while(is_rx_fifo_frame_available() == false);

            read_rx_fifo_frame(frame);

            clear_rx_fifo_frame_available();
        }

        // Read frame as soon as possible (with timeout)
        void read(Frame& frame, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTicker::now();

            while(is_rx_fifo_frame_available() == false && UsTicker::is_timeout(start, timeout_us) == false);

            read_rx_fifo_frame(frame);

            clear_rx_fifo_frame_available();
        }

        // Write frame as soon as possible (with infinite timeout),
        // returning the used Tx Message Buffer
        // NOTE: it is recommended to check after writing if the frame
        //       was transmitted successfully and clear its flag
        TxMessageBuffer write(const Frame& frame)
        {
            TxMessageBuffer tx_mb = get_free_tx_message_buffer();

            while(tx_mb == TxMessageBuffer::none_available)
            {
                tx_mb = get_free_tx_message_buffer();
            }

            write_frame(tx_mb, frame);

            return tx_mb;
        }

        // Write frame as soon as possible (with timeout),
        // returning the used Tx Message Buffer
        // NOTES: - if timeout expires returns TxMessageBuffer::none_available
        //        - it is recommended to check after writing if the frame
        //          was transmitted successfully and clear its flag
        TxMessageBuffer write(const Frame& frame, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTicker::now();

            TxMessageBuffer tx_mb = get_free_tx_message_buffer();

            while(tx_mb == TxMessageBuffer::none_available && UsTicker::is_timeout(start, timeout_us) == false);

            if(tx_mb != TxMessageBuffer::none_available)
            {
                write_frame(tx_mb, frame);

                return tx_mb;
            }

            return TxMessageBuffer::none_available;
        }

        // -------- RX FIFO STATUS FLAGS --------------------------------------

        bool is_rx_fifo_frame_available() const { return CanDriver::is_rx_fifo_frame_available(); }
        bool is_rx_fifo_warning()         const { return CanDriver::is_rx_fifo_warning(); }
        bool is_rx_fifo_overflow()        const { return CanDriver::is_rx_fifo_overflow(); }

        void clear_rx_fifo_frame_available() { CanDriver::clear_rx_fifo_frame_available(); }
        void clear_rx_fifo_warning()         { CanDriver::clear_rx_fifo_warning(); }
        void clear_rx_fifo_overflow()        { CanDriver::clear_rx_fifo_overflow(); }

        RxFifoStatusBitmask get_rx_fifo_status() const { return CanDriver::get_rx_fifo_status(); }

        void clear_rx_fifo_status(const RxFifoStatusBitmask bitmask) { CanDriver::clear_rx_fifo_status(bitmask); }

        // NOTES: - it will be performed in Freeze Mode
        //        - all Rx FIFO status flags must be cleared before execute this method
        void clear_rx_fifo() { CanDriver::clear_rx_fifo(); }

        // -------- TX MESSAGE BUFFER STATUS FLAGS ----------------------------

        bool was_frame_transmitted_successfully(const TxMessageBuffer tx_message_buffer) const { return CanDriver::was_frame_transmitted_successfully(tx_message_buffer); }

        void clear_frame_transmitted(const TxMessageBuffer tx_message_buffer) { CanDriver::clear_frame_transmitted(tx_message_buffer); }

        // -------- ERROR AND STATUS FLAGS ------------------------------------

        // For further details about CAN Bus Error Handling see the following link:
        // https://www.kvaser.com/about-can/the-can-protocol/can-error-handling/

        // Recommended procedure:
        // - call get_error_and_status method (this action also clear the
        //   respective bits that were set since the last read access)
        // - call clear_error_and_status method to clear the interrupt
        //   bits that has triggered the interrupt request and/or to clear
        //   the overrun_error bit if it is set

        ErrorAndStatusBitmask get_error_and_status() const { return CanDriver::get_error_and_status(); }

        void clear_error_and_status(const ErrorAndStatusBitmask bitmask) { CanDriver::clear_error_and_status(bitmask); }

        // -------- RX / TX BUS ERROR COUNTER ---------------------------------

        uint8_t get_rx_error_counter() const { return CanDriver::get_rx_error_counter(); }
        uint8_t get_tx_error_counter() const { return CanDriver::get_tx_error_counter(); }

        // -------- RX FIFO AND TX MESSAGE BUFFER INTERRUPTS ------------------

        void enable_ored_message_buffer_interrupts (const OredMessageBufferInterruptBitmask bitmask) { CanDriver::enable_ored_message_buffer_interrupts(bitmask); }
        void disable_ored_message_buffer_interrupts(const OredMessageBufferInterruptBitmask bitmask) { CanDriver::disable_ored_message_buffer_interrupts(bitmask); }
        OredMessageBufferInterruptBitmask get_ored_message_buffer_interrupts_enabled() const         { return CanDriver::get_ored_message_buffer_interrupts_enabled(); }

        // -------- REMAINING INTERRUPTS --------------------------------------

        void enable_interrupts (const InterruptBitmask bitmask) { CanDriver::enable_interrupts(bitmask); }
        void disable_interrupts(const InterruptBitmask bitmask) { CanDriver::disable_interrupts(bitmask); }
        InterruptBitmask get_interrupts_enabled() const         { return CanDriver::get_interrupts_enabled(); }

        // -------- OR'ED MESSAGE BUFFER IRQ / IRQ HANDLER --------------------

        void enable_ored_message_buffer_irq()     { CanDriver::enable_ored_message_buffer_irq(); }
        void disable_ored_message_buffer_irq()    { CanDriver::disable_ored_message_buffer_irq(); }
        bool is_ored_message_buffer_irq_enabled() { return CanDriver::is_ored_message_buffer_irq_enabled(); }

        void set_ored_message_buffer_irq_priority(const int32_t irq_priority) { CanDriver::set_ored_message_buffer_irq_priority(irq_priority); }

        void assign_ored_message_buffer_irq_handler(const IrqHandler& irq_handler) { CanDriver::assign_ored_message_buffer_irq_handler(irq_handler); }
        void remove_ored_message_buffer_irq_handler()                              { CanDriver::remove_ored_message_buffer_irq_handler(); }

        // -------- BUS OFF IRQ / IRQ HANDLER ---------------------------------

        void enable_bus_off_irq()     { CanDriver::enable_bus_off_irq(); }
        void disable_bus_off_irq()    { CanDriver::disable_bus_off_irq(); }
        bool is_bus_off_irq_enabled() { return CanDriver::is_bus_off_irq_enabled(); }

        void set_bus_off_irq_priority(const int32_t irq_priority) { CanDriver::set_bus_off_irq_priority(irq_priority); }

        void assign_bus_off_irq_handler(const IrqHandler& irq_handler) { CanDriver::assign_bus_off_irq_handler(irq_handler); }
        void remove_bus_off_irq_handler()                              { CanDriver::remove_bus_off_irq_handler(); }

        // -------- ERROR IRQ / IRQ HANDLER -----------------------------------

        void enable_error_irq()     { CanDriver::enable_error_irq(); }
        void disable_error_irq()    { CanDriver::disable_error_irq(); }
        bool is_error_irq_enabled() { return CanDriver::is_error_irq_enabled(); }

        void set_error_irq_priority(const int32_t irq_priority) { CanDriver::set_error_irq_priority(irq_priority); }

        void assign_error_irq_handler(const IrqHandler& irq_handler) { CanDriver::assign_error_irq_handler(irq_handler); }
        void remove_error_irq_handler()                              { CanDriver::remove_error_irq_handler(); }

        // -------- TRANSMIT WARNING IRQ / IRQ HANDLER ------------------------

        void enable_tx_warning_irq()     { CanDriver::enable_tx_warning_irq(); }
        void disable_tx_warning_irq()    { CanDriver::disable_tx_warning_irq(); }
        bool is_tx_warning_irq_enabled() { return CanDriver::is_tx_warning_irq_enabled(); }

        void set_tx_warning_irq_priority(const int32_t irq_priority) { CanDriver::set_tx_warning_irq_priority(irq_priority); }

        void assign_tx_warning_irq_handler(const IrqHandler& irq_handler) { CanDriver::assign_tx_warning_irq_handler(irq_handler); }
        void remove_tx_warning_irq_handler()                              { CanDriver::remove_tx_warning_irq_handler(); }

        // -------- RECEIVE WARNING IRQ / IRQ HANDLER -------------------------

        void enable_rx_warning_irq()     { CanDriver::enable_rx_warning_irq(); }
        void disable_rx_warning_irq()    { CanDriver::disable_rx_warning_irq(); }
        bool is_rx_warning_irq_enabled() { return CanDriver::is_rx_warning_irq_enabled(); }

        void set_rx_warning_irq_priority(const int32_t irq_priority) { CanDriver::set_rx_warning_irq_priority(irq_priority); }

        void assign_rx_warning_irq_handler(const IrqHandler& irq_handler) { CanDriver::assign_rx_warning_irq_handler(irq_handler); }
        void remove_rx_warning_irq_handler()                              { CanDriver::remove_rx_warning_irq_handler(); }

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

#include "targets/KV4x/kv4x_can.hpp"

namespace xarmlib
{
namespace hal
{

using Can = CanBase<targets::kv4x::CanDriver>;

} // namespace hal

using Can = hal::Can;

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Can = CanBase<targets::other_target::CanDriver>;

} // namespace hal

using Can = hal::Can;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_CAN_HPP

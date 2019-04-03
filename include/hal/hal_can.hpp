// ----------------------------------------------------------------------------
// @file    hal_can.hpp
// @brief   CAN HAL interface class.
// @notes   Rx FIFO is always used (up to 6 Message Buffers).
//          6 Message Buffers are defined as Tx MB.
//          16 Rx FIFO ID filter table elements are available as Type A
//          (one full ID (standard and extended) per ID Filter element).
// @date    29 March 2019
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

#ifndef __XARMLIB_HAL_CAN_HPP
#define __XARMLIB_HAL_CAN_HPP

#include "hal/hal_pin.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{
namespace hal
{




template <typename TargetCanDriver>
class CanHal : protected TargetCanDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Baudrate                          = typename TargetCanDriver::Baudrate;
        using LoopBackMode                      = typename TargetCanDriver::LoopBackMode;
        using Config                            = typename TargetCanDriver::Config;

        using FrameFormat                       = typename TargetCanDriver::FrameFormat;
        using FrameType                         = typename TargetCanDriver::FrameType;
        using FrameDataBytes                    = typename TargetCanDriver::FrameDataBytes;
        using Frame                             = typename TargetCanDriver::Frame;

        using RxFifoFilterElement               = typename TargetCanDriver::RxFifoFilterElement;

        using TxMessageBuffer                   = typename TargetCanDriver::TxMessageBuffer;

        using RxFifoStatus                      = typename TargetCanDriver::RxFifoStatus;
        using RxFifoStatusBitmask               = typename TargetCanDriver::RxFifoStatusBitmask;

        using ErrorAndStatus                    = typename TargetCanDriver::ErrorAndStatus;
        using ErrorAndStatusBitmask             = typename TargetCanDriver::ErrorAndStatusBitmask;

        using OredMessageBufferInterrupt        = typename TargetCanDriver::OredMessageBufferInterrupt;
        using OredMessageBufferInterruptBitmask = typename TargetCanDriver::OredMessageBufferInterruptBitmask;

        using Interrupt                         = typename TargetCanDriver::Interrupt;
        using InterruptBitmask                  = typename TargetCanDriver::InterruptBitmask;

        using IrqHandler                        = typename TargetCanDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        CanHal(const xarmlib::PinHal::Name txd, const xarmlib::PinHal::Name rxd, const Config& config) : TargetCanDriver(txd, rxd, config)
        {}

        // -------- RX FIFO CONFIGURATION -------------------------------------

        // NOTE: only 16 filter table elements are available
        //       (all as Type A - one full ID (standard and extended) per ID filter element)
        template<std::size_t Size>
        inline void config_rx_fifo_id_filter_table(const std::array<const RxFifoFilterElement, Size> element_array) { TargetCanDriver::config_rx_fifo_id_filter_table(element_array); }

        // -------- FREE TX MESSAGE BUFFER ------------------------------------

        inline TxMessageBuffer get_free_tx_message_buffer() const { return TargetCanDriver::get_free_tx_message_buffer(); }

        // -------- READ / WRITE FRAME ----------------------------------------

        // NOTE: clear the Rx FIFO frame available flag after reading the frame to update
        //       the output of the FIFO with the next frame, reissuing the interrupt
        inline Frame read_rx_fifo_frame() const { return TargetCanDriver::read_rx_fifo_frame(); }

        // NOTE: tx_message_buffer must be a free Message Buffer (use get_free_tx_message_buffer method)
        inline void write_frame(const TxMessageBuffer tx_message_buffer, const Frame& frame) { TargetCanDriver::write_frame(tx_message_buffer, frame); }

        // Read frame as soon as possible (with infinite timeout)
        inline Frame read()
        {
            while(is_rx_fifo_frame_available() == false);

            const Frame frame = read_rx_fifo_frame();

            clear_rx_fifo_frame_available();

            return frame;
        }

        // Read frame as soon as possible (with timeout)
        inline Frame read(const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTickerHal::now();

            while(is_rx_fifo_frame_available() == false && UsTickerHal::is_timeout(start, timeout_us) == false);

            const Frame frame = read_rx_fifo_frame();

            clear_rx_fifo_frame_available();

            return frame;
        }

        // Write frame as soon as possible (with infinite timeout),
        // returning the used Tx Message Buffer
        // NOTE: it is recommended to check after writing if the frame
        //       was transmitted successfully and clear its flag
        inline TxMessageBuffer write(const Frame& frame)
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
        inline TxMessageBuffer write(const Frame& frame, const std::chrono::microseconds timeout_us)
        {
            const auto start = UsTickerHal::now();

            TxMessageBuffer tx_mb = get_free_tx_message_buffer();

            while(tx_mb == TxMessageBuffer::none_available && UsTickerHal::is_timeout(start, timeout_us) == false);

            if(tx_mb != TxMessageBuffer::none_available)
            {
                write_frame(tx_mb, frame);

                return tx_mb;
            }

            return TxMessageBuffer::none_available;
        }

        // -------- RX FIFO STATUS FLAGS --------------------------------------

        inline bool is_rx_fifo_frame_available() const { return TargetCanDriver::is_rx_fifo_frame_available(); }
        inline bool is_rx_fifo_warning()         const { return TargetCanDriver::is_rx_fifo_warning(); }
        inline bool is_rx_fifo_overflow()        const { return TargetCanDriver::is_rx_fifo_overflow(); }

        inline void clear_rx_fifo_frame_available() { TargetCanDriver::clear_rx_fifo_frame_available(); }
        inline void clear_rx_fifo_warning()         { TargetCanDriver::clear_rx_fifo_warning(); }
        inline void clear_rx_fifo_overflow()        { TargetCanDriver::clear_rx_fifo_overflow(); }

        inline RxFifoStatusBitmask get_rx_fifo_status() const { return TargetCanDriver::get_rx_fifo_status(); }

        inline void clear_rx_fifo_status(const RxFifoStatusBitmask bitmask) { TargetCanDriver::clear_rx_fifo_status(bitmask); }

        // NOTES: - it will be performed in Freeze Mode
        //        - all Rx FIFO status flags must be cleared before execute this method
        inline void clear_rx_fifo() { TargetCanDriver::clear_rx_fifo(); }

        // -------- TX MESSAGE BUFFER STATUS FLAGS ----------------------------

        inline bool was_frame_transmitted_successfully(const TxMessageBuffer tx_message_buffer) const { return TargetCanDriver::was_frame_transmitted_successfully(tx_message_buffer); }

        inline void clear_frame_transmitted(const TxMessageBuffer tx_message_buffer) { TargetCanDriver::clear_frame_transmitted(tx_message_buffer); }

        // -------- ERROR AND STATUS FLAGS ------------------------------------

        // For further details about CAN Bus Error Handling see the following link:
        // https://www.kvaser.com/about-can/the-can-protocol/can-error-handling/

        // Recommended procedure:
        // - call get_error_and_status method (this action also clear the
        //   respective bits that were set since the last read access)
        // - call clear_error_and_status method to clear the interrupt
        //   bits that has triggered the interrupt request and/or to clear
        //   the overrun_error bit if it is set

        inline ErrorAndStatusBitmask get_error_and_status() const { return TargetCanDriver::get_error_and_status(); }

        inline void clear_error_and_status(const ErrorAndStatusBitmask bitmask) { TargetCanDriver::clear_error_and_status(bitmask); }

        // -------- RX / TX BUS ERROR COUNTER ---------------------------------

        inline uint8_t get_rx_error_counter() const { return TargetCanDriver::get_rx_error_counter(); }
        inline uint8_t get_tx_error_counter() const { return TargetCanDriver::get_tx_error_counter(); }

        // -------- RX FIFO AND TX MESSAGE BUFFER INTERRUPTS ------------------

        inline void enable_ored_message_buffer_interrupts (const OredMessageBufferInterruptBitmask bitmask) { TargetCanDriver::enable_ored_message_buffer_interrupts(bitmask); }
        inline void disable_ored_message_buffer_interrupts(const OredMessageBufferInterruptBitmask bitmask) { TargetCanDriver::disable_ored_message_buffer_interrupts(bitmask); }
        inline OredMessageBufferInterruptBitmask get_ored_message_buffer_interrupts_enabled() const         { return TargetCanDriver::get_ored_message_buffer_interrupts_enabled(); }

        // -------- REMAINING INTERRUPTS --------------------------------------

        inline void enable_interrupts (const InterruptBitmask bitmask) { TargetCanDriver::enable_interrupts(bitmask); }
        inline void disable_interrupts(const InterruptBitmask bitmask) { TargetCanDriver::disable_interrupts(bitmask); }
        inline InterruptBitmask get_interrupts_enabled() const         { return TargetCanDriver::get_interrupts_enabled(); }

        // -------- OR'ED MESSAGE BUFFER IRQ / IRQ HANDLER --------------------

        inline void enable_ored_message_buffer_irq()     { TargetCanDriver::enable_ored_message_buffer_irq(); }
        inline void disable_ored_message_buffer_irq()    { TargetCanDriver::disable_ored_message_buffer_irq(); }
        inline bool is_ored_message_buffer_irq_enabled() { return TargetCanDriver::is_ored_message_buffer_irq_enabled(); }

        inline void set_ored_message_buffer_irq_priority(const int32_t irq_priority) { TargetCanDriver::set_ored_message_buffer_irq_priority(irq_priority); }

        inline void assign_ored_message_buffer_irq_handler(const IrqHandler& irq_handler) { TargetCanDriver::assign_ored_message_buffer_irq_handler(irq_handler); }
        inline void remove_ored_message_buffer_irq_handler()                              { TargetCanDriver::remove_ored_message_buffer_irq_handler(); }

        // -------- BUS OFF IRQ / IRQ HANDLER ---------------------------------

        inline void enable_bus_off_irq()     { TargetCanDriver::enable_bus_off_irq(); }
        inline void disable_bus_off_irq()    { TargetCanDriver::disable_bus_off_irq(); }
        inline bool is_bus_off_irq_enabled() { return TargetCanDriver::is_bus_off_irq_enabled(); }

        inline void set_bus_off_irq_priority(const int32_t irq_priority) { TargetCanDriver::set_bus_off_irq_priority(irq_priority); }

        inline void assign_bus_off_irq_handler(const IrqHandler& irq_handler) { TargetCanDriver::assign_bus_off_irq_handler(irq_handler); }
        inline void remove_bus_off_irq_handler()                              { TargetCanDriver::remove_bus_off_irq_handler(); }

        // -------- ERROR IRQ / IRQ HANDLER -----------------------------------

        inline void enable_error_irq()     { TargetCanDriver::enable_error_irq(); }
        inline void disable_error_irq()    { TargetCanDriver::disable_error_irq(); }
        inline bool is_error_irq_enabled() { return TargetCanDriver::is_error_irq_enabled(); }

        inline void set_error_irq_priority(const int32_t irq_priority) { TargetCanDriver::set_error_irq_priority(irq_priority); }

        inline void assign_error_irq_handler(const IrqHandler& irq_handler) { TargetCanDriver::assign_error_irq_handler(irq_handler); }
        inline void remove_error_irq_handler()                              { TargetCanDriver::remove_error_irq_handler(); }

        // -------- TRANSMIT WARNING IRQ / IRQ HANDLER ------------------------

        inline void enable_tx_warning_irq()     { TargetCanDriver::enable_tx_warning_irq(); }
        inline void disable_tx_warning_irq()    { TargetCanDriver::disable_tx_warning_irq(); }
        inline bool is_tx_warning_irq_enabled() { return TargetCanDriver::is_tx_warning_irq_enabled(); }

        inline void set_tx_warning_irq_priority(const int32_t irq_priority) { TargetCanDriver::set_tx_warning_irq_priority(irq_priority); }

        inline void assign_tx_warning_irq_handler(const IrqHandler& irq_handler) { TargetCanDriver::assign_tx_warning_irq_handler(irq_handler); }
        inline void remove_tx_warning_irq_handler()                              { TargetCanDriver::remove_tx_warning_irq_handler(); }

        // -------- RECEIVE WARNING IRQ / IRQ HANDLER -------------------------

        inline void enable_rx_warning_irq()     { TargetCanDriver::enable_rx_warning_irq(); }
        inline void disable_rx_warning_irq()    { TargetCanDriver::disable_rx_warning_irq(); }
        inline bool is_rx_warning_irq_enabled() { return TargetCanDriver::is_rx_warning_irq_enabled(); }

        inline void set_rx_warning_irq_priority(const int32_t irq_priority) { TargetCanDriver::set_rx_warning_irq_priority(irq_priority); }

        inline void assign_rx_warning_irq_handler(const IrqHandler& irq_handler) { TargetCanDriver::assign_rx_warning_irq_handler(irq_handler); }
        inline void remove_rx_warning_irq_handler()                              { TargetCanDriver::remove_rx_warning_irq_handler(); }

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

#include "targets/KV4x/kv4x_can.hpp"

namespace xarmlib
{
using CanHal = hal::CanHal<targets::kv4x::CanDriver>;
using Can = CanHal;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using CanHal = hal::CanHal<targets::other_target::CanDriver>;
using Can = CanHal;
}

#endif




#endif // __XARMLIB_HAL_CAN_HPP

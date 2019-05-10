// ----------------------------------------------------------------------------
// @file    kv4x_can.hpp
// @brief   Kinetis KV4x Flex Controller Area Network (FlexCAN) class.
// @notes   Rx FIFO is always used (up to 6 Message Buffers).
//          6 Message Buffers are defined as Tx MB.
//          16 Rx FIFO ID filter table elements are available as Type A
//          (one full ID (standard and extended) per ID Filter element).
// @date    10 May 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_TARGETS_KV4X_CAN_HPP
#define __XARMLIB_TARGETS_KV4X_CAN_HPP

#include "external/bitmask.hpp"
#include "fsl_flexcan.h"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of IRQ handlers for all KV4x packages
extern "C" void CAN0_ORed_Message_buffer_IRQHandler(void);
extern "C" void CAN0_Bus_Off_IRQHandler            (void);
extern "C" void CAN0_Error_IRQHandler              (void);
extern "C" void CAN0_Tx_Warning_IRQHandler         (void);
extern "C" void CAN0_Rx_Warning_IRQHandler         (void);
//extern "C" void CAN0_Wake_Up_IRQHandler            (void);

#if (TARGET_CAN_COUNT == 2)
// Forward declaration of additional IRQ handlers
extern "C" void CAN1_ORed_Message_buffer_IRQHandler(void);
extern "C" void CAN1_Bus_Off_IRQHandler            (void);
extern "C" void CAN1_Error_IRQHandler              (void);
extern "C" void CAN1_Tx_Warning_IRQHandler         (void);
extern "C" void CAN1_Rx_Warning_IRQHandler         (void);
//extern "C" void CAN1_Wake_Up_IRQHandler            (void);
#endif




namespace xarmlib
{
namespace targets
{
namespace kv4x
{




namespace private_can
{

// CAN Rx FIFO status bits
enum class RxFifoStatus
{
    overflow          = CAN_IFLAG1_BUF7I_MASK,  // Rx FIFO overflow flag
    warning           = CAN_IFLAG1_BUF6I_MASK,  // Rx FIFO almost full flag
    frame_available   = CAN_IFLAG1_BUF5I_MASK,  // Frames available in Rx FIFO flag
    clear_all_bitmask = overflow
                      | warning
                      | frame_available,
    bitmask           = clear_all_bitmask
};




// CAN Error and Status 1 register (ESR1) bits
// For further details see the section 44.4.9 - Error and Status 1 register
// from the reference manual (KV4XP100M168RM)
enum class ErrorAndStatus
{
    // ERRORS
    overrun_error     = CAN_ESR1_ERROVR_MASK,       // Error condition occurred when
                                                    // any error from different frames
                                                    // had accumulated
    bit1_error        = CAN_ESR1_BIT1ERR_MASK,      // Unable to send recessive bit
    bit0_error        = CAN_ESR1_BIT0ERR_MASK,      // Unable to send dominant bit
    ack_error         = CAN_ESR1_ACKERR_MASK,       // Received no ACK on transmission
    crc_error         = CAN_ESR1_CRCERR_MASK,       // Cyclic Redundancy Check Error
    form_error        = CAN_ESR1_FRMERR_MASK,       // Form Error
    stuffing_error    = CAN_ESR1_STFERR_MASK,       // Stuffing Error
    all_errors        = overrun_error
                      | bit1_error
                      | bit0_error
                      | ack_error
                      | crc_error
                      | form_error
                      | stuffing_error,

    // STATUS
    synch             = CAN_ESR1_SYNCH_MASK,        // CAN Synchronization Status
    tx_error_warning  = CAN_ESR1_TXWRN_MASK,        // Tx Error Warning
    rx_error_warning  = CAN_ESR1_RXWRN_MASK,        // Rx Error Warning
    idle              = CAN_ESR1_IDLE_MASK,         // CAN IDLE State
    transmitting      = CAN_ESR1_TX_MASK,           // FlexCAN In Transmission
    fault_confinement = CAN_ESR1_FLTCONF_MASK,      // Fault Confinement State: 00 Error Active
                                                    //                          01 Error Passive
                                                    //                          1x Bus Off
    receiving         = CAN_ESR1_RX_MASK,           // FlexCAN In Reception

    // INTERRUPT FLAGS
    bus_off_done      = CAN_ESR1_BOFFDONEINT_MASK,  // Bus Off Done Interrupt Flag
    tx_warning        = CAN_ESR1_TWRNINT_MASK,      // Tx Warning Interrupt Flag
    rx_warning        = CAN_ESR1_RWRNINT_MASK,      // Rx Warning Interrupt Flag
    bus_off           = CAN_ESR1_BOFFINT_MASK,      // Bus Off Interrupt Flag
    error             = CAN_ESR1_ERRINT_MASK,       // Error Interrupt Flag
    wake_up           = CAN_ESR1_WAKINT_MASK,       // Wake-Up Interrupt Flag

    clear_all_bitmask = overrun_error
                      | bus_off_done
                      | tx_warning
                      | rx_warning
                      | bus_off
                      | error
                      | wake_up,
    bitmask           = all_errors
                      | synch
                      | tx_error_warning
                      | rx_error_warning
                      | idle
                      | transmitting
                      | fault_confinement
                      | receiving
                      | bus_off_done
                      | tx_warning
                      | rx_warning
                      | bus_off
                      | error
                      | wake_up
};




// CAN Tx Message Buffer and Rx FIFO interrupt sources
// NOTE: Rx FIFO occupy the MB0 ~ MB9 (Rx FIFO Engine and
//       ID Filter Table, so Tx Message Buffer occupy the
//       MB10 up to MB15
enum class OredMessageBufferInterrupt
{
    tx_message_buffer_1     = 1 << 10,                  // MB10 interrupt
    tx_message_buffer_2     = 1 << 11,                  // MB11 interrupt
    tx_message_buffer_3     = 1 << 12,                  // MB12 interrupt
    tx_message_buffer_4     = 1 << 13,                  // MB13 interrupt
    tx_message_buffer_5     = 1 << 14,                  // MB14 interrupt
    tx_message_buffer_6     = 1 << 15,                  // MB15 interrupt
    rx_fifo_overflow        = CAN_IFLAG1_BUF7I_MASK,    // Rx FIFO overflow interrupt
    rx_fifo_warning         = CAN_IFLAG1_BUF6I_MASK,    // Rx FIFO almost full interrupt
    rx_fifo_frame_available = CAN_IFLAG1_BUF5I_MASK,    // Frames available in Rx FIFO interrupt
    bitmask                 = tx_message_buffer_1
                            | tx_message_buffer_2
                            | tx_message_buffer_3
                            | tx_message_buffer_4
                            | tx_message_buffer_5
                            | tx_message_buffer_6
                            | rx_fifo_overflow
                            | rx_fifo_warning
                            | rx_fifo_frame_available
};




// CAN remaining interrupt sources
// NOTE: Tx Message Buffers and Rx FIFO have their own interrupts (defined above)
enum class Interrupt
{
    bus_off    = CAN_CTRL1_BOFFMSK_MASK,    // Bus Off interrupt
    error      = CAN_CTRL1_ERRMSK_MASK,     // Error interrupt
    tx_warning = CAN_CTRL1_TWRNMSK_MASK,    // Tx Warning interrupt
    rx_warning = CAN_CTRL1_RWRNMSK_MASK,    // Rx Warning interrupt
    //wake_up    = CAN_MCR_WAKMSK_MASK,       // Wake Up interrupt
    bitmask    = bus_off
               | error
               | tx_warning
               | rx_warning
               //| wake_up
};

BITMASK_DEFINE_VALUE_MASK(RxFifoStatus,               static_cast<uint32_t>(RxFifoStatus::bitmask))
BITMASK_DEFINE_VALUE_MASK(ErrorAndStatus,             static_cast<uint32_t>(ErrorAndStatus::bitmask))
BITMASK_DEFINE_VALUE_MASK(OredMessageBufferInterrupt, static_cast<uint32_t>(OredMessageBufferInterrupt::bitmask))
BITMASK_DEFINE_VALUE_MASK(Interrupt,                  static_cast<uint32_t>(Interrupt::bitmask))

} // namespace private_can




static constexpr uint32_t TARGET_CAN_MASK  = (1UL << TARGET_CAN_COUNT) - 1;

class CanDriver : private PeripheralRefCounter<CanDriver, TARGET_CAN_COUNT, TARGET_CAN_MASK>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::CAN0_ORed_Message_buffer_IRQHandler(void);
        friend void ::CAN0_Bus_Off_IRQHandler            (void);
        friend void ::CAN0_Error_IRQHandler              (void);
        friend void ::CAN0_Tx_Warning_IRQHandler         (void);
        friend void ::CAN0_Rx_Warning_IRQHandler         (void);
        //friend void ::CAN0_Wake_Up_IRQHandler            (void);
#if (TARGET_CAN_COUNT == 2)
        friend void ::CAN1_ORed_Message_buffer_IRQHandler(void);
        friend void ::CAN1_Bus_Off_IRQHandler            (void);
        friend void ::CAN1_Error_IRQHandler              (void);
        friend void ::CAN1_Tx_Warning_IRQHandler         (void);
        friend void ::CAN1_Rx_Warning_IRQHandler         (void);
        //friend void ::CAN1_Wake_Up_IRQHandler            (void);
#endif

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralCan = PeripheralRefCounter<CanDriver, TARGET_CAN_COUNT, TARGET_CAN_MASK>;

        // Baudrate selection
        enum class Baudrate : uint32_t
        {
            rate_20kbps   =   20000,
            rate_50kbps   =   50000,
            rate_100kbps  =  100000,
            rate_125kbps  =  125000,
            rate_250kbps  =  250000,
            rate_500kbps  =  500000,
            rate_1000kbps = 1000000
        };

        // Loop back mode
        enum class LoopBackMode
        {
            disabled = 0,
            enabled
        };

        /*
        // Self wake up
        enum class SelfWakeUp
        {
            disabled = 0,
            enabled
        };

        // Wake up source
        enum class WakeUpSource
        {
            unfiltered = 0, // FlexCAN uses unfiltered Rx input to detect edge
            filtered        // FlexCAN uses filtered Rx input to detect edge
        };

        // Doze mode enable
        enum class DozeMode
        {
            disabled = 0,   // FlexCAN is not enabled to enter low-power mode when Doze mode is requested
            enabled         // FlexCAN is enabled to enter low-power mode when Doze mode is requested
        };

        // Rx individual masking mode selection
        enum class RxIndividualMasking
        {
            disabled = 0,   // Rx global masking and queue feature are disabled (masking scheme with
                            // CAN_RXMGMASK, CAN_RX14MASK, CAN_RX15MASK and CAN_RXFGMASK for backwards
                            // compatibility with legacy applications)
            enabled         // Rx individual masking and queue feature are enabled
        };

        // Timer sync
        // NOTE: it enables a mechanism that resets the free-running timer each time a message is
        //       received in Message Buffer 0 / first available Mailbox. This feature provides means
        //       to synchronize multiple FlexCAN stations with a special “SYNC” message, that is,
        //       global network time.
        enum class TimerSync
        {
            disabled = 0,
            enabled
        };
        */

        struct Config
        {
            Baudrate     baudrate       = Baudrate::rate_1000kbps;
            LoopBackMode loop_back_mode = LoopBackMode::disabled;
        };

        class Frame
        {
            friend class CanDriver;

            public:

                // Frame format (IDE) selection
                enum class Format : uint8_t
                {
                    standard = 0,
                    extended
                };

                // Frame type (RTR) selection
                enum class Type : uint8_t
                {
                    data = 0,
                    remote
                };

                // Identifier (id) NOTE: in standard frame format, only the 11 least significant bits are used
                //                       in extended frame format, all the 29 bits are used

                // Default constructor
                Frame() : m_id { 0 }, m_format { Format::standard }, m_type { Type::data }, m_data_bytes { 0 }, m_data {}
                {}

                Frame(const uint32_t id, const Format format, const Type type, std::span<const uint8_t> data) :
                    m_id { id }, m_format { format }, m_type { type }, m_data_bytes { 0 }, m_data {}
                {
                    set_data(data);
                }

                uint32_t           get_id()     const { return m_id; }
                Format             get_format() const { return m_format; }
                Type               get_type()   const { return m_type; }
                std::span<uint8_t> get_data()         { return std::span(m_data).subspan(0, m_data_bytes); }

                void set_id(const uint32_t id)               { m_id = id; }
                void set_format(const Format format)         { m_format = format; }
                void set_type(const Type type)               { m_type = type; }
                void set_data(std::span<const uint8_t> data)
                {
                    assert(data.size() <= static_cast<std::ptrdiff_t>(m_data.size()));

                    m_data_bytes = data.size();

                    std::copy(data.begin(), data.end(), m_data.begin());
                }

            private:

                uint32_t                m_id;
                Format                  m_format;
                Type                    m_type;
                uint8_t                 m_data_bytes;   // [0 - 8]
                std::array<uint8_t, 8>  m_data;
                //uint16_t                m_timestamp;    // Internal Free-Running Counter Time Stamp
                //uint16_t                m_idhit;        // Identifier Acceptance Filter Hit Indicator
        };

        // Rx FIFO ID filter table element structure
        struct RxFifoFilterElement
        {
            // Frame Identifier
            // NOTE: in standard frame format, only the 11 least significant bits are used
            //       in extended frame format, all the 29 bits are used
            uint32_t      id;
            Frame::Format format;
            Frame::Type   type;
        };

        // Tx Message Buffer selection
        enum class TxMessageBuffer
        {
            number_1 = 10,
            number_2,
            number_3,
            number_4,
            number_5,
            number_6,
            none_available
        };

        // Type safe accessor to Rx FIFO status flags
        using RxFifoStatus        = private_can::RxFifoStatus;
        using RxFifoStatusBitmask = bitmask::bitmask<RxFifoStatus>;

        // Type safe accessor to error and status flags
        using ErrorAndStatus        = private_can::ErrorAndStatus;
        using ErrorAndStatusBitmask = bitmask::bitmask<ErrorAndStatus>;

        // Type safe accessor to Tx Message Buffer and Rx FIFO interrupt sources
        using OredMessageBufferInterrupt        = private_can::OredMessageBufferInterrupt;
        using OredMessageBufferInterruptBitmask = bitmask::bitmask<OredMessageBufferInterrupt>;

        // Type safe accessor to remaining interrupt sources
        using Interrupt        = private_can::Interrupt;
        using InterruptBitmask = bitmask::bitmask<Interrupt>;

        // IRQ handlers definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        CanDriver(const PinDriver::Name txd, const PinDriver::Name rxd, const Config& config) : PeripheralCan(*this, get_peripheral_index(txd, rxd))
        {
            const auto pin_config = get_pin_config(txd, rxd);

            PinDriver::set_pin_mux(txd, pin_config.pin_mux);
            PinDriver::set_pin_mux(rxd, pin_config.pin_mux);

            m_can_name = pin_config.can_name;

            switch(m_can_name)
            {
                case Name::can0: m_can_base = CAN0; break;
                case Name::can1: m_can_base = CAN1; break;
            };

            initialize(config);

            disable_ored_message_buffer_irq();
            disable_bus_off_irq();
            disable_error_irq();
            disable_tx_warning_irq();
            disable_rx_warning_irq();
            //disable_wake_up_irq();

            // Clear all error and status bits
            clear_error_and_status(ErrorAndStatus::clear_all_bitmask);
        }

        ~CanDriver()
        {
            disable_ored_message_buffer_irq();
            disable_bus_off_irq();
            disable_error_irq();
            disable_tx_warning_irq();
            disable_rx_warning_irq();
            //disable_wake_up_irq();

            // Clear all error and status bits
            clear_error_and_status(ErrorAndStatus::clear_all_bitmask);

            FLEXCAN_Deinit(m_can_base);
        }

        // -------- RX FIFO CONFIGURATION -------------------------------------

        // Configure the Rx FIFO ID filter table
        // NOTE: only 16 filter table elements are available
        //       (all as Type A - one full ID (standard and extended) per ID filter element)
        template<std::size_t Size>
        void config_rx_fifo_id_filter_table(const std::array<const RxFifoFilterElement, Size> element_array)
        {
            static_assert(Size <= 16, "Only up to 16 filter table elements are available");

            uint32_t rx_fifo_filter[Size];

            for(std::size_t i = 0; i <= Size; i++)
            {
                const auto element = element_array[i];

                rx_fifo_filter[i] = (element.format == Frame::Format::standard) ?
                                    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(element.id, element.type, element.format) :
                                    FLEXCAN_RX_FIFO_EXT_FILTER_TYPE_A(element.id, element.type, element.format);
            };

            const flexcan_rx_fifo_config_t rx_fifo_config =
            {
                rx_fifo_filter,
                static_cast<uint8_t>(Size),
                kFLEXCAN_RxFifoFilterTypeA,
                kFLEXCAN_RxFifoPrioHigh
            };

            FLEXCAN_SetRxFifoConfig(m_can_base, &rx_fifo_config, true);
        }

        // -------- FREE TX MESSAGE BUFFER ------------------------------------

        // Get the lowest free Tx Message Buffer
        TxMessageBuffer get_free_tx_message_buffer() const
        {
            if(is_tx_message_buffer_available(TxMessageBuffer::number_1) == true)
            {
                return TxMessageBuffer::number_1;
            }
            else if(is_tx_message_buffer_available(TxMessageBuffer::number_2) == true)
            {
                return TxMessageBuffer::number_2;
            }
            else if(is_tx_message_buffer_available(TxMessageBuffer::number_3) == true)
            {
                return TxMessageBuffer::number_3;
            }
            else if(is_tx_message_buffer_available(TxMessageBuffer::number_4) == true)
            {
                return TxMessageBuffer::number_4;
            }
            else if(is_tx_message_buffer_available(TxMessageBuffer::number_5) == true)
            {
                return TxMessageBuffer::number_5;
            }
            else if(is_tx_message_buffer_available(TxMessageBuffer::number_6) == true)
            {
                return TxMessageBuffer::number_6;
            }
            else
            {
                return TxMessageBuffer::none_available;
            }
        }

        // -------- READ / WRITE FRAME ----------------------------------------

        // Read a frame from the Rx FIFO
        // NOTE: clear the Rx FIFO frame available flag after reading the frame to update
        //       the output of the FIFO with the next frame, reissuing the interrupt
        void read_rx_fifo_frame(Frame& frame) const
        {
            flexcan_frame_t rx_frame;

            const int32_t result = FLEXCAN_ReadRxFifo(m_can_base, &rx_frame);

            // Assert if Rx FIFO is enabled (if it was previously configured)
            assert(result == 0);

            (void)result;

            frame.m_format     = static_cast<Frame::Format>(rx_frame.format);
            frame.m_type       = static_cast<Frame::Type>(rx_frame.type);
            frame.m_data_bytes = rx_frame.length;
            frame.m_id = (frame.m_format == Frame::Format::standard) ? (rx_frame.id >> CAN_ID_STD_SHIFT) : (rx_frame.id >> CAN_ID_EXT_SHIFT);
            frame.m_data[0] = rx_frame.dataByte0;
            frame.m_data[1] = rx_frame.dataByte1;
            frame.m_data[2] = rx_frame.dataByte2;
            frame.m_data[3] = rx_frame.dataByte3;
            frame.m_data[4] = rx_frame.dataByte4;
            frame.m_data[5] = rx_frame.dataByte5;
            frame.m_data[6] = rx_frame.dataByte6;
            frame.m_data[7] = rx_frame.dataByte7;
        }

        // Write a frame to a free Tx Message Buffer (use get_free_tx_message_buffer method)
        // NOTE: this function changes the Message Buffer state to start message
        //       transmit. After that the function returns immediately.
        void write_frame(const TxMessageBuffer tx_message_buffer, const Frame& frame)
        {
            assert(tx_message_buffer != TxMessageBuffer::none_available);

            flexcan_frame_t tx_frame;

            tx_frame.format    = static_cast<flexcan_frame_format_t>(frame.m_format);
            tx_frame.type      = static_cast<flexcan_frame_type_t>(frame.m_type);
            tx_frame.length    = frame.m_data_bytes;
            tx_frame.id        = (frame.m_format == Frame::Format::standard) ? FLEXCAN_ID_STD(frame.m_id) : FLEXCAN_ID_EXT(frame.m_id);
            tx_frame.dataByte0 = frame.m_data[0];
            tx_frame.dataByte1 = frame.m_data[1];
            tx_frame.dataByte2 = frame.m_data[2];
            tx_frame.dataByte3 = frame.m_data[3];
            tx_frame.dataByte4 = frame.m_data[4];
            tx_frame.dataByte5 = frame.m_data[5];
            tx_frame.dataByte6 = frame.m_data[6];
            tx_frame.dataByte7 = frame.m_data[7];

            const int32_t result = FLEXCAN_WriteTxMb(m_can_base, static_cast<uint8_t>(tx_message_buffer), &tx_frame);

            // Assert write tx message buffer successfully
            assert(result == 0);

            (void)result;
        }

        // -------- RX FIFO STATUS FLAGS --------------------------------------

        bool is_rx_fifo_frame_available() const { return (get_rx_fifo_status() & RxFifoStatus::frame_available) != 0; }
        bool is_rx_fifo_warning()         const { return (get_rx_fifo_status() & RxFifoStatus::warning)  != 0; }
        bool is_rx_fifo_overflow()        const { return (get_rx_fifo_status() & RxFifoStatus::overflow) != 0; }

        void clear_rx_fifo_frame_available() { clear_rx_fifo_status(RxFifoStatus::frame_available); }
        void clear_rx_fifo_warning()         { clear_rx_fifo_status(RxFifoStatus::warning);  }
        void clear_rx_fifo_overflow()        { clear_rx_fifo_status(RxFifoStatus::overflow); }

        RxFifoStatusBitmask get_rx_fifo_status() const
        {
            return static_cast<RxFifoStatus>(FLEXCAN_GetMbStatusFlags(m_can_base, static_cast<uint32_t>(RxFifoStatus::bitmask)));
        }

        void clear_rx_fifo_status(const RxFifoStatusBitmask bitmask)
        {
            FLEXCAN_ClearMbStatusFlags(m_can_base, bitmask.bits());
        }

        // NOTES: - it will be performed in Freeze Mode
        //        - all Rx FIFO status flags must be cleared before execute this method
        void clear_rx_fifo()
        {
            enter_freeze_mode();

            FLEXCAN_ClearMbStatusFlags(m_can_base, CAN_IFLAG1_BUF0I_MASK);

            exit_freeze_mode();
        }

        // -------- TX MESSAGE BUFFER STATUS FLAGS ----------------------------

        bool was_frame_transmitted_successfully(const TxMessageBuffer tx_message_buffer) const
        {
            assert(tx_message_buffer != TxMessageBuffer::none_available);

            return (FLEXCAN_GetMbStatusFlags(m_can_base, 1 << static_cast<uint32_t>(tx_message_buffer))) != 0;
        }

        void clear_frame_transmitted(const TxMessageBuffer tx_message_buffer)
        {
            assert(tx_message_buffer != TxMessageBuffer::none_available);

            FLEXCAN_ClearMbStatusFlags(m_can_base, 1 << static_cast<uint32_t>(tx_message_buffer));
        }

        // -------- ERROR AND STATUS FLAGS ------------------------------------

        // Recommended procedure:
        // - call get_error_and_status method (this action also clear the
        //   respective bits that were set since the last read access)
        // - call clear_error_and_status method to clear the interrupt
        //   bits that has triggered the interrupt request and/or to clear
        //   the overrun_error bit if it is set

        ErrorAndStatusBitmask get_error_and_status() const
        {
            return static_cast<ErrorAndStatus>(FLEXCAN_GetStatusFlags(m_can_base));
        }

        void clear_error_and_status(const ErrorAndStatusBitmask bitmask)
        {
            assert((bitmask.bits() & ~static_cast<uint32_t>(ErrorAndStatus::clear_all_bitmask)) == 0);

            FLEXCAN_ClearStatusFlags(m_can_base, bitmask.bits());
        }

        // -------- RX / TX BUS ERROR COUNTER ---------------------------------

        // For further details see the section 44.4.8 - Error Counter
        // from the reference manual (KV4XP100M168RM)

        uint8_t get_rx_error_counter() const
        {
            return static_cast<uint8_t>((m_can_base->ECR & CAN_ECR_RXERRCNT_MASK) >> CAN_ECR_RXERRCNT_SHIFT);
        }

        uint8_t get_tx_error_counter() const
        {
            return static_cast<uint8_t>((m_can_base->ECR & CAN_ECR_TXERRCNT_MASK) >> CAN_ECR_TXERRCNT_SHIFT);
        }

        // -------- RX FIFO AND TX MESSAGE BUFFER INTERRUPTS ------------------

        void enable_ored_message_buffer_interrupts(const OredMessageBufferInterruptBitmask bitmask)
        {
            FLEXCAN_EnableMbInterrupts(m_can_base, bitmask.bits());
        }

        void disable_ored_message_buffer_interrupts(const OredMessageBufferInterruptBitmask bitmask)
        {
            FLEXCAN_DisableMbInterrupts(m_can_base, bitmask.bits());
        }

        OredMessageBufferInterruptBitmask get_ored_message_buffer_interrupts_enabled() const
        {
            return static_cast<OredMessageBufferInterrupt>(m_can_base->IMASK1 & static_cast<uint32_t>(OredMessageBufferInterrupt::bitmask));
        }

        // -------- REMAINING INTERRUPTS --------------------------------------

        void enable_interrupts(const InterruptBitmask bitmask)
        {
            FLEXCAN_EnableInterrupts(m_can_base, bitmask.bits());
        }

        void disable_interrupts(const InterruptBitmask bitmask)
        {
            FLEXCAN_DisableInterrupts(m_can_base, bitmask.bits());
        }

        InterruptBitmask get_interrupts_enabled() const
        {
            return static_cast<Interrupt>(m_can_base->CTRL1 & static_cast<uint32_t>(Interrupt::bitmask));

            // Solve wake-up interrupt if it will be used...
        }

        // -------- OR'ED MESSAGE BUFFER IRQ / IRQ HANDLER --------------------

        void enable_ored_message_buffer_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_EnableIRQ(CAN1_ORed_Message_buffer_IRQn); break;
#endif
                default:                                                        break;
            }
        }

        void disable_ored_message_buffer_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_DisableIRQ(CAN0_ORed_Message_buffer_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_DisableIRQ(CAN1_ORed_Message_buffer_IRQn); break;
#endif
                default:                                                         break;
            }
        }

        bool is_ored_message_buffer_irq_enabled()
        {
            switch(m_can_name)
            {
                case Name::can0: return (NVIC_GetEnableIRQ(CAN0_ORed_Message_buffer_IRQn) != 0); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: return (NVIC_GetEnableIRQ(CAN1_ORed_Message_buffer_IRQn) != 0); break;
#endif
                default:         return false;                                                   break;
            }
        }

        void set_ored_message_buffer_irq_priority(const int32_t irq_priority)
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn, irq_priority); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_SetPriority(CAN1_ORed_Message_buffer_IRQn, irq_priority); break;
#endif
                default:                                                                        break;
            }
        }

        void assign_ored_message_buffer_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_ored_message_buffer_irq_handler = irq_handler;
        }

        void remove_ored_message_buffer_irq_handler()
        {
            m_ored_message_buffer_irq_handler = nullptr;
        }

        // -------- BUS OFF IRQ / IRQ HANDLER ---------------------------------

        void enable_bus_off_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_EnableIRQ(CAN0_Bus_Off_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_EnableIRQ(CAN1_Bus_Off_IRQn); break;
#endif
                default:                                            break;
            }
        }

        void disable_bus_off_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_DisableIRQ(CAN0_Bus_Off_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_DisableIRQ(CAN1_Bus_Off_IRQn); break;
#endif
                default:                                             break;
            }
        }

        bool is_bus_off_irq_enabled()
        {
            switch(m_can_name)
            {
                case Name::can0: return (NVIC_GetEnableIRQ(CAN0_Bus_Off_IRQn) != 0); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: return (NVIC_GetEnableIRQ(CAN1_Bus_Off_IRQn) != 0); break;
#endif
                default:         return false;                                       break;
            }
        }

        void set_bus_off_irq_priority(const int32_t irq_priority)
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_SetPriority(CAN0_Bus_Off_IRQn, irq_priority); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_SetPriority(CAN1_Bus_Off_IRQn, irq_priority); break;
#endif
                default:                                                            break;
            }
        }

        void assign_bus_off_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_bus_off_irq_handler = irq_handler;
        }

        void remove_bus_off_irq_handler()
        {
            m_bus_off_irq_handler = nullptr;
        }

        // -------- ERROR IRQ / IRQ HANDLER -----------------------------------

        void enable_error_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_EnableIRQ(CAN0_Error_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_EnableIRQ(CAN1_Error_IRQn); break;
#endif
                default:                                          break;
            }
        }

        void disable_error_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_DisableIRQ(CAN0_Error_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_DisableIRQ(CAN1_Error_IRQn); break;
#endif
                default:                                           break;
            }
        }

        bool is_error_irq_enabled()
        {
            switch(m_can_name)
            {
                case Name::can0: return (NVIC_GetEnableIRQ(CAN0_Error_IRQn) != 0); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: return (NVIC_GetEnableIRQ(CAN1_Error_IRQn) != 0); break;
#endif
                default:         return false;                                     break;
            }
        }

        void set_error_irq_priority(const int32_t irq_priority)
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_SetPriority(CAN0_Error_IRQn, irq_priority); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_SetPriority(CAN1_Error_IRQn, irq_priority); break;
#endif
                default:                                                          break;
            }
        }

        void assign_error_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_error_irq_handler = irq_handler;
        }

        void remove_error_irq_handler()
        {
            m_error_irq_handler = nullptr;
        }

        // -------- TRANSMIT WARNING IRQ / IRQ HANDLER ------------------------

        void enable_tx_warning_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_EnableIRQ(CAN0_Tx_Warning_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_EnableIRQ(CAN1_Tx_Warning_IRQn); break;
#endif
                default:                                               break;
            }
        }

        void disable_tx_warning_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_DisableIRQ(CAN0_Tx_Warning_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_DisableIRQ(CAN1_Tx_Warning_IRQn); break;
#endif
                default:                                                break;
            }
        }

        bool is_tx_warning_irq_enabled()
        {
            switch(m_can_name)
            {
                case Name::can0: return (NVIC_GetEnableIRQ(CAN0_Tx_Warning_IRQn) != 0); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: return (NVIC_GetEnableIRQ(CAN1_Tx_Warning_IRQn) != 0); break;
#endif
                default:         return false;                                          break;
            }
        }

        void set_tx_warning_irq_priority(const int32_t irq_priority)
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_SetPriority(CAN0_Tx_Warning_IRQn, irq_priority); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_SetPriority(CAN1_Tx_Warning_IRQn, irq_priority); break;
#endif
                default:                                                               break;
            }
        }

        void assign_tx_warning_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_tx_warning_irq_handler = irq_handler;
        }

        void remove_tx_warning_irq_handler()
        {
            m_tx_warning_irq_handler = nullptr;
        }

        // -------- RECEIVE WARNING IRQ / IRQ HANDLER -------------------------

        void enable_rx_warning_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_EnableIRQ(CAN0_Rx_Warning_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_EnableIRQ(CAN1_Rx_Warning_IRQn); break;
#endif
                default:                                               break;
            }
        }

        void disable_rx_warning_irq()
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_DisableIRQ(CAN0_Rx_Warning_IRQn); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_DisableIRQ(CAN1_Rx_Warning_IRQn); break;
#endif
                default:                                                break;
            }
        }

        bool is_rx_warning_irq_enabled()
        {
            switch(m_can_name)
            {
                case Name::can0: return (NVIC_GetEnableIRQ(CAN0_Rx_Warning_IRQn) != 0); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: return (NVIC_GetEnableIRQ(CAN1_Rx_Warning_IRQn) != 0); break;
#endif
                default:         return false;                                          break;
            }
        }

        void set_rx_warning_irq_priority(const int32_t irq_priority)
        {
            switch(m_can_name)
            {
                case Name::can0: NVIC_SetPriority(CAN0_Rx_Warning_IRQn, irq_priority); break;
#if (TARGET_CAN_COUNT == 2)
                case Name::can1: NVIC_SetPriority(CAN1_Rx_Warning_IRQn, irq_priority); break;
#endif
                default:                                                               break;
            }
        }

        void assign_rx_warning_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_rx_warning_irq_handler = irq_handler;
        }

        void remove_rx_warning_irq_handler()
        {
            m_rx_warning_irq_handler = nullptr;
        }

        // -------- WAKE UP IRQ / IRQ HANDLER ---------------------------------

        // ... (if it will be used)

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        enum class Name
        {
            can0 = 0,
#if (TARGET_CAN_COUNT == 2)
            can1
#endif
        };

        struct PinConfig
        {
            Name              can_name;
            PinDriver::PinMux pin_mux;
        };

        // Pin map type
        using PinMap = std::tuple<PinDriver::Name, PinDriver::Name, PinConfig>;

        // Pin map array type
        template <std::size_t Size>
        using PinMapArray = std::array<PinMap, Size>;

#if (TARGET_PACKAGE_PIN_COUNT == 100)
        static constexpr std::size_t m_pin_map_array_size { 5 };
#elif (TARGET_PACKAGE_PIN_COUNT == 64)
        static constexpr std::size_t m_pin_map_array_size { 4 };
#else
        static constexpr std::size_t m_pin_map_array_size { 1 };
#endif

        static constexpr PinMapArray<m_pin_map_array_size> m_pin_map_array
        { { //                   TXD                     RXD      PinConfig
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
              { PinDriver::Name::pe_24, PinDriver::Name::pe_25, { Name::can1, PinDriver::PinMux::alt2 } },
              { PinDriver::Name::pa_12, PinDriver::Name::pa_13, { Name::can0, PinDriver::PinMux::alt2 } },
#endif
              { PinDriver::Name::pb_16, PinDriver::Name::pb_17, { Name::can0, PinDriver::PinMux::alt5 } },
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
              { PinDriver::Name::pb_18, PinDriver::Name::pb_19, { Name::can0, PinDriver::PinMux::alt2 } },
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
              { PinDriver::Name::pc_16, PinDriver::Name::pc_17, { Name::can1, PinDriver::PinMux::alt2 } }
#endif
        } };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONFIGURATION / INITIALIZATION ----------------------------

        // Return the peripheral index according to the pins (constructor helper function):
        static constexpr int32_t get_peripheral_index(const PinDriver::Name txd, const PinDriver::Name rxd)
        {
            const auto pin_config = get_pin_config(txd, rxd);

            return static_cast<int32_t>(pin_config.can_name);
        }

        // Get the pin config struct if the specified txd and rxd are CAN pins
        static constexpr PinConfig get_pin_config(const PinDriver::Name txd, const PinDriver::Name rxd)
        {
            std::size_t index = 0;

            for(; index < m_pin_map_array.size(); ++index)
            {
                const auto pin_map = m_pin_map_array[index];

                if(std::get<0>(pin_map) == txd && std::get<1>(pin_map) == rxd)
                {
                    return std::get<2>(pin_map);
                }
            }

            // Assert txd and rxd are CAN pins
            assert(index < m_pin_map_array.size());

            return { Name::can0, PinDriver::PinMux::pin_disabled_or_analog };
        }

        // NOTE: implemented on the CPP file because it uses parameters from
        //       the library configuration file (xarmlib_config.h).
        void initialize(const Config& config);

        // Get Tx Message Buffer availability
        bool is_tx_message_buffer_available(const TxMessageBuffer tx_message_buffer) const
        {
            assert(tx_message_buffer != TxMessageBuffer::none_available);

            const int8_t mb = static_cast<int8_t>(tx_message_buffer);

            // Code for a Tx Data Frame or a Tx Remote Request Frame
            constexpr uint8_t code_tx_mb_data_or_remote = 0xC;

            return ((m_can_base->MB[mb].CS & CAN_CS_CODE_MASK) != CAN_CS_CODE(code_tx_mb_data_or_remote)) ? true : false;
        }

        void enter_freeze_mode()
        {
            // Set Freeze, halt bits
            m_can_base->MCR |= CAN_MCR_FRZ_MASK;
            m_can_base->MCR |= CAN_MCR_HALT_MASK;

            // Wait until the CAN Module enter freeze mode
            while((m_can_base->MCR & CAN_MCR_FRZACK_MASK) == 0);
        }

        void exit_freeze_mode()
        {
            // Clear Freeze, halt bits
            m_can_base->MCR &= ~CAN_MCR_HALT_MASK;
            m_can_base->MCR &= ~CAN_MCR_FRZ_MASK;

            // Wait until the CAN Module exit freeze mode
            while((m_can_base->MCR & CAN_MCR_FRZACK_MASK) != 0);
        }

        // -------- PRIVATE OR'ED MESSAGE BUFFER IRQ HANDLERS -----------------

        // OR'ed Message Buffer IRQ handler private implementation (call user IRQ handler)
        int32_t ored_message_buffer_irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_ored_message_buffer_irq_handler != nullptr)
            {
                yield = m_ored_message_buffer_irq_handler();
            }

            return yield;
        }

        // OR'ed Message Buffer IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t ored_message_buffer_irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return CanDriver::get_reference(index).ored_message_buffer_irq_handler();
        }

        // -------- PRIVATE BUS OFF IRQ HANDLERS ------------------------------

        // Bus Off IRQ handler private implementation (call user IRQ handler)
        int32_t bus_off_irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_bus_off_irq_handler != nullptr)
            {
                yield = m_bus_off_irq_handler();
            }

            return yield;
        }

        // Bus Off IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t bus_off_irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return CanDriver::get_reference(index).bus_off_irq_handler();
        }

        // -------- PRIVATE ERROR IRQ HANDLERS --------------------------------

        // Error IRQ handler private implementation (call user IRQ handler)
        int32_t error_irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_error_irq_handler != nullptr)
            {
                yield = m_error_irq_handler();
            }

            return yield;
        }

        // Error IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t error_irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return CanDriver::get_reference(index).error_irq_handler();
        }

        // -------- PRIVATE TRANSMIT WARNING IRQ HANDLERS ---------------------

        // Transmit Warning IRQ handler private implementation (call user IRQ handler)
        int32_t tx_warning_irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_tx_warning_irq_handler != nullptr)
            {
                yield = m_tx_warning_irq_handler();
            }

            return yield;
        }

        // Transmit Warning IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t tx_warning_irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return CanDriver::get_reference(index).tx_warning_irq_handler();
        }

        // -------- PRIVATE RECEIVE WARNING IRQ HANDLERS ----------------------

        // Receive Warning IRQ handler private implementation (call user IRQ handler)
        int32_t rx_warning_irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_rx_warning_irq_handler != nullptr)
            {
                yield = m_rx_warning_irq_handler();
            }

            return yield;
        }

        // Receive Warning IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t rx_warning_irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return CanDriver::get_reference(index).rx_warning_irq_handler();
        }

        // -------- PRIVATE WAKE UP IRQ HANDLERS ------------------------------

        /*
        // Wake Up IRQ handler private implementation (call user IRQ handler)
        int32_t wake_up_irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_wake_up_irq_handler != nullptr)
            {
                yield = m_wake_up_irq_handler();
            }

            return yield;
        }

        // Wake Up IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t wake_up_irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return CanDriver::get_reference(index).wake_up_irq_handler();
        }
        */

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        Name       m_can_name;
        CAN_Type*  m_can_base { nullptr };              // Pointer to the CMSIS CAN structure
        IrqHandler m_ored_message_buffer_irq_handler;   // User defined OR'ed Message Buffer IRQ handler
        IrqHandler m_bus_off_irq_handler;               // User defined Bus Off IRQ handler
        IrqHandler m_error_irq_handler;                 // User defined Error IRQ handler
        IrqHandler m_tx_warning_irq_handler;            // User defined Transmit Warning IRQ handler
        IrqHandler m_rx_warning_irq_handler;            // User defined Receive Warning IRQ handler
        //IrqHandler m_wake_up_irq_handler;               // User defined Wake Up IRQ handler
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_CAN_HPP

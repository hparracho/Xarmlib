// ----------------------------------------------------------------------------
// @file    kv4x_uart.hpp
// @brief   Kinetis KV4x UART class.
// @notes   TX and RX FIFOs are always used due to FSL driver implementation.
//          TX FIFO watermark = 0 and RX FIFO watermark = 1.
// @date    21 December 2018
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

#ifndef __XARMLIB_TARGETS_KV4X_UART_HPP
#define __XARMLIB_TARGETS_KV4X_UART_HPP

#include "external/bitmask.hpp"
#include "fsl_uart.h"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of IRQ handlers for all KV4x packages
extern "C" void UART0_RX_TX_IRQHandler(void);
extern "C" void UART0_ERR_IRQHandler  (void);
extern "C" void UART1_RX_TX_IRQHandler(void);
extern "C" void UART1_ERR_IRQHandler  (void);




namespace xarmlib
{
namespace targets
{
namespace kv4x
{




namespace private_uart
{

// UART status flags
enum class Status : uint32_t
{
    tx_data_register_empty           = (UART_S1_TDRE_MASK),             // TX data register empty flag
    tx_complete                      = (UART_S1_TC_MASK),               // Transmission complete flag
    rx_data_register_full            = (UART_S1_RDRF_MASK),             // RX data register full flag
    idle_line                        = (UART_S1_IDLE_MASK),             // Idle line detect flag
    rx_overrun                       = (UART_S1_OR_MASK),               // RX overrun flag
    noise_error                      = (UART_S1_NF_MASK),               // RX takes 3 samples of each received bit.
                                                                        // If any of these samples differ, noise flag sets
    framing_error                    = (UART_S1_FE_MASK),               // Frame error flag,
                                                                        // sets if logic 0 was detected where stop bit expected
    parity_error                     = (UART_S1_PF_MASK),               // If parity enabled, sets upon parity error detection
    lin_break                        = (UART_S2_LBKDIF_MASK    << 8),   // LIN break detect interrupt flag,
                                                                        // sets when LIN break char detected and LIN circuit enabled
    rx_active_edge                   = (UART_S2_RXEDGIF_MASK   << 8),   // RX pin active edge interrupt flag, sets when active edge detected
    rx_active                        = (UART_S2_RAF_MASK       << 8),   // Receiver Active Flag (RAF), sets at beginning of valid start bit
    noise_error_in_rx_data_register  = (UART_ED_NOISY_MASK     << 16),  // Noisy bit, sets if noise detected
    parity_error_in_rx_data_register = (UART_ED_PARITYE_MASK   << 16),  // Parity bit, sets if parity error detected
    tx_fifo_empty                    = (UART_SFIFO_TXEMPT_MASK << 24),  // TXEMPT bit, sets if TX buffer is empty
    rx_fifo_empty                    = (UART_SFIFO_RXEMPT_MASK << 24),  // RXEMPT bit, sets if RX buffer is empty
    tx_fifo_overflow                 = (UART_SFIFO_TXOF_MASK   << 24),  // TXOF bit, sets if TX buffer overflow occurred
    rx_fifo_overflow                 = (UART_SFIFO_RXOF_MASK   << 24),  // RXOF bit, sets if receive buffer overflow
    rx_fifo_underflow                = (UART_SFIFO_RXUF_MASK   << 24),  // RXUF bit, sets if receive buffer underflow

    // NOTE: The following flags can only clear or set by the hardware itself.
    //       (for each flag see the section 47.4 - Memory map and registers
    //       from the reference manual (KV4XP100M168RM))
    //
    //       tx_data_register_empty           (S1[TDRE])
    //       tx_complete                      (S1[TC])
    //       rx_data_register_full            (S1[RDRF])
    //       rx_active                        (S2[RAF])
    //       noise_error_in_rx_data_register  (ED[NOISY])
    //       parity_error_in_rx_data_register (ED[PARITYE])
    //       tx_fifo_empty                    (SFIFO[TXEMPT])
    //       rx_fifo_empty                    (SFIFO[RXEMPT])
    clear_all_manual_bitmask         = idle_line
                                     | rx_overrun
                                     | noise_error
                                     | framing_error
                                     | parity_error
                                     | lin_break
                                     | rx_active_edge
                                     | tx_fifo_overflow
                                     | rx_fifo_overflow
                                     | rx_fifo_underflow,

    bitmask                          = tx_data_register_empty
                                     | tx_complete
                                     | rx_data_register_full
                                     | idle_line
                                     | rx_overrun
                                     | noise_error
                                     | framing_error
                                     | parity_error
                                     | lin_break
                                     | rx_active_edge
                                     | rx_active
                                     | noise_error_in_rx_data_register
                                     | parity_error_in_rx_data_register
                                     | tx_fifo_empty
                                     | rx_fifo_empty
                                     | tx_fifo_overflow
                                     | rx_fifo_overflow
                                     | rx_fifo_underflow
};




// UART status interrupt sources
enum class StatusInterrupt : uint32_t
{
    lin_break              = (UART_BDH_LBKDIE_MASK),        // LIN break detect interrupt
    rx_active_edge         = (UART_BDH_RXEDGIE_MASK),       // RX active edge interrupt
    tx_data_register_empty = (UART_C2_TIE_MASK      << 8),  // Transmit data register empty interrupt
    tx_complete            = (UART_C2_TCIE_MASK     << 8),  // Transmission complete interrupt
    rx_data_register_full  = (UART_C2_RIE_MASK      << 8),  // Receiver data register full interrupt
    idle_line              = (UART_C2_ILIE_MASK     << 8),  // Idle line interrupt
    bitmask                = lin_break
                           | rx_active_edge
                           | tx_data_register_empty
                           | tx_complete
                           | rx_data_register_full
                           | idle_line
};

// UART error interrupt sources
enum class ErrorInterrupt : uint32_t
{
    rx_overrun             = (UART_C3_ORIE_MASK     << 16), // Receiver overrun interrupt
    noise_error            = (UART_C3_NEIE_MASK     << 16), // Noise error flag interrupt
    framing_error          = (UART_C3_FEIE_MASK     << 16), // Framing error flag interrupt
    parity_error           = (UART_C3_PEIE_MASK     << 16), // Parity error flag interrupt
    rx_fifo_overflow       = (UART_CFIFO_RXOFE_MASK << 24), // RX FIFO overflow interrupt
    tx_fifo_overflow       = (UART_CFIFO_TXOFE_MASK << 24), // TX FIFO overflow interrupt
    rx_fifo_underflow      = (UART_CFIFO_RXUFE_MASK << 24), // RX FIFO underflow interrupt
    bitmask                = rx_overrun
                           | noise_error
                           | framing_error
                           | parity_error
                           | rx_fifo_overflow
                           | tx_fifo_overflow
                           | rx_fifo_underflow
};

BITMASK_DEFINE_VALUE_MASK(Status,          static_cast<uint32_t>(Status::bitmask))
BITMASK_DEFINE_VALUE_MASK(StatusInterrupt, static_cast<uint32_t>(StatusInterrupt::bitmask))
BITMASK_DEFINE_VALUE_MASK(ErrorInterrupt,  static_cast<uint32_t>(ErrorInterrupt::bitmask))

} // namespace private_uart




class UartDriver : private PeripheralRefCounter<UartDriver, TARGET_UART_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::UART0_RX_TX_IRQHandler(void);
        friend void ::UART0_ERR_IRQHandler  (void);
        friend void ::UART1_RX_TX_IRQHandler(void);
        friend void ::UART1_ERR_IRQHandler  (void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralUart = PeripheralRefCounter<UartDriver, TARGET_UART_COUNT>;

        // Data length selection
        enum class DataBits
        {
            bits_8 = 0,     // 8 bit data mode
            bits_9          // 9 bit data mode
        };

        // Stop bits selection
        enum class StopBits
        {
            bits_1 = 0,     // 1 stop bit
            bits_2          // 2 stop bits
        };

        // Parity selection
        enum class Parity
        {
            none = 0,       // No parity
            even = 2,       // Even parity select
            odd             // Odd parity select
        };

        // Idle type selection
        enum class IdleType
        {
            start_bit = 0,  // Start counting after a valid start bit
            stop_bit        // Start counting after a stop bit
        };

        struct Config
        {
            int32_t  baudrate  = 9600;
            DataBits data_bits = DataBits::bits_8;
            StopBits stop_bits = StopBits::bits_1;
            Parity   parity    = Parity::none;
            IdleType idle_type = IdleType::start_bit;
        };

        // Type safe accessor to status flags
        using Status        = private_uart::Status;
        using StatusBitmask = bitmask::bitmask<Status>;

        // Type safe accessor to status interrupt sources
        using StatusInterrupt        = private_uart::StatusInterrupt;
        using StatusInterruptBitmask = bitmask::bitmask<StatusInterrupt>;

        // Type safe accessor to error interrupt sources
        using ErrorInterrupt        = private_uart::ErrorInterrupt;
        using ErrorInterruptBitmask = bitmask::bitmask<ErrorInterrupt>;

        // IRQ handlers definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        UartDriver(const PinDriver::Name txd, const PinDriver::Name rxd, const Config& config) : PeripheralUart(*this)
        {
            const std::size_t pin_map_index = get_pin_map_index(txd, rxd);

            // Assert txd and rxd are Uart pins
            assert(pin_map_index < m_pin_map_array.size());

            const auto pin_map = m_pin_map_array[pin_map_index];

            PinDriver::set_pin_mux(txd, pin_map.pin_mux);
            PinDriver::set_pin_mux(rxd, pin_map.pin_mux);

            select_data_sources(pin_map.uart_name, TransmitDataSource::tx_pin, ReceiveDataSource::rx_pin);

            m_uart_name = pin_map.uart_name;

            switch(m_uart_name)
            {
                case Name::uart0: m_uart_base = UART0; break;
                case Name::uart1: m_uart_base = UART1; break;
            };

            const int32_t result = initialize(config);

            // Assert baudrate less than 3%
            assert(result == 0);

            disable_status_irq();
            disable_error_irq();

            // Clear all manual status bits
            //clear_status(Status::clear_all_manual_bitmask);
        }

        ~UartDriver()
        {
            disable_status_irq();
            disable_error_irq();

            // Clear all manual status bits
            //clear_status(Status::clear_all_manual_bitmask);

            UART_Deinit(m_uart_base);
        }

        // -------- BAUDRATE --------------------------------------------------

        // NOTE: implemented on the CPP file because it uses parameters from
        //       the library configuration file (xarmlib_config.h).
        void set_baudrate(const int32_t baudrate);

        // -------- READ / WRITE ----------------------------------------------

        // Read data that has been received
        uint32_t read_data() const
        {
            return UART_ReadByte(m_uart_base);
        }

        // Write data to be transmitted
        void write_data(const uint32_t value)
        {
            UART_WriteByte(m_uart_base, value);
        }

        // -------- ENABLE / DISABLE TX AND RX --------------------------------

        // Enable TX and RX
        void enable()
        {
            UART_EnableTx(m_uart_base, true);
            UART_EnableRx(m_uart_base, true);
        }

        // Disable TX and RX
        void disable()
        {
            UART_EnableTx(m_uart_base, false);
            UART_EnableRx(m_uart_base, false);
        }

        // Gets the enable state
        bool is_enabled() const
        {
            return (((m_uart_base->C2 & UART_C2_TE_MASK) && (m_uart_base->C2 & UART_C2_RE_MASK)) != 0);
        }

        // -------- STATUS FLAGS ----------------------------------------------

        /* FSL driver method, however it doesn't clear the flag automatically
         *
         * bool is_rx_ready() const { return m_uart_base->RCFIFO != 0; }
         */
        bool is_rx_ready() const { return (get_status() & Status::rx_data_register_full) != 0; }
        bool is_rx_idle () const { return (get_status() & Status::rx_active) != 0; }
        bool is_tx_ready() const { return (get_status() & Status::tx_data_register_empty) != 0; }
        bool is_tx_idle () const { return (get_status() & Status::tx_complete) != 0; }

        StatusBitmask get_status() const
        {
            return static_cast<Status>(UART_GetStatusFlags(m_uart_base));
        }

        void clear_status(const StatusBitmask bitmask)
        {
            // NOTE: some flags can only clear or set by the hardware itself
            assert((bitmask.bits() & ~static_cast<uint32_t>(Status::clear_all_manual_bitmask)) == 0);

            while(is_tx_idle() == false && is_rx_idle() == false);

            const int32_t result = UART_ClearStatusFlags(m_uart_base, bitmask.bits());

            // NOTE: some flags can only clear or set by the hardware itself
            assert(result == 0);





            /* Using FSL driver to clear flags, if we want to clear idle_line
             * or noise_error or framing_error or parity_error flags
             * and rx_overrun flag, it will read D twice and it will set
             * rx_fifo_underflow flag!
             *
             * const int32_t result = UART_ClearStatusFlags(m_uart_base, bitmask.bits());

             * // NOTE: some flags can only clear or set by the hardware itself
             * assert(result == 0);
             */




//            const bool interrupt_underflow = ((get_error_interrupts_enabled() & ErrorInterrupt::rx_fifo_underflow)).bits() ? true : false;
//
//            disable_error_interrupts(ErrorInterrupt::rx_fifo_underflow);

//            const int32_t result = UART_ClearStatusFlags(m_uart_base, (bitmask & ~(Status::rx_overrun | Status::rx_fifo_underflow)).bits());
//
//            // NOTE: some flags can only clear or set by the hardware itself
//            assert(result == 0);
//
//            if(bitmask & Status::rx_overrun)
//            {
//                (void)m_uart_base->S1;
//                (void)m_uart_base->D;
//                m_uart_base->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
//                (void)m_uart_base->S1;
//                (void)m_uart_base->SFIFO;
//            }
//
////            if(interrupt_underflow == true)
//            {
//                enable_error_interrupts(ErrorInterrupt::rx_fifo_underflow);
//            }
//
//            if(bitmask & Status::rx_overrun)
//            {
//                /* Read base->D to clear the flags and Flush all data in FIFO. */
////                (void)m_uart_base->S1;
////                (void)m_uart_base->D;
////
////                /* Flush FIFO date, otherwise FIFO pointer will be in unknown state. */
////                m_uart_base->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
//
//                (void)m_uart_base->S1;
//                (void)m_uart_base->SFIFO;
//            }
//
//            enable_error_interrupts(ErrorInterrupt::rx_fifo_underflow);




            // NOTE: some flags can only clear or set by the hardware itself
//            assert((bitmask.bits() & ~static_cast<uint32_t>(Status::clear_all_manual_bitmask)) == 0);
//
////            const uint32_t mask = bitmask.bits();//(bitmask & Status::clear_all_manual_bitmask).bits();
//
//            const uint8_t register_s2 = m_uart_base->S2 & ~(UART_S2_RXEDGIF_MASK | UART_S2_LBKDIF_MASK);
//
//            m_uart_base->S2 = register_s2 | static_cast<uint8_t>(bitmask.bits() >> 8);
//
//            m_uart_base->SFIFO = static_cast<uint8_t>(bitmask.bits() >> 24);
//
//            if(bitmask & (Status::idle_line | Status::noise_error | Status::framing_error | Status::parity_error))
//            {
//                // Read m_uart_base->D to clear the flag(s)
//                (void)m_uart_base->S1;
//                (void)m_uart_base->D;
//            }
//
//            if(bitmask & Status::rx_overrun)
//            {
////                (void)m_uart_base->S1;
////                (void)m_uart_base->D;
////
////                // Flush all data in FIFO, otherwise FIFO pointer will be in unknown state
////                m_uart_base->CFIFO |= UART_CFIFO_RXFLUSH_MASK;
//            }
//
//            const auto status = get_status();
//
//            assert((status & Status::clear_all_manual_bitmask).bits() == 0);
        }

        // -------- STATUS INTERRUPTS -----------------------------------------

        void enable_status_interrupts(const StatusInterruptBitmask bitmask)
        {
            UART_EnableInterrupts(m_uart_base, bitmask.bits());
        }

        void disable_status_interrupts(const StatusInterruptBitmask bitmask)
        {
            UART_DisableInterrupts(m_uart_base, bitmask.bits());
        }

        StatusInterruptBitmask get_status_interrupts_enabled() const
        {
            return static_cast<StatusInterrupt>(UART_GetEnabledInterrupts(m_uart_base));
        }

        // -------- ERROR INTERRUPTS ------------------------------------------

        void enable_error_interrupts(const ErrorInterruptBitmask bitmask)
        {
            UART_EnableInterrupts(m_uart_base, bitmask.bits());
        }

        void disable_error_interrupts(const ErrorInterruptBitmask bitmask)
        {
            UART_DisableInterrupts(m_uart_base, bitmask.bits());
        }

        ErrorInterruptBitmask get_error_interrupts_enabled() const
        {
            return static_cast<ErrorInterrupt>(UART_GetEnabledInterrupts(m_uart_base));
        }

        // -------- STATUS IRQ / IRQ HANDLER ----------------------------------

        void enable_status_irq()
        {
            switch(m_uart_name)
            {
                case Name::uart0: NVIC_EnableIRQ(UART0_RX_TX_IRQn); break;
                case Name::uart1: NVIC_EnableIRQ(UART1_RX_TX_IRQn); break;
                default:                                            break;
            }
        }

        void disable_status_irq()
        {
            switch(m_uart_name)
            {
                case Name::uart0: NVIC_DisableIRQ(UART0_RX_TX_IRQn); break;
                case Name::uart1: NVIC_DisableIRQ(UART1_RX_TX_IRQn); break;
                default:                                             break;
            }
        }

        bool is_status_irq_enabled()
        {
            switch(m_uart_name)
            {
                case Name::uart0: return (NVIC_GetEnableIRQ(UART0_RX_TX_IRQn) != 0); break;
                case Name::uart1: return (NVIC_GetEnableIRQ(UART1_RX_TX_IRQn) != 0); break;
                default:          return false;                                      break;
            }
        }

        void set_status_irq_priority(const int32_t irq_priority)
        {
            switch(m_uart_name)
            {
                case Name::uart0: NVIC_SetPriority(UART0_RX_TX_IRQn, irq_priority); break;
                case Name::uart1: NVIC_SetPriority(UART1_RX_TX_IRQn, irq_priority); break;
                default:                                                            break;
            }
        }

        void assign_status_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_status_irq_handler = irq_handler;
        }

        void remove_status_irq_handler()
        {
            m_status_irq_handler = nullptr;
        }

        // -------- ERROR IRQ / IRQ HANDLER -----------------------------------

        void enable_error_irq()
        {
            switch(m_uart_name)
            {
                case Name::uart0: NVIC_EnableIRQ(UART0_ERR_IRQn); break;
                case Name::uart1: NVIC_EnableIRQ(UART1_ERR_IRQn); break;
                default:                                          break;
            }
        }

        void disable_error_irq()
        {
            switch(m_uart_name)
            {
                case Name::uart0: NVIC_DisableIRQ(UART0_ERR_IRQn); break;
                case Name::uart1: NVIC_DisableIRQ(UART1_ERR_IRQn); break;
                default:                                           break;
            }
        }

        bool is_error_irq_enabled()
        {
            switch(m_uart_name)
            {
                case Name::uart0: return (NVIC_GetEnableIRQ(UART0_ERR_IRQn) != 0); break;
                case Name::uart1: return (NVIC_GetEnableIRQ(UART1_ERR_IRQn) != 0); break;
                default:          return false;                                    break;
            }
        }

        void set_error_irq_priority(const int32_t irq_priority)
        {
            switch(m_uart_name)
            {
                case Name::uart0: NVIC_SetPriority(UART0_ERR_IRQn, irq_priority); break;
                case Name::uart1: NVIC_SetPriority(UART1_ERR_IRQn, irq_priority); break;
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

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        enum class Name
        {
            uart0 = 0,
            uart1
        };

        enum class TransmitDataSource
        {
            tx_pin = 0,
            ftm1_channel0_output
        };

        enum class ReceiveDataSource
        {
            rx_pin = 0,
            cmp0,
            cmp1
        };

        struct PinMap
        {
            PinDriver::Name   pin_name_txd;
            PinDriver::Name   pin_name_rxd;
            Name              uart_name;
            PinDriver::PinMux pin_mux;
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONFIGURATION ---------------------------------------------

        static constexpr std::size_t get_pin_map_index(const PinDriver::Name txd, const PinDriver::Name rxd)
        {
            std::size_t index = 0;

            for(; index < m_pin_map_array.size(); ++index)
            {
                const auto pin_map = m_pin_map_array[index];

                if(pin_map.pin_name_txd == txd && pin_map.pin_name_rxd == rxd)
                {
                    return index;
                }
            }

            return index;
        }

        static void select_data_sources(const Name name, const TransmitDataSource transmit_data_source,
                                                         const ReceiveDataSource  receive_data_source)
        {
            switch(name)
            {
                case Name::uart0:
                    SIM->SOPT5 = ((SIM->SOPT5 & (~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK)))           // Mask bits to zero which are setting
                                                 | SIM_SOPT5_UART0TXSRC(static_cast<uint32_t>(transmit_data_source))  // UART0 transmit data source select
                                                 | SIM_SOPT5_UART0RXSRC(static_cast<uint32_t>(receive_data_source))); // UART0 receive data source select
                    break;
                case Name::uart1:
                    SIM->SOPT5 = ((SIM->SOPT5 & (~(SIM_SOPT5_UART1TXSRC_MASK | SIM_SOPT5_UART1RXSRC_MASK)))           // Mask bits to zero which are setting
                                                 | SIM_SOPT5_UART1TXSRC(static_cast<uint32_t>(transmit_data_source))  // UART1 transmit data source select
                                                 | SIM_SOPT5_UART1RXSRC(static_cast<uint32_t>(receive_data_source))); // UART1 receive data source select
                    break;
            }
        }

        // NOTE: implemented on the CPP file because it uses parameters from
        //       the library configuration file (xarmlib_config.h).
        int32_t initialize(const Config& config);

        // -------- PRIVATE STATUS IRQ HANDLERS -------------------------------

        // Status IRQ handler private implementation (call user IRQ handler)
        int32_t status_irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_status_irq_handler != nullptr)
            {
                yield = m_status_irq_handler();
            }

            return yield;
        }

        // Status IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t status_irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return UartDriver::get_reference(index).status_irq_handler();
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

            return UartDriver::get_reference(index).error_irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        Name       m_uart_name;
        UART_Type* m_uart_base { nullptr }; // Pointer to the CMSIS UART structure
        IrqHandler m_status_irq_handler;    // User defined status IRQ handler
        IrqHandler m_error_irq_handler;     // User defined error IRQ handler

#if (TARGET_PACKAGE_PIN_COUNT == 100)
        static constexpr std::array<PinMap, 10> m_pin_map_array
#else
        static constexpr std::array<PinMap, 8> m_pin_map_array
#endif
        { { //                   TXD                     RXD
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
              { PinDriver::Name::pe_0,  PinDriver::Name::pe_1,  Name::uart1, PinDriver::PinMux::alt3 },
#endif
              { PinDriver::Name::pe_16, PinDriver::Name::pe_17, Name::uart1, PinDriver::PinMux::alt3 },
#if (TARGET_PACKAGE_PIN_COUNT == 48 || TARGET_PACKAGE_PIN_COUNT == 100)
              { PinDriver::Name::pe_20, PinDriver::Name::pe_21, Name::uart0, PinDriver::PinMux::alt4 },
#endif
              { PinDriver::Name::pa_2,  PinDriver::Name::pa_1,  Name::uart0, PinDriver::PinMux::alt2 },
#if (TARGET_PACKAGE_PIN_COUNT == 100)
              { PinDriver::Name::pa_14, PinDriver::Name::pa_15, Name::uart0, PinDriver::PinMux::alt3 },
#endif
              { PinDriver::Name::pb_1,  PinDriver::Name::pb_0,  Name::uart0, PinDriver::PinMux::alt7 },
              { PinDriver::Name::pb_17, PinDriver::Name::pb_16, Name::uart0, PinDriver::PinMux::alt3 },
              { PinDriver::Name::pc_4,  PinDriver::Name::pc_3,  Name::uart1, PinDriver::PinMux::alt3 },
              { PinDriver::Name::pc_7,  PinDriver::Name::pc_6,  Name::uart0, PinDriver::PinMux::alt5 },
              { PinDriver::Name::pd_7,  PinDriver::Name::pd_6,  Name::uart0, PinDriver::PinMux::alt3 }
        } };
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_UART_HPP

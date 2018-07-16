// ----------------------------------------------------------------------------
// @file    lpc84x_usart.hpp
// @brief   NXP LPC84x USART class (takes control of FRG0).
// @notes   Synchronous mode not implemented.
// @date    14 July 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_USART_HPP
#define __XARMLIB_TARGETS_LPC84X_USART_HPP

#include "external/bitmask.hpp"
#include "targets/LPC84x/lpc84x_cmsis.hpp"
#include "targets/LPC84x/lpc84x_pin.hpp"
#include "targets/LPC84x/lpc84x_swm.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"

#include <cmath>




// Forward declaration of IRQ handlers for both LPC844 and LPC845
extern "C" void USART0_IRQHandler(void);
extern "C" void USART1_IRQHandler(void);

#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
// Forward declaration of additional IRQ handlers for LPC845
extern "C" void USART2_IRQHandler(void);
extern "C" void PININT6_USART3_IRQHandler(void); // PININT6 / USART3 shared handler
extern "C" void PININT7_USART4_IRQHandler(void); // PININT7 / USART4 shared handler
#endif




namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




namespace private_usart
{

// USART Status register (STAT) bits
enum class Status
{
    RX_READY       = (1 << 0),  // Receiver ready
    RX_IDLE        = (1 << 1),  // Receiver idle
    TX_READY       = (1 << 2),  // Transmitter ready for data
    TX_IDLE        = (1 << 3),  // Transmitter idle
    CTS            = (1 << 4),  // Status of CTS signal
    CTS_DELTA      = (1 << 5),  // Change in CTS state
    TX_DISABLED    = (1 << 6),  // Transmitter disabled
    RX_OVERRUN     = (1 << 8),  // Overrun Error interrupt flag
    RX_BREAK       = (1 << 10), // Received break
    RX_BREAK_DELTA = (1 << 11), // Change in receive break detection
    START          = (1 << 12), // Start detected
    FRAME_ERROR    = (1 << 13), // Framing Error interrupt flag
    PARITY_ERROR   = (1 << 14), // Parity Error interrupt flag
    RX_NOISE       = (1 << 15), // Received Noise interrupt flag
    AUTOBAUD_ERROR = (1 << 16), // Auto-baud Error
    ALL            = 0x1F920    // 1'1111'1001'0010'0000
};

// USART Interrupt Enable Get, Set or Clear Register (INTSTAT / INTENSET / INTENCLR) bits
enum class Interrupt
{
    RX_READY       = (1 << 0),  // Receiver ready
    TX_READY       = (1 << 2),  // Transmitter ready for data
    TX_IDLE        = (1 << 3),  // Transmitter idle
    CTS_DELTA      = (1 << 5),  // Change in CTS state
    TX_DISABLEDT   = (1 << 6),  // Transmitter disabled
    RX_OVERRUN     = (1 << 8),  // Overrun Error interrupt flag
    RX_BREAK_DELTA = (1 << 11), // Change in receive break detection
    START          = (1 << 12), // Start detected
    FRAME_ERROR    = (1 << 13), // Framing Error interrupt flag
    PARITY_ERROR   = (1 << 14), // Parity Error interrupt flag
    RX_NOISE       = (1 << 15), // Received Noise interrupt flag
    AUTOBAUD_ERROR = (1 << 16), // Auto-baud Error
    ALL            = 0x1F96D    // 1'1111'1001'0110'1101
};

BITMASK_DEFINE_VALUE_MASK(Status,    static_cast<uint32_t>(Status::ALL))
BITMASK_DEFINE_VALUE_MASK(Interrupt, static_cast<uint32_t>(Interrupt::ALL))

} // namespace private_usart




class Usart : private PeripheralRefCounter<Usart, TARGET_USART_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::USART0_IRQHandler(void);
        friend void ::USART1_IRQHandler(void);
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
        friend void ::USART2_IRQHandler(void);
        friend void ::PININT6_USART3_IRQHandler(void); // PININT6 / USART3 shared handler
        friend void ::PININT7_USART4_IRQHandler(void); // PININT7 / USART4 shared handler
#endif

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralUsart = PeripheralRefCounter<Usart, TARGET_USART_COUNT>;

        // USART peripheral names selection
        enum class Name
        {
            USART0 = 0,
            USART1,
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
            USART2,
            USART3,
            USART4
#endif
        };

        // Data length selection (defined to map the CFG register directly)
        enum class DataBits
        {
            BITS_7 = (0 << 2),  // USART 7 bit data mode
            BITS_8 = (1 << 2),  // USART 8 bit data mode
            BITS_9 = (2 << 2)   // USART 9 bit data mode
        };

        // Stop bits selection (defined to map the CFG register directly)
        enum class StopBits
        {
            BITS_1 = (0 << 6),  // USART 1 stop bit
            BITS_2 = (1 << 6)   // USART 2 stop bits
        };

        // Parity selection (defined to map the CFG register directly)
        enum class Parity
        {
            NONE = (0 << 4),    // USART no parity
            EVEN = (2 << 4),    // USART even parity select
            ODD  = (3 << 4)     // USART odd parity select
        };

        // Type safe accessor to STAT register
        using Status        = private_usart::Status;
        using StatusBitmask = bitmask::bitmask<Status>;

        // Type safe accessor to INTSTAT / INTENSET / INTENCLR registers
        using Interrupt        = private_usart::Interrupt;
        using InterruptBitmask = bitmask::bitmask<Interrupt>;

        // IRQ handler definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        Usart(const Pin::Name txd,
              const Pin::Name rxd,
              const int32_t   baudrate,
              const DataBits  data_bits,
              const StopBits  stop_bits,
              const Parity    parity) : PeripheralUsart(*this)
        {
            // Initialize and configure FRG0 if this is the first USART peripheral instantiation
            if(get_used() == 1)
            {
                initialize_frg0();
            }

            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: m_usart = LPC_USART0;
                                   Clock::set_peripheral_clock_source(Clock::PeripheralClockSelect::USART0,
                                                                      Clock::PeripheralClockSource::FRG0_CLK);
                                   Clock::enable(Clock::Peripheral::USART0);
                                   Power::reset(Power::ResetPeripheral::USART0);
                                   Swm::assign(Swm::PinMovable::U0_RXD_I, rxd);
                                   Swm::assign(Swm::PinMovable::U0_TXD_O, txd);
                                   break;

                case Name::USART1: m_usart = LPC_USART1;
                                   Clock::set_peripheral_clock_source(Clock::PeripheralClockSelect::USART1,
                                                                      Clock::PeripheralClockSource::FRG0_CLK);
                                   Clock::enable(Clock::Peripheral::USART1);
                                   Power::reset(Power::ResetPeripheral::USART1);
                                   Swm::assign(Swm::PinMovable::U1_RXD_I, rxd);
                                   Swm::assign(Swm::PinMovable::U1_TXD_O, txd);
                                   break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::USART2: m_usart = LPC_USART2;
                                   Clock::set_peripheral_clock_source(Clock::PeripheralClockSelect::USART2,
                                                                      Clock::PeripheralClockSource::FRG0_CLK);
                                   Clock::enable(Clock::Peripheral::USART2);
                                   Power::reset(Power::ResetPeripheral::USART2);
                                   Swm::assign(Swm::PinMovable::U2_RXD_I, rxd);
                                   Swm::assign(Swm::PinMovable::U2_TXD_O, txd);
                                   break;

                case Name::USART3: m_usart = LPC_USART3;
                                   Clock::set_peripheral_clock_source(Clock::PeripheralClockSelect::USART3,
                                                                      Clock::PeripheralClockSource::FRG0_CLK);
                                   Clock::enable(Clock::Peripheral::USART3);
                                   Power::reset(Power::ResetPeripheral::USART3);
                                   Swm::assign(Swm::PinMovable::U3_RXD_I, rxd);
                                   Swm::assign(Swm::PinMovable::U3_TXD_O, txd);
                                   break;

                case Name::USART4: m_usart = LPC_USART4;
                                   Clock::set_peripheral_clock_source(Clock::PeripheralClockSelect::USART4,
                                                                      Clock::PeripheralClockSource::FRG0_CLK);
                                   Clock::enable(Clock::Peripheral::USART4);
                                   Power::reset(Power::ResetPeripheral::USART4);
                                   Swm::assign(Swm::PinMovable::U4_RXD_I, rxd);
                                   Swm::assign(Swm::PinMovable::U4_TXD_O, txd);
                                   break;
#endif
            };

            Pin::set_mode(txd, Pin::FunctionMode::HIZ);
            Pin::set_mode(rxd, Pin::FunctionMode::PULL_UP);

            disable_irq();

            // No continuous break, no address detect, no Tx disable, no CC, no CLRCC
            m_usart->CTL = 0;

            // Clear all status bits
            clear_status(Status::ALL);

            set_format(data_bits, stop_bits, parity);
            set_baudrate(baudrate);
        }

        ~Usart()
        {
            // Disable peripheral
            disable();

            const Name name = static_cast<Name>(get_index());

            // Disable peripheral clock sources and interrupts
            switch(name)
            {
                case Name::USART0: Clock::disable(Clock::Peripheral::USART0);
                                   NVIC_DisableIRQ(USART0_IRQn);
                                   break;
                case Name::USART1: Clock::disable(Clock::Peripheral::USART1);
                                   NVIC_DisableIRQ(USART1_IRQn);
                                   break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::USART2: Clock::disable(Clock::Peripheral::USART2);
                                   NVIC_DisableIRQ(USART2_IRQn);
                                   break;
                case Name::USART3: Clock::disable(Clock::Peripheral::USART3);
                                   /* DO NOT DISABLE SHARED INTERRUPTS */     // PININT6 / USART3 shared interrupt
                                   break;
                case Name::USART4: Clock::disable(Clock::Peripheral::USART4);
                                   /* DO NOT DISABLE SHARED INTERRUPTS */     // PININT7 / USART4 shared interrupt
                                   break;
#endif
            }
        }

        // -------- FORMAT / BAUDRATE -----------------------------------------

        void set_format(const DataBits data_bits, const StopBits stop_bits, const Parity parity)
        {
            set_data_bits(data_bits);
            set_stop_bits(stop_bits);
            set_parity(parity);
        }

        void set_data_bits(const DataBits data_bits)
        {
            const bool enabled = is_enabled();

            if(enabled == true)
            {
                // USART enabled...

                // Make sure the USART is not currently sending or receiving data
                while(is_tx_idle() == false || is_rx_idle() == false)
                {}

                // Disable USART
                disable();
            }

            // Set new data bits
            m_usart->CFG = (m_usart->CFG & ~CFG_DATALEN_BITMASK) | static_cast<uint32_t>(data_bits);

            if(enabled == true)
            {
                // If previously enabled, re-enable.
                enable();
            }
        }

        void set_stop_bits(const StopBits stop_bits)
        {
            const bool enabled = is_enabled();

            if(enabled == true)
            {
                // USART enabled...

                // Make sure the USART is not currently sending or receiving data
                while(is_tx_idle() == false || is_rx_idle() == false)
                {}

                // Disable USART
                disable();
            }

            // Set new stop bits
            m_usart->CFG = (m_usart->CFG & ~CFG_STOPLEN_BITMASK) | static_cast<uint32_t>(stop_bits);

            if(enabled == true)
            {
                // If previously enabled, re-enable.
                enable();
            }
        }

        void set_parity(const Parity parity)
        {
            const bool enabled = is_enabled();

            if(enabled == true)
            {
                // USART enabled...

                // Make sure the USART is not currently sending or receiving data
                while(is_tx_idle() == false || is_rx_idle() == false)
                {}

                // Disable USART
                disable();
            }

            // Set new parity
            m_usart->CFG = (m_usart->CFG & ~CFG_PARITY_BITMASK) | static_cast<uint32_t>(parity);

            if(enabled == true)
            {
                // If previously enabled, re-enable.
                enable();
            }
        }

        void set_baudrate(const int32_t baudrate)
        {
            assert(baudrate > 0);

            const bool enabled = is_enabled();

            if(enabled == true)
            {
                // USART enabled...

                // Make sure the USART is not currently sending or receiving data
                while(is_tx_idle() == false || is_rx_idle() == false)
                {}

                // Disable USART
                disable();
            }

            const int32_t div = get_baudrate_generator_div(baudrate);
            assert(div >= 1 && div <= 65536);

            // Set baudrate generator register
            m_usart->BRG = div - 1;

            if(enabled == true)
            {
                // If previously enabled, re-enable.
                enable();
            }
        }

        // -------- READ / WRITE ----------------------------------------------

        // Read data that has been received
        uint32_t read_data() const
        {
            // Strip off undefined reserved bits, keep 9 lower bits.
            return m_usart->RXDAT & 0x000001FF;
        }

        // Write data to be transmitted
        void write_data(const uint32_t value)
        {
            // Strip off undefined reserved bits, keep 9 lower bits.
            m_usart->TXDAT = value & 0x000001FF;
        }

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        void enable() { m_usart->CFG |= CFG_ENABLE; }

        // Disable peripheral
        void disable() { m_usart->CFG &= ~CFG_ENABLE; }

        // Gets the enable state
        bool is_enabled() const { return (m_usart->CFG & CFG_ENABLE) != 0; }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_rx_ready() const { return (get_status() & Status::RX_READY) != 0; }
        bool is_rx_idle () const { return (get_status() & Status::RX_IDLE ) != 0; }
        bool is_tx_ready() const { return (get_status() & Status::TX_READY) != 0; }
        bool is_tx_idle () const { return (get_status() & Status::TX_IDLE ) != 0; }

        StatusBitmask get_status() const
        {
            return static_cast<Status>(m_usart->STAT);
        }

        void clear_status(const StatusBitmask bitmask)
        {
            m_usart->STAT = bitmask.bits();
        }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_interrupts(const InterruptBitmask bitmask)
        {
            m_usart->INTENSET = bitmask.bits();
        }

        void disable_interrupts(const InterruptBitmask bitmask)
        {
            m_usart->INTENCLR = bitmask.bits();
        }

        InterruptBitmask get_interrupts_enabled() const
        {
            return static_cast<Interrupt>(m_usart->INTSTAT);
        }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: NVIC_EnableIRQ(USART0_IRQn);         break;
                case Name::USART1: NVIC_EnableIRQ(USART1_IRQn);         break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::USART2: NVIC_EnableIRQ(USART2_IRQn);         break;
                case Name::USART3: NVIC_EnableIRQ(PININT6_USART3_IRQn); break; // PININT6 / USART3 shared interrupt
                case Name::USART4: NVIC_EnableIRQ(PININT7_USART4_IRQn); break; // PININT7 / USART4 shared interrupt
#endif
                default:                                                break;
            }
        }

        void disable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: NVIC_DisableIRQ(USART0_IRQn);          break;
                case Name::USART1: NVIC_DisableIRQ(USART1_IRQn);          break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::USART2: NVIC_DisableIRQ(USART2_IRQn);          break;
                case Name::USART3: /* DO NOT DISABLE SHARED INTERRUPTS */ break; // PININT6 / USART3 shared interrupt
                case Name::USART4: /* DO NOT DISABLE SHARED INTERRUPTS */ break; // PININT7 / USART4 shared interrupt
#endif
                default:                                                  break;
            }
        }

        bool is_irq_enabled()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: return (NVIC_GetEnableIRQ(USART0_IRQn) != 0);         break;
                case Name::USART1: return (NVIC_GetEnableIRQ(USART1_IRQn) != 0);         break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::USART2: return (NVIC_GetEnableIRQ(USART2_IRQn) != 0);         break;
                case Name::USART3: return (NVIC_GetEnableIRQ(PININT6_USART3_IRQn) != 0); break; // PININT6 / USART3 shared interrupt
                case Name::USART4: return (NVIC_GetEnableIRQ(PININT7_USART4_IRQn) != 0); break; // PININT7 / USART4 shared interrupt
#endif
                default:           return false;                                         break;
            }
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: NVIC_SetPriority(USART0_IRQn,         irq_priority); break;
                case Name::USART1: NVIC_SetPriority(USART1_IRQn,         irq_priority); break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::USART2: NVIC_SetPriority(USART2_IRQn,         irq_priority); break;
                case Name::USART3: NVIC_SetPriority(PININT6_USART3_IRQn, irq_priority); break; // PININT6 / USART3 shared interrupt
                case Name::USART4: NVIC_SetPriority(PININT7_USART4_IRQn, irq_priority); break; // PININT7 / USART4 shared interrupt
#endif
                default:                                                                break;
            }
        }

        void assign_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_irq_handler = irq_handler;
        }

        void remove_irq_handler()
        {
            m_irq_handler = nullptr;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // USART Configuration Register (CFG) bits and masks
        enum CFG : uint32_t
        {
            CFG_ENABLE          = (1 << 0),     // USART enable
            CFG_DATALEN_BITMASK = (3 << 2),     // USART data mode bitmask
            CFG_STOPLEN_BITMASK = (1 << 6),     // USART stop bits bitmask
            CFG_PARITY_BITMASK  = (3 << 4)      // USART parity bitmask
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- FRG0 CONFIGURATION ----------------------------------------

        // Configure the FRG0 to be used and shared by all USART peripherals.
        // NOTE: implemented on the CPP file because it uses parameters from
        //       the library configuration file (xarmlib_config.h).
        void initialize_frg0();

        // Return the FRG MUL value for the supplied USART and main clock frequencies
        static constexpr uint8_t get_frg_mul(const int32_t usart_freq, const int32_t main_clk_freq)
        {
            const float mul = std::round((static_cast<float>(main_clk_freq) * 256 / static_cast<float>(usart_freq)) - 256);

            return static_cast<uint8_t>(mul);
        }

        // Return the maximum USART frequency that can be used to generate a
        // standard baudrate frequency for the supplied main clock frequency.
        static constexpr int32_t get_max_standard_frequency(const int32_t main_clk_freq)
        {
            // 58.982400MHz is the maximum attainable frequency from a 60MHz clock source
            int32_t max_standard_freq = 58982400;

            while(max_standard_freq > main_clk_freq)
            {
                max_standard_freq /= 2;
            }

            return max_standard_freq;
        }

        // Return the USART frequency divider (baudrate generator divider) to obtain the supplied baudrate frequency
        static int32_t get_baudrate_generator_div(const int32_t baudrate);

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_irq_handler != nullptr)
            {
                yield = m_irq_handler();
            }

            return yield;
        }

        // IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return Usart::get_reference(index).irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        LPC_USART_T* m_usart { nullptr };   // Pointer to the CMSIS USART structure
        IrqHandler   m_irq_handler;         // User defined IRQ handler
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_USART_HPP

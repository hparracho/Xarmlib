// ----------------------------------------------------------------------------
// @file    lpc84x_usart.hpp
// @brief   NXP LPC84x USART class (takes control of FRG0).
// @notes   Synchronous mode not implemented.
// @date    4 May 2018
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

#include <cmath>

#include "system/delegate"
#include "targets/peripheral_ref_counter.hpp"
#include "targets/LPC84x/lpc84x_cmsis.h"
#include "targets/LPC84x/lpc84x_pins.hpp"
#include "targets/LPC84x/lpc84x_swm.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace lpc84x
{




// Forward declaration of IRQ handlers for both LPC844 and LPC845
extern "C" void USART0_IRQHandler(void);
extern "C" void USART1_IRQHandler(void);

#if defined __LPC844__

// Number of available USART peripherals on LPC844
static constexpr std::size_t USART_COUNT { 2 };

#elif defined __LPC845__

// Forward declaration of additional IRQ handlers for LPC845
extern "C" void USART2_IRQHandler(void);
extern "C" void PININT6_IRQHandler(void);   // PIO INT6 shared slot with USART3
extern "C" void PININT7_IRQHandler(void);   // PIO INT7 shared slot with USART4

// Number of available USART peripherals on LPC845
static constexpr std::size_t USART_COUNT { 5 };

#endif // __LPC845__




class Usart : private PeripheralRefCounter<Usart, USART_COUNT>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralUsart = PeripheralRefCounter<Usart, USART_COUNT>;

        // USART peripheral names selection
        enum class Name
        {
            USART0 = 0,
            USART1,
#ifdef __LPC845__
            USART2,
            USART3,
            USART4
#endif
        };

        // Data length selection (defined to map the ??? register directly)
        enum class DataBits
        {
            BITS_7 = (0 << 2),  // USART 7 bit data mode
            BITS_8 = (1 << 2),  // USART 8 bit data mode
            BITS_9 = (2 << 2)   // USART 9 bit data mode
        };

        // Stop bits selection (defined to map the ??? register directly)
        enum class StopBits
        {
            BITS_1 = (0 << 6),  // USART 1 stop bit
            BITS_2 = (1 << 6)   // USART 2 stop bits
        };

        // Parity selection (defined to map the ??? register directly)
        enum class Parity
        {
            NONE = (0 << 4),    // USART no parity
            EVEN = (2 << 4),    // USART even parity select
            ODD  = (3 << 4)     // USART odd parity select
        };

        // IRQ handler definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

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

                // Make sure the USART is not currently sending or receiving data...
                while(is_rx_idle() == false || is_tx_idle() == false)
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

                // Make sure the USART is not currently sending or receiving data...
                while(is_rx_idle() == false || is_tx_idle() == false)
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

                // Make sure the USART is not currently sending or receiving data...
                while(is_rx_idle() == false || is_tx_idle() == false)
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

                // Make sure the USART is not currently sending or receiving data...
                while(is_rx_idle() == false || is_tx_idle() == false)
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

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable peripheral
        void enable() { m_usart->CFG |= CFG_ENABLE; }

        // Disable peripheral
        void disable() { m_usart->CFG &= ~CFG_ENABLE; }

        // Gets the enable state
        bool is_enabled() const { return (m_usart->CFG & CFG_ENABLE) != 0; }

        // -------- GET STATUS FLAGS ------------------------------------------

        bool is_rx_ready        () const { return (m_usart->STAT & STAT_RXRDY       ) != 0; }
        bool is_rx_idle         () const { return (m_usart->STAT & STAT_RXIDLE      ) != 0; }
        bool is_tx_ready        () const { return (m_usart->STAT & STAT_TXRDY       ) != 0; }
        bool is_tx_idle         () const { return (m_usart->STAT & STAT_TXIDLE      ) != 0; }
        bool is_cts             () const { return (m_usart->STAT & STAT_CTS         ) != 0; }
        bool is_cts_delta       () const { return (m_usart->STAT & STAT_DELTACTS    ) != 0; }
        bool is_tx_disabled_int () const { return (m_usart->STAT & STAT_TXDISINT    ) != 0; }
        bool is_rx_overrun_int  () const { return (m_usart->STAT & STAT_OVERRUNINT  ) != 0; }
        bool is_rx_break        () const { return (m_usart->STAT & STAT_RXBRK       ) != 0; }
        bool is_rx_break_delta  () const { return (m_usart->STAT & STAT_DELTARXBRK  ) != 0; }
        bool is_start           () const { return (m_usart->STAT & STAT_START       ) != 0; }
        bool is_frame_error_int () const { return (m_usart->STAT & STAT_FRAMERRINT  ) != 0; }
        bool is_parity_error_int() const { return (m_usart->STAT & STAT_PARITYERRINT) != 0; }
        bool is_rx_noise_int    () const { return (m_usart->STAT & STAT_RXNOISEINT  ) != 0; }
        bool is_autobaud_error  () const { return (m_usart->STAT & STAT_ABERR       ) != 0; }

        // -------- CLEAR STATUS FLAGS ----------------------------------------

        void clear_cts_delta       () { m_usart->STAT |= STAT_DELTACTS;     }
        void clear_rx_overrun_int  () { m_usart->STAT |= STAT_OVERRUNINT;   }
        void clear_rx_break_delta  () { m_usart->STAT |= STAT_DELTARXBRK;   }
        void clear_start           () { m_usart->STAT |= STAT_START;        }
        void clear_frame_error_int () { m_usart->STAT |= STAT_FRAMERRINT;   }
        void clear_parity_error_int() { m_usart->STAT |= STAT_PARITYERRINT; }
        void clear_rx_noise_int    () { m_usart->STAT |= STAT_RXNOISEINT;   }
        void clear_autobaud_error  () { m_usart->STAT |= STAT_ABERR;        }

        // -------- ENABLE INTERRUPTS -----------------------------------------

        void enable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: NVIC_EnableIRQ(USART0_IRQn); break;
                case Name::USART1: NVIC_EnableIRQ(USART1_IRQn); break;
#ifdef __LPC845__
                case Name::USART2: NVIC_EnableIRQ(USART2_IRQn); break;
                case Name::USART3: NVIC_EnableIRQ(USART3_IRQn); break;  // PIO INT6 shared slot with USART3
                case Name::USART4: NVIC_EnableIRQ(USART4_IRQn); break;  // PIO INT7 shared slot with USART4
#endif
                default:                                        break;
            }
        }

        void enable_irq_rx_ready() { m_usart->INTENSET |= INTEN_RXRDY; }
        void enable_irq_tx_ready() { m_usart->INTENSET |= INTEN_TXRDY; }

        // -------- DISABLE INTERRUPTS ----------------------------------------

        void disable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: NVIC_DisableIRQ(USART0_IRQn);          break;
                case Name::USART1: NVIC_DisableIRQ(USART1_IRQn);          break;
#ifdef __LPC845__
                case Name::USART2: NVIC_DisableIRQ(USART2_IRQn);          break;
                case Name::USART3: /* DO NOT DISABLE SHARED INTERRUPTS */ break;    // PIO INT6 shared slot with USART3
                case Name::USART4: /* DO NOT DISABLE SHARED INTERRUPTS */ break;    // PIO INT7 shared slot with USART4
#endif
                default:                                                  break;
            }
        }

        void disable_irq_rx_ready() { m_usart->INTENCLR |= INTEN_RXRDY; }
        void disable_irq_tx_ready() { m_usart->INTENCLR |= INTEN_TXRDY; }

        // -------- GET ENABLED INTERRUPTS ------------------------------------

        bool is_enabled_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: return (NVIC_GetEnableIRQ(USART0_IRQn) != 0); break;
                case Name::USART1: return (NVIC_GetEnableIRQ(USART1_IRQn) != 0); break;
#ifdef __LPC845__
                case Name::USART2: return (NVIC_GetEnableIRQ(USART2_IRQn) != 0); break;
                case Name::USART3: return (NVIC_GetEnableIRQ(USART3_IRQn) != 0); break;  // PIO INT6 shared slot with USART3
                case Name::USART4: return (NVIC_GetEnableIRQ(USART4_IRQn) != 0); break;  // PIO INT7 shared slot with USART4
#endif
                default:           return false;                                 break;
            }
        }

        bool is_enabled_irq_rx_ready     () const { return (m_usart->INTENSET & INTEN_RXRDY) != 0; }
        bool is_enabled_irq_tx_ready     () const { return (m_usart->INTENSET & INTEN_TXRDY) != 0; }

        // -------- IRQ HANDLER ASSIGNMENT ------------------------------------

        void set_irq_priority(const int32_t irq_priority)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::USART0: NVIC_SetPriority(USART0_IRQn, irq_priority); break;
                case Name::USART1: NVIC_SetPriority(USART1_IRQn, irq_priority); break;
#ifdef __LPC845__
                case Name::USART2: NVIC_SetPriority(USART2_IRQn, irq_priority); break;
                case Name::USART3: NVIC_SetPriority(USART3_IRQn, irq_priority); break;  // PIO INT6 shared slot with USART3
                case Name::USART4: NVIC_SetPriority(USART4_IRQn, irq_priority); break;  // PIO INT7 shared slot with USART4
#endif
                default:                                                        break;
            }
        }

        void assign_irq_handler_rx_ready(const IrqHandler& irq_handler_rx_ready)
        {
            assert(irq_handler_rx_ready != nullptr);

            m_irq_handler_rx_ready = irq_handler_rx_ready;
        }

        void assign_irq_handler_tx_ready(const IrqHandler& irq_handler_tx_ready)
        {
            assert(irq_handler_tx_ready != nullptr);

            m_irq_handler_tx_ready = irq_handler_tx_ready;
        }

        void remove_irq_handler_rx_ready()
        {
            m_irq_handler_rx_ready = nullptr;
        }

        void remove_irq_handler_tx_ready()
        {
            m_irq_handler_tx_ready = nullptr;
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

    //protected:
    public:

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        Usart() : PeripheralUsart(*this)
        {
            // Initialize and configure FRG0 if this is the first USART peripheral instantiation
            if(get_used() == 1)
            {
                initialize_frg0();
            }
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
#ifdef __LPC845__
                case Name::USART2: Clock::disable(Clock::Peripheral::USART2);
                                   NVIC_DisableIRQ(USART2_IRQn);
                                   break;
                case Name::USART3: Clock::disable(Clock::Peripheral::USART3);
                                   /* DO NOT DISABLE SHARED INTERRUPTS */           // PIO INT6 shared slot with USART3
                                   break;
                case Name::USART4: Clock::disable(Clock::Peripheral::USART4);
                                   /* DO NOT DISABLE SHARED INTERRUPTS */           // PIO INT7 shared slot with USART4
                                   break;
#endif
            }
        }

        // -------- INITIALIZATION / CONFIGURATION ----------------------------

        // Initialize USART peripheral structure and pins for asynchronous mode (constructor helper function)
        void initialize(const Pin::Name txd,
                        const Pin::Name rxd,
                        const int32_t   baudrate,
                        const DataBits  data_bits,
                        const StopBits  stop_bits,
                        const Parity    parity)
        {
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
#ifdef __LPC845__
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

            disable_irq();

            // No continuous break, no address detect, no Tx disable, no CC, no CLRCC
            m_usart->CTL = 0;

            // Clear all status bits
            m_usart->STAT = STAT_DELTACTS
                          | STAT_OVERRUNINT
                          | STAT_DELTARXBRK
                          | STAT_START
                          | STAT_FRAMERRINT
                          | STAT_PARITYERRINT
                          | STAT_RXNOISEINT
                          | STAT_ABERR;

            set_format(data_bits, stop_bits, parity);
            set_baudrate(baudrate);

            enable();
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C function to give access to private IRQ handler member function
        friend void USART0_IRQHandler(void);
        friend void USART1_IRQHandler(void);
#ifdef __LPC845__
// Forward declaration of IRQ handlers
        friend void USART2_IRQHandler(void);
        friend void PININT6_IRQHandler(void);   // PIO INT6 shared slot with USART3
        friend void PININT7_IRQHandler(void);   // PIO INT7 shared slot with USART4
#endif

        // USART Configuration Register (CFG) bits and masks
        enum CFG : uint32_t
        {
            CFG_ENABLE          = (1 << 0),     // USART enable
            CFG_DATALEN_BITMASK = (3 << 2),     // USART data mode bitmask
            CFG_STOPLEN_BITMASK = (1 << 6),     // USART stop bits bitmask
            CFG_PARITY_BITMASK  = (3 << 4)      // USART parity bitmask
        };

        // USART Status register (STAT) bits
        enum STAT : uint32_t
        {
            STAT_RXRDY          = (1 << 0),     // Receiver ready
            STAT_RXIDLE         = (1 << 1),     // Receiver idle
            STAT_TXRDY          = (1 << 2),     // Transmitter ready for data
            STAT_TXIDLE         = (1 << 3),     // Transmitter idle
            STAT_CTS            = (1 << 4),     // Status of CTS signal
            STAT_DELTACTS       = (1 << 5),     // Change in CTS state
            STAT_TXDISINT       = (1 << 6),     // Transmitter disabled
            STAT_OVERRUNINT     = (1 << 8),     // Overrun Error interrupt flag.
            STAT_RXBRK          = (1 << 10),    // Received break
            STAT_DELTARXBRK     = (1 << 11),    // Change in receive break detection
            STAT_START          = (1 << 12),    // Start detected
            STAT_FRAMERRINT     = (1 << 13),    // Framing Error interrupt flag
            STAT_PARITYERRINT   = (1 << 14),    // Parity Error interrupt flag
            STAT_RXNOISEINT     = (1 << 15),    // Received Noise interrupt flag
            STAT_ABERR          = (1 << 16)     // Auto-baud Error
        };

        // USART Interrupt Enable read and Set or Clear Register (INTENSET/INTENCLR) bits
        enum INTEN : uint32_t
        {
            INTEN_RXRDY         = (1 << 0),     // Receive Ready interrupt
            INTEN_TXRDY         = (1 << 2)      // Transmit Ready interrupt
        };

        // USART Interrupt Status Register (INTSTAT) bits
        enum INTSTAT : uint32_t
        {
            INTSTAT_RXRDY       = (1 << 0),     // Receive Ready interrupt
            INTSTAT_TXRDY       = (1 << 2)      // Transmit Ready interrupt
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

        // IRQ handler private implementation (manage interrupt flags and call user IRQ handlers)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if((m_usart->INTSTAT & INTSTAT_RXRDY) != 0 && m_irq_handler_rx_ready != nullptr)
            {
                yield |= m_irq_handler_rx_ready();
            }

            if((m_usart->INTSTAT & INTSTAT_TXRDY) != 0 && m_irq_handler_tx_ready != nullptr)
            {
                yield |= m_irq_handler_tx_ready();
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

        LPC_USART_T* m_usart { nullptr };

        IrqHandler   m_irq_handler_rx_ready;    // User defined RX ready IRQ handler
        IrqHandler   m_irq_handler_tx_ready;    // User defined TX ready IRQ handler
};




} // namespace lpc84x
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_USART_HPP

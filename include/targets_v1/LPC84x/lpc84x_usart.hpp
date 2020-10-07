// ----------------------------------------------------------------------------
// @file    lpc84x_usart.hpp
// @brief   NXP LPC84x USART class (takes control of FRG0).
// @notes   Synchronous mode not implemented.
// @date    20 September 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#include "xarmlib_config.hpp"
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
    rx_ready           = (1 << 0),  // Receiver ready
    rx_idle            = (1 << 1),  // Receiver idle
    tx_ready           = (1 << 2),  // Transmitter ready for data
    tx_idle            = (1 << 3),  // Transmitter idle
    cts                = (1 << 4),  // Status of CTS signal
    cts_delta          = (1 << 5),  // Change in CTS state
    tx_disabled        = (1 << 6),  // Transmitter disabled
    rx_overrun         = (1 << 8),  // Overrun Error interrupt flag
    rx_break           = (1 << 10), // Received break
    rx_break_delta     = (1 << 11), // Change in receive break detection
    start              = (1 << 12), // Start detected
    frame_error        = (1 << 13), // Framing Error interrupt flag
    parity_error       = (1 << 14), // Parity Error interrupt flag
    rx_noise           = (1 << 15), // Received Noise interrupt flag
    autobaud_error     = (1 << 16), // Auto-baud Error
    clear_all_bitmask  = 0x1F920,   // Clear all bitmask (1'1111'1001'0010'0000)
    bitmask            = 0x1FD7F    // Full bitmask (1'1111'1101'0111'1111)
};

// USART Interrupt Enable Get, Set or Clear Register (INTSTAT / INTENSET / INTENCLR) bits
enum class Interrupt
{
    rx_ready       = (1 << 0),  // Receiver ready
    tx_ready       = (1 << 2),  // Transmitter ready for data
    tx_idle        = (1 << 3),  // Transmitter idle
    cts_delta      = (1 << 5),  // Change in CTS state
    tx_disabledt   = (1 << 6),  // Transmitter disabled
    rx_overrun     = (1 << 8),  // Overrun Error interrupt flag
    rx_break_delta = (1 << 11), // Change in receive break detection
    start          = (1 << 12), // Start detected
    frame_error    = (1 << 13), // Framing Error interrupt flag
    parity_error   = (1 << 14), // Parity Error interrupt flag
    rx_noise       = (1 << 15), // Received Noise interrupt flag
    autobaud_error = (1 << 16), // Auto-baud Error
    bitmask        = 0x1F96D    // Full bitmask (1'1111'1001'0110'1101)
};

BITMASK_DEFINE_VALUE_MASK(Status,    static_cast<uint32_t>(Status::bitmask))
BITMASK_DEFINE_VALUE_MASK(Interrupt, static_cast<uint32_t>(Interrupt::bitmask))

} // namespace private_usart




class UsartDriver : private PeripheralRefCounter<UsartDriver, TARGET_USART_COUNT>
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
        using PeripheralUsart = PeripheralRefCounter<UsartDriver, TARGET_USART_COUNT>;

        // Data length selection (defined to map the CFG register directly)
        enum class DataBits
        {
            bits_7 = (0 << 2),  // USART 7 bit data mode
            bits_8 = (1 << 2),  // USART 8 bit data mode
            bits_9 = (2 << 2)   // USART 9 bit data mode
        };

        // Stop bits selection (defined to map the CFG register directly)
        enum class StopBits
        {
            bits_1 = (0 << 6),  // USART 1 stop bit
            bits_2 = (1 << 6)   // USART 2 stop bits
        };

        // Parity selection (defined to map the CFG register directly)
        enum class Parity
        {
            none = (0 << 4),    // USART no parity
            even = (2 << 4),    // USART even parity select
            odd  = (3 << 4)     // USART odd parity select
        };

        struct Config
        {
            int32_t  baudrate  = 9600;
            DataBits data_bits = DataBits::bits_8;
            StopBits stop_bits = StopBits::bits_1;
            Parity   parity    = Parity::none;
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

        UsartDriver(const PinDriver::Name txd, const PinDriver::Name rxd, const Config& config) : PeripheralUsart(*this)
        {
            // Initialize and configure FRG0 if this is the first USART peripheral instantiation
            if(get_used() == 1)
            {
                initialize_frg0();
            }

            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::usart0: m_usart = LPC_USART0;
                                   ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::usart0,
                                                                            ClockDriver::PeripheralClockSource::frg0_clk);
                                   ClockDriver::enable(ClockDriver::Peripheral::usart0);
                                   PowerDriver::reset(PowerDriver::ResetPeripheral::usart0);
                                   SwmDriver::assign(SwmDriver::PinMovable::u0_rxd_i, rxd);
                                   SwmDriver::assign(SwmDriver::PinMovable::u0_txd_o, txd);
                                   break;

                case Name::usart1: m_usart = LPC_USART1;
                                   ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::usart1,
                                                                            ClockDriver::PeripheralClockSource::frg0_clk);
                                   ClockDriver::enable(ClockDriver::Peripheral::usart1);
                                   PowerDriver::reset(PowerDriver::ResetPeripheral::usart1);
                                   SwmDriver::assign(SwmDriver::PinMovable::u1_rxd_i, rxd);
                                   SwmDriver::assign(SwmDriver::PinMovable::u1_txd_o, txd);
                                   break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::usart2: m_usart = LPC_USART2;
                                   ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::usart2,
                                                                            ClockDriver::PeripheralClockSource::frg0_clk);
                                   ClockDriver::enable(ClockDriver::Peripheral::usart2);
                                   PowerDriver::reset(PowerDriver::ResetPeripheral::usart2);
                                   SwmDriver::assign(SwmDriver::PinMovable::u2_rxd_i, rxd);
                                   SwmDriver::assign(SwmDriver::PinMovable::u2_txd_o, txd);
                                   break;

                case Name::usart3: m_usart = LPC_USART3;
                                   ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::usart3,
                                                                            ClockDriver::PeripheralClockSource::frg0_clk);
                                   ClockDriver::enable(ClockDriver::Peripheral::usart3);
                                   PowerDriver::reset(PowerDriver::ResetPeripheral::usart3);
                                   SwmDriver::assign(SwmDriver::PinMovable::u3_rxd_i, rxd);
                                   SwmDriver::assign(SwmDriver::PinMovable::u3_txd_o, txd);
                                   break;

                case Name::usart4: m_usart = LPC_USART4;
                                   ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::usart4,
                                                                            ClockDriver::PeripheralClockSource::frg0_clk);
                                   ClockDriver::enable(ClockDriver::Peripheral::usart4);
                                   PowerDriver::reset(PowerDriver::ResetPeripheral::usart4);
                                   SwmDriver::assign(SwmDriver::PinMovable::u4_rxd_i, rxd);
                                   SwmDriver::assign(SwmDriver::PinMovable::u4_txd_o, txd);
                                   break;
#endif
            };

            PinDriver::set_mode(txd, PinDriver::FunctionMode::hiz);
            PinDriver::set_mode(rxd, PinDriver::FunctionMode::pull_up);

            disable_irq();

            // No continuous break, no address detect, no Tx disable, no CC, no CLRCC
            m_usart->CTL = 0;

            // Clear all status bits
            clear_status(Status::clear_all_bitmask);

            set_format(config.data_bits, config.stop_bits, config.parity);
            set_baudrate(config.baudrate);
        }

        ~UsartDriver()
        {
            // Disable peripheral
            disable();

            const Name name = static_cast<Name>(get_index());

            // Disable peripheral clock sources and interrupts
            switch(name)
            {
                case Name::usart0: ClockDriver::disable(ClockDriver::Peripheral::usart0);
                                   NVIC_DisableIRQ(USART0_IRQn);
                                   break;
                case Name::usart1: ClockDriver::disable(ClockDriver::Peripheral::usart1);
                                   NVIC_DisableIRQ(USART1_IRQn);
                                   break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::usart2: ClockDriver::disable(ClockDriver::Peripheral::usart2);
                                   NVIC_DisableIRQ(USART2_IRQn);
                                   break;
                case Name::usart3: ClockDriver::disable(ClockDriver::Peripheral::usart3);
                                   /* DO NOT DISABLE SHARED INTERRUPTS */     // PININT6 / USART3 shared interrupt
                                   break;
                case Name::usart4: ClockDriver::disable(ClockDriver::Peripheral::usart4);
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
            m_usart->CFG = (m_usart->CFG & ~cfg_datalen_bitmask) | static_cast<uint32_t>(data_bits);

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
            m_usart->CFG = (m_usart->CFG & ~cfg_stoplen_bitmask) | static_cast<uint32_t>(stop_bits);

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
            m_usart->CFG = (m_usart->CFG & ~cfg_parity_bitmask) | static_cast<uint32_t>(parity);

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
        void enable() { m_usart->CFG |= cfg_enable; }

        // Disable peripheral
        void disable() { m_usart->CFG &= ~cfg_enable; }

        // Gets the enable state
        bool is_enabled() const { return (m_usart->CFG & cfg_enable) != 0; }

        // -------- STATUS FLAGS ----------------------------------------------

        bool is_rx_ready() const { return (get_status() & Status::rx_ready) != 0; }
        bool is_rx_idle () const { return (get_status() & Status::rx_idle ) != 0; }
        bool is_tx_ready() const { return (get_status() & Status::tx_ready) != 0; }
        bool is_tx_idle () const { return (get_status() & Status::tx_idle ) != 0; }

        StatusBitmask get_status() const
        {
            return static_cast<Status>(m_usart->STAT);
        }

        void clear_status(const StatusBitmask bitmask)
        {
            m_usart->STAT = (bitmask & Status::clear_all_bitmask).bits();
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
                case Name::usart0: NVIC_EnableIRQ(USART0_IRQn);         break;
                case Name::usart1: NVIC_EnableIRQ(USART1_IRQn);         break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::usart2: NVIC_EnableIRQ(USART2_IRQn);         break;
                case Name::usart3: NVIC_EnableIRQ(PININT6_USART3_IRQn); break; // PININT6 / USART3 shared interrupt
                case Name::usart4: NVIC_EnableIRQ(PININT7_USART4_IRQn); break; // PININT7 / USART4 shared interrupt
#endif
                default:                                                break;
            }
        }

        void disable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::usart0: NVIC_DisableIRQ(USART0_IRQn);          break;
                case Name::usart1: NVIC_DisableIRQ(USART1_IRQn);          break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::usart2: NVIC_DisableIRQ(USART2_IRQn);          break;
                case Name::usart3: /* DO NOT DISABLE SHARED INTERRUPTS */ break; // PININT6 / USART3 shared interrupt
                case Name::usart4: /* DO NOT DISABLE SHARED INTERRUPTS */ break; // PININT7 / USART4 shared interrupt
#endif
                default:                                                  break;
            }
        }

        bool is_irq_enabled()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::usart0: return (NVIC_GetEnableIRQ(USART0_IRQn) != 0);         break;
                case Name::usart1: return (NVIC_GetEnableIRQ(USART1_IRQn) != 0);         break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::usart2: return (NVIC_GetEnableIRQ(USART2_IRQn) != 0);         break;
                case Name::usart3: return (NVIC_GetEnableIRQ(PININT6_USART3_IRQn) != 0); break; // PININT6 / USART3 shared interrupt
                case Name::usart4: return (NVIC_GetEnableIRQ(PININT7_USART4_IRQn) != 0); break; // PININT7 / USART4 shared interrupt
#endif
                default:           return false;                                         break;
            }
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::usart0: NVIC_SetPriority(USART0_IRQn,         irq_priority); break;
                case Name::usart1: NVIC_SetPriority(USART1_IRQn,         irq_priority); break;
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
                case Name::usart2: NVIC_SetPriority(USART2_IRQn,         irq_priority); break;
                case Name::usart3: NVIC_SetPriority(PININT6_USART3_IRQn, irq_priority); break; // PININT6 / USART3 shared interrupt
                case Name::usart4: NVIC_SetPriority(PININT7_USART4_IRQn, irq_priority); break; // PININT7 / USART4 shared interrupt
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

        // USART peripheral names selection
        enum class Name
        {
            usart0 = 0,
            usart1,
#if (TARGET_USART_COUNT == 5) /* __LPC845__ */
            usart2,
            usart3,
            usart4
#endif
        };

        // USART Configuration Register (CFG) bits and masks
        enum CFG : uint32_t
        {
            cfg_enable          = (1 << 0),     // USART enable
            cfg_datalen_bitmask = (3 << 2),     // USART data mode bitmask
            cfg_stoplen_bitmask = (1 << 6),     // USART stop bits bitmask
            cfg_parity_bitmask  = (3 << 4)      // USART parity bitmask
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- FRG0 CONFIGURATION ----------------------------------------

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
        static int32_t get_baudrate_generator_div(const int32_t baudrate)
        {
            constexpr int32_t main_clk_freq = SystemDriver::get_main_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK);
            constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

            return usart_freq / 16 / baudrate;
        }

        // Configure the FRG0 to be used and shared by all USART peripherals
        void initialize_frg0()
        {
            constexpr int32_t main_clk_freq = SystemDriver::get_main_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK);
            constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

            constexpr uint8_t mul = get_frg_mul(usart_freq, main_clk_freq);
            constexpr uint8_t div = 0xFF; // Fixed value to use with the fractional baudrate generator

            // Select main clock as the source for FRG0
            ClockDriver::set_frg_clock_source(ClockDriver::FrgClockSelect::frg0, ClockDriver::FrgClockSource::main_clk);

            // Set the FRG0 fractional divider
            ClockDriver::set_frg_clock_divider(ClockDriver::FrgClockSelect::frg0, mul, div);
        }

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

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

            return UsartDriver::get_reference(index).irq_handler();
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

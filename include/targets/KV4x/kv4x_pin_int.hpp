// ----------------------------------------------------------------------------
// @file    kv4x_pin_int.hpp
// @brief   Kinetis KV4x pin interrupt class.
// @date    18 May 2020
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

#ifndef __XARMLIB_TARGETS_KV4X_PIN_INT_HPP
#define __XARMLIB_TARGETS_KV4X_PIN_INT_HPP

#include "xarmlib_config.hpp"
#include "targets/KV4x/kv4x_gpio.hpp"
#include "targets/KV4x/kv4x_port.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of PORT IRQ handlers
extern "C" void PORTA_IRQHandler(void);
extern "C" void PORTB_IRQHandler(void);
extern "C" void PORTC_IRQHandler(void);
extern "C" void PORTD_IRQHandler(void);
extern "C" void PORTE_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class PinIntDriver : private PeripheralRefCounter<PinIntDriver, XARMLIB_CONFIG_PIN_INTERRUPT_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend PORT IRQ handler C functions to give access to private IRQ handler member function
        friend void ::PORTA_IRQHandler(void);
        friend void ::PORTB_IRQHandler(void);
        friend void ::PORTC_IRQHandler(void);
        friend void ::PORTD_IRQHandler(void);
        friend void ::PORTE_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralPinInt = PeripheralRefCounter<PinIntDriver, XARMLIB_CONFIG_PIN_INTERRUPT_COUNT>;

        using InputMode     = GpioDriver::InputMode;
        using PassiveFilter = GpioDriver::PassiveFilter;
        using LockRegister  = GpioDriver::LockRegister;

        using InputModeConfig              = GpioDriver::InputModeConfig;
        using InputModeTrueOpenDrainConfig = GpioDriver::InputModeTrueOpenDrainConfig;

        // Pin interrupt mode
        enum class IntMode
        {
            //disabled               = 0x0U,  // Interrupt/DMA request is disabled
            dma_rising_edge        = 0x1U,  // DMA request on rising edge
            dma_falling_edge       = 0x2U,  // DMA request on falling edge
            dma_either_edge        = 0x3U,  // DMA request on either edge
            interrupt_logic_zero   = 0x8U,  // Interrupt when logic zero
            interrupt_rising_edge  = 0x9U,  // Interrupt on rising edge
            interrupt_falling_edge = 0xAU,  // Interrupt on falling edge
            interrupt_either_edge  = 0xBU,  // Interrupt on either edge
            interrupt_logic_one    = 0xCU   // Interrupt when logic one
        };

        // IRQ handlers definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        // Normal input pin constructor
        PinIntDriver(const PinDriver::Name  pin_name,
                     const InputModeConfig& config,
                     const IntMode          int_mode) : PeripheralPinInt(*this),
                                                        m_int_mode { int_mode },
                                                        m_port_index { PinDriver::get_port_index(pin_name) },
                                                        m_port_base { get_port_base(m_port_index) },
                                                        m_gpio(pin_name, config)
        {
            assert(pin_name != PinDriver::Name::nc);
            assert(pin_name != PinDriver::Name::pc_6 && pin_name != PinDriver::Name::pc_7);

            disable_interrupt();
            clear_interrupt_pending();
        }

        // True open-drain input pin constructor (only available on PC_6 and PC_7)
        PinIntDriver(const PinDriver::Name               pin_name,
                     const InputModeTrueOpenDrainConfig& config,
                     const IntMode                       int_mode) : PeripheralPinInt(*this),
                                                                     m_int_mode { int_mode },
                                                                     m_port_index { PinDriver::get_port_index(pin_name) },
                                                                     m_port_base { get_port_base(m_port_index) },
                                                                     m_gpio(pin_name, config)
        {
            assert(pin_name == PinDriver::Name::pc_6 || pin_name == PinDriver::Name::pc_7);

            disable_interrupt();
            clear_interrupt_pending();
        }

        ~PinIntDriver()
        {
            disable_interrupt();
            clear_interrupt_pending();

            // Disable all Port IRQs if this is the last pin interrupt deleted
            if(get_used() == 1)
            {
                disable_port_irq(PortDriver::Name::porta);
                disable_port_irq(PortDriver::Name::portb);
                disable_port_irq(PortDriver::Name::portc);
                disable_port_irq(PortDriver::Name::portd);
                disable_port_irq(PortDriver::Name::porte);
            }
        }

        // -------- CONFIGURATION ---------------------------------------------

        void set_mode(const InputModeConfig& config)              { m_gpio.set_mode(config); }
        void set_mode(const InputModeTrueOpenDrainConfig& config) { m_gpio.set_mode(config); }

        PinDriver::Name get_pin_name() const { return m_gpio.get_pin_name(); }

        // -------- READ ------------------------------------------------------

        uint32_t read() const { return m_gpio.read(); }

        // -------- PIN INTERRUPT ---------------------------------------------

        void enable_interrupt()
        {
            PORT_SetPinInterruptConfig(m_port_base, m_gpio.m_pin, static_cast<port_interrupt_t>(m_int_mode));
        }

        void disable_interrupt()
        {
            PORT_SetPinInterruptConfig(m_port_base, m_gpio.m_pin, kPORT_InterruptOrDMADisabled);
        }

        bool is_interrupt_enabled() const
        {
            return ((m_port_base->PCR[m_gpio.m_pin] & PORT_PCR_IRQC_MASK) != 0);
        }

        bool is_interrupt_pending() const
        {
            return ((m_port_base->PCR[m_gpio.m_pin] & PORT_PCR_ISF_MASK) != 0);
        }

        void clear_interrupt_pending()
        {
            m_port_base->PCR[m_gpio.m_pin] |= PORT_PCR_ISF_MASK;
        }

        // -------- PORT IRQ --------------------------------------------------

        static void enable_port_irq(const PortDriver::Name port_name)
        {
            switch(port_name)
            {
                case PortDriver::Name::porta: NVIC_EnableIRQ(PORTA_IRQn); break;
                case PortDriver::Name::portb: NVIC_EnableIRQ(PORTB_IRQn); break;
                case PortDriver::Name::portc: NVIC_EnableIRQ(PORTC_IRQn); break;
                case PortDriver::Name::portd: NVIC_EnableIRQ(PORTD_IRQn); break;
                case PortDriver::Name::porte: NVIC_EnableIRQ(PORTE_IRQn); break;
                default:                                                  break;
            }
        }

        static void disable_port_irq(const PortDriver::Name port_name)
        {
            switch(port_name)
            {
                case PortDriver::Name::porta: NVIC_DisableIRQ(PORTA_IRQn); break;
                case PortDriver::Name::portb: NVIC_DisableIRQ(PORTB_IRQn); break;
                case PortDriver::Name::portc: NVIC_DisableIRQ(PORTC_IRQn); break;
                case PortDriver::Name::portd: NVIC_DisableIRQ(PORTD_IRQn); break;
                case PortDriver::Name::porte: NVIC_DisableIRQ(PORTE_IRQn); break;
                default:                                                   break;
            }
        }

        static bool is_port_irq_enabled(const PortDriver::Name port_name)
        {
            switch(port_name)
            {
                case PortDriver::Name::porta: return (NVIC_GetEnableIRQ(PORTA_IRQn) != 0);
                case PortDriver::Name::portb: return (NVIC_GetEnableIRQ(PORTB_IRQn) != 0);
                case PortDriver::Name::portc: return (NVIC_GetEnableIRQ(PORTC_IRQn) != 0);
                case PortDriver::Name::portd: return (NVIC_GetEnableIRQ(PORTD_IRQn) != 0);
                case PortDriver::Name::porte: return (NVIC_GetEnableIRQ(PORTE_IRQn) != 0);
                default:                      return false;
            }
        }

        static void set_port_irq_priority(const PortDriver::Name port_name, const int32_t irq_priority)
        {
            switch(port_name)
            {
                case PortDriver::Name::porta: NVIC_SetPriority(PORTA_IRQn, irq_priority); break;
                case PortDriver::Name::portb: NVIC_SetPriority(PORTB_IRQn, irq_priority); break;
                case PortDriver::Name::portc: NVIC_SetPriority(PORTC_IRQn, irq_priority); break;
                case PortDriver::Name::portd: NVIC_SetPriority(PORTD_IRQn, irq_priority); break;
                case PortDriver::Name::porte: NVIC_SetPriority(PORTE_IRQn, irq_priority); break;
                default:                                                                  break;
            }
        }

        // -------- PIN IRQ HANDLER -------------------------------------------

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
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- PORT INTERRUPTS -------------------------------------------

        static uint32_t get_port_interrupts(const std::size_t port_index)
        {
            return PORT_GetPinsInterruptFlags(get_port_base(port_index));
        }

        static void clear_port_interrupts(const std::size_t port_index, const uint32_t mask)
        {
            PORT_ClearPinsInterruptFlags(get_port_base(port_index), mask);
        }

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
        static int32_t irq_handler(const PortDriver::Name port_name)
        {
            const auto port_index     = static_cast<std::size_t>(port_name);
            const auto pint_int_count = static_cast<std::size_t>(PinIntDriver::get_used());

            const uint32_t flags = get_port_interrupts(port_index);

            int32_t yield = 0;

            for(std::size_t p = 0; p < pint_int_count; ++p)
            {
                if(PinIntDriver::get_reference(p).m_port_index == port_index)
                {
                    const uint32_t pin_mask = 1 << PinIntDriver::get_reference(p).m_gpio.m_pin;

                    if(flags & pin_mask)
                    {
                        yield |= PinIntDriver::get_reference(p).irq_handler();
                    }
                }
            }

            clear_port_interrupts(port_index, flags);

            return yield;
        }

        // -------- HELPERS ---------------------------------------------------

        static constexpr PORT_Type* get_port_base(const std::size_t port_index)
        {
            assert(port_index < 5);

            switch(port_index)
            {
                case 0: return PORTA;
                case 1: return PORTB;
                case 2: return PORTC;
                case 3: return PORTD;
                case 4: return PORTE;
            }

            return nullptr;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const IntMode     m_int_mode;
        const std::size_t m_port_index;
        PORT_Type*        m_port_base { nullptr };
        GpioDriver        m_gpio;
        IrqHandler        m_irq_handler; // User defined IRQ handler
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_PIN_INT_HPP

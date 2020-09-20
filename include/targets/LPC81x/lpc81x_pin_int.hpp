// ----------------------------------------------------------------------------
// @file    lpc81x_pin_int.hpp
// @brief   NXP LPC81x pin interrupt class.
// @date    15 September 2020
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

#ifndef __XARMLIB_TARGETS_LPC81X_PIN_INT_HPP
#define __XARMLIB_TARGETS_LPC81X_PIN_INT_HPP

#include "xarmlib_config.hpp"
#include "targets/LPC81x/lpc81x_gpio.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of PININT IRQ handlers
extern "C" void PININT0_IRQHandler(void);
extern "C" void PININT1_IRQHandler(void);
extern "C" void PININT2_IRQHandler(void);
extern "C" void PININT3_IRQHandler(void);
extern "C" void PININT4_IRQHandler(void);
extern "C" void PININT5_IRQHandler(void);
extern "C" void PININT6_IRQHandler(void);
extern "C" void PININT7_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class PinIntDriver : private PeripheralRefCounter<PinIntDriver, TARGET_PIN_INTERRUPT_COUNT>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend PORT IRQ handler C functions to give access to private IRQ handler member function
        friend void ::PININT0_IRQHandler(void);
        friend void ::PININT1_IRQHandler(void);
        friend void ::PININT2_IRQHandler(void);
        friend void ::PININT3_IRQHandler(void);
        friend void ::PININT4_IRQHandler(void);
        friend void ::PININT5_IRQHandler(void);
        friend void ::PININT6_IRQHandler(void);
        friend void ::PININT7_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralPinInt = PeripheralRefCounter<PinIntDriver, TARGET_PIN_INTERRUPT_COUNT>;

        using InputMode = GpioDriver::InputMode;

        using InputModeConfig              = GpioDriver::InputModeConfig;
        using InputModeTrueOpenDrainConfig = GpioDriver::InputModeTrueOpenDrainConfig;

        // Pin interrupt mode
        enum class IntMode
        {
            rising_edge  = 0x1U,  // Interrupt on rising edge
            falling_edge = 0x2U,  // Interrupt on falling edge
            either_edge  = 0x3U,  // Interrupt on either edge
            low_level    = 0x10U, // Interrupt when logic zero
            high_level   = 0x12U  // Interrupt when logic one
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
                                                        m_gpio(pin_name, config),
                                                        m_int_mode { int_mode }
        {
            assert(pin_name != PinDriver::Name::nc);
            assert(pin_name != PinDriver::Name::p0_10 && pin_name != PinDriver::Name::p0_11);

            disable_irq();
            disable_interrupt();
            clear_interrupt_pending();

            // Assign the specified pin to the first free interrupt slot
            LPC_SYSCON->PINTSEL[get_index()] = static_cast<uint32_t>(m_gpio.get_pin_name());
        }

        // True open-drain input pin constructor (only available on P0_10 and P0_11)
        PinIntDriver(const PinDriver::Name               pin_name,
                     const InputModeTrueOpenDrainConfig& config,
                     const IntMode                       int_mode) : PeripheralPinInt(*this),
                                                                     m_gpio(pin_name, config),
                                                                     m_int_mode { int_mode }
        {
            assert(pin_name == PinDriver::Name::p0_10 || pin_name == PinDriver::Name::p0_11);

            disable_irq();
            disable_interrupt();
            clear_interrupt_pending();

            // Assign the specified pin to the first free interrupt slot
            LPC_SYSCON->PINTSEL[get_index()] = static_cast<uint32_t>(m_gpio.get_pin_name());
        }

        ~PinIntDriver()
        {
            disable_irq();
            disable_interrupt();
            clear_interrupt_pending();
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
            const auto index_bitmask = (1UL << get_index());
            const bool edge_sensitive = m_int_mode <= IntMode::either_edge;

            if(edge_sensitive)
            {
                // Set pin interrupt mode sensitive to edge
                LPC_PININT->ISEL = LPC_PININT->ISEL & ~index_bitmask;

                // Enable rising edge
                if(m_int_mode == IntMode::rising_edge || m_int_mode == IntMode::either_edge)
                {
                    LPC_PININT->SIENR = index_bitmask;
                }

                // Enable falling edge
                if(m_int_mode == IntMode::falling_edge || m_int_mode == IntMode::either_edge)
                {
                    LPC_PININT->SIENF = index_bitmask;
                }
            }
            else
            {
                // Set and enable pin interrupt mode sensitive to level
                LPC_PININT->ISEL = LPC_PININT->ISEL | index_bitmask;
                LPC_PININT->SIENR =  index_bitmask;

                // Enable low or high level
                if(m_int_mode == IntMode::low_level)
                {
                    LPC_PININT->CIENF = index_bitmask;
                }
                else
                {
                    LPC_PININT->SIENF = index_bitmask;
                }
            }
        }

        void disable_interrupt()
        {
            const auto index_bitmask = (1UL << get_index());

            // Disable rising edge or level
            LPC_PININT->CIENR = index_bitmask;
            // Disable falling edge
            LPC_PININT->CIENF = index_bitmask;

            LPC_PININT->RISE = 0;
            LPC_PININT->FALL = 0;
        }

        bool is_interrupt_enabled() const
        {
            const auto index_bitmask = (1UL << get_index());

            return ((LPC_PININT->IENR | LPC_PININT->IENF) & index_bitmask) != 0;
        }

        bool is_interrupt_pending() const
        {
            const auto index_bitmask = (1UL << get_index());

            return (LPC_PININT->IST & index_bitmask) != 0;
        }

        void clear_interrupt_pending()
        {
            const bool edge_sensitive = m_int_mode <= IntMode::either_edge;

            // Clear interrupt pending status only when the pin was triggered by edge-sensitive
            if(edge_sensitive)
            {
                const auto index_bitmask = (1UL << get_index());

                LPC_PININT->IST = LPC_PININT->IST | index_bitmask;
            }
        }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()
        {
            NVIC_EnableIRQ(static_cast<IRQn_Type>(PININT0_IRQn + get_index()));
        }

        void disable_irq()
        {
            NVIC_DisableIRQ(static_cast<IRQn_Type>(PININT0_IRQn + get_index()));
        }

        bool is_irq_enabled()
        {
            return NVIC_GetEnableIRQ(static_cast<IRQn_Type>(PININT0_IRQn + get_index())) != 0;
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            NVIC_SetPriority(static_cast<IRQn_Type>(PININT0_IRQn + get_index()), irq_priority);
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

        // Pin interrupt peripheral names selection
        enum class Name
        {
            pin_int0 = 0,
            pin_int1,
            pin_int2,
            pin_int3,
            pin_int4,
            pin_int5,
            pin_int6,
            pin_int7
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // Used by FreeRTOS

            if(m_irq_handler != nullptr)
            {
                yield = m_irq_handler();
            }

            // Only edge-sensitive interrupts will be cleared
            clear_interrupt_pending();

            return yield;
        }

        // IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

           return PinIntDriver::get_reference(index).irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        GpioDriver      m_gpio;
        const IntMode   m_int_mode;
        IrqHandler      m_irq_handler;  // User defined IRQ handler
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_PIN_INT_HPP

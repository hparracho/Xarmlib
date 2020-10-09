// ----------------------------------------------------------------------------
// @file    lpc81x_pin_int.hpp
// @brief   NXP LPC81x pin interrupt class.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------


#ifndef XARMLIB_TARGETS_LPC81X_PIN_INT_HPP
#define XARMLIB_TARGETS_LPC81X_PIN_INT_HPP

#include "core/bitmask.hpp"
#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_gpio.hpp"
#include "targets/LPC81x/lpc81x_pin.hpp"
#include "targets/LPC81x/lpc81x_specs.hpp"
#include "hal/hal_pin_int_base.hpp"




// Forward declaration of PININT IRQ handlers
extern "C" void PININT0_IRQHandler(void);
extern "C" void PININT1_IRQHandler(void);
extern "C" void PININT2_IRQHandler(void);
extern "C" void PININT3_IRQHandler(void);
extern "C" void PININT4_IRQHandler(void);
extern "C" void PININT5_IRQHandler(void);
extern "C" void PININT6_IRQHandler(void);
extern "C" void PININT7_IRQHandler(void);




namespace xarmlib::targets::lpc81x
{

class PinInt : public hal::PinIntBase<PinInt>
{
    // Give IRQ handlers C functions access to private member functions
    friend void ::PININT0_IRQHandler(void);
    friend void ::PININT1_IRQHandler(void);
    friend void ::PININT2_IRQHandler(void);
    friend void ::PININT3_IRQHandler(void);
    friend void ::PININT4_IRQHandler(void);
    friend void ::PININT5_IRQHandler(void);
    friend void ::PININT6_IRQHandler(void);
    friend void ::PININT7_IRQHandler(void);

public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTORS / DESTRUCTOR -------------------------------------

    // Normal input pin constructor
    PinInt(const Pin::Name              pin_name,
           const Gpio::InputModeConfig& config,
           const Mode                   mode) : hal::PinIntBase<PinInt>(*this, mode),
                                                m_gpio(pin_name, config)
    {
        if(pin_name != Pin::Name::nc)
        {
            disable_interrupt();

            // Assign the specified pin to the first free interrupt slot
            LPC_SYSCON->PINTSEL[m_ref_counter.get_this_index()] = static_cast<uint32_t>(m_gpio.get_pin_name());
        }
    }

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    // True open-drain input pin constructor (only available on pins P0_10 and P0_11)
    PinInt(const Pin::Name                           pin_name,
           const Gpio::InputModeTrueOpenDrainConfig& config,
           const Mode                                mode) : hal::PinIntBase<PinInt>(*this, mode),
                                                             m_gpio(pin_name, config)
    {
        if(pin_name != Pin::Name::nc)
        {
            disable_interrupt();

            // Assign the specified pin to the first free interrupt slot
            LPC_SYSCON->PINTSEL[m_ref_counter.get_this_index()] = static_cast<uint32_t>(m_gpio.get_pin_name());
        }
    }
#endif

    // -------- CONFIGURATION -------------------------------------------------

    void set_mode(const Gpio::InputModeConfig& config)              { m_gpio.set_mode(config); }
#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    void set_mode(const Gpio::InputModeTrueOpenDrainConfig& config) { m_gpio.set_mode(config); }
#endif

    Pin::Name get_pin_name() const { return m_gpio.get_pin_name(); }

    // -------- READ ----------------------------------------------------------

    uint32_t read() const { return m_gpio.read(); }

    // -------- INTERRUPT -----------------------------------------------------

    void enable_interrupt()
    {
        const bool irq_enabled = is_irq_enabled();

        if(irq_enabled)
        {
            NVIC_DisableIRQ(get_irq_name());
        }

        const auto index_bitmask = (1UL << m_ref_counter.get_this_index());
        const bool edge_sensitive = (Bitmask<Mode>{m_mode} & Mode::either_edge) != 0;

        if(edge_sensitive)
        {
            // Set pin interrupt mode sensitive to edge
            LPC_PININT->ISEL = LPC_PININT->ISEL & ~index_bitmask;

            // Enable falling edge
            if((Bitmask<Mode>{m_mode} & Mode::falling_edge) != 0)
            {
                LPC_PININT->SIENF = index_bitmask;
            }

            // Enable rising edge
            if((Bitmask<Mode>{m_mode} & Mode::rising_edge) != 0)
            {
                LPC_PININT->SIENR = index_bitmask;
            }
        }
        else
        {
            // Set and enable pin interrupt mode sensitive to level
            LPC_PININT->ISEL = LPC_PININT->ISEL | index_bitmask;
            LPC_PININT->SIENR =  index_bitmask;

            // Enable low or high level
            if(m_mode == Mode::low_level)
            {
                LPC_PININT->CIENF = index_bitmask;
            }
            else
            {
                LPC_PININT->SIENF = index_bitmask;
            }
        }

        if(irq_enabled)
        {
            clear_interrupt_pending();
            NVIC_EnableIRQ(get_irq_name());
        }
    }

    void disable_interrupt()
    {
        const auto index_bitmask = 1UL << m_ref_counter.get_this_index();

        // Disable rising edge or level
        LPC_PININT->CIENR = index_bitmask;
        // Disable falling edge
        LPC_PININT->CIENF = index_bitmask;

        clear_interrupt_pending();
    }

    bool is_interrupt_enabled() const
    {
        const auto index_bitmask = 1UL << m_ref_counter.get_this_index();

        return ((LPC_PININT->IENR | LPC_PININT->IENF) & index_bitmask) != 0;
    }

    bool is_interrupt_pending() const
    {
        const auto index_bitmask = 1UL << m_ref_counter.get_this_index();

        return (LPC_PININT->IST & index_bitmask) != 0;
    }

    void clear_interrupt_pending()
    {
        const bool edge_sensitive = (Bitmask<Mode>{m_mode} & Mode::either_edge) != 0;

        // Clear interrupt pending status only when the pin was triggered by edge-sensitive
        if(edge_sensitive)
        {
            const auto index_bitmask = 1UL << m_ref_counter.get_this_index();

            LPC_PININT->IST |= index_bitmask;
        }
    }

    Bitmask<Mode> get_interrupt_pending() const
    {
        const auto index_bitmask = 1UL << m_ref_counter.get_this_index();
        const bool edge_sensitive = (Bitmask<Mode>{m_mode} & Mode::either_edge) != 0;

        Bitmask<Mode> pending_int;

        if(edge_sensitive)
        {
            if((LPC_PININT->FALL & index_bitmask) != 0) { pending_int  =  Mode::falling_edge; }
            if((LPC_PININT->RISE & index_bitmask) != 0) { pending_int |=  Mode::rising_edge;  }
        }
        else
        {
            if((LPC_PININT->IST & index_bitmask) != 0)
            {
                if((LPC_PININT->IENF & index_bitmask) == 0)
                {
                    pending_int = Mode::low_level;
                }
                else
                {
                    pending_int = Mode::high_level;
                }
            }
        }

        return pending_int;
    }

    // -------- IRQ -----------------------------------------------------------

    // Get the IRQ type associated with this peripheral
    IRQn_Type get_irq_name() const
    {
        return static_cast<IRQn_Type>(PININT0_IRQn + m_ref_counter.get_this_index());
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    Gpio            m_gpio;
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_PIN_INT_HPP

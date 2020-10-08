// ----------------------------------------------------------------------------
// @file    hal_pin_int_base.hpp
// @brief   Pin Interrupt HAL interface class.
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

#ifndef XARMLIB_HAL_PIN_INT_BASE_HPP
#define XARMLIB_HAL_PIN_INT_BASE_HPP

#include "hal/hal_gpio.hpp"
#include "hal/hal_peripheral_irq.hpp"




namespace xarmlib::hal
{

template <typename Driver>
using PinIntPeripheral = PeripheralIrq<Driver, TARGET_PIN_INTERRUPT_COUNT>;

template <typename Driver>
class PinIntBase : public PinIntPeripheral<Driver>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // Pin interrupt mode selection
    enum class Mode : uint32_t
    {
        falling_edge = (1UL << 0),  // Interrupt on falling edge
        rising_edge  = (1UL << 1),  // Interrupt on rising edge
        either_edge  = (3UL << 0),  // Interrupt on either edge
        low_level    = (1UL << 2),  // Interrupt when logic zero
        high_level   = (1UL << 3),  // Interrupt when logic one
        bitmask      = 0x0F         // Full bitmask (0000'0000'0000'1111)
    };

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONFIGURATION -------------------------------------------------

    // Set normal input pin mode
    void set_mode(const Gpio::InputModeConfig& config)
    {
        static_cast<Driver*>(this)->set_mode(config);
    }

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    // Set true open-drain input pin mode
    void set_mode(const Gpio::InputModeTrueOpenDrainConfig& config)
    {
        static_cast<Driver*>(this)->set_mode(config);
    }
#endif

    // Set or change the pin interrupt mode
    void set_mode(const Mode mode)
    {
        if(static_cast<Driver*>(this)->is_interrupt_enabled())
        {
            static_cast<Driver*>(this)->disable_interrupt();
            static_cast<Driver*>(this)->clear_interrupt_pending();
            m_mode = mode;
            static_cast<Driver*>(this)->enable_interrupt();
        }
        else
        {
            m_mode = mode;
        }
    }

    Pin::Name get_pin_name() const
    {
        return static_cast<const Driver*>(this)->get_pin_name();
    }

    // -------- READ ----------------------------------------------------------

    uint32_t read() const { return static_cast<const Driver*>(this)->read(); }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTOR ---------------------------------------------------
    PinIntBase(Driver& driver, Mode mode)
        : PinIntPeripheral<Driver>(driver),
          m_mode {mode}
    {}

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER VARIABLES
    // ------------------------------------------------------------------------

    Mode        m_mode {0};
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_PIN_INT_BASE_HPP

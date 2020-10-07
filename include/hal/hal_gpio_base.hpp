// ----------------------------------------------------------------------------
// @file    hal_gpio_base.hpp
// @brief   GPIO HAL interface class.
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

#ifndef XARMLIB_HAL_GPIO_BASE_HPP
#define XARMLIB_HAL_GPIO_BASE_HPP

#include "hal/hal_pin.hpp"




namespace xarmlib::hal
{

template <typename Driver, typename Traits>
class GpioBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using InputModeConfig               = typename Traits::InputModeConfig;
    using OutputModeConfig              = typename Traits::OutputModeConfig;
#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    using InputModeTrueOpenDrainConfig  = typename Traits::InputModeTrueOpenDrainConfig;
    using OutputModeTrueOpenDrainConfig = typename Traits::OutputModeTrueOpenDrainConfig;
#endif

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONFIGURATION -------------------------------------------------

    // Set normal input pin mode
    void set_mode(const InputModeConfig& config)
    {
        static_cast<Driver*>(this)->set_mode(config);
    }

    // Set normal output pin mode
    void set_mode(const OutputModeConfig& config)
    {
        static_cast<Driver*>(this)->set_mode(config);
    }

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    // Set true open-drain input pin mode
    void set_mode(const InputModeTrueOpenDrainConfig& config)
    {
        static_cast<Driver*>(this)->set_mode(config);
    }

    // Set true open-drain output pin mode
    void set_mode(const OutputModeTrueOpenDrainConfig config)
    {
        static_cast<Driver*>(this)->set_mode(config);
    }
#endif

    Pin::Name get_pin_name() const
    {
        return m_pin_name;
    }

    // -------- READ / WRITE --------------------------------------------------

    uint32_t read() const
    {
        return static_cast<const Driver*>(this)->read();
    }

    void write(const uint32_t value)
    {
        static_cast<Driver*>(this)->write(value);
    }

protected:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTOR ---------------------------------------------------
    GpioBase(const Pin::Name pin_name) : m_pin_name {pin_name}
    {}

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER VARIABLES
    // ------------------------------------------------------------------------

    const Pin::Name m_pin_name {Pin::Name::nc};
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_GPIO_BASE_HPP

// ----------------------------------------------------------------------------
// @file    api_digital_in.hpp
// @brief   API digital input class.
// @date    29 September 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_API_DIGITAL_IN_HPP
#define XARMLIB_API_DIGITAL_IN_HPP

#include "core/target_gpio.hpp"
#include "core/target_pin.hpp"
#include "hal/hal_gpio.hpp"




namespace xarmlib
{

class DigitalIn : private Gpio
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    DigitalIn(const Pin::Name pin_name, const Gpio::InputModeConfig& config = Gpio::InputModeConfig{})
        : Gpio(pin_name, config) {}

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    DigitalIn(const Pin::Name pin_name, const Gpio::InputModeTrueOpenDrainConfig& config)
        : Gpio(pin_name, config) {}
#endif

    // -------- CONFIGURATION -------------------------------------------------

    using hal::Gpio::get_pin_name;

    // -------- READ ----------------------------------------------------------

    using hal::Gpio::read;

    operator uint32_t () const
    {
        return hal::Gpio::read();
    }

    // Read negated value operator
    uint32_t operator ! () const
    {
        return !hal::Gpio::read();
    }
};

} // namespace xarmlib




#endif // XARMLIB_API_DIGITAL_IN_HPP

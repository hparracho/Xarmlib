// ----------------------------------------------------------------------------
// @file    api_digital_out.hpp
// @brief   API digital output class.
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

#ifndef XARMLIB_API_DIGITAL_OUT_HPP
#define XARMLIB_API_DIGITAL_OUT_HPP

#include "core/target_gpio.hpp"
#include "core/target_pin.hpp"
#include "hal/hal_gpio.hpp"




namespace xarmlib
{

class DigitalOut : private Gpio
{
public:

    // --------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // --------------------------------------------------------------------

    DigitalOut(const Pin::Name pin_name, const Gpio::OutputModeConfig& config = Gpio::OutputModeConfig{})
        : Gpio(pin_name, config) {}

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    DigitalOut(const Pin::Name pin_name, const Gpio::OutputModeTrueOpenDrainConfig& config)
        : Gpio(pin_name, config) {}
#endif

    // -------- CONFIGURATION ---------------------------------------------

    using hal::Gpio::get_pin_name;

    // -------- READ ------------------------------------------------------

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

    // -------- WRITE -----------------------------------------------------

    using hal::Gpio::write;

    DigitalOut& operator = (const uint32_t value)
    {
        hal::Gpio::write(value);
        return (*this);
    }

    DigitalOut& operator = (const DigitalOut &rhs)
    {
        hal::Gpio::write(rhs.read());
        return (*this);
    }
};

} // namespace xarmlib




#endif // XARMLIB_API_DIGITAL_OUT_HPP

// ----------------------------------------------------------------------------
// @file    api_digital_out.hpp
// @brief   API digital output class.
// @date    14 January 2020
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

#ifndef __XARMLIB_API_DIGITAL_OUT_HPP
#define __XARMLIB_API_DIGITAL_OUT_HPP

#include "hal/hal_gpio.hpp"

namespace xarmlib
{




class DigitalOut : private hal::Gpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        DigitalOut(const hal::Pin::Name pin_name, const hal::Gpio::OutputModeConfig& config) : hal::Gpio(pin_name, config)
        {}

#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        DigitalOut(const hal::Pin::Name pin_name, const hal::Gpio::OutputModeTrueOpenDrainConfig& config) : hal::Gpio(pin_name, config)
        {}
#endif

        // -------- CONFIGURATION ---------------------------------------------

        hal::Pin::Name get_pin_name() const { return hal::Gpio::get_pin_name(); }

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

#endif // __XARMLIB_API_DIGITAL_OUT_HPP

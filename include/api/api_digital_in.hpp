// ----------------------------------------------------------------------------
// @file    api_digital_in.hpp
// @brief   API digital input class.
// @date    29 November 2018
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

#ifndef __XARMLIB_API_DIGITAL_IN_HPP
#define __XARMLIB_API_DIGITAL_IN_HPP

#include "hal/hal_gpio.hpp"

namespace xarmlib
{




class DigitalIn : private GpioHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        DigitalIn(const PinHal::Name pin_name, const GpioHal::InputModeConfig& config) : GpioHal(pin_name, config)
        {}

        DigitalIn(const PinHal::Name pin_name, const GpioHal::InputModeTrueOpenDrainConfig& config) : GpioHal(pin_name, config)
        {}

        // -------- READ ------------------------------------------------------

        using GpioHal::read;

        operator uint32_t () const
        {
            return GpioHal::read();
        }

        // Read negated value operator
        uint32_t operator ! () const
        {
            return !GpioHal::read();
        }
};




} // namespace xarmlib

#endif // __XARMLIB_API_DIGITAL_IN_HPP

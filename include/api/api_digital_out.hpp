// ----------------------------------------------------------------------------
// @file    api_digital_out.hpp
// @brief   API digital output class.
// @date    21 November 2018
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

#ifndef __XARMLIB_API_DIGITAL_OUT_HPP
#define __XARMLIB_API_DIGITAL_OUT_HPP

#include "hal/hal_gpio.hpp"

namespace xarmlib
{




class DigitalOut : private Gpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

#if defined __KV4X__
        DigitalOut(const Pin::Name           pin_name,
                   const Gpio::OutputMode    output_mode,
                   const Gpio::SlewRate      slew_rate      = Gpio::SlewRate::kPORT_FastSlewRate,
                   const Gpio::DriveStrength drive_strength = Gpio::DriveStrength::kPORT_LowDriveStrength,
                   const Gpio::LockRegister  lock_register  = Gpio::LockRegister::kPORT_UnlockRegister) : Gpio(pin_name,
                                                                                                               output_mode,
                                                                                                               slew_rate,
                                                                                                               drive_strength,
                                                                                                               lock_register)
        {}

        DigitalOut(const Pin::Name                     pin_name,
                   const Gpio::OutputModeTrueOpenDrain output_mode,
                   const Gpio::SlewRate                slew_rate      = Gpio::SlewRate::kPORT_FastSlewRate,
                   const Gpio::DriveStrength           drive_strength = Gpio::DriveStrength::kPORT_LowDriveStrength,
                   const Gpio::LockRegister            lock_register  = Gpio::LockRegister::kPORT_UnlockRegister) : Gpio(pin_name,
                                                                                                                         output_mode,
                                                                                                                         slew_rate,
                                                                                                                         drive_strength,
                                                                                                                         lock_register)
        {}
#elif defined __LPC84X__ || __LPC81X__
        DigitalOut(const Pin::Name pin_name, const Gpio::OutputMode output_mode) : Gpio(pin_name, output_mode)
        {}

        DigitalOut(const Pin::Name pin_name, const Gpio::OutputModeTrueOpenDrain output_mode) : Gpio(pin_name, output_mode)
        {}
#endif

        // -------- READ ------------------------------------------------------

        using Gpio::read;

        operator uint32_t () const
        {
            return Gpio::read();
        }

        // Read negated value operator
        uint32_t operator ! () const
        {
            return !Gpio::read();
        }

        // -------- WRITE -----------------------------------------------------

        using Gpio::write;

        DigitalOut& operator = (const uint32_t value)
        {
            Gpio::write(value);
            return (*this);
        }

        DigitalOut& operator = (const DigitalOut &rhs)
        {
            Gpio::write(rhs.read());
            return (*this);
        }
};




} // namespace xarmlib

#endif // __XARMLIB_API_DIGITAL_OUT_HPP

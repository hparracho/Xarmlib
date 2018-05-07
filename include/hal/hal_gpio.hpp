// ----------------------------------------------------------------------------
// @file    hal_gpio.hpp
// @brief   GPIO HAL interface class.
// @date    7 May 2018
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

#ifndef __XARMLIB_HAL_GPIO_HPP
#define __XARMLIB_HAL_GPIO_HPP

#include "hal/hal_pins.hpp"

namespace xarmlib
{
namespace hal
{




template <class TargetGpio>
class Gpio : private TargetGpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        using Pin = xarmlib::Pin;

        using InputMode               = typename TargetGpio::InputMode;
        using OutputMode              = typename TargetGpio::OutputMode;
        using InputModeTrueOpenDrain  = typename TargetGpio::InputModeTrueOpenDrain;
        using OutputModeTrueOpenDrain = typename TargetGpio::OutputModeTrueOpenDrain;

        using InputFilter             = typename TargetGpio::InputFilter;
        using InputInvert             = typename TargetGpio::InputInvert;
        using InputHysteresis         = typename TargetGpio::InputHysteresis;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Normal input pin constructor
        Gpio(const Pin::Name       pin_name,
             const InputMode       input_mode,
             const InputFilter     input_filter     = InputFilter::BYPASS,
             const InputInvert     input_invert     = InputInvert::NORMAL,
             const InputHysteresis input_hysteresis = InputHysteresis::ENABLE) : TargetGpio(pin_name,
                                                                                            input_mode,
                                                                                            input_filter,
                                                                                            input_invert,
                                                                                            input_hysteresis)
        {}

        // Normal output pin constructor
        Gpio(const Pin::Name  pin_name,
             const OutputMode output_mode) : TargetGpio(pin_name,
                                                        output_mode)
        {}

        // True open-drain input pin constructor
        Gpio(const Pin::Name              pin_name,
             const InputModeTrueOpenDrain input_mode,
             const InputFilter            input_filter = InputFilter::BYPASS,
             const InputInvert            input_invert = InputInvert::NORMAL) : TargetGpio(pin_name,
                                                                                           input_mode,
                                                                                           input_filter,
                                                                                           input_invert)
        {}

        // True open-drain output pin constructor
        Gpio(const Pin::Name               pin_name,
             const OutputModeTrueOpenDrain output_mode) : TargetGpio(pin_name,
                                                                     output_mode)
        {}

        // -------- MODE ------------------------------------------------------

        using TargetGpio::set_mode;

        // -------- READ / WRITE ----------------------------------------------

        using TargetGpio::read;
        using TargetGpio::write;
};




} // namespace hal
} // namespace xarmlib




#include "system/target.h"




#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_gpio.hpp"

namespace xarmlib
{
using Gpio = hal::Gpio<lpc84x::Gpio>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Gpio = hal::Gpio<other_target::Gpio>;
}

#endif




#endif // __XARMLIB_HAL_GPIO_HPP

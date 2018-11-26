// ----------------------------------------------------------------------------
// @file    hal_gpio.hpp
// @brief   GPIO HAL interface class.
// @date    23 November 2018
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

#include "hal/hal_pin.hpp"

namespace xarmlib
{
namespace hal
{




template <class TargetGpio>
class Gpio : public TargetGpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using InputModeConfig               = typename TargetGpio::InputModeConfig;
        using OutputModeConfig              = typename TargetGpio::OutputModeConfig;
        using InputModeTrueOpenDrainConfig  = typename TargetGpio::InputModeTrueOpenDrainConfig;
        using OutputModeTrueOpenDrainConfig = typename TargetGpio::OutputModeTrueOpenDrainConfig;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Default constructor (assign a NC pin)
        Gpio() = default;

        // Normal input pin constructor
        Gpio(const xarmlib::Pin::Name pin_name, const InputModeConfig config) : TargetGpio(pin_name, config)
        {}

        // Normal output pin constructor
        Gpio(const xarmlib::Pin::Name pin_name, const OutputModeConfig config) : TargetGpio(pin_name, config)
        {}

        // True open-drain input pin constructor
        Gpio(const xarmlib::Pin::Name pin_name, const InputModeTrueOpenDrainConfig config) : TargetGpio(pin_name, config)
        {}

        // True open-drain output pin constructor
        Gpio(const xarmlib::Pin::Name pin_name, const OutputModeTrueOpenDrainConfig config) : TargetGpio(pin_name, config)
        {}

        // -------- CONFIGURATION ---------------------------------------------

        using TargetGpio::set_mode;

        // -------- READ / WRITE ----------------------------------------------

        using TargetGpio::read;
        using TargetGpio::write;
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_gpio.hpp"

namespace xarmlib
{
using Gpio = hal::Gpio<targets::kv4x::Gpio>;
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_gpio.hpp"

namespace xarmlib
{
using Gpio = hal::Gpio<targets::lpc84x::Gpio>;
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_gpio.hpp"

namespace xarmlib
{
using Gpio = hal::Gpio<targets::lpc81x::Gpio>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Gpio = hal::Gpio<targets::other_target::Gpio>;
}

#endif




#endif // __XARMLIB_HAL_GPIO_HPP

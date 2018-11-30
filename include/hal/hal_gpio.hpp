// ----------------------------------------------------------------------------
// @file    hal_gpio.hpp
// @brief   GPIO HAL interface class.
// @date    30 November 2018
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




template <typename TargetGpioDriver>
class GpioHal : protected TargetGpioDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using InputMode                     = typename TargetGpioDriver::InputMode;
        using OutputMode                    = typename TargetGpioDriver::OutputMode;
        using OutputModeTrueOpenDrain       = typename TargetGpioDriver::OutputModeTrueOpenDrain;

        using InputModeConfig               = typename TargetGpioDriver::InputModeConfig;
        using OutputModeConfig              = typename TargetGpioDriver::OutputModeConfig;
        using InputModeTrueOpenDrainConfig  = typename TargetGpioDriver::InputModeTrueOpenDrainConfig;
        using OutputModeTrueOpenDrainConfig = typename TargetGpioDriver::OutputModeTrueOpenDrainConfig;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Default constructor (assign a NC pin)
        GpioHal() = default;

        // Normal input pin constructor
        GpioHal(const xarmlib::PinHal::Name pin_name, const InputModeConfig& config) : TargetGpioDriver(pin_name, config)
        {}

        // Normal output pin constructor
        GpioHal(const xarmlib::PinHal::Name pin_name, const OutputModeConfig& config) : TargetGpioDriver(pin_name, config)
        {}

        // True open-drain input pin constructor
        GpioHal(const xarmlib::PinHal::Name pin_name, const InputModeTrueOpenDrainConfig& config) : TargetGpioDriver(pin_name, config)
        {}

        // True open-drain output pin constructor
        GpioHal(const xarmlib::PinHal::Name pin_name, const OutputModeTrueOpenDrainConfig& config) : TargetGpioDriver(pin_name, config)
        {}

        // -------- CONFIGURATION ---------------------------------------------

        inline void set_mode(const InputModeConfig& config)               { TargetGpioDriver::set_mode(config); }
        inline void set_mode(const OutputModeConfig& config)              { TargetGpioDriver::set_mode(config); }
        inline void set_mode(const InputModeTrueOpenDrainConfig& config)  { TargetGpioDriver::set_mode(config); }
        inline void set_mode(const OutputModeTrueOpenDrainConfig& config) { TargetGpioDriver::set_mode(config); }

        // -------- READ / WRITE ----------------------------------------------

        inline uint32_t read() const                { return TargetGpioDriver::read(); }
        inline void     write(const uint32_t value) { TargetGpioDriver::write(value); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_gpio.hpp"

namespace xarmlib
{
using GpioHal = hal::GpioHal<targets::kv4x::GpioDriver>;

//class Gpio : public GpioHal
//{
//    public:
//
//        // --------------------------------------------------------------------
//        // PUBLIC TYPE ALIASES
//        // --------------------------------------------------------------------
//
//        using SlewRate      = typename GpioHal::SlewRate;
//        using PassiveFilter = typename GpioHal::PassiveFilter;
//        using DriveStrength = typename GpioHal::DriveStrength;
//        using LockRegister  = typename GpioHal::LockRegister;
//};
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_gpio.hpp"

namespace xarmlib
{
using GpioHal = hal::GpioHal<targets::lpc84x::GpioDriver>;

//class Gpio : public GpioHal
//{
//    public:
//
//        // --------------------------------------------------------------------
//        // PUBLIC TYPE ALIASES
//        // --------------------------------------------------------------------
//
//        using InputFilterClockDivider = typename GpioHal::InputFilterClockDivider;
//        using InputFilter             = typename GpioHal::InputFilter;
//        using InputInvert             = typename GpioHal::InputInvert;
//        using InputHysteresis         = typename GpioHal::InputHysteresis;
//
//        // --------------------------------------------------------------------
//        // PUBLIC MEMBER FUNCTIONS
//        // --------------------------------------------------------------------
//
//        static inline void set_input_filter_clock_divider(const InputFilterClockDivider clock_div, const uint8_t div)
//        {
//            GpioHal::set_input_filter_clock_divider(clock_div, div);
//        }
//};
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_gpio.hpp"

namespace xarmlib
{
using GpioHal = hal::GpioHal<targets::lpc81x::GpioDriver>;

//class Gpio : public GpioHal
//{
//    public:
//
//        // --------------------------------------------------------------------
//        // PUBLIC TYPE ALIASES
//        // --------------------------------------------------------------------
//
//        using InputFilterClockDivider = typename GpioHal::InputFilterClockDivider;
//        using InputFilter             = typename GpioHal::InputFilter;
//        using InputInvert             = typename GpioHal::InputInvert;
//        using InputHysteresis         = typename GpioHal::InputHysteresis;
//
//        // --------------------------------------------------------------------
//        // PUBLIC MEMBER FUNCTIONS
//        // --------------------------------------------------------------------
//
//        static inline void set_input_filter_clock_divider(const InputFilterClockDivider clock_div, const uint8_t div)
//        {
//            GpioHal::set_input_filter_clock_divider(clock_div, div);
//        }
//};
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using GpioHal = hal::GpioHal<targets::other_target::GpioDriver>;
using Gpio = GpioHal;
}

#endif




#endif // __XARMLIB_HAL_GPIO_HPP

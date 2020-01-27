// ----------------------------------------------------------------------------
// @file    hal_gpio.hpp
// @brief   GPIO HAL interface class.
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

#ifndef __XARMLIB_HAL_GPIO_HPP
#define __XARMLIB_HAL_GPIO_HPP

#include "hal/hal_pin.hpp"

namespace xarmlib
{
namespace hal
{




template <typename GpioDriver>
class GpioBase : protected GpioDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using InputMode                     = typename GpioDriver::InputMode;
        using OutputMode                    = typename GpioDriver::OutputMode;
#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        using OutputModeTrueOpenDrain       = typename GpioDriver::OutputModeTrueOpenDrain;
#endif

        using InputModeConfig               = typename GpioDriver::InputModeConfig;
        using OutputModeConfig              = typename GpioDriver::OutputModeConfig;
#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        using InputModeTrueOpenDrainConfig  = typename GpioDriver::InputModeTrueOpenDrainConfig;
        using OutputModeTrueOpenDrainConfig = typename GpioDriver::OutputModeTrueOpenDrainConfig;
#endif

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Default constructor (assign a NC pin)
        GpioBase() = default;

        // Normal input pin constructor
        GpioBase(const hal::Pin::Name pin_name, const InputModeConfig& config) : GpioDriver(pin_name, config)
        {}

        // Normal output pin constructor
        GpioBase(const hal::Pin::Name pin_name, const OutputModeConfig& config) : GpioDriver(pin_name, config)
        {}

#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        // True open-drain input pin constructor
        GpioBase(const hal::Pin::Name pin_name, const InputModeTrueOpenDrainConfig& config) : GpioDriver(pin_name, config)
        {}

        // True open-drain output pin constructor
        GpioBase(const hal::Pin::Name pin_name, const OutputModeTrueOpenDrainConfig& config) : GpioDriver(pin_name, config)
        {}
#endif

        // -------- CONFIGURATION ---------------------------------------------

        void set_mode(const InputModeConfig& config)               { GpioDriver::set_mode(config); }
        void set_mode(const OutputModeConfig& config)              { GpioDriver::set_mode(config); }
#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        void set_mode(const InputModeTrueOpenDrainConfig& config)  { GpioDriver::set_mode(config); }
        void set_mode(const OutputModeTrueOpenDrainConfig& config) { GpioDriver::set_mode(config); }
#endif

        hal::Pin::Name get_pin_name() const { return GpioDriver::get_pin_name(); }

        // -------- READ / WRITE ----------------------------------------------

        uint32_t read() const                { return GpioDriver::read(); }
        void     write(const uint32_t value) { GpioDriver::write(value); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_gpio.hpp"

namespace xarmlib
{
namespace hal
{

using Gpio = GpioBase<targets::kv5x::GpioDriver>;

} // namespace hal

class Gpio : public hal::Gpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Gpio;

        using SlewRate      = typename Hal::SlewRate;
        using PassiveFilter = typename Hal::PassiveFilter;
        using DriveStrength = typename Hal::DriveStrength;
        using LockRegister  = typename Hal::LockRegister;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_gpio.hpp"

namespace xarmlib
{
namespace hal
{

using Gpio = GpioBase<targets::kv4x::GpioDriver>;

} // namespace hal

class Gpio : public hal::Gpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Gpio;

        using SlewRate      = typename Hal::SlewRate;
        using PassiveFilter = typename Hal::PassiveFilter;
        using DriveStrength = typename Hal::DriveStrength;
        using LockRegister  = typename Hal::LockRegister;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;
};

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_gpio.hpp"

namespace xarmlib
{
namespace hal
{

using Gpio = GpioBase<targets::lpc84x::GpioDriver>;

} // namespace hal

class Gpio : public hal::Gpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------
        using Hal = hal::Gpio;

        using InputFilterClockDivider = typename Hal::InputFilterClockDivider;
        using InputFilter             = typename Hal::InputFilter;
        using InputInvert             = typename Hal::InputInvert;
        using InputHysteresis         = typename Hal::InputHysteresis;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        static void set_input_filter_clock_divider(const InputFilterClockDivider clock_div, const uint8_t div)
        {
            Hal::set_input_filter_clock_divider(clock_div, div);
        }
};

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_gpio.hpp"

namespace xarmlib
{
namespace hal
{

using Gpio = GpioBase<targets::lpc81x::GpioDriver>;

} // namespace hal

class Gpio : public hal::Gpio
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Gpio;

        using InputFilterClockDivider = typename Hal::InputFilterClockDivider;
        using InputFilter             = typename Hal::InputFilter;
        using InputInvert             = typename Hal::InputInvert;
        using InputHysteresis         = typename Hal::InputHysteresis;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal::Hal;

        static void set_input_filter_clock_divider(const InputFilterClockDivider clock_div, const uint8_t div)
        {
            Hal::set_input_filter_clock_divider(clock_div, div);
        }
};

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Gpio = GpioBase<targets::other_target::GpioDriver>;

} // namespace hal

using Gpio = hal::Gpio;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_GPIO_HPP

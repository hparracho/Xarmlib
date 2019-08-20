// ----------------------------------------------------------------------------
// @file    lpc81x_gpio.hpp
// @brief   NXP LPC81x GPIO class.
// @date    4 March 2019
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

#ifndef __XARMLIB_TARGETS_LPC81X_GPIO_HPP
#define __XARMLIB_TARGETS_LPC81X_GPIO_HPP

#include "targets/LPC81x/lpc81x_pin.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class GpioDriver
{
    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Input pin modes
        enum class InputMode
        {
            hiz = 0,
            pull_down,
            pull_up,
            repeater
        };

        // Output pin modes
        enum class OutputMode
        {
            push_pull_low = 0,
            push_pull_high,
            open_drain_low,
            open_drain_hiz
        };

        // True open-drain output pin modes
        enum class OutputModeTrueOpenDrain
        {
            low = 0,
            hiz
        };

        using InputFilterClockDivider = ClockDriver::IoconClockDividerSelect;
        using InputFilter             = PinDriver::InputFilter;
        using InputInvert             = PinDriver::InputInvert;
        using InputHysteresis         = PinDriver::InputHysteresis;

        struct InputModeConfig
        {
            InputMode       input_mode       = InputMode::pull_up;
            InputFilter     input_filter     = InputFilter::bypass;
            InputInvert     input_invert     = InputInvert::normal;
            InputHysteresis input_hysteresis = InputHysteresis::enable;
        };

        struct OutputModeConfig
        {
            OutputMode output_mode = OutputMode::push_pull_low;
        };

        struct InputModeTrueOpenDrainConfig
        {
            // input mode: HIZ
            InputFilter input_filter = InputFilter::bypass;
            InputInvert input_invert = InputInvert::normal;
        };

        struct OutputModeTrueOpenDrainConfig
        {
            OutputModeTrueOpenDrain output_mode = OutputModeTrueOpenDrain::low;
        };

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Default constructor (assign a NC pin)
        GpioDriver() : m_pin_name { PinDriver::Name::nc }
        {}

        // Normal input pin constructor
        GpioDriver(const PinDriver::Name pin_name, const InputModeConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name != PinDriver::Name::nc)
            {
                config_port();
                set_mode(config);
            }
        }

        // Normal output pin constructor
        GpioDriver(const PinDriver::Name pin_name, const OutputModeConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name != PinDriver::Name::nc)
            {
                config_port();
                set_mode(config);
            }
        }

        // True open-drain input pin constructor (only available on P0_10 and P0_11)
        GpioDriver(const PinDriver::Name pin_name, const InputModeTrueOpenDrainConfig& config) : m_pin_name { pin_name }
        {
#if (TARGET_PACKAGE_PIN_COUNT >= 16)
            if(pin_name == PinDriver::Name::p0_10 || pin_name == PinDriver::Name::p0_11)
            {
                config_port();
                set_mode(config);
            }
#else
            (void)config.input_filter;
            (void)config.input_invert;
            assert(TARGET_PACKAGE_PIN_COUNT >= 16 /* Invalid function on packages without true open-drain pins */);
#endif
        }

        // True open-drain output pin constructor (only available on P0_10 and P0_11)
        GpioDriver(const PinDriver::Name pin_name, const OutputModeTrueOpenDrainConfig config) : m_pin_name { pin_name }
        {
#if (TARGET_PACKAGE_PIN_COUNT >= 16)
            if(pin_name == PinDriver::Name::p0_10 || pin_name == PinDriver::Name::p0_11)
            {
                config_port();
                set_mode(config);
            }
#else
            (void)config.output_mode;
            assert(TARGET_PACKAGE_PIN_COUNT >= 16 /* Invalid function on packages without true open-drain pins */);
#endif
        }

        // -------- CONFIGURATION ---------------------------------------------

        // Set normal input pin mode
        void set_mode(const InputModeConfig& config)
        {
            // Exclude NC
            assert(m_pin_name != PinDriver::Name::nc);

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
            // Exclude true open-drain pins
            assert(m_pin_name != PinDriver::Name::p0_10 && m_pin_name != PinDriver::Name::p0_11);
#endif

            PinDriver::FunctionMode function_mode;

            switch(config.input_mode)
            {
                case InputMode::hiz:       function_mode = PinDriver::FunctionMode::hiz;       break;
                case InputMode::pull_down: function_mode = PinDriver::FunctionMode::pull_down; break;
                case InputMode::repeater:  function_mode = PinDriver::FunctionMode::repeater;  break;
                case InputMode::pull_up:
                default:                   function_mode = PinDriver::FunctionMode::pull_up;   break;
            }

            write(0);
            set_direction(Direction::input);

            PinDriver::set_mode(m_pin_name, function_mode,
                                            PinDriver::OpenDrain::disable,
                                            config.input_filter,
                                            config.input_invert,
                                            config.input_hysteresis);
        }

        // Set normal output pin mode
        void set_mode(const OutputModeConfig& config)
        {
            // Exclude NC
            assert(m_pin_name != PinDriver::Name::nc);

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
            // Exclude true open-drain pins
            assert(m_pin_name != PinDriver::Name::p0_10 && m_pin_name != PinDriver::Name::p0_11);
#endif

            uint32_t             pin_value;
            PinDriver::OpenDrain open_drain;

            switch(config.output_mode)
            {
                case OutputMode::push_pull_low:  pin_value = 0; open_drain = PinDriver::OpenDrain::disable; break;
                case OutputMode::open_drain_low: pin_value = 0; open_drain = PinDriver::OpenDrain::enable;  break;
                case OutputMode::open_drain_hiz: pin_value = 1; open_drain = PinDriver::OpenDrain::enable;  break;
                case OutputMode::push_pull_high:
                default:                         pin_value = 1; open_drain = PinDriver::OpenDrain::disable; break;
            }

            write(pin_value);
            set_direction(Direction::output);

            PinDriver::set_mode(m_pin_name, PinDriver::FunctionMode::hiz,
                                            open_drain,
                                            PinDriver::InputFilter::bypass,
                                            PinDriver::InputInvert::normal,
                                            PinDriver::InputHysteresis::enable);
        }

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
        // Set true open-drain input pin mode (only available on P0_10 and P0_11)
        void set_mode(const InputModeTrueOpenDrainConfig& config)
        {
            // Available only on true open-drain pins
            assert(m_pin_name == PinDriver::Name::p0_10 || m_pin_name == PinDriver::Name::p0_11);

            write(0);
            set_direction(Direction::input);

            PinDriver::set_mode(m_pin_name, PinDriver::I2cMode::standard_gpio, config.input_filter, config.input_invert);
        }

        // Set true open-drain output pin mode (only available on P0_10 and P0_11)
        void set_mode(const OutputModeTrueOpenDrainConfig config)
        {
            // Available only on true open-drain pins
            assert(m_pin_name == PinDriver::Name::p0_10 || m_pin_name == PinDriver::Name::p0_11);

            write((config.output_mode == OutputModeTrueOpenDrain::low) ? 0 : 1);
            set_direction(Direction::output);

            PinDriver::set_mode(m_pin_name, PinDriver::I2cMode::standard_gpio, PinDriver::InputFilter::bypass, PinDriver::InputInvert::normal);
        }
#endif

        // -------- READ / WRITE ----------------------------------------------

        uint32_t read() const
        {
            if(reg_w != nullptr)
            {
                return (*reg_w != 0) ? 1 : 0;
            }

            return 0;
        }

        void write(const uint32_t value)
        {
            if(reg_w != nullptr)
            {
                *reg_w = value;
            }
        }

        // -------- INPUT FILTER CLOCK DIVIDER SELECTION ----------------------

        // Set the value of the supplied IOCON clock divider (used by input filters)
        // NOTE: The input filter source (where the divider is applied) is the MAIN clock
        static void set_input_filter_clock_divider(const InputFilterClockDivider clock_div, const uint8_t div)
        {
            ClockDriver::set_iocon_clock_divider(clock_div, div);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Pin direction
        enum class Direction
        {
            input = 0,
            output
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void config_port()
        {
            m_pin_mask = 1 << static_cast<uint32_t>(m_pin_name);

            reg_w   = &LPC_GPIO->W0[static_cast<uint32_t>(m_pin_name)];
            reg_dir = &LPC_GPIO->DIR0;

            if(ClockDriver::is_enabled(ClockDriver::Peripheral::gpio) == false)
            {
                // Enable GPIO
                ClockDriver::enable(ClockDriver::Peripheral::gpio);
                PowerDriver::reset(PowerDriver::ResetPeripheral::gpio);
            }
        }

        // Set pin direction
        void set_direction(const Direction direction)
        {
            if(reg_dir != nullptr)
            {
                switch(direction)
                {
                    case Direction::input:  *reg_dir &= ~m_pin_mask; break;
                    case Direction::output: *reg_dir |=  m_pin_mask; break;
                }
            }
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const PinDriver::Name m_pin_name;
             uint32_t         m_pin_mask { 0 };
        __IO uint32_t*        reg_w      { nullptr };
        __IO uint32_t*        reg_dir    { nullptr };
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_GPIO_HPP

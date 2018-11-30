// ----------------------------------------------------------------------------
// @file    lpc84x_gpio.hpp
// @brief   NXP LPC84x GPIO class.
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

#ifndef __XARMLIB_TARGETS_LPC84X_GPIO_HPP
#define __XARMLIB_TARGETS_LPC84X_GPIO_HPP

#include "targets/LPC84x/lpc84x_pin.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
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
            HIZ = 0,
            PULL_DOWN,
            PULL_UP,
            REPEATER
        };

        // Output pin modes
        enum class OutputMode
        {
            PUSH_PULL_LOW = 0,
            PUSH_PULL_HIGH,
            OPEN_DRAIN_LOW,
            OPEN_DRAIN_HIZ
        };

        // True open-drain output pin modes
        enum class OutputModeTrueOpenDrain
        {
            LOW = 0,
            HIZ
        };

        using InputFilterClockDivider = ClockDriver::IoconClockDividerSelect;
        using InputFilter             = PinDriver::InputFilter;
        using InputInvert             = PinDriver::InputInvert;
        using InputHysteresis         = PinDriver::InputHysteresis;

        struct InputModeConfig
        {
            InputMode       input_mode       = InputMode::PULL_UP;
            InputFilter     input_filter     = InputFilter::BYPASS;
            InputInvert     input_invert     = InputInvert::NORMAL;
            InputHysteresis input_hysteresis = InputHysteresis::ENABLE;
        };

        struct OutputModeConfig
        {
            OutputMode output_mode = OutputMode::PUSH_PULL_LOW;
        };

        struct InputModeTrueOpenDrainConfig
        {
            // input mode: HIZ
            InputFilter input_filter = InputFilter::BYPASS;
            InputInvert input_invert = InputInvert::NORMAL;
        };

        struct OutputModeTrueOpenDrainConfig
        {
            OutputModeTrueOpenDrain output_mode = OutputModeTrueOpenDrain::LOW;
        };

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Default constructor (assign a NC pin)
        GpioDriver() : m_pin_name { PinDriver::Name::NC }
        {}

        // Normal input pin constructor
        GpioDriver(const PinDriver::Name pin_name, const InputModeConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name != PinDriver::Name::NC)
            {
                config_port();
                set_mode(config);
            }
        }

        // Normal output pin constructor
        GpioDriver(const PinDriver::Name pin_name, const OutputModeConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name != PinDriver::Name::NC)
            {
                config_port();
                set_mode(config);
            }
        }

        // True open-drain input pin constructor (only available on P0_10 and P0_11)
        GpioDriver(const PinDriver::Name pin_name, const InputModeTrueOpenDrainConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name == PinDriver::Name::P0_10 || pin_name == PinDriver::Name::P0_11)
            {
                config_port();
                set_mode(config);
            }
        }

        // True open-drain output pin constructor (only available on P0_10 and P0_11)
        GpioDriver(const PinDriver::Name pin_name, const OutputModeTrueOpenDrainConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name == PinDriver::Name::P0_10 || pin_name == PinDriver::Name::P0_11)
            {
                config_port();
                set_mode(config);
            }
        }

        // -------- CONFIGURATION ---------------------------------------------

        // Set normal input pin mode
        void set_mode(const InputModeConfig& config)
        {
            // Exclude NC and true open-drain pins
            assert(m_pin_name != PinDriver::Name::NC && m_pin_name != PinDriver::Name::P0_10 && m_pin_name != PinDriver::Name::P0_11);

            PinDriver::FunctionMode function_mode;

            switch(config.input_mode)
            {
                case InputMode::HIZ:       function_mode = PinDriver::FunctionMode::HIZ;       break;
                case InputMode::PULL_DOWN: function_mode = PinDriver::FunctionMode::PULL_DOWN; break;
                case InputMode::REPEATER:  function_mode = PinDriver::FunctionMode::REPEATER;  break;
                case InputMode::PULL_UP:
                default:                   function_mode = PinDriver::FunctionMode::PULL_UP;   break;
            }

            write(0);
            set_direction(Direction::INPUT);

            PinDriver::set_mode(m_pin_name, function_mode,
                                            PinDriver::OpenDrain::DISABLE,
                                            config.input_filter,
                                            config.input_invert,
                                            config.input_hysteresis);
        }

        // Set normal output pin mode
        void set_mode(const OutputModeConfig& config)
        {
            // Exclude NC and true open-drain pins
            assert(m_pin_name != PinDriver::Name::NC && m_pin_name != PinDriver::Name::P0_10 && m_pin_name != PinDriver::Name::P0_11);

            uint32_t             pin_value;
            PinDriver::OpenDrain open_drain;

            switch(config.output_mode)
            {
                case OutputMode::PUSH_PULL_LOW:  pin_value = 0; open_drain = PinDriver::OpenDrain::DISABLE; break;
                case OutputMode::OPEN_DRAIN_LOW: pin_value = 0; open_drain = PinDriver::OpenDrain::ENABLE;  break;
                case OutputMode::OPEN_DRAIN_HIZ: pin_value = 1; open_drain = PinDriver::OpenDrain::ENABLE;  break;
                case OutputMode::PUSH_PULL_HIGH:
                default:                         pin_value = 1; open_drain = PinDriver::OpenDrain::DISABLE; break;
            }

            write(pin_value);
            set_direction(Direction::OUTPUT);

            PinDriver::set_mode(m_pin_name, PinDriver::FunctionMode::HIZ,
                                            open_drain,
                                            PinDriver::InputFilter::BYPASS,
                                            PinDriver::InputInvert::NORMAL,
                                            PinDriver::InputHysteresis::ENABLE);
        }

        // Set true open-drain input pin mode (only available on P0_10 and P0_11)
        void set_mode(const InputModeTrueOpenDrainConfig& config)
        {
            // Available only on true open-drain pins
            assert(m_pin_name == PinDriver::Name::P0_10 || m_pin_name == PinDriver::Name::P0_11);

            write(0);
            set_direction(Direction::INPUT);

            PinDriver::set_mode(m_pin_name, PinDriver::I2cMode::STANDARD_GPIO, config.input_filter, config.input_invert);
        }

        // Set true open-drain output pin mode (only available on P0_10 and P0_11)
        void set_mode(const OutputModeTrueOpenDrainConfig& config)
        {
            // Available only on true open-drain pins
            assert(m_pin_name == PinDriver::Name::P0_10 || m_pin_name == PinDriver::Name::P0_11);

            write((config.output_mode == OutputModeTrueOpenDrain::LOW) ? 0 : 1);
            set_direction(Direction::OUTPUT);

            PinDriver::set_mode(m_pin_name, PinDriver::I2cMode::STANDARD_GPIO, PinDriver::InputFilter::BYPASS, PinDriver::InputInvert::NORMAL);
        }

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
            INPUT = 0,
            OUTPUT
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void config_port()
        {
            if(static_cast<uint32_t>(m_pin_name) < 32)
            {
                m_pin_mask = 1 << static_cast<uint32_t>(m_pin_name);

                reg_w   = &LPC_GPIO->W0[static_cast<uint32_t>(m_pin_name)];
                reg_dir = &LPC_GPIO->DIR0;

                if(ClockDriver::is_enabled(ClockDriver::Peripheral::GPIO0) == false)
                {
                    // Enable GPIO port 0
                    ClockDriver::enable(ClockDriver::Peripheral::GPIO0);
                    PowerDriver::reset(PowerDriver::ResetPeripheral::GPIO0);
                }
            }
            else
            {
                m_pin_mask = 1 << (static_cast<uint32_t>(m_pin_name) - 32);

                reg_w   = &LPC_GPIO->W1[static_cast<uint32_t>(m_pin_name) - 32];
                reg_dir = &LPC_GPIO->DIR1;

                if(ClockDriver::is_enabled(ClockDriver::Peripheral::GPIO1) == false)
                {
                    // Enable GPIO port 1
                    ClockDriver::enable(ClockDriver::Peripheral::GPIO1);
                    PowerDriver::reset(PowerDriver::ResetPeripheral::GPIO1);
                }
            }
        }

        // Set pin direction
        void set_direction(const Direction direction)
        {
            if(reg_dir != nullptr)
            {
                switch(direction)
                {
                    case Direction::INPUT:  *reg_dir &= ~m_pin_mask; break;
                    case Direction::OUTPUT: *reg_dir |=  m_pin_mask; break;
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




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_GPIO_HPP

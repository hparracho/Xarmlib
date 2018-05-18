// ----------------------------------------------------------------------------
// @file    lpc84x_gpio.hpp
// @brief   NXP LPC84x GPIO class.
// @date    18 May 2018
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

#include <cstdint>

#include "targets/LPC84x/lpc84x_pin.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




class Gpio
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

        // True open-drain input pin modes
        enum class InputModeTrueOpenDrain
        {
            HIZ = 0
        };

        // True open-drain output pin modes
        enum class OutputModeTrueOpenDrain
        {
            LOW = 0,
            HIZ
        };

        using InputFilterClockDiv = Clock::IoconClockDivSelect;
        using InputFilter         = Pin::InputFilter;
        using InputInvert         = Pin::InputInvert;
        using InputHysteresis     = Pin::InputHysteresis;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Normal input pin constructor
        Gpio(const Pin::Name       pin_name,
             const InputMode       input_mode,
             const InputFilter     input_filter,
             const InputInvert     input_invert,
             const InputHysteresis input_hysteresis) : m_pin {pin_name}
        {
            if(pin_name != Pin::Name::NC)
            {
                config_port();
                set_mode(input_mode, input_filter, input_invert, input_hysteresis);
            }
        }

        // Normal output pin constructor
        Gpio(const Pin::Name  pin_name,
             const OutputMode output_mode) : m_pin {pin_name}
        {
            if(pin_name != Pin::Name::NC)
            {
                config_port();
                set_mode(output_mode);
            }
        }

#if (__LPC84X_GPIOS__ == 54)
        // True open-drain input pin constructor (only available on P0_10 and P0_11)
        Gpio(const Pin::Name pin_name,
             const InputModeTrueOpenDrain input_mode,
             const InputFilter            input_filter,
             const InputInvert            input_invert) : m_pin {pin_name}
        {
            if(pin_name == Pin::Name::P0_10 || pin_name == Pin::Name::P0_11)
            {
                config_port();
                set_mode(input_mode, input_filter, input_invert);
            }
        }

        // True open-drain output pin constructor (only available on P0_10 and P0_11)
        Gpio(const Pin::Name               pin_name,
             const OutputModeTrueOpenDrain output_mode) : m_pin {pin_name}
        {
            if(pin_name == Pin::Name::P0_10 || pin_name == Pin::Name::P0_11)
            {
                config_port();
                set_mode(output_mode);
            }
        }
#endif

        // -------- CONFIGURATION ---------------------------------------------

        // Set normal input pin mode
        void set_mode(const InputMode       input_mode,
                      const InputFilter     input_filter     = InputFilter::BYPASS,
                      const InputInvert     input_invert     = InputInvert::NORMAL,
                      const InputHysteresis input_hysteresis = InputHysteresis::ENABLE)
        {
            assert(m_pin != Pin::Name::NC);

#if (__LPC84X_GPIOS__ == 54)
            // Exclude true open-drain pins
            assert(m_pin != Pin::Name::P0_10 && m_pin != Pin::Name::P0_11);
#endif

            Pin::FunctionMode function_mode;

            switch(input_mode)
            {
                case InputMode::HIZ:       function_mode = Pin::FunctionMode::HIZ;       break;
                case InputMode::PULL_DOWN: function_mode = Pin::FunctionMode::PULL_DOWN; break;
                case InputMode::REPEATER:  function_mode = Pin::FunctionMode::REPEATER;  break;
                case InputMode::PULL_UP:
                default:                   function_mode = Pin::FunctionMode::PULL_UP;   break;
            }

            write(0);
            set_direction(Direction::INPUT);

            Pin::set_mode(m_pin, function_mode,
                                 Pin::OpenDrain::DISABLE,
                                 input_filter,
                                 input_invert,
                                 input_hysteresis);
        }

        // Set normal output pin mode
        void set_mode(const OutputMode output_mode)
        {
            assert(m_pin != Pin::Name::NC);

#if (__LPC84X_GPIOS__ == 54)
            // Exclude true open-drain pins
            assert(m_pin != Pin::Name::P0_10 && m_pin != Pin::Name::P0_11);
#endif

            uint32_t pin_value;
            Pin::OpenDrain open_drain;

            switch(output_mode)
            {
                case OutputMode::PUSH_PULL_LOW:  pin_value = 0; open_drain = Pin::OpenDrain::DISABLE; break;
                case OutputMode::OPEN_DRAIN_LOW: pin_value = 0; open_drain = Pin::OpenDrain::ENABLE;  break;
                case OutputMode::OPEN_DRAIN_HIZ: pin_value = 1; open_drain = Pin::OpenDrain::ENABLE;  break;
                case OutputMode::PUSH_PULL_HIGH:
                default:                         pin_value = 1; open_drain = Pin::OpenDrain::DISABLE; break;
            }

            write(pin_value);
            set_direction(Direction::OUTPUT);

            Pin::set_mode(m_pin, Pin::FunctionMode::HIZ,
                                 open_drain,
                                 Pin::InputFilter::BYPASS,
                                 Pin::InputInvert::NORMAL,
                                 Pin::InputHysteresis::ENABLE);
        }

#if (__LPC84X_GPIOS__ == 54)
        // Set true open-drain input pin mode (only available on P0_10 and P0_11)
        void set_mode(const InputModeTrueOpenDrain input_mode,
                      const InputFilter            input_filter = InputFilter::BYPASS,
                      const InputInvert            input_invert = InputInvert::NORMAL)
        {
            (void)input_mode; // Input mode only used to identify the type of pin

            // Available only on true open-drain pins
            assert(m_pin == Pin::Name::P0_10 || m_pin == Pin::Name::P0_11);

            write(0);
            set_direction(Direction::INPUT);

            Pin::set_mode(m_pin, Pin::I2cMode::STANDARD_GPIO, input_filter, input_invert);
        }

        // Set true open-drain output pin mode (only available on P0_10 and P0_11)
        void set_mode(const OutputModeTrueOpenDrain output_mode)
        {
            // Available only on true open-drain pins
            assert(m_pin == Pin::Name::P0_10 || m_pin == Pin::Name::P0_11);

            write((output_mode == OutputModeTrueOpenDrain::LOW) ? 0 : 1);
            set_direction(Direction::OUTPUT);

            Pin::set_mode(m_pin, Pin::I2cMode::STANDARD_GPIO, Pin::InputFilter::BYPASS, Pin::InputInvert::NORMAL);
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

        // -------- INPUT FILTER CLOCK DIV SELECTION --------------------------

        // Set the value of the supplied IOCON clock divider (used by input filters)
        // NOTE: The input filter source (where the divider is applied) is the MAIN clock
        static void set_input_filter_clock_div(const InputFilterClockDiv clock_div, const uint8_t div)
        {
            Clock::set_iocon_clock_div(clock_div, div);
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
            if(static_cast<uint32_t>(m_pin) < 32)
            {
                m_mask = 1 << static_cast<uint32_t>(m_pin);

                reg_w   = &LPC_GPIO->W0[static_cast<uint32_t>(m_pin)];
                reg_dir = &LPC_GPIO->DIR0;

                if(Clock::is_enabled(Clock::Peripheral::GPIO0) == false)
                {
                    // Enable GPIO port 0
                    Clock::enable(Clock::Peripheral::GPIO0);
                    Power::reset(Power::ResetPeripheral::GPIO0);
                }
            }
            else
            {
                m_mask = 1 << (static_cast<uint32_t>(m_pin) - 32);

                reg_w   = &LPC_GPIO->W1[static_cast<uint32_t>(m_pin) - 32];
                reg_dir = &LPC_GPIO->DIR1;

                if(Clock::is_enabled(Clock::Peripheral::GPIO1) == false)
                {
                    // Enable GPIO port 1
                    Clock::enable(Clock::Peripheral::GPIO1);
                    Power::reset(Power::ResetPeripheral::GPIO1);
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
                    case Direction::INPUT:  *reg_dir &= ~m_mask; break;
                    case Direction::OUTPUT: *reg_dir |=  m_mask; break;
                }
            }
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const Pin::Name     m_pin;
             uint32_t       m_mask  { 0 };
        __IO uint32_t*      reg_w   { nullptr };
        __IO uint32_t*      reg_dir { nullptr };
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_GPIO_HPP

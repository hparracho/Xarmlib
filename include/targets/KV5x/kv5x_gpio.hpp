// ----------------------------------------------------------------------------
// @file    kv5x_gpio.hpp
// @brief   Kinetis KV5x GPIO class.
// @date    13 January 2020
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

#ifndef __XARMLIB_TARGETS_KV5X_GPIO_HPP
#define __XARMLIB_TARGETS_KV5X_GPIO_HPP

#include "targets/KV5x/kv5x_pin.hpp"
#include "fsl_gpio.h"

namespace xarmlib
{
namespace targets
{
namespace kv5x
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
            pull_up
        };

        // Output pin modes
        enum class OutputMode
        {
            push_pull_low = 0,
            push_pull_high,
            open_drain_low,
            open_drain_hiz
        };

        using SlewRate      = PinDriver::SlewRate;
        using PassiveFilter = PinDriver::PassiveFilter;
        using DriveStrength = PinDriver::DriveStrength;
        using LockRegister  = PinDriver::LockRegister;

        struct InputModeConfig
        {
            InputMode     input_mode     = InputMode::pull_up;
            PassiveFilter passive_filter = PassiveFilter::disable;
            LockRegister  lock_register  = LockRegister::unlock;
        };

        struct OutputModeConfig
        {
            OutputMode    output_mode    = OutputMode::push_pull_low;
            SlewRate      slew_rate      = SlewRate::fast;
            DriveStrength drive_strength = DriveStrength::low;
            LockRegister  lock_register  = LockRegister::unlock;
        };

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Default constructor (assign a NC pin)
        GpioDriver() : m_pin_name { PinDriver::Name::nc }
        {}

        // Input pin constructor
        GpioDriver(const PinDriver::Name pin_name, const InputModeConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name != PinDriver::Name::nc)
            {
                config_port();
                set_mode(config);
            }
        }

        // Output pin constructor
        GpioDriver(const PinDriver::Name pin_name, const OutputModeConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name != PinDriver::Name::nc)
            {
                config_port();
                set_mode(config);
            }
        }

        // -------- CONFIGURATION ---------------------------------------------

        // Set input pin mode
        void set_mode(const InputModeConfig& config)
        {
            // Exclude NC
            assert(m_pin_name != PinDriver::Name::nc);

            PinDriver::FunctionMode function_mode;

            switch(config.input_mode)
            {
                case InputMode::hiz:       function_mode = PinDriver::FunctionMode::hiz;       break;
                case InputMode::pull_down: function_mode = PinDriver::FunctionMode::pull_down; break;
                case InputMode::pull_up:
                default:                   function_mode = PinDriver::FunctionMode::pull_up;   break;
            }

            PinDriver::set_mode(m_pin_name, function_mode,
                                            PinDriver::PinMux::gpio,
                                            SlewRate::fast,
                                            config.passive_filter,
                                            PinDriver::OpenDrain::disable,
                                            DriveStrength::low,
                                            config.lock_register);

            initialize(Direction::input, 0);
        }

        // Set output pin mode
        void set_mode(const OutputModeConfig& config)
        {
            // Exclude NC
            assert(m_pin_name != PinDriver::Name::nc);

            uint8_t              pin_value;
            PinDriver::OpenDrain open_drain;

            switch(config.output_mode)
            {
                case OutputMode::push_pull_low:  pin_value = 0; open_drain = PinDriver::OpenDrain::disable; break;
                case OutputMode::open_drain_low: pin_value = 0; open_drain = PinDriver::OpenDrain::enable;  break;
                case OutputMode::open_drain_hiz: pin_value = 1; open_drain = PinDriver::OpenDrain::enable;  break;
                case OutputMode::push_pull_high:
                default:                         pin_value = 1; open_drain = PinDriver::OpenDrain::disable; break;
            }

            PinDriver::set_mode(m_pin_name, PinDriver::FunctionMode::hiz,
                                            PinDriver::PinMux::gpio,
                                            config.slew_rate,
                                            PassiveFilter::disable,
                                            open_drain,
                                            config.drive_strength,
                                            config.lock_register);

            initialize(Direction::output, pin_value);
        }

        PinDriver::Name get_pin_name() const { return m_pin_name; }

        // -------- READ / WRITE ----------------------------------------------

        uint32_t read() const
        {
            if(m_gpio_base != nullptr)
            {
                return GPIO_PinRead(m_gpio_base, m_pin);
            }

            return 0;
        }

        void write(const uint32_t value)
        {
            if(m_gpio_base != nullptr)
            {
                GPIO_PinWrite(m_gpio_base, m_pin, value);
            }
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
            const uint32_t port_index = PinDriver::get_port_index(m_pin_name);
                           m_pin      = PinDriver::get_pin_bit(m_pin_name);

            switch(port_index)
            {
                case 0: m_gpio_base = GPIOA; break;
                case 1: m_gpio_base = GPIOB; break;
                case 2: m_gpio_base = GPIOC; break;
                case 3: m_gpio_base = GPIOD; break;
                case 4: m_gpio_base = GPIOE; break;
            }
        }

        void initialize(const Direction direction, const uint8_t output_value)
        {
            const gpio_pin_config_t pin_config =
            {
                 static_cast<gpio_pin_direction_t>(direction),
                 output_value
            };

            GPIO_PinInit(m_gpio_base, m_pin, &pin_config);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const PinDriver::Name m_pin_name;
        GPIO_Type*            m_gpio_base { nullptr };
        uint32_t              m_pin { 0 };
};




} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV5X_GPIO_HPP

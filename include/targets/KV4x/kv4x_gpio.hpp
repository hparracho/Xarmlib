// ----------------------------------------------------------------------------
// @file    kv4x_gpio.hpp
// @brief   Kinetis KV4x GPIO class.
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

#ifndef __XARMLIB_TARGETS_KV4X_GPIO_HPP
#define __XARMLIB_TARGETS_KV4X_GPIO_HPP

#include "targets/KV4x/kv4x_pin.hpp"
#include "fsl_gpio.h"

namespace xarmlib
{
namespace targets
{
namespace kv4x
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
            PULL_UP
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

        using SlewRate      = PinDriver::SlewRate;
        using PassiveFilter = PinDriver::PassiveFilter;
        using DriveStrength = PinDriver::DriveStrength;
        using LockRegister  = PinDriver::LockRegister;

        struct InputModeConfig
        {
            InputMode     input_mode     = InputMode::PULL_UP;
            PassiveFilter passive_filter = PassiveFilter::DISABLE;
            LockRegister  lock_register  = LockRegister::UNLOCK;
        };

        struct OutputModeConfig
        {
            OutputMode    output_mode    = OutputMode::PUSH_PULL_LOW;
            SlewRate      slew_rate      = SlewRate::FAST;
            DriveStrength drive_strength = DriveStrength::LOW;
            LockRegister  lock_register  = LockRegister::UNLOCK;
        };

        struct InputModeTrueOpenDrainConfig
        {
            // input mode: HIZ
            LockRegister lock_register = LockRegister::UNLOCK;
        };

        struct OutputModeTrueOpenDrainConfig
        {
            OutputModeTrueOpenDrain output_mode    = OutputModeTrueOpenDrain::LOW;
            SlewRate                slew_rate      = SlewRate::FAST;
            DriveStrength           drive_strength = DriveStrength::LOW;
            LockRegister            lock_register  = LockRegister::UNLOCK;
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

        // True open-drain input pin constructor (only available on PC_6 and PC_7)
        GpioDriver(const PinDriver::Name pin_name, const InputModeTrueOpenDrainConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name == PinDriver::Name::PC_6 || pin_name == PinDriver::Name::PC_7)
            {
                config_port();
                set_mode(config);
            }
        }

        // True open-drain output pin constructor (only available on PC_6 and PC_7)
        GpioDriver(const PinDriver::Name pin_name, const OutputModeTrueOpenDrainConfig& config) : m_pin_name { pin_name }
        {
            if(pin_name == PinDriver::Name::PC_6 || pin_name == PinDriver::Name::PC_7)
            {
                config_port();
                set_mode(config);
            }
        }

        // -------- CONFIGURATION ---------------------------------------------

        // Set normal input pin mode
        void set_mode(const InputModeConfig& config)
        {
            // Exclude NC
            assert(m_pin_name != PinDriver::Name::NC);

            // Exclude true open-drain pins
            assert(m_pin_name != PinDriver::Name::PC_6 && m_pin_name != PinDriver::Name::PC_7);

            PinDriver::FunctionMode function_mode;

            switch(config.input_mode)
            {
                case InputMode::HIZ:       function_mode = PinDriver::FunctionMode::HIZ;       break;
                case InputMode::PULL_DOWN: function_mode = PinDriver::FunctionMode::PULL_DOWN; break;
                case InputMode::PULL_UP:
                default:                   function_mode = PinDriver::FunctionMode::PULL_UP;   break;
            }

            PinDriver::set_mode(m_pin_name, function_mode,
                                            PinDriver::PinMux::GPIO,
                                            SlewRate::FAST,
                                            config.passive_filter,
                                            PinDriver::OpenDrain::DISABLE,
                                            DriveStrength::LOW,
                                            config.lock_register);

            initialize(Direction::INPUT, 0);
        }

        // Set normal output pin mode
        void set_mode(const OutputModeConfig& config)
        {
            // Exclude NC
            assert(m_pin_name != PinDriver::Name::NC);

            // Exclude true open-drain pins
            assert(m_pin_name != PinDriver::Name::PC_6 && m_pin_name != PinDriver::Name::PC_7);

            uint8_t              pin_value;
            PinDriver::OpenDrain open_drain;

            switch(config.output_mode)
            {
                case OutputMode::PUSH_PULL_LOW:  pin_value = 0; open_drain = PinDriver::OpenDrain::DISABLE; break;
                case OutputMode::OPEN_DRAIN_LOW: pin_value = 0; open_drain = PinDriver::OpenDrain::ENABLE;  break;
                case OutputMode::OPEN_DRAIN_HIZ: pin_value = 1; open_drain = PinDriver::OpenDrain::ENABLE;  break;
                case OutputMode::PUSH_PULL_HIGH:
                default:                         pin_value = 1; open_drain = PinDriver::OpenDrain::DISABLE; break;
            }

            PinDriver::set_mode(m_pin_name, PinDriver::FunctionMode::HIZ,
                                            PinDriver::PinMux::GPIO,
                                            config.slew_rate,
                                            PassiveFilter::DISABLE,
                                            open_drain,
                                            config.drive_strength,
                                            config.lock_register);

            initialize(Direction::OUTPUT, pin_value);
        }

        // Set true open-drain input pin mode (only available on PC_6 and PC_7)
        void set_mode(const InputModeTrueOpenDrainConfig& config)
        {
            // Available only on true open-drain pins
            assert(m_pin_name == PinDriver::Name::PC_6 || m_pin_name == PinDriver::Name::PC_7);

            PinDriver::set_mode(m_pin_name, PinDriver::PinMux::GPIO,
                                            SlewRate::FAST,
                                            PassiveFilter::DISABLE,
                                            DriveStrength::LOW,
                                            config.lock_register);

            initialize(Direction::INPUT, 0);
        }

        // Set true open-drain output pin mode (only available on PC_6 and PC_7)
        void set_mode(const OutputModeTrueOpenDrainConfig& config)
        {
            // Available only on true open-drain pins
            assert(m_pin_name == PinDriver::Name::PC_6 || m_pin_name == PinDriver::Name::PC_7);

            PinDriver::set_mode(m_pin_name, PinDriver::PinMux::GPIO,
                                            config.slew_rate,
                                            PassiveFilter::DISABLE,
                                            config.drive_strength,
                                            config.lock_register);

            initialize(Direction::OUTPUT, (config.output_mode == OutputModeTrueOpenDrain::LOW) ? 0 : 1);
        }

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
            INPUT = 0,
            OUTPUT
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




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_GPIO_HPP

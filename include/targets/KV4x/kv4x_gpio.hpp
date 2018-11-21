// ----------------------------------------------------------------------------
// @file    kv4x_gpio.hpp
// @brief   Kinetis KV4x GPIO class.
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

        using SlewRate      = Pin::SlewRate;
        using PassiveFilter = Pin::PassiveFilter;
        using DriveStrength = Pin::DriveStrength;
        using LockRegister  = Pin::LockRegister;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTORS ----------------------------------------------

        // Default constructor (assign a NC pin)
        Gpio() : m_pin_name { Pin::Name::NC }
        {}

        // Normal input pin constructor
        Gpio(const Pin::Name     pin_name,
             const InputMode     input_mode,
             const PassiveFilter passive_filter,
             const LockRegister  lock_register) : m_pin_name {pin_name}
        {
            if(pin_name != Pin::Name::NC)
            {
                set_mode(input_mode, passive_filter, lock_register);
            }
        }

        // Normal output pin constructor
        Gpio(const Pin::Name     pin_name,
             const OutputMode    output_mode,
             const SlewRate      slew_rate,
             const DriveStrength drive_strength,
             const LockRegister  lock_register) : m_pin_name {pin_name}
        {
            if(pin_name != Pin::Name::NC)
            {
                set_mode(output_mode, slew_rate, drive_strength, lock_register);
            }
        }

        // True open-drain input pin constructor (only available on PC_6 and PC_7)
        Gpio(const Pin::Name              pin_name,
             const InputModeTrueOpenDrain input_mode,
             const LockRegister           lock_register) : m_pin_name {pin_name}
        {
            if(pin_name == Pin::Name::PC_6 || pin_name == Pin::Name::PC_7)
            {
                set_mode(input_mode, lock_register);
            }
        }

        // True open-drain output pin constructor (only available on PC_6 and PC_7)
        Gpio(const Pin::Name               pin_name,
             const OutputModeTrueOpenDrain output_mode,
             const SlewRate                slew_rate,
             const DriveStrength           drive_strength,
             const LockRegister            lock_register) : m_pin_name {pin_name}
        {
            if(pin_name == Pin::Name::PC_6 || pin_name == Pin::Name::PC_7)
            {
                set_mode(output_mode, slew_rate, drive_strength, lock_register);
            }
        }

        // -------- CONFIGURATION ---------------------------------------------

        // Set normal input pin mode
        void set_mode(const InputMode     input_mode,
                      const PassiveFilter passive_filter = PassiveFilter::kPORT_PassiveFilterDisable,
                      const LockRegister  lock_register  = LockRegister::kPORT_UnlockRegister)
        {
            // Exclude NC
            assert(m_pin_name != Pin::Name::NC);

            // Exclude true open-drain pins
            assert(m_pin_name != Pin::Name::PC_6 && m_pin_name != Pin::Name::PC_7);

            Pin::FunctionMode function_mode;

            switch(input_mode)
            {
                case InputMode::HIZ:       function_mode = Pin::FunctionMode::kPORT_PullDisable; break;
                case InputMode::PULL_DOWN: function_mode = Pin::FunctionMode::kPORT_PullDown;    break;
                case InputMode::PULL_UP:
                default:                   function_mode = Pin::FunctionMode::kPORT_PullUp;      break;
            }

            Pin::set_mode(m_pin_name, function_mode,
                                      Pin::PinMuxControl::kPORT_MuxAsGpio,
                                      SlewRate::kPORT_FastSlewRate,
                                      passive_filter,
                                      Pin::OpenDrain::kPORT_OpenDrainDisable,
                                      DriveStrength::kPORT_LowDriveStrength,
                                      lock_register);

            initialize(Direction::kGPIO_DigitalInput, 0);
        }

        // Set normal output pin mode
        void set_mode(const OutputMode    output_mode,
                      const SlewRate      slew_rate      = SlewRate::kPORT_FastSlewRate,
                      const DriveStrength drive_strength = DriveStrength::kPORT_LowDriveStrength,
                      const LockRegister  lock_register  = LockRegister::kPORT_UnlockRegister)
        {
            // Exclude NC
            assert(m_pin_name != Pin::Name::NC);

            // Exclude true open-drain pins
            assert(m_pin_name != Pin::Name::PC_6 && m_pin_name != Pin::Name::PC_7);

            uint32_t       pin_value;
            Pin::OpenDrain open_drain;

            switch(output_mode)
            {
                case OutputMode::PUSH_PULL_LOW:  pin_value = 0; open_drain = Pin::OpenDrain::kPORT_OpenDrainDisable; break;
                case OutputMode::OPEN_DRAIN_LOW: pin_value = 0; open_drain = Pin::OpenDrain::kPORT_OpenDrainEnable;  break;
                case OutputMode::OPEN_DRAIN_HIZ: pin_value = 1; open_drain = Pin::OpenDrain::kPORT_OpenDrainEnable;  break;
                case OutputMode::PUSH_PULL_HIGH:
                default:                         pin_value = 1; open_drain = Pin::OpenDrain::kPORT_OpenDrainDisable; break;
            }

            Pin::set_mode(m_pin_name, Pin::FunctionMode::kPORT_PullDisable,
                                      Pin::PinMuxControl::kPORT_MuxAsGpio,
                                      slew_rate,
                                      PassiveFilter::kPORT_PassiveFilterDisable,
                                      open_drain,
                                      drive_strength,
                                      lock_register);

            initialize(Direction::kGPIO_DigitalOutput, pin_value);
        }

        // Set true open-drain input pin mode (only available on PC_6 and PC_7)
        void set_mode(const InputModeTrueOpenDrain input_mode,
                      const LockRegister           lock_register = LockRegister::kPORT_UnlockRegister)
        {
            (void)input_mode; // Input mode only used to identify the type of pin

            // Available only on true open-drain pins
            assert(m_pin_name == Pin::Name::PC_6 || m_pin_name == Pin::Name::PC_7);

            Pin::set_mode(m_pin_name, Pin::PinMuxControl::kPORT_MuxAsGpio,
                                      SlewRate::kPORT_FastSlewRate,
                                      PassiveFilter::kPORT_PassiveFilterDisable,
                                      DriveStrength::kPORT_LowDriveStrength,
                                      lock_register);

            initialize(Direction::kGPIO_DigitalInput, 0);
        }

        // Set true open-drain output pin mode (only available on PC_6 and PC_7)
        void set_mode(const OutputModeTrueOpenDrain output_mode,
                      const SlewRate                slew_rate      = SlewRate::kPORT_FastSlewRate,
                      const DriveStrength           drive_strength = DriveStrength::kPORT_LowDriveStrength,
                      const LockRegister            lock_register  = LockRegister::kPORT_UnlockRegister)
        {
            // Available only on true open-drain pins
            assert(m_pin_name == Pin::Name::PC_6 || m_pin_name == Pin::Name::PC_7);

            Pin::set_mode(m_pin_name, Pin::PinMuxControl::kPORT_MuxAsGpio,
                                      slew_rate,
                                      PassiveFilter::kPORT_PassiveFilterDisable,
                                      drive_strength,
                                      lock_register);

            initialize(Direction::kGPIO_DigitalOutput, (output_mode == OutputModeTrueOpenDrain::LOW) ? 0 : 1);
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
        using Direction = _gpio_pin_direction;

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void initialize(const Direction direction, const uint32_t output_value)
        {
            const uint32_t port_index = Pin::get_port_index(m_pin_name);
                           m_pin      = Pin::get_pin_bit(m_pin_name);

            switch(port_index)
            {
                case 0: m_gpio_base = GPIOA; break;
                case 1: m_gpio_base = GPIOB; break;
                case 2: m_gpio_base = GPIOC; break;
                case 3: m_gpio_base = GPIOD; break;
                case 4: m_gpio_base = GPIOE; break;
            }

            const gpio_pin_config_t pin_config =
            {
                 direction,
                 output_value
            };

            GPIO_PinInit(m_gpio_base, m_pin, &pin_config);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const Pin::Name m_pin_name;
        GPIO_Type*      m_gpio_base { nullptr };
        uint32_t        m_pin { 0 };
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_GPIO_HPP

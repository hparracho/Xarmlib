// ----------------------------------------------------------------------------
// @file    kv4x_port.hpp
// @brief   Kinetis KV4x port class.
// @date    26 November 2018
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

#ifndef __XARMLIB_TARGETS_KV4X_PORT_HPP
#define __XARMLIB_TARGETS_KV4X_PORT_HPP

#include "targets/KV4x/kv4x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class Port
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Port names (all packages have 5 ports)
        enum class Name
        {
            A = 0,
            B,
            C,
            D,
            E,
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void set_direction(const Name port)
        {
            GPIO_Type* gpio_base = get_gpio_base(port);

            // NOTE: Ones configure as outputs
            gpio_base->PDDR = 0xFFFFFFFF;
        }

        static void clear_direction(const Name port)
        {
            GPIO_Type* gpio_base = get_gpio_base(port);

            // NOTE: Zeros configure as inputs
            gpio_base->PDDR = 0;
        }

        static void set_direction(const Name port, const uint32_t mask)
        {
            GPIO_Type* gpio_base = get_gpio_base(port);

            // NOTE: Ones configure as outputs
            gpio_base->PDDR |= mask;
        }

        static void clear_direction(const Name port, const uint32_t mask)
        {
            GPIO_Type* gpio_base = get_gpio_base(port);

            // NOTE: Zeros configure as inputs
            gpio_base->PDDR &= ~mask;
        }

        static void set_direction(const Pin::Name pin)
        {
            assert(pin >= Pin::Name::NC);

            GPIO_Type* gpio_base = get_gpio_base(pin);

            // NOTE: Ones configure as outputs
            gpio_base->PDDR |= 1UL << Pin::get_pin_bit(pin);
        }

        static void clear_direction(const Pin::Name pin)
        {
            assert(pin >= Pin::Name::NC);

            GPIO_Type* gpio_base = get_gpio_base(pin);

            // NOTE: Zeros configure as inputs
            gpio_base->PDDR &= ~(1UL << Pin::get_pin_bit(pin));
        }

        static void write_direction(const Name port, const uint32_t mask, const uint32_t value)
        {
            GPIO_Type* gpio_base = get_gpio_base(port);

            gpio_base->PDDR = (gpio_base->PDDR & ~mask) | (value & mask);
        }

        static uint32_t read(const Name port)
        {
            const GPIO_Type* gpio_base = get_gpio_base(port);

            return gpio_base->PDIR;
        }

        static uint32_t read(const Name port, const uint32_t mask)
        {
            const GPIO_Type* gpio_base = get_gpio_base(port);

            return gpio_base->PDIR & mask;
        }

        static void write(const Name port, const uint32_t value)
        {
            GPIO_Type* gpio_base = get_gpio_base(port);

            gpio_base->PDOR = value;
        }

        static void write(const Name port, const uint32_t mask, const uint32_t value)
        {
            GPIO_Type* gpio_base = get_gpio_base(port);

            gpio_base->PDOR = (gpio_base->PDOR & ~mask) | (value & mask);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr GPIO_Type* get_gpio_base(const Name port)
        {
            return get_gpio_base(static_cast<std::size_t>(port));
        }

        static constexpr GPIO_Type* get_gpio_base(const Pin::Name pin_name)
        {
            return get_gpio_base(Pin::get_port_index(pin_name));
        }

        static constexpr GPIO_Type* get_gpio_base(const std::size_t port_index)
        {
            switch(port_index)
            {
                case 0:  return GPIOA; break;
                case 1:  return GPIOB; break;
                case 2:  return GPIOC; break;
                case 3:  return GPIOD; break;
                case 4:  return GPIOE; break;
                default: return 0;     break;
            }
        }
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_PORT_HPP

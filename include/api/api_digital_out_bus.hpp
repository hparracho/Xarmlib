// ----------------------------------------------------------------------------
// @file    api_digital_out_bus.hpp
// @brief   API digital output bus class.
// @date    27 August 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_API_DIGITAL_OUT_BUS_HPP
#define __XARMLIB_API_DIGITAL_OUT_BUS_HPP

#include "api/api_pin_bus.hpp"
#include "hal/hal_gpio.hpp"
#include "core/non_copyable.hpp"

#include <dynarray>
#include <memory>

namespace xarmlib
{




class DigitalOutBus : private NonCopyable<DigitalOutBus>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        DigitalOutBus(const PinNameBus& pin_name_bus, const hal::Gpio::OutputModeConfig& config) : m_bus(pin_name_bus.get_size())
        {
            std::size_t index = 0;
            for(auto pin_name : pin_name_bus)
            {
                m_bus[index++] = std::make_unique<hal::Gpio>(pin_name, config);
            }
        }

        DigitalOutBus(const PinNameBus& pin_name_bus, const hal::Gpio::OutputModeTrueOpenDrainConfig& config) : m_bus(pin_name_bus.get_size())
        {
            std::size_t index = 0;
            for(auto pin_name : pin_name_bus)
            {
                m_bus[index++] = std::make_unique<hal::Gpio>(pin_name, config);
            }
        }

        // -------- READ ------------------------------------------------------

        uint32_t read() const
        {
            uint32_t value = 0;

            for(std::size_t pin = 0; pin < m_bus.size(); ++pin)
            {
                value |= m_bus[pin]->read() << pin;
            }

            return value;
        }

        operator uint32_t () const
        {
            return read();
        }

        // Read negated value operator
        uint32_t operator ! () const
        {
            return !read();
        }

        // -------- WRITE -----------------------------------------------------

        void write(const uint32_t value)
        {
            for(std::size_t pin = 0; pin < m_bus.size(); ++pin)
            {
                m_bus[pin]->write(value & (1 << pin));
            }
        }

        DigitalOutBus& operator = (const uint32_t value)
        {
            write(value);
            return (*this);
        }

        DigitalOutBus& operator = (const DigitalOutBus &rhs)
        {
            write(rhs.read());
            return (*this);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::dynarray<std::unique_ptr<hal::Gpio>> m_bus;
};




} // namespace xarmlib

#endif // __XARMLIB_API_DIGITAL_OUT_BUS_HPP

// ----------------------------------------------------------------------------
// @file    api_gpio_source.hpp
// @brief   API GPIO source class.
// @date    17 July 2018
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

#ifndef __XARMLIB_API_GPIO_SOURCE_HPP
#define __XARMLIB_API_GPIO_SOURCE_HPP

#include "api/api_pin_source.hpp"
#include "hal/hal_port.hpp"

#include <array>

namespace xarmlib
{




class GpioSource : public PinSource
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

#ifdef DEBUG
        GpioSource()
        {
            static bool initialized = false;
            assert(initialized == false);
            initialized = true;
        }
#endif

        // Get the handler that is intended to be used as a pin source reader handler of the PinScanner class
        PinScanner::PinSourceHandler get_pin_source_handler() override
        {
            return PinScanner::PinSourceHandler::create<GpioSource, &GpioSource::pin_source_handler>(this);
        }

        std::size_t get_port_count() const override
        {
            return TARGET_PORT_COUNT;
        }

        uint32_t get_current_read(const std::size_t port_index) const override
        {
            assert(port_index < m_current_reads.size());

            return m_current_reads[port_index];
        }

        uint32_t get_current_read_bit(const std::size_t port_index, const std::size_t pin_bit) const override
        {
            assert(port_index < m_current_reads.size());
            assert(pin_bit < 32);

            return m_current_reads[port_index] & (1UL << pin_bit);
        }

     private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void pin_source_handler()
        {
            for(std::size_t port_index = 0; port_index < TARGET_PORT_COUNT; ++port_index)
            {
                m_current_reads[port_index] = Port::read(static_cast<Port::Name>(port_index));
            }
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::array<uint32_t, TARGET_PORT_COUNT> m_current_reads { 0 };
};




} // namespace xarmlib

#endif // __XARMLIB_API_GPIO_SOURCE_HPP

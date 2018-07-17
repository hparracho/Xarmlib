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

        uint32_t get_read(const std::size_t port_index) const override
        {
            assert(port_index < m_reads.size());

            return m_reads[port_index];
        }

        uint32_t get_read_bit(const std::size_t port_index, const std::size_t pin_bit) const override
        {
            assert(port_index < m_reads.size());
            assert(pin_bit < 32);

            return m_reads[port_index] & (1UL << pin_bit);
        }

        uint32_t get_output_bit(const std::size_t port_index, const std::size_t pin_bit) const override
        {
            assert(port_index < m_outputs.size());
            assert(pin_bit < 32);

            return m_outputs[port_index] & (1UL << pin_bit);
        }

        void set_output_bit(const std::size_t port_index, const std::size_t pin_bit, const uint32_t value) override
        {
            assert(port_index < m_outputs.size());
            assert(pin_bit < 32);

            const uint32_t pin_mask = (1UL << pin_bit);

            m_outputs[port_index] = (m_outputs[port_index] & (~pin_mask)) | (value & pin_mask);

            m_outputs_mask[port_index] |= pin_mask;
        }

        static constexpr int8_t get_port_index(const Pin::Name pin_name)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_name) >> 5);    // (pin_name / 32)
        }

        static constexpr int8_t get_pin_bit(const Pin::Name pin_name)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_name) & 0x1F);  // (pin_name % 32)
        }

     private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void pin_source_handler()
        {
            for(std::size_t port_index = 0; port_index < TARGET_PORT_COUNT; ++port_index)
            {
                const Port::Name port_name = static_cast<Port::Name>(port_index);

                // Set output direction for the specified mask
                Port::set_direction(port_name, m_outputs_mask[port_index]);

                Port::write(port_name, m_outputs[port_index]);

                // Back to input direction
                Port::clear_direction(port_name, m_outputs_mask[port_index]);

                // Clear mask
                m_outputs_mask[port_index] = 0;

                m_reads[port_index] = Port::read(port_name);
            }
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::array<uint32_t, TARGET_PORT_COUNT> m_outputs { 0xFFFFFFFF };
        std::array<uint32_t, TARGET_PORT_COUNT> m_outputs_mask { 0 };
        std::array<uint32_t, TARGET_PORT_COUNT> m_reads { 0 };
};




} // namespace xarmlib

#endif // __XARMLIB_API_GPIO_SOURCE_HPP

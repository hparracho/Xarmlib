// ----------------------------------------------------------------------------
// @file    api_port_in.hpp
// @brief   API port input class.
// @date    22 June 2018
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

#ifndef __XARMLIB_API_PORT_IN_HPP
#define __XARMLIB_API_PORT_IN_HPP

#include "hal/hal_gpio.hpp"
#include "hal/hal_port.hpp"

namespace xarmlib
{




class PortIn
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void config_pin(const Pin::Name             pin_name,
                        const Gpio::InputMode       input_mode,
                        const Gpio::InputFilter     input_filter     = Gpio::InputFilter::BYPASS,
                        const Gpio::InputInvert     input_invert     = Gpio::InputInvert::NORMAL,
                        const Gpio::InputHysteresis input_hysteresis = Gpio::InputHysteresis::ENABLE)
        {
            Gpio local_gpio(pin_name, input_mode, input_filter, input_invert, input_hysteresis);

            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            m_pins_mask[port_index] |= 1UL << pin_index;
        }

        void config_pin(const Pin::Name                    pin_name,
                        const Gpio::InputModeTrueOpenDrain input_mode,
                        const Gpio::InputFilter            input_filter = Gpio::InputFilter::BYPASS,
                        const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
        {
            Gpio local_gpio(pin_name, input_mode, input_filter, input_invert);

            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            m_pins_mask[port_index] |= 1UL << pin_index;
        }

        void remove_pin(const Pin::Name pin_name)
        {
            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

            m_pins_mask[port_index] &= ~(1UL << pin_index);
        }

        void read(std::array<uint32_t, Port::COUNT>& ports)
        {
            for(std::size_t port_index = 0; port_index < Port::COUNT; port_index++)
            {
                ports[port_index] = Port::read(static_cast<Port::Name>(port_index));
            }
        }

        uint32_t current_debounced(const Pin::Name pin_name)
        {
            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

            const bool current_pin_debounced = (~m_current_debounced[port_index]) & (1 << pin_index);

            return static_cast<uint32_t>(current_pin_debounced);
        }

        uint32_t new_debounced(const Pin::Name pin_name)
        {
            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

            const bool new_pin_debounced = (m_current_debounced[port_index] & (1 << pin_index)) ^ (m_last_debounced[port_index] & (1 << pin_index));

            return static_cast<uint32_t>(new_pin_debounced);
        }

        uint32_t sampling(const Pin::Name pin_name)
        {
                const std::size_t port_index = get_port_index(pin_name);
                const std::size_t pin_index  = get_pin_index(pin_name);

                assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                const bool pin_sampling = m_sampling[port_index] & (1 << pin_index);

                return static_cast<uint32_t>(pin_sampling);
        }

        // Set the value of the supplied IOCON clock divider (used by input filters)
        // NOTE: The input filter source (where the divider is applied) is the MAIN clock
        static void set_input_filter_clock_divider(const Gpio::InputFilterClockDivider clock_div, const uint8_t div)
        {
            Gpio::set_input_filter_clock_divider(clock_div, div);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr std::size_t get_port_index(const Pin::Name pin_name)
        {
            assert(pin_name != Pin::Name::NC);

            return static_cast<std::size_t>(static_cast<uint32_t>(pin_name) >> 5);  // divide by 32
        }

        static constexpr std::size_t get_pin_index(const Pin::Name pin_name)
        {
            assert(pin_name != Pin::Name::NC);

            return static_cast<std::size_t>(static_cast<uint32_t>(pin_name) % 32);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::array<uint32_t, Port::COUNT>    m_pins_mask;

        std::array<uint32_t, Port::COUNT>    m_last_read_pins;

        std::array<uint32_t, Port::COUNT>    m_current_debounced;
        std::array<uint32_t, Port::COUNT>    m_last_debounced;
        std::array<uint32_t, Port::COUNT>    m_sampling;                 // Inputs that are being sampled and not yet filtered
};




} // namespace xarmlib

#endif // __XARMLIB_API_PORT_IN_HPP

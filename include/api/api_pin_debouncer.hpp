// ----------------------------------------------------------------------------
// @file    api_pin_debouncer.hpp
// @brief   API MCU pin debouncer class.
// @date    6 July 2018
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

#ifndef __XARMLIB_API_PIN_DEBOUNCER_HPP
#define __XARMLIB_API_PIN_DEBOUNCER_HPP

#include "api/api_input_debouncer.hpp"
#include "api/api_input_scanner.hpp"
#include "hal/hal_port.hpp"

#include <array>

namespace xarmlib
{




class PinDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void assign_pins(const PinNameBus&               pin_name_bus,
                                const std::chrono::milliseconds filter_ms_low,
                                const std::chrono::milliseconds filter_ms_high,
                                const Gpio::InputMode           input_mode,
                                const Gpio::InputInvert         input_invert     = Gpio::InputInvert::NORMAL,
                                const Gpio::InputHysteresis     input_hysteresis = Gpio::InputHysteresis::ENABLE)
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert, input_hysteresis);
            }

            config_pins(pin_name_bus, filter_ms_low, filter_ms_high);
        }

        static void assign_pins(const PinNameBus&                  pin_name_bus,
                                const std::chrono::milliseconds    filter_ms_low,
                                const std::chrono::milliseconds    filter_ms_high,
                                const Gpio::InputModeTrueOpenDrain input_mode,
                                const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert);
            }

            config_pins(pin_name_bus, filter_ms_low, filter_ms_high);
        }

        // Debounce handler that is intended to be used as an input handler of the InputScanner class
        static bool debounce_handler()
        {
            for(std::size_t port_index = 0; port_index < Port::COUNT; ++port_index)
            {
                m_ports[port_index].current_read = Port::read(static_cast<Port::Name>(port_index));
            }

            return InputDebouncer::debounce((gsl::span<InputDebouncer::Input>{ m_pins }).first(m_assigned_pin_count), m_ports);
        }

        static uint32_t get_filtered(const PinNameBus& pin_name_bus)
        {
            uint32_t filtered = 0;
            std::size_t filtered_shift_index = 0;

            for(auto pin_name : pin_name_bus)
            {
                const int8_t port_index = get_port_index(pin_name);
                const int8_t pin_bit = get_pin_bit(pin_name);
                const uint32_t pin_mask = (1UL << pin_bit);

                // Assertion fails if the pin wasn't previously assigned to the debouncer list
                assert((m_ports[port_index].enabled & pin_mask) != 0);

                const bool filtered_bit = (m_ports[port_index].filtered & pin_mask) != 0;

                filtered |= static_cast<uint32_t>(filtered_bit) << filtered_shift_index++;
            }

            return filtered;
        }

        static uint32_t get_sampling(const PinNameBus& pin_name_bus)
        {
            uint32_t sampling = 0;
            std::size_t sampling_shift_index = 0;

            for(auto pin_name : pin_name_bus)
            {
                const int8_t port_index = get_port_index(pin_name);
                const int8_t pin_bit = get_pin_bit(pin_name);
                const uint32_t pin_mask = (1UL << pin_bit);

                // Assertion fails if the pin wasn't previously assigned to the debouncer list
                assert((m_ports[port_index].enabled & pin_mask) != 0);

                const bool sampling_bit = (m_ports[port_index].sampling & pin_mask) != 0;

                sampling |= static_cast<uint32_t>(sampling_bit) << sampling_shift_index++;
            }

            return sampling;
        }

     private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void config_pins(const PinNameBus&                pin_name_bus,
                                const std::chrono::milliseconds& filter_low_ms,
                                const std::chrono::milliseconds& filter_high_ms)
        {
            assert(filter_low_ms.count()  > 0 && filter_low_ms.count()  <= std::numeric_limits<int16_t>::max());
            assert(filter_high_ms.count() > 0 && filter_high_ms.count() <= std::numeric_limits<int16_t>::max());

            const bool resume = InputScanner::is_running();

            InputScanner::stop();

            for(auto pin_name : pin_name_bus)
            {
                config_pin(pin_name, static_cast<int16_t>(filter_low_ms.count()),
                                     static_cast<int16_t>(filter_high_ms.count()));
            }

            if(resume == true)
            {
                InputScanner::resume();
            }
        }

        static void config_pin(const Pin::Name pin_name, const int16_t filter_low_ms, const int16_t filter_high_ms)
        {
            const int8_t port_index = get_port_index(pin_name);
            const int8_t pin_bit = get_pin_bit(pin_name);

            std::size_t assigned_pin = 0;
            bool found = false;

            while(found == false && assigned_pin < m_assigned_pin_count)
            {
                if(m_pins[assigned_pin].port_index == port_index && m_pins[assigned_pin].pin_bit == pin_bit)
                {
                    found = true;
                }
                else
                {
                    assigned_pin++;
                }
            }

            if(found == false)
            {
                m_assigned_pin_count = assigned_pin + 1;
                assert(m_assigned_pin_count <= m_pins.size());

                m_ports[port_index].enabled |= 1UL << pin_bit;
            }

            m_pins[assigned_pin] = {port_index, pin_bit, filter_low_ms, filter_high_ms, 0 };
        }

        static constexpr int8_t get_port_index(const Pin::Name pin_name)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_name) >> 5);    // (pin_name / 32)
        }

        static constexpr int8_t get_pin_bit(const Pin::Name pin_name)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_name) & 0x1F);  // (pin_name % 32)
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        static std::array<InputDebouncer::PortMask, Port::COUNT> m_ports;
        static std::dynarray<InputDebouncer::Input>              m_pins;
        static std::size_t                                       m_assigned_pin_count;
};




} // namespace xarmlib

#endif // __XARMLIB_API_PIN_DEBOUNCER_HPP

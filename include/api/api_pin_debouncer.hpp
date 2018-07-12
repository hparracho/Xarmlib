// ----------------------------------------------------------------------------
// @file    api_pin_debouncer.hpp
// @brief   API MCU pin debouncer class.
// @date    12 July 2018
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
                                const std::chrono::milliseconds low_filter,
                                const std::chrono::milliseconds high_filter,
                                const Gpio::InputMode           input_mode,
                                const Gpio::InputInvert         input_invert     = Gpio::InputInvert::NORMAL,
                                const Gpio::InputHysteresis     input_hysteresis = Gpio::InputHysteresis::ENABLE)
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert, input_hysteresis);
            }

            config_pins(pin_name_bus, low_filter, high_filter);
        }

        static void assign_pins(const PinNameBus&                  pin_name_bus,
                                const std::chrono::milliseconds    low_filter,
                                const std::chrono::milliseconds    high_filter,
                                const Gpio::InputModeTrueOpenDrain input_mode,
                                const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert);
            }

            config_pins(pin_name_bus, low_filter, high_filter);
        }

        // Get the handler that is intended to be used as an input handler of the InputScanner class
        static InputScanner::InputHandler get_input_handler()
        {
            return InputScanner::InputHandler::create<&input_handler>();
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
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        struct Input
        {
            int8_t      port_index     { -1 };  // Input's port index
            int8_t      pin_bit        { -1 };  // Input's bit within a port
            int16_t     low_filter_ms  { 0 };   // Milliseconds that a pin must be steady at low level to be accepted as filtered (debounced)
            int16_t     high_filter_ms { 0 };   // Milliseconds that a pin must be steady at high level to be accepted as filtered (debounced)
            int16_t     counter_ms     { 0 };   // Filter time counter
        };

        struct PortMask
        {
            uint32_t    current_read { 0 };     // Current iteration inputs
            uint32_t    last_read    { 0 };     // Previous iteration inputs
            uint32_t    filtered     { 0 };     // Filtered inputs
            uint32_t    sampling     { 0 };     // Inputs that are being sampled and not yet filtered
            uint32_t    enabled      { 0 };     // Enabled inputs
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void config_pins(const PinNameBus&                pin_name_bus,
                                const std::chrono::milliseconds& low_filter,
                                const std::chrono::milliseconds& high_filter)
        {
            assert(low_filter.count()  > 0 && low_filter.count()  <= std::numeric_limits<int16_t>::max());
            assert(high_filter.count() > 0 && high_filter.count() <= std::numeric_limits<int16_t>::max());

            const bool resume = InputScanner::is_running();

            InputScanner::stop();

            for(auto pin_name : pin_name_bus)
            {
                config_pin(pin_name, static_cast<int16_t>(low_filter.count()),
                                     static_cast<int16_t>(high_filter.count()));
            }

            if(resume == true)
            {
                InputScanner::resume();
            }
        }

        static void config_pin(const Pin::Name pin_name, const int16_t low_filter_ms, const int16_t high_filter_ms)
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

            m_pins[assigned_pin] = {port_index, pin_bit, low_filter_ms, high_filter_ms, 0 };
        }

        // Handler that is intended to be used as an input handler of the InputScanner class
        static bool input_handler()
        {
            for(std::size_t port_index = 0; port_index < Port::COUNT; ++port_index)
            {
                m_ports[port_index].current_read = Port::read(static_cast<Port::Name>(port_index));
            }

            if(m_is_first_debounce == true)
            {
                for(auto& port : m_ports)
                {
                    port.last_read = port.filtered = port.current_read;
                }

                m_is_first_debounce = false;

                return true;
            }

            bool new_input = false;

            for(std::size_t assigned_pin = 0; assigned_pin < m_assigned_pin_count; ++assigned_pin)
            {
                auto& input = m_pins[assigned_pin];
                auto& port  = m_ports[input.port_index];

                const uint32_t pin_mask = (1UL << input.pin_bit);

                const uint32_t current_read_bit = port.current_read & pin_mask;
                const uint32_t last_read_bit    = port.last_read    & pin_mask;

                if(current_read_bit != last_read_bit)
                {
                    // Inputs are different. Reload counter with filter time.
                    input.counter_ms = (current_read_bit == 0) ? input.low_filter_ms : input.high_filter_ms;

                    // Set sampling flag
                    port.sampling |= pin_mask;

                    // Update last read input
                    port.last_read = (port.last_read & (~pin_mask)) | current_read_bit;
                }
                else
                {
                    if(input.counter_ms > 0)
                    {
                        input.counter_ms--;
                    }

                    if(input.counter_ms == 0)
                    {
                        const uint32_t filtered_bit = port.filtered & pin_mask;

                        if(current_read_bit != filtered_bit)
                        {
                            // Update filtered input
                            port.filtered = (port.filtered & (~pin_mask)) | current_read_bit;

                            // Clear sampling flag
                            port.sampling &= ~pin_mask;

                            // Set new input flag
                            new_input = true;
                        }
                    }
                }
            }

            return new_input;
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

        static std::array<PortMask, Port::COUNT> m_ports;
        static std::dynarray<Input>              m_pins;
        static std::size_t                       m_assigned_pin_count;
        static bool                              m_is_first_debounce;
};




} // namespace xarmlib

#endif // __XARMLIB_API_PIN_DEBOUNCER_HPP

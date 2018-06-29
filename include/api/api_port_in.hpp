// ----------------------------------------------------------------------------
// @file    api_port_in.hpp
// @brief   API port input class.
// @date    29 June 2018
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

#include "api/api_input_debouncer.hpp"
#include "api/api_input_scanner.hpp"
#include "api/api_pin_bus.hpp"
#include "api/api_pin_bus_debounced.hpp"

#include "hal/hal_gpio.hpp"
#include "hal/hal_port.hpp"

#include "system/dynarray"
#include "system/gsl"

#include <limits>

namespace xarmlib
{




class PortIn
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template<Pin::Name... pins>
        static void config_pins(const PinBus<pins...>&          pin_bus,
                                const std::chrono::milliseconds sample_count_high,
                                const std::chrono::milliseconds sample_count_low,
                                const Gpio::InputMode           input_mode,
                                const Gpio::InputFilter         input_filter     = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert         input_invert     = Gpio::InputInvert::NORMAL,
                                const Gpio::InputHysteresis     input_hysteresis = Gpio::InputHysteresis::ENABLE)
        {
            for(const auto pin_name : pin_bus)
            {
                Gpio local_gpio(pin_name, input_mode, input_filter, input_invert, input_hysteresis);
            }

            assign_pins(pin_bus, sample_count_high, sample_count_low);
        }

        template<Pin::Name... pins>
        static void config_pins(const PinBus<pins...>&             pin_bus,
                                const std::chrono::milliseconds    sample_count_high,
                                const std::chrono::milliseconds    sample_count_low,
                                const Gpio::InputModeTrueOpenDrain input_mode,
                                const Gpio::InputFilter            input_filter = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
        {
            for(const auto pin_name : pin_bus)
            {
                Gpio local_gpio(pin_name, input_mode, input_filter, input_invert);
            }

            assign_pins(pin_bus, sample_count_high, sample_count_low);
        }

        template<Pin::Name... pins>
        static void config_pins(const PinBusDebounced<pins...>& pin_bus_debounced,
                                const std::chrono::milliseconds sample_count_high,
                                const std::chrono::milliseconds sample_count_low,
                                const Gpio::InputMode           input_mode,
                                const Gpio::InputFilter         input_filter     = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert         input_invert     = Gpio::InputInvert::NORMAL,
                                const Gpio::InputHysteresis     input_hysteresis = Gpio::InputHysteresis::ENABLE)
        {
            config_pins(pins..., sample_count_high, sample_count_low, input_mode, input_filter, input_invert, input_hysteresis);
        }

        template<Pin::Name... pins>
        static void config_pins(const PinBusDebounced<pins...>&    pin_bus_debounced,
                                const std::chrono::milliseconds    sample_count_high,
                                const std::chrono::milliseconds    sample_count_low,
                                const Gpio::InputModeTrueOpenDrain input_mode,
                                const Gpio::InputFilter            input_filter = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
        {
            config_pins(pins..., sample_count_high, sample_count_low, input_mode, input_filter, input_invert);
        }

        template<Pin::Name... pins>
        static void remove_pins(const PinBus<pins...>& pin_bus)
        {
            const bool timer_is_running = InputScanner::is_running();

            InputScanner::stop();

            for(const auto pin_name : pin_bus)
            {
                const std::size_t port_index = get_port_index(pin_name);
                const std::size_t pin_index  = get_pin_index(pin_name);

                assert((m_input_ports_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                m_input_ports_mask[port_index] &= ~(1UL << pin_index);

                for(std::size_t input_index = 0; input_index < m_inputs_count; input_index++)
                {
                    if(m_inputs[input_index].port_index == port_index && m_inputs[input_index].bit_index == pin_index)
                    {
                        for(; input_index < (m_inputs_count - 1); input_index++)
                        {
                            m_inputs[input_index].port_index           = m_inputs[input_index + 1].port_index;
                            m_inputs[input_index].bit_index            = m_inputs[input_index + 1].bit_index;
                            m_inputs[input_index].sample_count_ms_high = m_inputs[input_index + 1].sample_count_ms_high;
                            m_inputs[input_index].sample_count_ms_low  = m_inputs[input_index + 1].sample_count_ms_low;
                            m_inputs[input_index].sample_counter       = m_inputs[input_index + 1].sample_counter;
                        }

                        m_inputs_count--;

                        break;
                    }
                }
            }

            //if(m_inputs_count > 0) => ! POSSIBILITY TO BE ANALYZED !
            {
                if(timer_is_running == true)
                {
                    InputScanner::resume();
                }
            }
        }

        template <Pin::Name... pins>
        static void remove_pins(const PinBusDebounced<pins...>& pin_bus_debounced)
        {
            remove_pins(pins...);
        }

        template<Pin::Name... pins>
        static uint32_t current_debounced(const PinBus<pins...>& pin_bus)
        {
            static_assert(pin_bus.get_array().size() <= 32, "Pin bus over limits!");

            uint32_t value = 0;

            int32_t value_index = 0;

            for(const auto pin_name : pin_bus)
            {
                const std::size_t port_index = get_port_index(pin_name);
                const std::size_t pin_index  = get_pin_index(pin_name);

                assert((m_input_ports_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                const bool current_pin_debounced = (~m_input_ports[port_index].current_debounced) & (1UL << pin_index);

                value |= static_cast<uint32_t>(current_pin_debounced) << value_index;

                value_index++;
            }

            return value;
        }

        template<Pin::Name... pins>
        static uint32_t new_debounced(const PinBus<pins...>& pin_bus)
        {
            static_assert(pin_bus.get_array().size() <= 32, "Pin bus over limits!");

            uint32_t value = 0;

            int32_t value_index = 0;

            for(const auto pin_name : pin_bus)
            {
                const std::size_t port_index = get_port_index(pin_name);
                const std::size_t pin_index  = get_pin_index(pin_name);

                assert((m_input_ports_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                const bool new_pin_debounced = (m_input_ports[port_index].current_debounced & (1UL << pin_index)) ^ (m_input_ports[port_index].last_debounced & (1UL << pin_index));

                value |= static_cast<uint32_t>(new_pin_debounced) << value_index;

                value_index++;
            }

            return value;
        }

        template<Pin::Name... pins>
        static uint32_t sampling(const PinBus<pins...>& pin_bus)
        {
            static_assert(pin_bus.get_array().size() <= 32, "Pin bus over limits!");

            uint32_t value = 0;

            int32_t value_index = 0;

            for(const auto pin_name : pin_bus)
            {
                const std::size_t port_index = get_port_index(pin_name);
                const std::size_t pin_index  = get_pin_index(pin_name);

                assert((m_input_ports_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                const bool pin_sampling = m_input_ports[port_index].sampling & (1UL << pin_index);

                value |= static_cast<uint32_t>(pin_sampling) << value_index;

                value_index++;
            }

            return value;
        }

        // Returns true if the specified pin bus is not being sampled and if its value is new
        template<Pin::Name... pins>
        static bool pin_bus_debounced(const PinBusDebounced<pins...>& pin_bus_debounced)
        {
            if(sampling(pins...) == 0)
            {
                if((pin_bus_debounced.m_value ^ current_debounced(pins...)) != 0)
                {
                    pin_bus_debounced.m_value = current_debounced(pins...);

                    return true;
                }
            }

            return false;
        }

        // Input reader handler to use with InputScanner class
        static bool debounce_port_inputs()
        {
            for(std::size_t port_index = 0; port_index < Port::COUNT; port_index++)
            {
                m_input_ports[port_index].current_read = Port::read(static_cast<Port::Name>(port_index));
            }

            gsl::span<InputDebouncer::Input> inputs{ m_inputs };
            auto inputs_used = inputs.first(m_inputs_count);

            return InputDebouncer::debounce(inputs_used, m_input_ports);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template<Pin::Name... pins>
        static void assign_pins(const PinBus<pins...>& pin_bus, const std::chrono::milliseconds sample_count_high, const std::chrono::milliseconds sample_count_low)
        {
            const bool timer_is_running = InputScanner::is_running();

            InputScanner::stop();

            for(const auto pin_name : pin_bus)
            {
                assign_pin(pin_name, sample_count_high, sample_count_low);
            }

            if(timer_is_running == true)
            {
                InputScanner::resume();
            }
        }

        static void assign_pin(const Pin::Name pin_name, const std::chrono::milliseconds sample_count_high, const std::chrono::milliseconds sample_count_low)
        {
            assert(sample_count_high.count() <= std::numeric_limits<int16_t>::max());
            assert(sample_count_low.count()  <= std::numeric_limits<int16_t>::max());

            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            // Check if pin is already assigned
            for(std::size_t input_index = 0; input_index < m_inputs_count; input_index++)
            {
                if(m_inputs[input_index].port_index == port_index && m_inputs[input_index].bit_index == pin_index)
                {
                    m_inputs[input_index].sample_count_ms_high = static_cast<int16_t>(sample_count_high.count());
                    m_inputs[input_index].sample_count_ms_low  = static_cast<int16_t>(sample_count_low.count());
                    m_inputs[input_index].sample_counter = 0;

                    return;
                }
            }

            // New pin to assign...

            assert(m_inputs_count < m_inputs.size() /* "XARMLIB_CONFIG_PORT_IN_INPUTS_COUNT was reached!" */);

            m_inputs[m_inputs_count].port_index = port_index;
            m_inputs[m_inputs_count].bit_index  = pin_index;
            m_inputs[m_inputs_count].sample_count_ms_high = static_cast<int16_t>(sample_count_high.count());
            m_inputs[m_inputs_count].sample_count_ms_low  = static_cast<int16_t>(sample_count_low.count());
            m_inputs[m_inputs_count].sample_counter = 0;

            m_input_ports_mask[port_index] |= 1UL << pin_index;

            m_inputs_count++;
        }

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

        static dynarray<InputDebouncer::Input>                      m_inputs;
        static std::size_t                                          m_inputs_count;

        static std::array<InputDebouncer::InputPort, Port::COUNT>   m_input_ports;

        static std::array<uint32_t, Port::COUNT>                    m_input_ports_mask;
};




} // namespace xarmlib

#endif // __XARMLIB_API_PORT_IN_HPP

// ----------------------------------------------------------------------------
// @file    api_port_in.hpp
// @brief   API port input class.
// @date    27 June 2018
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

#include <initializer_list>
#include <limits>
#include "system/gsl"
#include "hal/hal_gpio.hpp"
#include "hal/hal_port.hpp"
#include "api/api_group_debounced.hpp"
#include "api/api_input_debouncer.hpp"
#include "api/api_input_scanner.hpp"

namespace xarmlib
{




class PortIn
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void config_pins(const std::initializer_list<Pin::Name> pin_names,
                                const std::chrono::milliseconds        sample_count_high,
                                const std::chrono::milliseconds        sample_count_low,
                                const Gpio::InputMode                  input_mode,
                                const Gpio::InputFilter                input_filter     = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert                input_invert     = Gpio::InputInvert::NORMAL,
                                const Gpio::InputHysteresis            input_hysteresis = Gpio::InputHysteresis::ENABLE)
        {
            for(const auto pin_name : pin_names)
            {
                Gpio local_gpio(pin_name, input_mode, input_filter, input_invert, input_hysteresis);
            }

            assign_pins(pin_names, sample_count_high, sample_count_low);
        }

        static void config_pins(const std::initializer_list<Pin::Name> pin_names,
                                const std::chrono::milliseconds        sample_count_high,
                                const std::chrono::milliseconds        sample_count_low,
                                const Gpio::InputModeTrueOpenDrain     input_mode,
                                const Gpio::InputFilter                input_filter = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert                input_invert = Gpio::InputInvert::NORMAL)
        {
            for(const auto pin_name : pin_names)
            {
                Gpio local_gpio(pin_name, input_mode, input_filter, input_invert);
            }

            assign_pins(pin_names, sample_count_high, sample_count_low);
        }

        static void config_pins(const GroupDebounced            group_debounced,
                                const std::chrono::milliseconds sample_count_high,
                                const std::chrono::milliseconds sample_count_low,
                                const Gpio::InputMode           input_mode,
                                const Gpio::InputFilter         input_filter     = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert         input_invert     = Gpio::InputInvert::NORMAL,
                                const Gpio::InputHysteresis     input_hysteresis = Gpio::InputHysteresis::ENABLE)
        {
            config_pins(group_debounced.m_pin_names, sample_count_high, sample_count_low, input_mode, input_filter, input_invert, input_hysteresis);
        }

        static void config_pins(const GroupDebounced               group_debounced,
                                const std::chrono::milliseconds    sample_count_high,
                                const std::chrono::milliseconds    sample_count_low,
                                const Gpio::InputModeTrueOpenDrain input_mode,
                                const Gpio::InputFilter            input_filter = Gpio::InputFilter::BYPASS,
                                const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
        {
            config_pins(group_debounced.m_pin_names, sample_count_high, sample_count_low, input_mode, input_filter, input_invert);
        }

        static void remove_pins(const std::initializer_list<Pin::Name> pin_names)
        {
            const bool timer_is_running = InputScanner::is_running();

            InputScanner::stop();
//            m_timer.disable_irq();

            for(const auto pin_name : pin_names)
            {
                const std::size_t port_index = get_port_index(pin_name);
                const std::size_t pin_index  = get_pin_index(pin_name);

                assert((m_input_ports_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                m_input_ports_mask[port_index] &= ~(1UL << pin_index);

                for(std::ptrdiff_t input_index = 0; input_index < m_inputs_count; input_index++)
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

            if(timer_is_running == true)
            {
//                    m_timer.enable_irq();
//                    m_timer.reload();
                InputScanner::start(std::chrono::milliseconds(1));
            }

//            if(m_inputs_count > 0)
//            {
//                if(timer_is_running == true)
//                {
////                    m_timer.enable_irq();
////                    m_timer.reload();
//                    InputScanner::start(std::chrono::milliseconds(1));
//                }
//            }
//            else
//            {
//                //remove_new_debounced_handler();
//
//                m_timer.remove_irq_handler();
//            }
        }

        static void remove_pins(const GroupDebounced group_debounced)
        {
            remove_pins(group_debounced.m_pin_names);
        }

        static uint32_t current_debounced(const std::initializer_list<Pin::Name> pin_names)
        {
            assert(pin_names.size() <= 32);

            uint32_t value = 0;

            int32_t value_index = 0;

            for(const auto pin_name : pin_names)
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

        static uint32_t new_debounced(const std::initializer_list<Pin::Name> pin_names)
        {
            assert(pin_names.size() <= 32);

            uint32_t value = 0;

            int32_t value_index = 0;

            for(const auto pin_name : pin_names)
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

        static uint32_t sampling(const std::initializer_list<Pin::Name> pin_names)
        {
            assert(pin_names.size() <= 32);

            uint32_t value = 0;

            int32_t value_index = 0;

            for(const auto pin_name : pin_names)
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

        // Returns true if the group pins are not being sampled and if its value is new
        static bool group_debounced(GroupDebounced& group_debounced)
        {
            if(sampling(group_debounced.m_pin_names) == 0)
            {
                if((group_debounced.m_value ^ current_debounced(group_debounced.m_pin_names)) != 0)
                {
                    group_debounced.m_value = current_debounced(group_debounced.m_pin_names);

                    return true;
                }
            }

            return false;
        }

        // Set the value of the supplied IOCON clock divider (used by input filters)
        // NOTE: The input filter source (where the divider is applied) is the MAIN clock
        static void set_input_filter_clock_divider(const Gpio::InputFilterClockDivider clock_div, const uint8_t div)
        {
            Gpio::set_input_filter_clock_divider(clock_div, div);
        }

        static bool debounce_input_ports()
        {
            for(std::size_t port_index = 0; port_index < Port::COUNT; port_index++)
            {
                m_input_ports[port_index].current_read = Port::read(static_cast<Port::Name>(port_index));
            }

            return InputDebouncer::debounce(m_inputs, m_input_ports);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void assign_pins(const std::initializer_list<Pin::Name> pin_names, const std::chrono::milliseconds sample_count_high, const std::chrono::milliseconds sample_count_low)
        {
            const bool timer_is_running = InputScanner::is_running();

            InputScanner::stop();
//            m_timer.disable_irq();

            for(const auto pin_name : pin_names)
            {
                assign_pin(pin_name, sample_count_high, sample_count_low);
            }

            if(timer_is_running == true)
            {
//                m_timer.enable_irq();
//                m_timer.reload();
                InputScanner::start(std::chrono::milliseconds(1));
            }
        }

        static void assign_pin(const Pin::Name pin_name, const std::chrono::milliseconds sample_count_high, const std::chrono::milliseconds sample_count_low)
        {
            assert(sample_count_high.count() <= std::numeric_limits<int16_t>::max());
            assert(sample_count_low.count()  <= std::numeric_limits<int16_t>::max());

            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            // Check if pin is already assigned
            for(std::ptrdiff_t input_index = 0; input_index < m_inputs_count; input_index++)
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

            assert(m_inputs_count < m_inputs.size() /* "XARMLIB_CONFIG_DEBOUNCER_INPUT_COUNT_MAX was reached!" */);

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

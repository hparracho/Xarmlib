// ----------------------------------------------------------------------------
// @file    api_debouncer.hpp
// @brief   API debouncer class (takes control of one available Timer).
// @date    23 May 2018
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

#ifndef __XARMLIB_API_DEBOUNCER_HPP
#define __XARMLIB_API_DEBOUNCER_HPP

#include <initializer_list>
#include <limits>
#include "system/gsl"
#include "hal/hal_timer.hpp"
#include "hal/hal_gpio.hpp"
#include "hal/hal_port.hpp"
#include "api/api_group_debounced.hpp"

namespace xarmlib
{




namespace private_debouncer
{
    struct Input
    {
        Pin::Name   pin_name;
        int16_t     sample_count_ms_high;   // Number of samples (in ms) to accept an input as debounced at low level
        int16_t     sample_count_ms_low;    // Number of samples (in ms) to accept an input as debounced at high level
        int16_t     sample_counter;
    };
}




class Debouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // New debounced handler definition
        using NewDebouncedHandlerType = int32_t();
        using NewDebouncedHandler     = Delegate<NewDebouncedHandlerType>;

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
            const bool timer_is_running = m_timer.is_running();

            m_timer.stop();
            m_timer.disable_irq();

            for(const auto pin_name : pin_names)
            {
                const std::size_t port_index = get_port_index(pin_name);
                const std::size_t pin_index  = get_pin_index(pin_name);

                assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                m_pins_mask[port_index] &= ~(1UL << pin_index);

                for(std::ptrdiff_t input_index = 0; input_index < m_input_count; input_index++)
                {
                    if(m_input[input_index].pin_name == pin_name)
                    {
                        for(; input_index < (m_input_count - 1); input_index++)
                        {
                            m_input[input_index].pin_name             = m_input[input_index + 1].pin_name;
                            m_input[input_index].sample_count_ms_high = m_input[input_index + 1].sample_count_ms_high;
                            m_input[input_index].sample_count_ms_low  = m_input[input_index + 1].sample_count_ms_low;
                            m_input[input_index].sample_counter       = m_input[input_index + 1].sample_counter;
                        }

                        m_input_count--;

                        break;
                    }
                }
            }

            if(m_input_count > 0)
            {
                if(timer_is_running == true)
                {
                    m_timer.enable_irq();
                    m_timer.reload();
                }
            }
            else
            {
                //remove_new_debounced_handler();

                m_timer.remove_irq_handler();
            }
        }

        static void remove_pins(const GroupDebounced group_debounced)
        {
            remove_pins(group_debounced.m_pin_names);
        }

        static void assign_new_debounced_handler(const NewDebouncedHandler& new_debounced_handler)
        {
            assert(new_debounced_handler != nullptr);

            m_new_debounced_handler = new_debounced_handler;
        }

        static void remove_new_debounced_handler()
        {
            m_new_debounced_handler = nullptr;
        }

        static void start(const int32_t timer_irq_priority)
        {
            assert(m_timer.is_enabled_irq() == false);

            for(std::size_t port_index = 0; port_index < Port::COUNT; port_index++)
            {
                m_current_debounced[port_index] = m_last_debounced[port_index] = m_last_read_pins[port_index] = Port::read(static_cast<Port::Name>(port_index));
                m_sampling[port_index] = 0x00000000;
            }

            const auto timer_callback = Timer::IrqHandler::create<&timer_irq_handler>();

#ifdef __TARGET_TIMER_TYPE_IS_MRT__
            Timer::set_mrt_irq_priority(timer_irq_priority);
#else
            m_timer.set_irq_priority(timer_irq_priority);
#endif
            m_timer.assign_irq_handler(timer_callback);
            m_timer.enable_irq();

            if(m_new_debounced_handler != nullptr)
            {
                m_new_debounced_handler();
            }

            m_timer.start(std::chrono::milliseconds(1), Timer::Mode::FREE_RUNNING);
        }

        static void restart()
        {
            assert(m_timer.is_enabled_irq() == true);

            m_timer.stop();
            m_timer.disable_irq();

            for(std::size_t port_index = 0; port_index < Port::COUNT; port_index++)
            {
                m_current_debounced[port_index] = m_last_debounced[port_index] = m_last_read_pins[port_index] = Port::read(static_cast<Port::Name>(port_index));
                m_sampling[port_index] = 0x00000000;
            }

            if(m_new_debounced_handler != nullptr)
            {
                m_new_debounced_handler();
            }

            m_timer.enable_irq();
            m_timer.reload();
        }

        static void stop()
        {
            m_timer.stop();
            m_timer.clear_pending_irq();
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

                assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                const bool current_pin_debounced = (~m_current_debounced[port_index]) & (1 << pin_index);

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

                assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                const bool new_pin_debounced = (m_current_debounced[port_index] & (1 << pin_index)) ^ (m_last_debounced[port_index] & (1 << pin_index));

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

                assert((m_pins_mask[port_index] & (1UL << pin_index)) != 0 /* "Pin is not configured!" */);

                const bool pin_sampling = m_sampling[port_index] & (1 << pin_index);

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

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void assign_pins(const std::initializer_list<Pin::Name> pin_names, const std::chrono::milliseconds sample_count_high, const std::chrono::milliseconds sample_count_low)
        {
            const bool timer_is_running = m_timer.is_running();

            m_timer.stop();
            m_timer.disable_irq();

            for(const auto pin_name : pin_names)
            {
                assign_pin(pin_name, sample_count_high, sample_count_low);
            }

            if(timer_is_running == true)
            {
                m_timer.enable_irq();
                m_timer.reload();
            }
        }

        static void assign_pin(const Pin::Name pin_name, const std::chrono::milliseconds sample_count_high, const std::chrono::milliseconds sample_count_low)
        {
            assert(sample_count_high.count() <= std::numeric_limits<int16_t>::max());
            assert(sample_count_low.count()  <= std::numeric_limits<int16_t>::max());

            // Check if pin is already assigned
            for(std::ptrdiff_t input_index = 0; input_index < m_input_count; input_index++)
            {
                if(m_input[input_index].pin_name == pin_name)
                {
                    m_input[input_index].sample_count_ms_high = static_cast<int16_t>(sample_count_high.count());
                    m_input[input_index].sample_count_ms_low  = static_cast<int16_t>(sample_count_low.count());
                    m_input[input_index].sample_counter = 0;

                    return;
                }
            }

            // New pin to assign...

            assert(m_input_count < m_input.size() /* "XARMLIB_CONFIG_DEBOUNCER_INPUT_COUNT_MAX was reached!" */);

            m_input[m_input_count].pin_name = pin_name;
            m_input[m_input_count].sample_count_ms_high = static_cast<int16_t>(sample_count_high.count());
            m_input[m_input_count].sample_count_ms_low  = static_cast<int16_t>(sample_count_low.count());
            m_input[m_input_count].sample_counter = 0;

            const std::size_t port_index = get_port_index(pin_name);
            const std::size_t pin_index  = get_pin_index(pin_name);

            m_pins_mask[port_index] |= 1UL << pin_index;

            m_input_count++;
        }

        static int32_t timer_irq_handler()
        {
            temp_m_led_red.write(0); //@TODO: APAGAR !!

            int32_t yield = 0;  // User in FreeRTOS

            if(debounce_port_pins() == true && m_new_debounced_handler != nullptr)
            {
                yield |= m_new_debounced_handler();
            }

            temp_m_led_red.write(1); //@TODO: APAGAR !!

            return yield;
        }

        static bool debounce_port_pins()
        {
            uint32_t current_read_pins[Port::COUNT] = { 0 };
            uint32_t current_debounced[Port::COUNT] = { 0 };
            uint32_t sampling[Port::COUNT] = { 0 };

            for(std::size_t port_index = 0; port_index < Port::COUNT; port_index++)
            {
                current_read_pins[port_index] = Port::read(static_cast<Port::Name>(port_index));

                current_debounced[port_index] = m_current_debounced[port_index];
            }

            for(std::ptrdiff_t input_index = 0; input_index < m_input_count; input_index++)
            {
                const std::size_t port_index = get_port_index(m_input[input_index].pin_name);
                const std::size_t pin_index  = get_pin_index(m_input[input_index].pin_name);

                const uint32_t current_read_pin = current_read_pins[port_index] & (1 << pin_index);
                const uint32_t last_read_pin    = m_last_read_pins[port_index]  & (1 << pin_index);

                if(current_read_pin != last_read_pin)
                {
                    if(current_read_pin)
                    {
                        // High level

                        // Reset samples
                        m_input[input_index].sample_counter = m_input[input_index].sample_count_ms_high;
                    }
                    else
                    {
                        // Low level

                        // Reset samples
                        m_input[input_index].sample_counter = m_input[input_index].sample_count_ms_low;
                    }
                }

                if(m_input[input_index].sample_counter == 0)
                {
                    // Clear pin mask
                    current_debounced[port_index] &= ~(1 << pin_index);

                    // Input debounced
                    current_debounced[port_index] |= current_read_pin;
                }
                else
                {
                    // Sampling
                    sampling[port_index] |= (1 << pin_index);

                    m_input[input_index].sample_counter--;
                }
            }

            bool result = false;

            for(std::size_t port_index = 0; port_index < Port::COUNT; port_index++)
            {
                m_last_read_pins[port_index] = current_read_pins[port_index];

                if(current_debounced[port_index] != m_current_debounced[port_index])
                {
                    m_last_debounced[port_index] = m_current_debounced[port_index];

                    m_current_debounced[port_index] = current_debounced[port_index];

                    m_sampling[port_index] = sampling[port_index];

                    result = true;
                }
            }

            return result;
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

        static Timer                                m_timer;

        //@TODO: APAGAR !!
        static Gpio                                 temp_m_led_red;

        static gsl::span<private_debouncer::Input>  m_input;
        static std::ptrdiff_t                       m_input_count;

        static std::array<uint32_t, Port::COUNT>    m_pins_mask;

        static std::array<uint32_t, Port::COUNT>    m_last_read_pins;

        static std::array<uint32_t, Port::COUNT>    m_current_debounced;
        static std::array<uint32_t, Port::COUNT>    m_last_debounced;
        static std::array<uint32_t, Port::COUNT>    m_sampling;                 // Inputs that are being sampled and not yet filtered

        static NewDebouncedHandler                  m_new_debounced_handler;    // User defined new debounced input handler
};




} // namespace xarmlib

#endif // __XARMLIB_API_DEBOUNCER_HPP

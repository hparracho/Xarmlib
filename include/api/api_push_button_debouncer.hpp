// ----------------------------------------------------------------------------
// @file    api_push_button_debouncer.hpp
// @brief   API push-button debouncer class.
// @date    9 October 2018
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

#ifndef __XARMLIB_API_PUSH_BUTTON_DEBOUNCER_HPP
#define __XARMLIB_API_PUSH_BUTTON_DEBOUNCER_HPP

#include "spi_io_source.hpp"
#include "api/api_gpio_source.hpp"
#include "api/api_push_button_bus.hpp"
#include "hal/hal_gpio.hpp"

namespace xarmlib
{




template <class Type>
class PushButtonDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        PushButtonDebouncer(      GpioSource&           gpio_source,
                                  PushButtonBus<Type>&  push_button_bus,
                            const int16_t               scan_time_stage1_samples,
                            const int16_t               scan_time_stage2_samples,
                            const int16_t               scan_time_timeout_samples,
                            const int16_t               scan_time_output_error_samples,
                            const Gpio::InputMode       input_mode,
                            const Gpio::InputInvert     input_invert     = Gpio::InputInvert::NORMAL,
                            const Gpio::InputHysteresis input_hysteresis = Gpio::InputHysteresis::ENABLE) :
            m_push_button_bus { push_button_bus },
            m_pin_source { gpio_source },
            m_push_buttons(push_button_bus.get_size()),
            m_stage_samples { scan_time_stage1_samples, scan_time_stage2_samples },
            m_timeout_samples { scan_time_timeout_samples },
            m_output_error_samples { scan_time_output_error_samples },
            m_last_read_bus { 0 },
            m_sampling_bus { 0 },
            m_stage1_reached_bus { 0 },
            m_stage2_reached_bus { 0 },
            m_input_error_bus { 0 },
            m_output_error_bus { 0 },
            m_output_bus { push_button_bus.get_mask() },
            m_output_waved_bus { 0 },
            m_continuous_wave_value { 0 },
            m_continuous_wave_counter { static_cast<int16_t>(scan_time_stage1_samples - m_continuous_wave_high_samples) }
        {
            for(const auto push_button : push_button_bus)
            {
                Gpio gpio(push_button.pin, input_mode, Gpio::InputFilter::BYPASS, input_invert, input_hysteresis);
            }

            config_pins<GpioSource>();
        }

        PushButtonDebouncer(      SpiIoSource&         spi_io_source,
                                  PushButtonBus<Type>& push_button_bus,
                            const int16_t              scan_time_stage1_samples,
                            const int16_t              scan_time_stage2_samples,
                            const int16_t              scan_time_timeout_samples,
                            const int16_t              scan_time_output_error_samples) :
            m_push_button_bus { push_button_bus },
            m_pin_source { spi_io_source },
            m_push_buttons(push_button_bus.get_size()),
            m_stage_samples { scan_time_stage1_samples, scan_time_stage2_samples },
            m_timeout_samples { scan_time_timeout_samples },
            m_output_error_samples { scan_time_output_error_samples },
            m_last_read_bus { 0 },
            m_sampling_bus { 0 },
            m_stage1_reached_bus { 0 },
            m_stage2_reached_bus { 0 },
            m_input_error_bus { 0 },
            m_output_error_bus { 0 },
            m_output_bus { push_button_bus.get_mask() },
            m_output_waved_bus { 0 },
            m_continuous_wave_value { 0 },
            m_continuous_wave_counter { static_cast<int16_t>(scan_time_stage1_samples - m_continuous_wave_high_samples) }
        {
            config_pins<SpiIoSource>();
        }

        // Get the handler that is intended to be used as a debouncer handler of the PinScanner class
        PinScanner::DebouncerHandler get_debouncer_handler()
        {
            return PinScanner::DebouncerHandler::create<PushButtonDebouncer, &PushButtonDebouncer::debouncer_handler>(this);
        }

        // NOTE: these flags are cleared after reading
        uint32_t get_stage1_reached_bus()
        {
            const uint32_t stage1_reached_bus = m_stage1_reached_bus;

            m_stage1_reached_bus = 0;

            return stage1_reached_bus;
        }

        // NOTE: these flags are cleared after reading
        uint32_t get_stage2_reached_bus()
        {
            const uint32_t stage2_reached_bus = m_stage2_reached_bus;

            m_stage2_reached_bus = 0;

            return stage2_reached_bus;
        }

        // NOTE: these flags are active errors
        uint32_t get_input_error_bus() const
        {
            return m_input_error_bus;
        }

        uint32_t get_output_error_bus() const
        {
            return m_output_error_bus;
        }

        void clear_output_error_bus(const uint32_t mask)
        {
            m_output_error_bus &= ~mask;
        }

        uint32_t get_output_bus() const
        {
            return m_output_bus;
        }

        // NOTES: - the value will be written in the next transfer
        //        - the maskable push-buttons behaves as input
        void set_output_bus(const uint32_t mask)
        {
            for(std::size_t push_button_index = 0; push_button_index < m_push_buttons.size(); ++push_button_index)
            {
                const uint32_t push_button_mask = 1UL << push_button_index;

                if((mask & push_button_mask) != 0)
                {
                    auto& push_button = m_push_buttons[push_button_index];

                    const uint32_t output_bit = 1UL << push_button.pin_bit;

                    m_pin_source.write_output_bit(push_button.port_index, push_button.pin_bit, output_bit);

                    m_output_bus       |= push_button_mask;
                    m_sampling_bus     &= ~push_button_mask;
                    m_output_waved_bus &= ~push_button_mask;

                    push_button.counter = m_timeout_samples;
                }
            }
        }

        // NOTES: - the value will be written in the next transfer
        //        - the maskable push-buttons behaves as output
        void clear_output_bus(const uint32_t mask)
        {
            for(std::size_t push_button_index = 0; push_button_index < m_push_buttons.size(); ++push_button_index)
            {
                const uint32_t push_button_mask = 1UL << push_button_index;

                if((mask & push_button_mask) != 0)
                {
                    auto& push_button = m_push_buttons[push_button_index];

                    m_pin_source.write_output_bit(push_button.port_index, push_button.pin_bit, 0);

                    m_output_bus       &= ~push_button_mask;
                    m_sampling_bus     &= ~push_button_mask;
                    m_output_waved_bus &= ~push_button_mask;

                    push_button.counter = m_output_error_samples;
                }
            }
        }

        // NOTES: - the value will be written in the next transfer
        //        - with this method the maskable push-buttons behaves as input/output at same time
        //        - a cleared output will never be at '0' for more than scan_time_stage1_samples minor twice samples of scan time
        void set_output_waved_bus(const uint32_t mask)
        {
            for(std::size_t push_button_index = 0; push_button_index < m_push_buttons.size(); ++push_button_index)
            {
                const uint32_t push_button_mask = 1UL << push_button_index;

                if((mask & push_button_mask) != 0)
                {
                    auto& push_button = m_push_buttons[push_button_index];

                    const uint32_t output_bit = 1UL << push_button.pin_bit;

                    m_pin_source.write_output_bit(push_button.port_index, push_button.pin_bit, output_bit);

                    m_output_bus       |= push_button_mask;
                    m_output_waved_bus |= push_button_mask;
                }
            }
        }

        // NOTES: - the value will be written in the next transfer
        //        - with this method the maskable push-buttons behaves as input/output at same time
        //        - a cleared output will never be at '0' for more than scan_time_stage1_samples minor twice samples of scan time
        void clear_output_waved_bus(const uint32_t mask)
        {
            for(std::size_t push_button_index = 0; push_button_index < m_push_buttons.size(); ++push_button_index)
            {
                const uint32_t push_button_mask = 1UL << push_button_index;

                if((mask & push_button_mask) != 0)
                {
                    auto& push_button = m_push_buttons[push_button_index];

                    const uint32_t output_bit = m_continuous_wave_value << push_button.pin_bit;

                    m_pin_source.write_output_bit(push_button.port_index, push_button.pin_bit, output_bit);

                    m_output_bus       &= ~push_button_mask;
                    m_output_waved_bus |=  push_button_mask;
                }
            }
        }

    protected:

        // --------------------------------------------------------------------
        // PROTECTED MEMBER VARIABLES
        // --------------------------------------------------------------------

        PushButtonBus<Type>& m_push_button_bus;

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        struct PushButton
        {
            int8_t  port_index { -1 };  // Input's port index
            int8_t  pin_bit    { -1 };  // Input's bit within a port
            int16_t counter    {  0 };  // Samples counter
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template <typename Source>
        void config_pins()
        {
            assert(m_stage_samples[0]     > 1);
            assert(m_stage_samples[1]     > 1);
            assert(m_timeout_samples      > 1);
            assert(m_output_error_samples > 1);

            assert(m_stage_samples[0] > m_output_error_samples);
            assert(m_stage_samples[1] > m_stage_samples[0]);
            assert(m_timeout_samples  > m_stage_samples[1]);

            std::size_t assigned_push_button = 0;

            const bool resume = PinScanner::is_running();

            PinScanner::stop();

            for(auto push_button : m_push_button_bus)
            {
                const int8_t port_index = Source::get_port_index(push_button.pin);
                const int8_t pin_bit = Source::get_pin_bit(push_button.pin);

                m_push_buttons[assigned_push_button++] = { port_index, pin_bit, 0 };
            }

            if(resume == true)
            {
                PinScanner::resume();
            }
        }

        // Handler that is intended to be used as a debouncer handler of the PinScanner class
        bool debouncer_handler(const bool is_starting)
        {
            bool continuous_wave_value_changed = false;

            m_continuous_wave_counter--;

            if(m_continuous_wave_counter == 0)
            {
                if(m_continuous_wave_value == 0)
                {
                    m_continuous_wave_value = 1;
                    m_continuous_wave_counter = m_continuous_wave_high_samples;
                }
                else
                {
                    m_continuous_wave_value = 0;
                    m_continuous_wave_counter = m_stage_samples[0] - m_continuous_wave_high_samples;
                }

                continuous_wave_value_changed = true;
            }

            if(is_starting == true)
            {
                for(std::size_t push_button_index = 0; push_button_index < m_push_buttons.size(); ++push_button_index)
                {
                    const auto push_button = m_push_buttons[push_button_index];

                    const bool read_bit = m_pin_source.get_read_bit(push_button.port_index, push_button.pin_bit) != 0;

                    m_last_read_bus |= static_cast<uint32_t>(read_bit) << push_button_index;
                }

                m_input_error_bus = ~m_last_read_bus & m_push_button_bus.get_mask();

                return true;
            }

            bool new_push_button = false;

            for(std::size_t push_button_index = 0; push_button_index < m_push_buttons.size(); ++push_button_index)
            {
                auto& push_button = m_push_buttons[push_button_index];

                const uint32_t push_button_mask = 1UL << push_button_index;

                const bool current_read_bit = m_pin_source.get_read_bit(push_button.port_index, push_button.pin_bit) != 0;
                const bool last_read_bit    = (m_last_read_bus & push_button_mask) != 0;

                if(current_read_bit != last_read_bit)
                {
                    // Inputs are different

                    const bool output_bit = (m_output_bus & push_button_mask) != 0;

                    const bool output_waved_bit = (m_output_waved_bus & push_button_mask) != 0;

                    if(output_bit != 0 || output_waved_bit != 0)
                    {
                        // Input mode

                        if(current_read_bit == 0)
                        {
                            // Push-button was pressed

                            // Get the selected stage
                            const std::size_t stage_index = m_push_button_bus.get_stage2_selected(push_button_index);

                            // Reload counter with state1 or stage2 samples
                            push_button.counter = m_stage_samples[stage_index];

                            // Set sampling flag
                            m_sampling_bus |= push_button_mask;

                            // Clear flags
                            m_stage1_reached_bus &= ~push_button_mask;
                            m_stage2_reached_bus &= ~push_button_mask;
                        }
                        else
                        {
                            // Push-button was released

                            if((m_sampling_bus & push_button_mask) != 0
                             && m_push_button_bus.get_stage2_selected(push_button_index) == true
                             && push_button.counter <= (m_stage_samples[1] - m_stage_samples[0]))
                            {
                                // Set stage 1 reached flag
                                m_stage1_reached_bus |= push_button_mask;

                                // Set new push-button flag
                                new_push_button = true;
                            }

                            push_button.counter = 0;

                            // Clear sampling flag
                            m_sampling_bus &= ~push_button_mask;

                            // Clear input error flag
                            m_input_error_bus &= ~push_button_mask;
                        }
                    }
                    else
                    {
                        // Output mode

                        // Sampling possible over-current or stop
                        push_button.counter = (current_read_bit != 0) ? m_output_error_samples : 0;
                    }

                    // Update last read push-button
                    m_last_read_bus = (m_last_read_bus & (~push_button_mask)) | static_cast<uint32_t>(current_read_bit) << push_button_index;
                }
                else
                {
                    if(push_button.counter > 0)
                    {
                        push_button.counter--;

                        if(push_button.counter == 0)
                        {
                            if((m_sampling_bus & push_button_mask) != 0)
                            {
                                if(m_push_button_bus.get_stage2_selected(push_button_index) == false)
                                {
                                    // Stage 1 filter time reached

                                    // Set stage 1 reached flag
                                    m_stage1_reached_bus |= push_button_mask;

                                    // Reload counter with the relative timeout time
                                    push_button.counter = m_timeout_samples - m_stage_samples[0];
                                }
                                else
                                {
                                    // Stage 2 filter time reached

                                    // Set stage 2 reached flag
                                    m_stage2_reached_bus |= push_button_mask;

                                    // Reload counter with the relative timeout time
                                    push_button.counter = m_timeout_samples - m_stage_samples[1];
                                }

                                // Clear sampling flag
                                m_sampling_bus &= ~push_button_mask;

                                // Set new push-button flag
                                new_push_button = true;
                            }
                            else
                            {
                                const bool output_bit = (m_output_bus & push_button_mask) != 0;

                                if(output_bit != current_read_bit)
                                {
                                    if(current_read_bit == 0)
                                    {
                                        // Set input error flag
                                        m_input_error_bus |= push_button_mask;
                                    }
                                    else
                                    {
                                        // Set output error flag
                                        m_output_error_bus |= push_button_mask;

                                        // Unable to clear output (over-current?) -> set output
                                        m_pin_source.write_output_bit(push_button.port_index, push_button.pin_bit, 1UL << push_button.pin_bit);

                                        m_output_bus |= push_button_mask;
                                    }
                                }
                            }
                        }
                    }
                }

                if(continuous_wave_value_changed == true && (m_output_bus & push_button_mask) == 0 && (m_output_waved_bus & push_button_mask) != 0)
                {
                    const uint32_t output_bit = m_continuous_wave_value << push_button.pin_bit;

                    m_pin_source.write_output_bit(push_button.port_index, push_button.pin_bit, output_bit);
                }
            }

            return new_push_button;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        PinSource&                   m_pin_source;
        std::dynarray<PushButton>    m_push_buttons;

        const std::array<int16_t, 2> m_stage_samples;        // Number of samples of scan time that a pin must be steady at low level to be accepted at stage1/stage2
                                                             // NOTE: index 0 -> stage1; index 1 -> stage2
        const int16_t                m_timeout_samples;      // Number of samples of scan time that a pin must be steady at low level to be considered timeout
        const int16_t                m_output_error_samples; // Number of samples of scan time that a output must be equal to the input to valid the output written

        // Bitmasks aligned with PushButtonBus
        uint32_t                     m_last_read_bus;        // Previous iteration inputs
        uint32_t                     m_sampling_bus;         // Inputs that are being sampled and not yet filtered

        uint32_t                     m_stage1_reached_bus;
        uint32_t                     m_stage2_reached_bus;
        uint32_t                     m_input_error_bus;
        uint32_t                     m_output_error_bus;

        uint32_t                     m_output_bus;           // Outputs written

        uint32_t                     m_output_waved_bus;     // Outputs using the continuous wave

        // Continuous wave helpers
        static constexpr int16_t     m_continuous_wave_high_samples { 2 }; // Number of samples of scan time for the continuous wave be steady at high level
        uint32_t                     m_continuous_wave_value;
        int16_t                      m_continuous_wave_counter;
};




} // namespace xarmlib

#endif // __XARMLIB_API_PUSH_BUTTON_DEBOUNCER_HPP

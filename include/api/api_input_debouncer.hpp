// ----------------------------------------------------------------------------
// @file    api_input_debouncer.hpp
// @brief   API input debouncer class.
// @date    14 January 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_API_INPUT_DEBOUNCER_HPP
#define __XARMLIB_API_INPUT_DEBOUNCER_HPP

#include "devices/spi_io_source.hpp"
#include "api/api_gpio_source.hpp"
#include "api/api_pin_bus.hpp"
#include "hal/hal_gpio.hpp"

namespace xarmlib
{




class InputDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        InputDebouncer(      GpioSource&                 gpio_source,
                       const PinNameBus&                 pin_name_bus,
                       const hal::Gpio::InputModeConfig& pin_bus_config,
                       const int16_t                     scan_time_low_samples,
                       const int16_t                     scan_time_high_samples) : m_pin_source { gpio_source },
                                                                                   m_inputs(pin_name_bus.get_size()),
                                                                                   m_low_samples { scan_time_low_samples },
                                                                                   m_high_samples { scan_time_high_samples },
                                                                                   m_last_read_bus { 0 },
                                                                                   m_filtered_bus { 0 },
                                                                                   m_sampling_bus { 0 }
        {
            for(const auto pin_name : pin_name_bus)
            {
                hal::Gpio gpio(pin_name, pin_bus_config);
            }

            config_pins<GpioSource>(pin_name_bus);
        }

#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        InputDebouncer(      GpioSource&                              gpio_source,
                       const PinNameBus&                              pin_name_bus,
                       const hal::Gpio::InputModeTrueOpenDrainConfig& pin_bus_config,
                       const int16_t                                  scan_time_low_samples,
                       const int16_t                                  scan_time_high_samples) : m_pin_source { gpio_source },
                                                                                                m_inputs(pin_name_bus.get_size()),
                                                                                                m_low_samples { scan_time_low_samples },
                                                                                                m_high_samples { scan_time_high_samples },
                                                                                                m_last_read_bus { 0 },
                                                                                                m_filtered_bus { 0 },
                                                                                                m_sampling_bus { 0 }
        {
            for(const auto pin_name : pin_name_bus)
            {
                hal::Gpio gpio(pin_name, pin_bus_config);
            }

            config_pins<GpioSource>(pin_name_bus);
        }
#endif

        InputDebouncer(      SpiIoSource& spi_io_source,
                       const PinIndexBus& pin_index_bus,
                       const int16_t      scan_time_low_samples,
                       const int16_t      scan_time_high_samples) : m_pin_source { spi_io_source },
                                                                    m_inputs(pin_index_bus.get_size()),
                                                                    m_low_samples { scan_time_low_samples },
                                                                    m_high_samples { scan_time_high_samples },
                                                                    m_last_read_bus { 0 },
                                                                    m_filtered_bus { 0 },
                                                                    m_sampling_bus { 0 }
        {
            config_pins<SpiIoSource>(pin_index_bus);
        }

        // Get the handler that is intended to be used as a debouncer handler of the PinScanner class
        PinScanner::DebouncerHandler get_debouncer_handler()
        {
            return PinScanner::DebouncerHandler::create<InputDebouncer, &InputDebouncer::debouncer_handler>(this);
        }

        uint32_t get_filtered_bus() const
        {
            return m_filtered_bus;
        }

        uint32_t get_sampling_bus() const
        {
            return m_sampling_bus;
        }

     private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        struct Input
        {
            int8_t      port_index { -1 };  // Input's port index
            int8_t      pin_bit    { -1 };  // Input's bit within a port
            int16_t     counter    {  0 };  // Samples counter
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template <typename PinBusSource, class PinBusType>
        void config_pins(const PinBus<PinBusType>& pin_bus)
        {
            assert(m_low_samples  > 1);
            assert(m_high_samples > 1);

            std::size_t assigned_input = 0;

            const bool resume = PinScanner::is_running();

            PinScanner::stop();

            for(auto pin : pin_bus)
            {
                const int8_t port_index = PinBusSource::get_port_index(pin);
                const int8_t pin_bit = PinBusSource::get_pin_bit(pin);

                m_inputs[assigned_input++] = { port_index, pin_bit, 0 };
            }

            if(resume == true)
            {
                PinScanner::resume();
            }
        }

        // Handler that is intended to be used as a debouncer handler of the PinScanner class
        bool debouncer_handler(const bool is_starting)
        {
            if(is_starting == true)
            {
                for(std::size_t input_index = 0; input_index < m_inputs.size(); ++input_index)
                {
                    const auto input = m_inputs[input_index];

                    const bool read_bit = m_pin_source.get_read_bit(input.port_index, input.pin_bit) != 0;

                    m_last_read_bus |= static_cast<uint32_t>(read_bit) << input_index;
                }

                m_filtered_bus = m_last_read_bus;

                return true;
            }

            bool new_input = false;

            for(std::size_t input_index = 0; input_index < m_inputs.size(); ++input_index)
            {
                auto& input = m_inputs[input_index];

                const uint32_t input_mask = 1UL << input_index;

                const bool current_read_bit = m_pin_source.get_read_bit(input.port_index, input.pin_bit) != 0;
                const bool last_read_bit    = (m_last_read_bus & input_mask) != 0;

                if(current_read_bit != last_read_bit)
                {
                    // Inputs are different. Reload counter with number of samples.
                    input.counter = (current_read_bit == 0) ? m_low_samples : m_high_samples;

                    // Set sampling flag
                     m_sampling_bus |= input_mask;

                    // Update last read input
                    m_last_read_bus = (m_last_read_bus & (~input_mask)) | static_cast<uint32_t>(current_read_bit) << input_index;
                }
                else
                {
                    if(input.counter > 0)
                    {
                        input.counter--;

                        if(input.counter == 0)
                        {
                            const bool filtered_bit = (m_filtered_bus & input_mask) != 0;

                            if(current_read_bit != filtered_bit)
                            {
                                // Update filtered input
                                m_filtered_bus = (m_filtered_bus & (~input_mask)) | static_cast<uint32_t>(current_read_bit) << input_index;

                                // Set new input flag
                                new_input = true;
                            }

                            // Clear sampling flag
                            m_sampling_bus &= ~input_mask;
                        }
                    }
                }
            }

            return new_input;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        PinSource&           m_pin_source;
        std::dynarray<Input> m_inputs;

        const int16_t        m_low_samples;     // Number of samples of scan time that a pin must be steady at low level to be accepted as filtered (debounced)
        const int16_t        m_high_samples;    // Number of samples of scan time that a pin must be steady at high level to be accepted as filtered (debounced)

        // Bitmasks aligned with PinBus
        uint32_t             m_last_read_bus;   // Previous iteration inputs
        uint32_t             m_filtered_bus;    // Filtered inputs
        uint32_t             m_sampling_bus;    // Inputs that are being sampled and not yet filtered
};




} // namespace xarmlib

#endif // __XARMLIB_API_INPUT_DEBOUNCER_HPP

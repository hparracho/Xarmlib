// ----------------------------------------------------------------------------
// @file    api_io_debouncer.hpp
// @brief   API I/O debouncer class.
// @date    26 July 2018
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

#ifndef __XARMLIB_API_IO_DEBOUNCER_HPP
#define __XARMLIB_API_IO_DEBOUNCER_HPP

#include "spi_io_source.hpp"
#include "api/api_gpio_source.hpp"
#include "api/api_pin_bus.hpp"
#include "hal/hal_gpio.hpp"

namespace xarmlib
{




class IoDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        IoDebouncer(      GpioSource&           gpio_source,
                    const PinNameBus&           pin_name_bus,
                    const int16_t               scan_time_low_samples,
                    const int16_t               scan_time_high_samples,
                    const int16_t               scan_time_over_current_samples,
                    const Gpio::InputMode       input_mode,
                    const Gpio::InputInvert     input_invert     = Gpio::InputInvert::NORMAL,
                    const Gpio::InputHysteresis input_hysteresis = Gpio::InputHysteresis::ENABLE) : m_pin_source { gpio_source },
                                                                                                    m_ports(gpio_source.get_port_count()),
                                                                                                    m_inputs(pin_name_bus.get_size()),
                                                                                                    m_over_current_samples { 0 }
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert, input_hysteresis);
            }

            config_pins<GpioSource>(pin_name_bus, scan_time_low_samples, scan_time_high_samples, scan_time_over_current_samples);
        }

        IoDebouncer(      GpioSource&                  gpio_source,
                    const PinNameBus&                  pin_name_bus,
                    const int16_t                      scan_time_low_samples,
                    const int16_t                      scan_time_high_samples,
                    const int16_t                      scan_time_over_current_samples,
                    const Gpio::InputModeTrueOpenDrain input_mode,
                    const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL) : m_pin_source { gpio_source },
                                                                                                   m_ports(gpio_source.get_port_count()),
                                                                                                   m_inputs(pin_name_bus.get_size()),
                                                                                                   m_over_current_samples { 0 }
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert);
            }

            config_pins<GpioSource>(pin_name_bus, scan_time_low_samples, scan_time_high_samples, scan_time_over_current_samples);
        }

        IoDebouncer(      SpiIoSource& spi_io_source,
                    const PinIndexBus& pin_index_bus,
                    const int16_t      scan_time_low_samples,
                    const int16_t      scan_time_high_samples,
                    const int16_t      scan_time_over_current_samples) : m_pin_source { spi_io_source },
                                                                         m_ports(spi_io_source.get_port_count()),
                                                                         m_inputs(pin_index_bus.get_size()),
                                                                         m_over_current_samples { 0 }
        {
            config_pins<SpiIoSource>(pin_index_bus, scan_time_low_samples, scan_time_high_samples, scan_time_over_current_samples);
        }

        // Get the handler that is intended to be used as a debouncer handler of the PinScanner class
        PinScanner::DebouncerHandler get_debouncer_handler()
        {
            return PinScanner::DebouncerHandler::create<IoDebouncer, &IoDebouncer::debouncer_handler>(this);
        }

        uint32_t get_filtered()
        {
            uint32_t filtered = 0;
            std::size_t filtered_shift_index = 0;

            for(auto input : m_inputs)
            {
                const uint32_t pin_mask = (1UL << input.pin_bit);

                const bool filtered_bit = (m_ports[input.port_index].filtered & pin_mask) != 0;

                filtered |= static_cast<uint32_t>(filtered_bit) << filtered_shift_index++;
            }

            return filtered;
        }

        uint32_t get_sampling()
        {
            uint32_t sampling = 0;
            std::size_t sampling_shift_index = 0;

            for(auto input : m_inputs)
            {
                const uint32_t pin_mask = (1UL << input.pin_bit);

                const bool sampling_bit = (m_ports[input.port_index].sampling & pin_mask) != 0;

                sampling |= static_cast<uint32_t>(sampling_bit) << sampling_shift_index++;
            }

            return sampling;
        }

        // Write output value to be written in the next transfer
        void write_output(const uint32_t value)
        {
            std::size_t value_shift_index = 0;

            for(auto& input : m_inputs)
            {
                const bool value_bit = (value & (1UL << value_shift_index++)) != 0;
                const uint32_t output_bit = static_cast<uint32_t>(value_bit) << input.pin_bit;

                m_pin_source.write_output_bit(input.port_index, input.pin_bit, output_bit);

                if(output_bit == 0)
                {
                    // Reload counter with over-current samples
                    input.counter = m_over_current_samples;
                }
            }
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        struct Input
        {
            int8_t      port_index   { -1 };    // Input's port index
            int8_t      pin_bit      { -1 };    // Input's bit within a port
            int16_t     low_samples  { 0 };     // Number of samples of scan time that a pin must be steady at low level to be accepted as filtered (debounced)
            int16_t     high_samples { 0 };     // Number of samples of scan time that a pin must be steady at high level to be accepted as filtered (debounced)
            int16_t     counter      { 0 };     // Samples counter
        };

        struct PortMask
        {
            uint32_t     last_read { 0 };       // Previous iteration inputs
            uint32_t     filtered  { 0 };       // Filtered inputs
            uint32_t     sampling  { 0 };       // Inputs that are being sampled and not yet filtered
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template <typename PinBusSource, class PinBusType>
        void config_pins(const PinBus<PinBusType>& pin_bus, const int16_t low_samples, const int16_t high_samples, const int16_t over_current_samples)
        {
            assert(low_samples  > 0);
            assert(high_samples > 0);
            assert(over_current_samples > 0);

            m_over_current_samples = over_current_samples;

            std::size_t assigned_input = 0;

            const bool resume = PinScanner::is_running();

            PinScanner::stop();

            for(auto pin : pin_bus)
            {
                const int8_t port_index = PinBusSource::get_port_index(pin);
                const int8_t pin_bit = PinBusSource::get_pin_bit(pin);

                m_inputs[assigned_input++] = {port_index, pin_bit, low_samples, high_samples, 0 };
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
                for(std::size_t port_index = 0; port_index < m_ports.size(); ++port_index)
                {
                    m_ports[port_index].last_read = m_ports[port_index].filtered = m_pin_source.get_read(port_index);
                }

                return true;
            }

            bool new_input = false;

            for(auto& input : m_inputs)
            {
                auto& port = m_ports[input.port_index];

                const uint32_t pin_mask = (1UL << input.pin_bit);

                const uint32_t current_read_bit = m_pin_source.get_read_bit(input.port_index, input.pin_bit);
                const uint32_t last_read_bit    = port.last_read & pin_mask;

                if(current_read_bit != last_read_bit)
                {
                    const uint32_t output_bit = m_pin_source.get_output_bit(input.port_index, input.pin_bit);

                    // Inputs are different. Reload counter with number of samples.
                    if(output_bit == 0 && current_read_bit != 0)
                    {
                        input.counter = m_over_current_samples;
                    }
                    else if(output_bit != 0 || input.counter == 0)
                    {
                        input.counter = (current_read_bit == 0) ? input.low_samples : input.high_samples;

                        // Set sampling flag
                        port.sampling |= pin_mask;
                    }

                    // Update last read input
                    port.last_read = (port.last_read & (~pin_mask)) | current_read_bit;
                }
                else
                {
                    if(input.counter > 0)
                    {
                        input.counter--;
                    }

                    if(input.counter == 0)
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

                        const uint32_t output_bit = m_pin_source.get_output_bit(input.port_index, input.pin_bit);

                        if(current_read_bit != 0 && output_bit == 0)
                        {
                            // Unable to set output (over-current?) -> clear output
                            m_pin_source.write_output_bit(input.port_index, input.pin_bit, pin_mask);
                        }
                    }
                }
            }

            return new_input;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        PinSource&              m_pin_source;
        std::dynarray<PortMask> m_ports;
        std::dynarray<Input>    m_inputs;
        int16_t                 m_over_current_samples;

};




} // namespace xarmlib

#endif // __XARMLIB_API_IO_DEBOUNCER_HPP

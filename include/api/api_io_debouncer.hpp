// ----------------------------------------------------------------------------
// @file    api_io_debouncer.hpp
// @brief   API I/O debouncer class.
// @date    16 July 2018
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
#include "api/api_pin_bus.hpp"
#include "api/api_pin_scanner.hpp"

namespace xarmlib
{




class IoDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        IoDebouncer(      SpiIoSource&              spi_io_source,
                    const PinIndexBus&              pin_index_bus,
                    const std::chrono::milliseconds low_filter,
                    const std::chrono::milliseconds high_filter,
                    const std::chrono::milliseconds over_current_filter = std::chrono::milliseconds(20)) : m_pin_source { spi_io_source },
                                                                                                           m_ports(spi_io_source.get_port_count()),
                                                                                                           m_inputs(pin_index_bus.get_size()),
                                                                                                           m_over_current_filter { 0 }
        {
            assert(low_filter.count()          > 0 && low_filter.count()          <= std::numeric_limits<int16_t>::max());
            assert(high_filter.count()         > 0 && high_filter.count()         <= std::numeric_limits<int16_t>::max());
            assert(over_current_filter.count() > 0 && over_current_filter.count() <= std::numeric_limits<int16_t>::max());

            const int16_t low_filter_ms  = static_cast<int16_t>(low_filter.count());
            const int16_t high_filter_ms = static_cast<int16_t>(high_filter.count());

            m_over_current_filter = static_cast<int16_t>(over_current_filter.count());

            std::size_t assigned_input = 0;

            const bool resume = PinScanner::is_running();

            PinScanner::stop();

            for(auto pin_index : pin_index_bus)
            {
                const int8_t port_index = get_port_index(pin_index);
                const int8_t pin_bit = get_pin_bit(pin_index);

                m_inputs[assigned_input++] = {port_index, pin_bit, low_filter_ms, high_filter_ms, 0 };
            }

            if(resume == true)
            {
                PinScanner::resume();
            }
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
                const uint8_t pin_mask = (1U << input.pin_bit);

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
                const uint8_t pin_mask = (1U << input.pin_bit);

                const bool sampling_bit = (m_ports[input.port_index].sampling & pin_mask) != 0;

                sampling |= static_cast<uint32_t>(sampling_bit) << sampling_shift_index++;
            }

            return sampling;
        }

        // Set output value to be written in the next transfer
        void set_output(const uint32_t value)
        {
            std::size_t value_shift_index = 0;

            for(auto input : m_inputs)
            {
                const uint8_t pin_mask = (1U << input.pin_bit);

                const bool value_bit = (value & (1UL << value_shift_index++)) != 0;
                const uint8_t output_bit = static_cast<uint8_t>(value_bit) << input.pin_bit;

                //@TODO: get/set spi_io_source.outputs AND reload over-current filter
//                m_outputs[port_index] = (m_outputs[port_index] & (~pin_mask)) | output_bit;

                // Set filter output flag if output bit = 0
                m_ports[input.port_index].filter_output |= (~output_bit) & pin_mask;
            }
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
            uint8_t     last_read     { 0 };    // Previous iteration inputs
            uint8_t     filtered      { 0 };    // Filtered inputs
            uint8_t     sampling      { 0 };    // Inputs that are being sampled and not yet filtered
            uint8_t     filter_output { 0 };    // Outputs to load counter with over-current filter time in the input handler routine at top
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Handler that is intended to be used as a debouncer handler of the PinScanner class
        bool debouncer_handler(const bool is_starting)
        {
            if(is_starting == true)
            {
                for(std::size_t port_index = 0; port_index < m_ports.size(); ++port_index)
                {
                    m_ports[port_index].last_read = m_ports[port_index].filtered = m_pin_source.get_current_read(port_index);
                }

                return true;
            }

            bool new_input = false;

            for(auto& input : m_inputs)
            {
                auto& port = m_ports[input.port_index];

                const uint8_t pin_mask = (1U << input.pin_bit);

                const uint32_t current_read_bit = m_pin_source.get_current_read_bit(input.port_index, input.pin_bit);
                const uint8_t last_read_bit     = port.last_read & pin_mask;

                if((port.filter_output & pin_mask) != 0)
                {
                    // Reload counter with over-current filter time
                    input.counter_ms = m_over_current_filter;

                    // Clear filter output flag
                    port.filter_output &= ~pin_mask;
                }
                else if(current_read_bit != last_read_bit)
                {
                    const uint8_t output_bit = m_outputs[input.port_index] & pin_mask;

                    // Inputs are different. Reload counter with respective filter time.
                    if(output_bit == 0 && current_read_bit != 0)
                    {
                        input.counter_ms = m_over_current_filter;
                    }
                    else if(output_bit != 0 || input.counter_ms == 0)
                    {
                        input.counter_ms = (current_read_bit == 0) ? input.low_filter_ms : input.high_filter_ms;

                        // Set sampling flag
                        port.sampling |= pin_mask;
                    }

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
                        const uint8_t filtered_bit = port.filtered & pin_mask;

                        if(current_read_bit != filtered_bit)
                        {
                            // Update filtered input
                            port.filtered = (port.filtered & (~pin_mask)) | current_read_bit;

                            // Clear sampling flag
                            port.sampling &= ~pin_mask;

                            // Set new input flag
                            new_input = true;
                        }

                        const uint8_t output_bit = m_outputs[input.port_index] & pin_mask;

                        if(current_read_bit != 0 && output_bit == 0)
                        {
                            // Unable to set output (over-current?) -> clear output
                            m_outputs[input.port_index] |= pin_mask;
                        }
                    }
                }
            }

            return new_input;
        }

        static constexpr int8_t get_port_index(const int8_t pin_index)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_index) >> 3);    // (pin_index / 8)
        }

        static constexpr int8_t get_pin_bit(const int8_t pin_index)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_index) & 0x07);  // (pin_index % 8)
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        PinSource&              m_pin_source;
        std::dynarray<PortMask> m_ports;
        std::dynarray<Input>    m_inputs;
        int16_t                 m_over_current_filter;

};




} // namespace xarmlib

#endif // __XARMLIB_API_IO_DEBOUNCER_HPP

// ----------------------------------------------------------------------------
// @file    spi_io_port_debouncer.hpp
// @brief   SPI I/O port debouncer class (based on SpiIoPort class).
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

#ifndef __XARMLIB_DEVICES_SPI_IO_PORT_DEBOUNCER_HPP
#define __XARMLIB_DEVICES_SPI_IO_PORT_DEBOUNCER_HPP

#include "spi_io_port.hpp"
#include "api/api_input_scanner.hpp"
#include "api/api_pin_bus.hpp"

namespace xarmlib
{




class SpiIoPortDebouncer
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiIoPortDebouncer(      SpiIoPort&                spi_io_port,
                           const std::size_t               port_count,
                           const std::chrono::milliseconds over_current_filter = std::chrono::milliseconds(20))
            : m_spi_io_port { spi_io_port },
              m_outputs(port_count, static_cast<uint8_t>(0xFF)),
              m_current_reads(port_count, static_cast<uint8_t>(0)),
              m_ports(port_count),
              // maximum number of pins allowed to assign: port_count * 8 pins
              m_pins(port_count << 3),
              m_assigned_pin_count { 0 },
              m_over_current_filter { 0 },
              m_is_first_debounce { true }
        {
           assert(port_count > 0);
           assert(over_current_filter.count() > 0 && over_current_filter.count() <= std::numeric_limits<int16_t>::max());

           m_over_current_filter = static_cast<int16_t>(over_current_filter.count());
        }

        void assign_pins(const PinIndexBus&              pin_index_bus,
                         const std::chrono::milliseconds low_filter,
                         const std::chrono::milliseconds high_filter)
        {
            assert(low_filter.count()  > 0 && low_filter.count()  <= std::numeric_limits<int16_t>::max());
            assert(high_filter.count() > 0 && high_filter.count() <= std::numeric_limits<int16_t>::max());

            const bool resume = InputScanner::is_running();

            InputScanner::stop();

            for(auto pin_index : pin_index_bus)
            {
                config_pin(pin_index, static_cast<int16_t>(low_filter.count()),
                                      static_cast<int16_t>(high_filter.count()));
            }

            if(resume == true)
            {
                InputScanner::resume();
            }
        }

        // Get the handler that is intended to be used as an input handler of the InputScanner class
        InputScanner::InputHandler get_input_handler()
        {
            return InputScanner::InputHandler::create<SpiIoPortDebouncer, &SpiIoPortDebouncer::input_handler>(this);
        }

        uint32_t get_filtered(const PinIndexBus& pin_index_bus)
        {
            uint32_t filtered = 0;
            std::size_t filtered_shift_index = 0;

            for(auto pin_index : pin_index_bus)
            {
                const int8_t port_index = get_port_index(pin_index);
                const int8_t pin_bit = get_pin_bit(pin_index);
                const uint8_t pin_mask = (1U << pin_bit);

                // Assertion fails if the pin wasn't previously assigned to the debouncer list
                assert((m_ports[port_index].enabled & pin_mask) != 0);

                const bool filtered_bit = (m_ports[port_index].filtered & pin_mask) != 0;

                filtered |= static_cast<uint32_t>(filtered_bit) << filtered_shift_index++;
            }

            return filtered;
        }

        uint32_t get_sampling(const PinIndexBus& pin_index_bus)
        {
            uint32_t sampling = 0;
            std::size_t sampling_shift_index = 0;

            for(auto pin_index : pin_index_bus)
            {
                const int8_t port_index = get_port_index(pin_index);
                const int8_t pin_bit = get_pin_bit(pin_index);
                const uint8_t pin_mask = (1U << pin_bit);

                // Assertion fails if the pin wasn't previously assigned to the debouncer list
                assert((m_ports[port_index].enabled & pin_mask) != 0);

                const bool sampling_bit = (m_ports[port_index].sampling & pin_mask) != 0;

                sampling |= static_cast<uint32_t>(sampling_bit) << sampling_shift_index++;
            }

            return sampling;
        }

        // Set output value to be written in the next transfer
        void set_output(const PinIndexBus& pin_index_bus, const uint32_t value)
        {
            std::size_t value_shift_index = 0;

            for(auto pin_index : pin_index_bus)
            {
                const int8_t port_index = get_port_index(pin_index);
                const int8_t pin_bit = get_pin_bit(pin_index);
                const uint8_t pin_mask = (1U << pin_bit);

                // Assertion fails if the pin wasn't previously assigned to the debouncer list
                assert((m_ports[port_index].enabled & pin_mask) != 0);

                const bool value_bit = (value & (1UL << value_shift_index++)) != 0;
                const uint8_t output_bit = static_cast<uint8_t>(value_bit) << pin_bit;

                m_outputs[port_index] = (m_outputs[port_index] & (~pin_mask)) | output_bit;

                // Set filter output flag if output bit = 0
                m_ports[port_index].filter_output |= (~output_bit) & pin_mask;
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
            uint8_t     enabled       { 0 };    // Enabled inputs
            uint8_t     filter_output { 0 };    // Outputs to load counter with over-current filter time in the input handler routine at top
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void config_pin(const int8_t pin_index, const int16_t low_filter_ms, const int16_t high_filter_ms)
        {
            const int8_t port_index = get_port_index(pin_index);
            const int8_t pin_bit = get_pin_bit(pin_index);

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
        bool input_handler()
        {
            m_spi_io_port.transfer(m_outputs, m_current_reads);

            if(m_is_first_debounce == true)
            {
                for(std::size_t port_index = 0; port_index < m_ports.size(); ++port_index)
                {
                    m_ports[port_index].last_read = m_ports[port_index].filtered = m_current_reads[port_index];
                }

                m_is_first_debounce = false;

                return true;
            }

            bool new_input = false;

            for(std::size_t assigned_pin = 0; assigned_pin < m_assigned_pin_count; ++assigned_pin)
            {
                auto& input = m_pins[assigned_pin];
                auto& port  = m_ports[input.port_index];

                const uint8_t pin_mask = (1U << input.pin_bit);

                const uint8_t current_read_bit = m_current_reads[input.port_index] & pin_mask;
                const uint8_t last_read_bit    = port.last_read                    & pin_mask;

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
                    else if((output_bit == 0 && input.counter_ms > 0) == false)
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

        SpiIoPort&              m_spi_io_port;

        std::dynarray<uint8_t>  m_outputs;
        std::dynarray<uint8_t>  m_current_reads;

        std::dynarray<PortMask> m_ports;

        std::dynarray<Input>    m_pins;
        std::size_t             m_assigned_pin_count;

        int16_t                 m_over_current_filter;

        bool                    m_is_first_debounce;
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_IO_PORT_DEBOUNCER_HPP

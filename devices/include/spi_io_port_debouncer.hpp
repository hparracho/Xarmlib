// ----------------------------------------------------------------------------
// @file    spi_io_port_debouncer.hpp
// @brief   SPI I/O port debouncer class (based on SpiIoPort class).
// @date    11 July 2018
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
#include "api/api_input_debouncer.hpp"
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

        SpiIoPortDebouncer(SpiIoPort& spi_io_port, const std::size_t port_count) : m_spi_io_port { spi_io_port },
                                                                                   m_port_count { port_count },
                                                                                   // m_ports size ceiled: ((port_count - 1) / 4) + 1)
                                                                                   m_ports(((port_count - 1) >> 2) + 1),
                                                                                   // maximum number of pins allowed to assign: port_count * 8 pins
                                                                                   m_pins(port_count << 3),
                                                                                   m_assigned_pin_count { 0 },
                                                                                   m_outputs(m_ports.size(), static_cast<uint32_t>(0xFFFFFFFF)),
                                                                                   m_is_first_debounce { true }
        {
           assert(port_count > 0);
        }

        void assign_input_pins(const PinIndexBus&              pin_index_bus,
                               const std::chrono::milliseconds filter_ms_low,
                               const std::chrono::milliseconds filter_ms_high)
        {
            config_pins(pin_index_bus, filter_ms_low, filter_ms_high);
        }

        void assign_output_pins(const PinIndexBus& pin_index_bus)
        {
            config_pins(pin_index_bus, std::chrono::milliseconds(20), std::chrono::milliseconds(20));
        }

        void assign_io_pins(const PinIndexBus&              pin_index_bus,
                            const std::chrono::milliseconds filter_ms_low)
        {
            config_pins(pin_index_bus, filter_ms_low, std::chrono::milliseconds(20));
        }

        // Get the handler that is intended to be used as an input handler of the InputScanner class
        InputScanner::InputHandler get_input_handler()
        {
            return InputScanner::InputHandler::create<SpiIoPortDebouncer, &SpiIoPortDebouncer::debounce_handler>(this);
        }

        uint32_t get_filtered(const PinIndexBus& pin_index_bus)
        {
            uint32_t filtered = 0;
            std::size_t filtered_shift_index = 0;

            for(auto pin_index : pin_index_bus)
            {
                const int8_t port_index = get_port_index(pin_index);
                const int8_t pin_bit = get_pin_bit(pin_index);
                const uint32_t pin_mask = (1UL << pin_bit);

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
                const uint32_t pin_mask = (1UL << pin_bit);

                // Assertion fails if the pin wasn't previously assigned to the debouncer list
                assert((m_ports[port_index].enabled & pin_mask) != 0);

                const bool sampling_bit = (m_ports[port_index].sampling & pin_mask) != 0;

                sampling |= static_cast<uint32_t>(sampling_bit) << sampling_shift_index++;
            }

            return sampling;
        }

        // Update output value to be written in the next transfer
        void update_output(const PinIndexBus& pin_index_bus, const uint32_t value)
        {
            std::size_t value_shift_index = 0;

            for(auto pin_index : pin_index_bus)
            {
                const int8_t port_index = get_port_index(pin_index);
                const int8_t pin_bit = get_pin_bit(pin_index);
                const uint32_t pin_mask = (1UL << pin_bit);

                // Assertion fails if the pin wasn't previously assigned to the debouncer list
                assert((m_ports[port_index].enabled & pin_mask) != 0);

                const bool value_bit = (value & (1UL << value_shift_index++)) != 0;

                m_outputs[port_index] = (m_outputs[port_index] & (~pin_mask)) | (static_cast<uint32_t>(value_bit) << pin_bit);

                m_ports[port_index].sampling |= pin_mask;

                for(std::size_t assigned_pin = 0; assigned_pin < m_assigned_pin_count; ++assigned_pin)
                {
                    if(m_pins[assigned_pin].port_index == port_index && m_pins[assigned_pin].pin_bit == pin_bit)
                    {
                        m_pins[assigned_pin].counter_ms = m_pins[assigned_pin].filter_high_ms;

                        break;
                    }
                }
            }
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        void config_pins(const PinIndexBus&               pin_index_bus,
                         const std::chrono::milliseconds& filter_low_ms,
                         const std::chrono::milliseconds& filter_high_ms)
        {
            assert(filter_low_ms.count()  > 0 && filter_low_ms.count()  <= std::numeric_limits<int16_t>::max());
            assert(filter_high_ms.count() > 0 && filter_high_ms.count() <= std::numeric_limits<int16_t>::max());

            const bool resume = InputScanner::is_running();

            InputScanner::stop();

            for(auto pin_index : pin_index_bus)
            {
                config_pin(pin_index, static_cast<int16_t>(filter_low_ms.count()),
                                      static_cast<int16_t>(filter_high_ms.count()));
            }

            if(resume == true)
            {
                InputScanner::resume();
            }
        }

        void config_pin(const int8_t pin_index, const int16_t filter_low_ms, const int16_t filter_high_ms)
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

            m_pins[assigned_pin] = {port_index, pin_bit, filter_low_ms, filter_high_ms, 0 };
        }

        // Debounce handler that is intended to be used as an input handler of the InputScanner class
        bool debounce_handler()
        {
            m_spi_io_port.transfer(m_outputs, m_ports, m_port_count);

            if(m_is_first_debounce == true)
            {
                for(auto& port : m_ports)
                {
                    port.last_read = port.filtered = port.current_read;
                    //port.sampling = 0;
                }

                m_is_first_debounce = false;

                return true;
            }

            const bool result = InputDebouncer::debounce((gsl::span<InputDebouncer::Input>{ m_pins }).first(m_assigned_pin_count), m_ports);

            for(std::size_t port_index = 0; port_index < m_ports.size(); ++port_index)
            {
                const auto port = m_ports[port_index];

                // If sampling == 0 && filtered != outputs => unable to set output (over-current?) -> clear output
                m_outputs[port_index] |= (port.filtered & (~port.sampling)) ^ m_outputs[port_index];
            }

            return result;
        }

        static constexpr int8_t get_port_index(const int8_t pin_index)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_index) >> 5);    // (pin_index / 32)
        }

        static constexpr int8_t get_pin_bit(const int8_t pin_index)
        {
            return static_cast<int8_t>(static_cast<std::size_t>(pin_index) & 0x1F);  // (pin_index % 32)
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        SpiIoPort&                              m_spi_io_port;
        const std::size_t                       m_port_count;

        std::dynarray<InputDebouncer::PortMask> m_ports;

        std::dynarray<InputDebouncer::Input>    m_pins;
        std::size_t                             m_assigned_pin_count;

        std::dynarray<uint32_t>                 m_outputs;

        bool                                    m_is_first_debounce;
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_IO_PORT_DEBOUNCER_HPP

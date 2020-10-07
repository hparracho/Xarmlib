// ----------------------------------------------------------------------------
// @file    api_output_driver.hpp
// @brief   API output driver class.
// @date    8 September 2020
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
/*
#ifndef __XARMLIB_API_OUTPUT_DRIVER_HPP
#define __XARMLIB_API_OUTPUT_DRIVER_HPP

#include "devices/spi_io_source.hpp"
#include "api/api_gpio_source.hpp"
#include "api/api_pin_bus.hpp"
#include "hal/hal_gpio.hpp"

namespace xarmlib
{




template <PinPolarity Polarity>
class OutputDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Constructor for a generic PinSource not using GPIO
        template <typename PinSource>
        OutputDriver(      PinSource&   pin_source,
                     const PinIndexBus& pin_index_bus) : m_pin_source { pin_source },
                                                         m_outputs(pin_index_bus.get_size()),
                                                         m_output_bus { get_default_output_bus(pin_index_bus) }
        {
            config_pins<PinSource>(pin_index_bus);
        }

        OutputDriver(      GpioSource<Polarity>&        gpio_source,
                     const PinNameBus&                  pin_name_bus,
                     const hal::Gpio::OutputModeConfig& pin_bus_config) : m_pin_source { gpio_source },
                                                                          m_outputs(pin_name_bus.get_size()),
                                                                          m_output_bus { get_default_output_bus(pin_name_bus) }
        {
            for(const auto pin_name : pin_name_bus)
            {
                hal::Gpio gpio(pin_name, pin_bus_config);
            }

            config_pins<GpioSource<Polarity>>(pin_name_bus);
        }

#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        OutputDriver(      GpioSource<Polarity>&                     gpio_source,
                     const PinNameBus&                               pin_name_bus,
                     const hal::Gpio::OutputModeTrueOpenDrainConfig& pin_bus_config) : m_pin_source { gpio_source },
                                                                                       m_outputs(pin_name_bus.get_size()),
                                                                                       m_output_bus { get_default_output_bus(pin_name_bus) }
        {
            for(const auto pin_name : pin_name_bus)
            {
                hal::Gpio gpio(pin_name, pin_bus_config);
            }

            config_pins<GpioSource<Polarity>>(pin_name_bus);
        }
#endif

        OutputDriver(      SpiIoSource<Polarity>& spi_io_source,
                     const PinIndexBus&           pin_index_bus) : m_pin_source { spi_io_source },
                                                                   m_outputs(pin_index_bus.get_size()),
                                                                   m_output_bus { get_default_output_bus(pin_index_bus) }
        {
            config_pins<SpiIoSource<Polarity>>(pin_index_bus);
        }

        uint32_t get_output_bus() const
        {
            return m_output_bus;
        }

        // NOTE: the value will be written in the next transfer
        void set_output_bus(const uint32_t mask)
        {
            for(std::size_t output_index = 0; output_index < m_outputs.size(); ++output_index)
            {
                const uint32_t output_mask = 1UL << output_index;

                if((mask & output_mask) != 0)
                {
                    auto& output = m_outputs[output_index];

                    const uint32_t output_bit = 1UL << output.pin_bit;

                    m_pin_source.write_output_bit(output.port_index, output.pin_bit, output_bit);

                    m_output_bus |= output_mask;
                }
            }
        }

        // NOTE: the value will be written in the next transfer
        void clear_output_bus(const uint32_t mask)
        {
            for(std::size_t output_index = 0; output_index < m_outputs.size(); ++output_index)
            {
                const uint32_t output_mask = 1UL << output_index;

                if((mask & output_mask) != 0)
                {
                    auto& output = m_outputs[output_index];

                    m_pin_source.write_output_bit(output.port_index, output.pin_bit, 0);

                    m_output_bus &= ~output_mask;
                }
            }
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        struct Output
        {
            int8_t      port_index { -1 };  // Output's port index
            int8_t      pin_bit    { -1 };  // Output's bit within a port
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template <class PinBusType>
        static constexpr uint32_t get_default_output_bus(const PinBus<PinBusType>& pin_bus)
        {
            return (Polarity == PinPolarity::negative) ? pin_bus.get_mask() : 0;
        }

        template <typename PinBusSource, class PinBusType>
        void config_pins(const PinBus<PinBusType>& pin_bus)
        {
            std::size_t assigned_output = 0;

            const bool resume = PinScanner::is_running();

            PinScanner::stop();

            for(auto pin : pin_bus)
            {
                const int8_t port_index = PinBusSource::get_port_index(pin);
                const int8_t pin_bit = PinBusSource::get_pin_bit(pin);

                m_outputs[assigned_output++] = { port_index, pin_bit };
            }

            if(resume == true)
            {
                PinScanner::resume();
            }
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        PinSource&            m_pin_source;
        std::dynarray<Output> m_outputs;

        // Bitmask aligned with PinBus
        uint32_t              m_output_bus;	// Outputs written
};




} // namespace xarmlib

#endif // __XARMLIB_API_OUTPUT_DRIVER_HPP
*/

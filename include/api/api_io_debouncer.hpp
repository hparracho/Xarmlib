// ----------------------------------------------------------------------------
// @file    api_io_debouncer.hpp
// @brief   API I/O debouncer class.
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
                    const int16_t               scan_time_output_error_samples,
                    const Gpio::InputMode       input_mode,
                    const Gpio::InputInvert     input_invert     = Gpio::InputInvert::NORMAL,
                    const Gpio::InputHysteresis input_hysteresis = Gpio::InputHysteresis::ENABLE) : m_pin_source { gpio_source },
                                                                                                    m_ios(pin_name_bus.get_size()),
                                                                                                    m_low_samples { scan_time_low_samples },
                                                                                                    m_high_samples { scan_time_high_samples },
                                                                                                    m_output_error_samples { scan_time_output_error_samples },
                                                                                                    m_last_read_bus { 0 },
                                                                                                    m_filtered_bus { 0 },
                                                                                                    m_sampling_bus { 0 },
                                                                                                    m_input_error_bus { 0 },
                                                                                                    m_output_error_bus { 0 },
                                                                                                    m_output_bus { pin_name_bus.get_mask() }
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert, input_hysteresis);
            }

            config_pins<GpioSource>(pin_name_bus);
        }

        IoDebouncer(      GpioSource&                  gpio_source,
                    const PinNameBus&                  pin_name_bus,
                    const int16_t                      scan_time_low_samples,
                    const int16_t                      scan_time_high_samples,
                    const int16_t                      scan_time_output_error_samples,
                    const Gpio::InputModeTrueOpenDrain input_mode,
                    const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL) : m_pin_source { gpio_source },
                                                                                                   m_ios(pin_name_bus.get_size()),
                                                                                                   m_low_samples { scan_time_low_samples },
                                                                                                   m_high_samples { scan_time_high_samples },
                                                                                                   m_output_error_samples { scan_time_output_error_samples },
                                                                                                   m_last_read_bus { 0 },
                                                                                                   m_filtered_bus { 0 },
                                                                                                   m_sampling_bus { 0 },
                                                                                                   m_input_error_bus { 0 },
                                                                                                   m_output_error_bus { 0 },
                                                                                                   m_output_bus { pin_name_bus.get_mask() }
        {
            for(const auto pin_name : pin_name_bus)
            {
                Gpio gpio(pin_name, input_mode, Gpio::InputFilter::BYPASS, input_invert);
            }

            config_pins<GpioSource>(pin_name_bus);
        }

        IoDebouncer(      SpiIoSource& spi_io_source,
                    const PinIndexBus& pin_index_bus,
                    const int16_t      scan_time_low_samples,
                    const int16_t      scan_time_high_samples,
                    const int16_t      scan_time_output_error_samples) : m_pin_source { spi_io_source },
                                                                         m_ios(pin_index_bus.get_size()),
                                                                         m_low_samples { scan_time_low_samples },
                                                                         m_high_samples { scan_time_high_samples },
                                                                         m_output_error_samples { scan_time_output_error_samples },
                                                                         m_last_read_bus { 0 },
                                                                         m_filtered_bus { 0 },
                                                                         m_sampling_bus { 0 },
                                                                         m_input_error_bus { 0 },
                                                                         m_output_error_bus { 0 },
                                                                         m_output_bus { pin_index_bus.get_mask() }
        {
            config_pins<SpiIoSource>(pin_index_bus);
        }

        // Get the handler that is intended to be used as a debouncer handler of the PinScanner class
        PinScanner::DebouncerHandler get_debouncer_handler()
        {
            return PinScanner::DebouncerHandler::create<IoDebouncer, &IoDebouncer::debouncer_handler>(this);
        }

        uint32_t get_filtered_bus() const
        {
            return m_filtered_bus;
        }

        uint32_t get_sampling_bus() const
        {
            return m_sampling_bus;
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
        //        - the maskable pins behaves as input
        void set_output_bus(const uint32_t mask)
        {
            for(std::size_t io_index = 0; io_index < m_ios.size(); ++io_index)
            {
                const uint32_t io_mask = 1UL << io_index;

                if((mask & io_mask) != 0)
                {
                    auto& io = m_ios[io_index];

                    const uint32_t output_bit = 1UL << io.pin_bit;

                    m_pin_source.write_output_bit(io.port_index, io.pin_bit, output_bit);

                    m_output_bus   |=  io_mask;
                    m_filtered_bus |=  io_mask;;
                    m_sampling_bus &= ~io_mask;

                    io.counter = m_output_error_samples;
                }
            }
        }

        // NOTES: - the value will be written in the next transfer
        //        - the maskable pins behaves as output
        void clear_output_bus(const uint32_t mask)
        {
            for(std::size_t io_index = 0; io_index < m_ios.size(); ++io_index)
            {
                const uint32_t io_mask = 1UL << io_index;

                if((mask & io_mask) != 0)
                {
                    auto& io = m_ios[io_index];

                    m_pin_source.write_output_bit(io.port_index, io.pin_bit, 0);

                    m_output_bus   &= ~io_mask;
                    m_filtered_bus &= ~io_mask;;
                    m_sampling_bus &= ~io_mask;

                    io.counter = m_output_error_samples;
                }
            }
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        struct Io
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
            assert(m_low_samples          > 1);
            assert(m_high_samples         > 1);
            assert(m_output_error_samples > 1);

            assert(m_low_samples  > m_output_error_samples);
            assert(m_high_samples > m_output_error_samples);

            std::size_t assigned_io = 0;

            const bool resume = PinScanner::is_running();

            PinScanner::stop();

            for(auto pin : pin_bus)
            {
                const int8_t port_index = PinBusSource::get_port_index(pin);
                const int8_t pin_bit = PinBusSource::get_pin_bit(pin);

                m_ios[assigned_io++] = { port_index, pin_bit, 0 };
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
                for(std::size_t io_index = 0; io_index < m_ios.size(); ++io_index)
                {
                    const auto io = m_ios[io_index];

                    const bool read_bit = m_pin_source.get_read_bit(io.port_index, io.pin_bit) != 0;

                    m_last_read_bus |= static_cast<uint32_t>(read_bit) << io_index;
                }

                m_filtered_bus = m_last_read_bus;

                m_input_error_bus = ~m_last_read_bus & static_cast<uint32_t>((1UL << m_ios.size()) - 1);

                return true;
            }

            bool new_input = false;

            for(std::size_t io_index = 0; io_index < m_ios.size(); ++io_index)
            {
                auto& io = m_ios[io_index];

                const uint32_t io_mask = 1UL << io_index;

                const bool current_read_bit = m_pin_source.get_read_bit(io.port_index, io.pin_bit) != 0;
                const bool last_read_bit    = (m_last_read_bus & io_mask) != 0;

                if(current_read_bit != last_read_bit)
                {
                    // Inputs are different

                    const bool output_bit = (m_output_bus & io_mask) != 0;

                    if(output_bit != 0)
                    {
                        // Input mode

                        if(current_read_bit == 0)
                        {
                            // Reload counter with low samples
                            io.counter = m_low_samples;
                        }
                        else
                        {
                            // Reload counter with high samples
                            io.counter = m_high_samples;

                            // Clear input error flag
                            m_input_error_bus &= ~io_mask;
                        }

                        // Set sampling flag
                        m_sampling_bus |= io_mask;
                    }
                    else
                    {
                        // Output mode

                        // Sampling possible over-current or stop
                        io.counter = (current_read_bit != 0) ? m_output_error_samples : 0;
                    }

                    // Update last read io
                    m_last_read_bus = (m_last_read_bus & (~io_mask)) | static_cast<uint32_t>(current_read_bit) << io_index;
                }
                else
                {
                    if(io.counter > 0)
                    {
                        io.counter--;

                        if(io.counter == 0)
                        {
                            if((m_sampling_bus & io_mask) != 0)
                            {
                                const bool filtered_bit = m_filtered_bus & io_mask;

                                if(current_read_bit != filtered_bit)
                                {
                                    // Update filtered input
                                    m_filtered_bus = (m_filtered_bus & (~io_mask)) | static_cast<uint32_t>(current_read_bit) << io_index;

                                    // Set new input flag
                                    new_input = true;
                                }

                                // Clear sampling flag
                                m_sampling_bus &= ~io_mask;
                            }
                            else
                            {
                                const bool output_bit = (m_output_bus & io_mask) != 0;

                                if(output_bit != current_read_bit)
                                {
                                    if(current_read_bit == 0)
                                    {
                                        // Set input error flag
                                        m_input_error_bus |= io_mask;
                                    }
                                    else
                                    {
                                        // Set output error flag
                                        m_output_error_bus |= io_mask;

                                        // Unable to clear output (over-current?) -> set output
                                        m_pin_source.write_output_bit(io.port_index, io.pin_bit, 1UL << io.pin_bit);

                                        m_output_bus   |= io_mask;
                                        m_filtered_bus |= io_mask;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            return new_input;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        PinSource&        m_pin_source;
        std::dynarray<Io> m_ios;

        const int16_t     m_low_samples;            // Number of samples of scan time that a pin must be steady at low level to be accepted as filtered (debounced)
        const int16_t     m_high_samples;           // Number of samples of scan time that a pin must be steady at high level to be accepted as filtered (debounced)
        const int16_t     m_output_error_samples;   // Number of samples of scan time that a output must be equal to the input to valid the output written

        // Bitmasks aligned with PinBus
        uint32_t          m_last_read_bus;          // Previous iteration inputs
        uint32_t          m_filtered_bus;           // Filtered inputs
        uint32_t          m_sampling_bus;           // Inputs that are being sampled and not yet filtered

        uint32_t          m_input_error_bus;
        uint32_t          m_output_error_bus;

        uint32_t          m_output_bus;             // Outputs written
};




} // namespace xarmlib

#endif // __XARMLIB_API_IO_DEBOUNCER_HPP

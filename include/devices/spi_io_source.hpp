// ----------------------------------------------------------------------------
// @file    spi_io_source.hpp
// @brief   SPI I/O source class.
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

#ifndef __XARMLIB_DEVICES_SPI_IO_SOURCE_HPP
#define __XARMLIB_DEVICES_SPI_IO_SOURCE_HPP

#include "api/api_digital_out.hpp"
#include "api/api_pin_source.hpp"
#include "hal/hal_spi.hpp"

namespace xarmlib
{




template <PinPolarity Polarity>
class SpiIoSource : public PinSource
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiIoSource(      hal::SpiMaster& spi_master,
                    const hal::Pin::Name  latch,
                    const hal::Pin::Name  enable,
                    const std::size_t     port_count) : m_spi_master { spi_master },
                                                        m_latch(latch, { hal::Gpio::OutputMode::push_pull_high }),
                                                        m_enable(enable, { hal::Gpio::OutputMode::push_pull_high }),
                                                        m_outputs(port_count, get_default_outputs()),
                                                        m_reads(port_count, static_cast<uint8_t>(0))
        {
            assert(spi_master.is_enabled() == true);
            assert(port_count > 0);
        }

        // Get the handler that is intended to be used as a pin source reader handler of the PinScanner class
        PinScanner::PinSourceHandler get_pin_source_handler() override
        {
            return PinScanner::PinSourceHandler::create<SpiIoSource, &SpiIoSource::pin_source_handler>(this);
        }

        std::size_t get_port_count() const override
        {
            return m_reads.size();
        }

        uint32_t get_read(const std::size_t port_index) const override
        {
            assert(port_index < m_reads.size());

            return m_reads[port_index];
        }

        uint32_t get_read_bit(const std::size_t port_index, const std::size_t pin_bit) const override
        {
            assert(port_index < m_reads.size());
            assert(pin_bit < 8);

            return m_reads[port_index] & (1UL << pin_bit);
        }

        uint32_t get_output_bit(const std::size_t port_index, const std::size_t pin_bit) const override
        {
            assert(port_index < m_outputs.size());
            assert(pin_bit < 8);

            return m_outputs[port_index] & (1UL << pin_bit);
        }

        void write_output_bit(const std::size_t port_index, const std::size_t pin_bit, const uint32_t output_bit) override
        {
            assert(port_index < m_outputs.size());
            assert(pin_bit < 8);

            const uint32_t pin_mask = (1UL << pin_bit);

            m_outputs[port_index] = static_cast<uint8_t>((m_outputs[port_index] & ~pin_mask) | (output_bit & pin_mask));
        }

        static constexpr int8_t get_port_index(const int8_t pin_index)
        {
            assert(pin_index >= 0);

            return static_cast<int8_t>(static_cast<std::size_t>(pin_index) >> 3);    // (pin_index / 8)
        }

        static constexpr int8_t get_pin_bit(const int8_t pin_index)
        {
            assert(pin_index >= 0);

            return static_cast<int8_t>(static_cast<std::size_t>(pin_index) & 0x07);  // (pin_index % 8)
        }

        // Transfer a buffer (simultaneous write and read)
        void transfer(const std::span<const uint8_t> output_buffer, const std::span<uint8_t> input_buffer)
        {
            assert(output_buffer.size() > 0);
            assert(output_buffer.size() == input_buffer.size());

            m_latch = 0;
            m_latch = 1;

            m_spi_master.mutex_take();

            for(std::ptrdiff_t port_index = output_buffer.size() - 1; port_index >= 0; --port_index)
            {
                input_buffer[port_index] = static_cast<uint8_t>(m_spi_master.transfer(output_buffer[port_index]));
            }

            m_spi_master.mutex_give();

            m_latch = 0;
            m_latch = 1;
        }

        // Enable IO port
        void enable() { m_enable = 0; }

        // Disable IO port
        void disable() { m_enable = 1; }

        // Gets the enable state
        bool is_enabled() const { return (m_enable == 0); }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr uint8_t get_default_outputs()
        {
            return (Polarity == PinPolarity::negative) ? static_cast<uint8_t>(0xFF) : 0;
        }

        void pin_source_handler()
        {
            transfer(m_outputs, m_reads);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        hal::SpiMaster&        m_spi_master;
        DigitalOut             m_latch;
        DigitalOut             m_enable;

        std::dynarray<uint8_t> m_outputs;
        std::dynarray<uint8_t> m_reads;
};




using SpiPositiveIoSource = SpiIoSource<PinPolarity::positive>;
using SpiNegativeIoSource = SpiIoSource<PinPolarity::negative>;




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_IO_SOURCE_HPP

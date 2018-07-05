// ----------------------------------------------------------------------------
// @file    spi_io_port.hpp
// @brief   SPI I/O port class (based on module FPIO8SM).
// @date    5 July 2018
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

#ifndef __XARMLIB_DEVICES_SPI_IO_PORT_HPP
#define __XARMLIB_DEVICES_SPI_IO_PORT_HPP

#include "api/api_digital_out.hpp"
#include "hal/hal_spi.hpp"

namespace xarmlib
{




class SpiIoPort
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiIoPort(      SpiMaster&  spi_master,
                  const Pin::Name   latch,
                  const Pin::Name   enable) : m_spi_master { spi_master },
                                              m_latch(latch, Gpio::OutputMode::PUSH_PULL_HIGH),
                                              m_enable(enable, Gpio::OutputMode::PUSH_PULL_HIGH)
        {
            assert(spi_master.is_enabled() == true);
        }

        // Transfer a buffer (simultaneous write and read)
        void transfer(const gsl::span<const uint8_t> output_buffer, const gsl::span<uint8_t> input_buffer)
        {
            assert(output_buffer.size() == input_buffer.size());
            assert(output_buffer.size() > 0);

            m_latch = 0;
            m_latch = 1;

            m_spi_master.mutex_take();

            for(std::ptrdiff_t port_index = output_buffer.size() - 1; port_index >= 0; --port_index)
            {
                input_buffer[port_index] = m_spi_master.transfer(output_buffer[port_index]);
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
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        SpiMaster&  m_spi_master;
        DigitalOut  m_latch;
        DigitalOut  m_enable;
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_IO_PORT_HPP

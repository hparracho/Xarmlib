// ----------------------------------------------------------------------------
// @file    devices_spi_io_module.hpp
// @brief   Devices SPI I/O module (FPIO8SM) class.
// @date    4 July 2018
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

#ifndef __XARMLIB_DEVICES_SPI_IO_MODULE_HPP
#define __XARMLIB_DEVICES_SPI_IO_MODULE_HPP

#include "api/api_digital_out.hpp"
#include "hal/hal_spi.hpp"

namespace xarmlib
{




class SpiIoModule
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiIoModule(      SpiMaster&  spi_master,
                    const Pin::Name   latch,
                    const Pin::Name   enable,
                    const std::size_t count) : m_spi_master { spi_master },
                                               m_latch(latch, Gpio::OutputMode::PUSH_PULL_HIGH),
                                               m_enable(enable, Gpio::OutputMode::PUSH_PULL_HIGH),
                                               m_count { count }
        {
            assert(count >= 1 && count <= 8);
            assert(spi_master.is_enabled() == true);

            // Clear shift registers
            transfer(0xFFFFFFFFFFFFFFFF);
        }

        // Transfer a value (simultaneous write and read)
        // NOTE: Return the input value
        uint32_t transfer(const uint32_t output_value)
        {
            assert(m_count <= 4);

            return transfer<uint32_t>(output_value);
        }

        uint64_t transfer(const uint64_t output_value)
        {
            return transfer<uint64_t>(output_value);
        }

        // Enable IO module
        void enable() { m_enable = 0; }

        // Disable IO module
        void disable() { m_enable = 1; }

        // Gets the enable state
        bool is_enabled() const { return (m_enable == 0); }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template<typename T>
        T transfer(const T output_value)
        {
            T input_value = 0;

            int32_t shift = get_shift(m_count);

            m_latch = 0;
            m_latch = 1;

            m_spi_master.MutexTake();
            for(std::size_t m = 0; m < m_count; ++m)
            {
                const uint32_t output_module = (static_cast<uint32_t>(output_value >> shift)) & 0xFF;

                const uint32_t input_module = m_spi_master.transfer(output_module) & 0xFF;

                input_value |= static_cast<T>(input_module) << shift;

                shift -= 8;
            }
            m_spi_master.MutexGive();

            m_latch = 0;
            m_latch = 1;

            return input_value;
        }

        static constexpr int32_t get_shift(const std::size_t count)
        {
            return (count - 1) * 8;
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

              SpiMaster&    m_spi_master;
              DigitalOut    m_latch;
              DigitalOut    m_enable;
        const std::size_t   m_count;
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_IO_MODULE_HPP

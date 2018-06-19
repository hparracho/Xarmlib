// ----------------------------------------------------------------------------
// @file    api_spi_io_module.hpp
// @brief   API SPI I/O module (FPIO8SM) class.
// @date    18 June 2018
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

#ifndef __XARMLIB_API_SPI_IO_MODULE_HPP
#define __XARMLIB_API_SPI_IO_MODULE_HPP

#include "hal/hal_spi.hpp"
#include "api/api_digital_out.hpp"

namespace xarmlib
{




namespace private_spi_io_module
{

template<std::size_t IO_MODULE_COUNT_MAX>
struct is_io_module_count_max_1_to_8
{
    static constexpr bool value = IO_MODULE_COUNT_MAX >= 1 && IO_MODULE_COUNT_MAX <= 8;
};

} // namespace private_spi_io_module




// SpiIoModule is enabled via a template parameter
template<std::size_t IO_MODULE_COUNT_MAX, class Enable = void>
class SpiIoModule; // undefined

template<std::size_t IO_MODULE_COUNT_MAX>
class SpiIoModule<IO_MODULE_COUNT_MAX, typename std::enable_if<private_spi_io_module::is_io_module_count_max_1_to_8<IO_MODULE_COUNT_MAX>::value>::type>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        using Type = typename std::conditional<IO_MODULE_COUNT_MAX <= 4, uint32_t, uint64_t>::type;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiIoModule(SpiMaster& spi_master, const Pin::Name latch, const Pin::Name enable) : m_spi_master(spi_master),
                                                                                            m_latch(latch, DigitalOut::OutputMode::PUSH_PULL_HIGH),
                                                                                            m_enable(enable, DigitalOut::OutputMode::PUSH_PULL_HIGH)
        {
            // Just to make sure SPI is enable
            m_spi_master.enable();

            // Clear shift registers
            read_write(static_cast<Type>(0xFFFFFFFFFFFFFFFF));
        }

        // Reads and writes to all modules
        Type read_write(const Type output_value)
        {
            return transfer(IO_MODULE_COUNT_MAX, output_value);
        }

        // Reads and writes to a specified number of modules (up to IO_MODULE_COUNT_MAX)
        Type read_write(const std::size_t io_module_count, const Type output_value)
        {
            assert(io_module_count >= 1 && io_module_count <= IO_MODULE_COUNT_MAX);

            return transfer(io_module_count, output_value);
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

        Type transfer(const std::size_t io_module_count, const Type output_value)
        {
            Type input_value = 0;

            int32_t shift = get_shift(io_module_count);

            m_latch = 0;
            m_latch = 1;

            for(std::size_t m = 0; m < io_module_count; m++)
            {
                const uint32_t output_module = (static_cast<uint32_t>(output_value >> shift)) & 0xFF;

                const uint32_t input_module = m_spi_master.transfer(output_module) & 0xFF;

                input_value |= static_cast<Type>(input_module) << shift;

                shift -= 8;
            }

            m_latch = 0;
            m_latch = 1;

            return input_value;
        }

        static constexpr int32_t get_shift(const std::size_t io_module_count)
        {
            return (io_module_count - 1) * 8;
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        SpiMaster&  m_spi_master;

        DigitalOut  m_latch;
        DigitalOut  m_enable;
};




} // namespace xarmlib

#endif // __XARMLIB_API_SPI_IO_MODULE_HPP

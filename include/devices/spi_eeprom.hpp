// ----------------------------------------------------------------------------
// @file    spi_eeprom.hpp
// @brief   SPI EEPROM class.
// @note    Implemented at least for the families M95xxx-W, M95xxx-R and
//          M95xxx-DF. The M95xxx-D offers an identification page, whose
//          functions are not implemented.
// @date    20 September 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_DEVICES_SPI_EEPROM_HPP
#define __XARMLIB_DEVICES_SPI_EEPROM_HPP

#include "api/api_digital_out.hpp"
#include "hal/hal_spi.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{




template<std::size_t Capacity, std::size_t PageSize>
class SpiEeprom
{
    // SPI M95xxx: 1 Kbit to 2 Mbits
    static_assert(Capacity >= 128 && Capacity <= 262144 && (Capacity % 128) == 0, "SPI EEPROM: Invalid capacity supplied");
    static_assert(PageSize > 0 && (Capacity % PageSize) == 0, "SPI EEPROM: Invalid page size supplied");

    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiEeprom(      hal::SpiMaster& spi_master,
                  const hal::Pin::Name  cs,
                  const hal::Pin::Name  wp,
                  const bool            readonly): m_spi_master { spi_master },
                                                   m_cs(cs, { hal::Gpio::OutputMode::push_pull_high }),
                                                   m_wp(wp, { hal::Gpio::OutputMode::push_pull_low })
        {
            if(readonly != is_readonly())
            {
                m_spi_master.mutex_take();

                const bool success = set_readonly(readonly);

                m_spi_master.mutex_give();

                assert(success == true);
                (void)success;
            }
        }

        bool is_readonly()
        {
            // Read SRWD, BP1 and BP0
            constexpr uint8_t status_protect = (1 << STATUS_REGISTER_WRITE_DISABLE_BIT) |
                                               (1 << STATUS_BLOCK_PROTECT_1_BIT)        |
                                               (1 << STATUS_BLOCK_PROTECT_0_BIT);

            m_spi_master.mutex_take();

            if(wait_for_ready() == false)
            {
                m_spi_master.mutex_give();

                return false;
            }

            const bool is_readonly = ((get_status_register() & status_protect) == status_protect);

            m_spi_master.mutex_give();

            return is_readonly;
        }

        bool read(const uint32_t address, uint8_t *data_array, std::size_t data_array_size)
        {
            assert(data_array != nullptr);
            assert((address + data_array_size - 1) <= (Capacity - 1));

            m_spi_master.mutex_take();

            const bool success = wait_for_ready();

            if(success == true)
            {
                m_cs = 0;

                m_spi_master.transfer(CMD_READ_FROM_MEMORY_ARRAY);

                send_address(address);

                while(data_array_size-- > 0)
                {
                    // Read data
                    *data_array++ = m_spi_master.transfer(0xFF);
                }

                m_cs = 1;
            }

            m_spi_master.mutex_give();

            return success;
        }

        bool write(uint32_t address, const uint8_t* data_array, const std::size_t data_array_size)
        {
            assert(data_array != nullptr);
            assert((address + data_array_size - 1) <= (Capacity - 1));

            // Calculate the byte within the first page
            const std::size_t byte = address % PageSize;

            const std::size_t first_page_size = (data_array_size > (PageSize - byte)) ? PageSize - byte : data_array_size;

            m_spi_master.mutex_take();

            // First page, partial or complete
            if(write_page(address, data_array, first_page_size) == false)
            {
                m_spi_master.mutex_give();

                return false;
            }

            address    += first_page_size;
            data_array += first_page_size;

            std::ptrdiff_t size = data_array_size - first_page_size;

            while(size > 0)
            {
                const std::size_t write_size = (static_cast<std::size_t>(size) > PageSize) ? PageSize : size;

                // Remaining pages...
                if(write_page(address, data_array, write_size) == false)
                {
                    m_spi_master.mutex_give();

                    return false;
                }

                address    += write_size;
                data_array += write_size;
                size       -= write_size;
            }

            m_spi_master.mutex_give();

            return true;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE TYPE ALIASES
        // --------------------------------------------------------------------

        using UsTicker = hal::UsTicker;

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // Status bits and masks
        enum STATUS : uint8_t
        {
            STATUS_WRITE_IN_PROGRESS_BIT      = 0, // WIP
            STATUS_WRITE_ENABLE_LATCH_BIT     = 1, // WEL
            STATUS_BLOCK_PROTECT_0_BIT        = 2, // BP0
            STATUS_BLOCK_PROTECT_1_BIT        = 3, // BP1
            STATUS_REGISTER_WRITE_DISABLE_BIT = 7, // SRWD
            STATUS_MASK                       = 0x8F
        };

        // Command definitions
        enum CMD : uint8_t
        {
            CMD_WRITE_ENABLE           = 0x06,
            CMD_WRITE_DISABLE          = 0x04,
            CMD_READ_STATUS_REGISTER   = 0x05,
            CMD_WRITE_STATUS_REGISTER  = 0x01,
            CMD_READ_FROM_MEMORY_ARRAY = 0x03,
            CMD_WRITE_TO_MEMORY_ARRAY  = 0x02
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        uint8_t get_status_register()
        {
            m_cs = 0;

            m_spi_master.transfer(CMD_READ_STATUS_REGISTER);

            // Read status (reserved bit masked out)
            const uint8_t data = m_spi_master.transfer(0xFF) & STATUS_MASK;

            m_cs = 1;

            return data;
        }

        bool is_busy()
        {
            return ((get_status_register() & (1 << STATUS_WRITE_IN_PROGRESS_BIT)) != 0);
        }

        bool wait_for_ready()
        {
            const auto start = UsTicker::now();

            do
            {
                if(is_busy() == false)
                {
                    return true;
                }
            }while(UsTicker::is_timeout(start, std::chrono::milliseconds(500)) == false);

            return false;
        }

        void enable_write()
        {
            m_cs = 0;

            m_spi_master.transfer(CMD_WRITE_ENABLE);

            m_cs = 1;
        }

        bool set_readonly(const bool enable)
        {
            if(wait_for_ready() == true)
            {
                // case enable:  set SRWD and set   BP1 and BP0
                // case disable: set SRWD and clear BP1 and BP0
                const uint8_t status_protect = (enable == true) ? (1 << STATUS_REGISTER_WRITE_DISABLE_BIT) |
                                                                  (1 << STATUS_BLOCK_PROTECT_1_BIT)        |
                                                                  (1 << STATUS_BLOCK_PROTECT_0_BIT)
                                                                : (1 << STATUS_REGISTER_WRITE_DISABLE_BIT);

                enable_write();

                // Unprotect status register and select EEPROM
                m_wp = 1;
                m_cs = 0;

                m_spi_master.transfer(CMD_WRITE_STATUS_REGISTER);   // Send command
                m_spi_master.transfer(status_protect);              // Write status block protect

                // Deselect EEPROM and Protect status register
                m_cs = 1;
                m_wp = 0;

                return true;
            }

            return false;
        }

        void send_address(const uint32_t address)
        {
            if constexpr((Capacity - 1) > 0xFFFFFF)
            {
                m_spi_master.transfer(static_cast<uint8_t>(address >> 24));
            }

            if constexpr((Capacity - 1) > 0xFFFF)
            {
                m_spi_master.transfer(static_cast<uint8_t>(address >> 16));
            }

            if constexpr((Capacity - 1) > 0xFF)
            {
                m_spi_master.transfer(static_cast<uint8_t>(address >> 8));
            }

            m_spi_master.transfer(static_cast<uint8_t>(address));
        }

        bool write_page(const uint32_t address, const uint8_t* data_array, std::size_t data_array_size)
        {
            // Truncate the data array size so it won't overflow the page size
            if(data_array_size > (PageSize - (address & (PageSize - 1))))
            {
                data_array_size = (PageSize - (address & (PageSize - 1)));
            }

            if(wait_for_ready() == true)
            {
                enable_write();

                m_cs = 0;

                m_spi_master.transfer(CMD_WRITE_TO_MEMORY_ARRAY);

                send_address(address);

                while(data_array_size-- > 0)
                {
                    m_spi_master.transfer(*data_array++);
                }

                m_cs = 1;

                return true;
            }

            return false;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        hal::SpiMaster&   m_spi_master; // SPI bus
        DigitalOut        m_cs;         // Chip select
        DigitalOut        m_wp;         // Write protect
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_EEPROM_HPP

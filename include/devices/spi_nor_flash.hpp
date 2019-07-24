// ----------------------------------------------------------------------------
// @file    spi_nor_flash.hpp
// @brief   SPI NOR flash class.
// @notes   Implemented at least according to the families:
//          - Spansion/Cypress' S25FL2xx [4Mb - 16Mb];
//          - Winbond's W25X and W25Q [512Kb - 256Mb].
//          Exclusive to use in FatFs.
// @date    22 July 2019
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

#ifndef __XARMLIB_DEVICES_SPI_NOR_FLASH_HPP
#define __XARMLIB_DEVICES_SPI_NOR_FLASH_HPP

#include "diskio.h"
#include "api/api_digital_out.hpp"
#include "hal/hal_spi.hpp"
#include "hal/hal_us_ticker.hpp"

namespace xarmlib
{




namespace private_spi_nor_flash
{

// Memory organizations                                             512Kb  1Mb   2Mb   4Mb   8Mb  16Mb   32Mb   64Mb  128Mb   256Mb
static constexpr std::array<std::size_t, 10> lookup_table_sectors = {  16,  32,   64,  128,  256,  512,  1024,  2048,  4096,   8192 };
//static constexpr std::array<std::size_t, 10> lookup_table_pages   = { 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536, 131072 };




class SpiNorFlash
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        SpiNorFlash(      hal::SpiMaster& spi_master,
                    const hal::Pin::Name  cs,
                    const hal::Pin::Name  wp,
                    const bool            readonly): m_spi_master { spi_master },
                                                     m_cs(cs, { hal::Gpio::OutputMode::push_pull_high }),
                                                     m_wp(wp, { hal::Gpio::OutputMode::push_pull_low }),
                                                     m_readonly { readonly }
        {
            assert(m_sector_size == FF_MAX_SS);
        }

        // Read the flash configuration
        bool read_configuration()
        {
            m_spi_master.mutex_take();

            // Select flash
            m_cs = 0;

            // Read Manufacturer, Memory Type and Capacity ID (JEDEC ID)
            m_spi_master.transfer(CMD_JEDEC_ID_READ);           // Send command
            m_spi_master.transfer(0xFF);                        // Skip manufacturer ID
            m_spi_master.transfer(0xFF);                        // Skip memory type ID

            const uint8_t data = m_spi_master.transfer(0xFF);   // Read capacity ID

            // Deselect flash
            m_cs = 1;

            m_spi_master.mutex_give();

            // Extract memory size from capacity ID
            const uint8_t id = data & 0x0F;

            assert(id <= 9);

            m_sectors = lookup_table_sectors[id];
            //m_pages   = lookup_table_pages[id];

            //                                    SRP | BP1 | BP0                   :  SRP | BP3/TB | BP2 | BP1 | BP0
            m_status_block_protect = (id <= 1) ? (1 << STATUS_REGISTER_PROTECT_BIT) |
                                                 (1 << STATUS_BLOCK_PROTECT_1_BIT)  |
                                                 (1 << STATUS_BLOCK_PROTECT_0_BIT)  : (1 << STATUS_REGISTER_PROTECT_BIT) |
                                                                                      (1 << STATUS_BLOCK_PROTECT_3_BIT)  |
                                                                                      (1 << STATUS_BLOCK_PROTECT_2_BIT)  |
                                                                                      (1 << STATUS_BLOCK_PROTECT_1_BIT)  |
                                                                                      (1 << STATUS_BLOCK_PROTECT_0_BIT);

            bool success = true;

            const bool is_currently_readonly = is_readonly();

            if(m_readonly == true && is_currently_readonly == false)
            {
                m_spi_master.mutex_take();

                success = enable_readonly();

                m_spi_master.mutex_give();
            }
            else if(m_readonly == false && is_currently_readonly == true)
            {
                m_spi_master.mutex_take();

                success = disable_readonly();

                m_spi_master.mutex_give();
            }

            return success;
        }

        /*
        void DumpToSdCard()
        {
            // Work area (file system object) for logical drive
            FATFS fs;

            // Forced mount the flash memory drive
            if(f_mount(&fs, "1:", 1) == FR_OK)
            {
                // File object
                FIL file;

                // Open the file
                if(f_open(&file, "1:dump.dat", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
                {
                    for(int32_t sector = 0; sector < m_sectors; sector++)
                    {
                        uint8_t buffer[4096];

                        Read(sector, buffer, 1);

                        uint32_t bytes = 0;

                        f_write(&file, buffer, 4096, &bytes);
                    }

                    // Close the file
                    f_close(&file);
                }
            }

            // Unmount memory flash file system
            f_mount(NULL, "1:", 0);
        }
        */

        // Get the read-only status
        bool is_readonly()
        {
            m_spi_master.mutex_take();

            if(wait_for_ready() == false)
            {
                m_spi_master.mutex_give();

                return false;
            }

            const bool is_readonly = ((get_status_register() & m_status_block_protect) == m_status_block_protect);

            m_spi_master.mutex_give();

            return is_readonly;
        }

        // Perform a chip erase
        bool erase_chip()
        {
            m_spi_master.mutex_take();

            if(wait_for_ready() == false)
            {
                m_spi_master.mutex_give();

                return false;
            }

            enable_write();

            //m_wp = 1;
            m_cs = 0;

            m_spi_master.transfer(CMD_CHIP_ERASE);

            m_cs = 1;
            //m_wp = 0;

            m_spi_master.mutex_give();

            return true;
        }

        // Perform an erase of the specified memory range (sector by sector)
        bool erase_sector(const uint32_t starting_sector, std::size_t sector_count)
        {
            uint32_t address = starting_sector * m_sector_size;

            m_spi_master.mutex_take();

            while(sector_count-- > 0)
            {
                if(wait_for_ready() == false)
                {
                    m_spi_master.mutex_give();

                    return false;
                }

                enable_write();

                //m_wp = 1;
                m_cs = 0;

                m_spi_master.transfer(CMD_SECTOR_ERASE);

                send_address(address);

                m_cs = 1;
                //m_wp = 0;

                address += m_sector_size;
            }

            m_spi_master.mutex_give();

            return true;
        }

        // Perform a continuous memory read (sector by sector)
        bool read_sector(const uint32_t starting_sector, uint8_t* data_array, const std::size_t sector_count)
        {
            assert(data_array != nullptr);

            m_spi_master.mutex_take();

            const bool success = wait_for_ready();

            if(success == true)
            {
                const uint32_t address = starting_sector * m_sector_size;
                std::size_t    size    = sector_count    * m_sector_size;

                m_cs = 0;

                m_spi_master.transfer(CMD_DATA_READ);

                send_address(address);

                while(size-- > 0)
                {
                    // Read data
                    *data_array++ = m_spi_master.transfer(0xFF);
                }

                m_cs = 1;
            }

            m_spi_master.mutex_give();

            return success;
        }

        // Perform a continuous memory program (sector by sector)
        bool program_sector(const uint32_t starting_sector, const uint8_t* data_array, const std::size_t sector_count)
        {
            assert(data_array != nullptr);

            uint32_t    address    = starting_sector * m_sector_size;
            std::size_t page_count = (sector_count   * m_sector_size) / m_page_size;

            m_spi_master.mutex_take();

            while(page_count-- > 0)
            {
                if(program_page(address, data_array) == false)
                {
                    m_spi_master.mutex_give();

                    return false;
                }

                address    += m_page_size;
                data_array += m_page_size;
            }

            m_spi_master.mutex_give();

            return true;
        }

        // Perform a continuous memory read/verify (sector by sector)
        bool compare_sector(const uint32_t starting_sector, const uint8_t* data_array, const std::size_t sector_count)
        {
            assert(data_array != nullptr);

            m_spi_master.mutex_take();

            bool success = wait_for_ready();

            if(success == true)
            {
                const uint32_t address = starting_sector * m_sector_size;
                std::size_t    size    = sector_count    * m_sector_size;

                m_cs = 0;

                m_spi_master.transfer(CMD_DATA_READ);

                send_address(address);

                while(size-- > 0)
                {
                    // Compare data
                    if(*data_array++ != m_spi_master.transfer(0xFF))
                    {
                        success = false;
                        break;
                    }
                }

                m_cs = 1;
            }

            m_spi_master.mutex_give();

            return success;
        }

        // Control device specific features and miscellaneous functions other than generic read/write (FatFs specific)
        bool control(const uint8_t code, void* data)
        {
            bool success = true;

            switch(code)
            {
                // Make sure that no pending write process
                case CTRL_SYNC:
                    m_spi_master.mutex_take();

                    if(wait_for_ready() == false)
                    {
                        success = false;
                    }

                    m_spi_master.mutex_give();
                    break;

                // Get number of sectors on the disk (DWORD)
                case GET_SECTOR_COUNT:
                    *(uint32_t *)data = m_sectors;
                    break;

                // Get R/W sector size (WORD)
                case GET_SECTOR_SIZE:
                    *(uint32_t *)data = m_sector_size;
                    break;

                // Get erase block size in unit of sector (DWORD)
                case GET_BLOCK_SIZE:
                    *(uint32_t *)data = 1;
                    break;

                // Enable/Disable write protect
                case CTRL_PROTECT:
                    const bool protect = *(bool *)data;

                    if(protect == true)
                    {
                        m_spi_master.mutex_take();

                        enable_readonly();

                        m_spi_master.mutex_give();
                    }
                    else
                    {
                        m_spi_master.mutex_take();

                        disable_readonly();

                        m_spi_master.mutex_give();
                    }

                    break;
            }

            return success;
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
            STATUS_WRITE_IN_PROGRESS_BIT = 0,
            STATUS_BLOCK_PROTECT_0_BIT   = 2,
            STATUS_BLOCK_PROTECT_1_BIT   = 3,
            STATUS_BLOCK_PROTECT_2_BIT   = 4,
            STATUS_BLOCK_PROTECT_3_BIT   = 5,
            STATUS_REGISTER_PROTECT_BIT  = 7,
            STATUS_MASK                  = 0xBF
        };

        // Command definitions
        enum CMD : uint8_t
        {
            // Erase commands
            CMD_SECTOR_ERASE = 0x20,
            CMD_BLOCK_ERASE  = 0xD8,
            CMD_CHIP_ERASE   = 0xC7,

            // Read and Program commands
            CMD_DATA_READ    = 0x03,
            CMD_FAST_READ    = 0x0B,
            CMD_PAGE_PROGRAM = 0x02,

            // Additional commands
            CMD_ENABLE_WRITE          = 0x06,
            CMD_STATUS_REGISTER_READ  = 0x05,
            CMD_STATUS_REGISTER_WRITE = 0x01,
            CMD_JEDEC_ID_READ         = 0x9F,
            CMD_UNIQUE_ID_READ        = 0x4B
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Get the status register
        uint8_t get_status_register()
        {
            m_cs = 0;

            m_spi_master.transfer(CMD_STATUS_REGISTER_READ);

            // Read status (reserved bit masked out)
            const uint8_t data = m_spi_master.transfer(0xFF) & STATUS_MASK;

            m_cs = 1;

            return data;
        }

        // Get the ready/busy status
        bool is_busy()
        {
            return ((get_status_register() & (1 << STATUS_WRITE_IN_PROGRESS_BIT)) != 0);
        }

        // Wait for the flash to become ready
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

        // Enable write
        void enable_write()
        {
            //m_wp = 1;
            m_cs = 0;

            m_spi_master.transfer(CMD_ENABLE_WRITE);

            m_cs = 1;
            //m_wp = 0;
        }

        // Enable read-only
        bool enable_readonly()
        {
            if(wait_for_ready() == true)
            {
                enable_write();

                // Unprotect status register and Select flash
                m_wp = 1;
                m_cs = 0;

                m_spi_master.transfer(CMD_STATUS_REGISTER_WRITE);   // Send command
                m_spi_master.transfer(m_status_block_protect);      // Write status block protect

                // Deselect flash and Protect status register
                m_cs = 1;
                m_wp = 0;

                return true;
            }

            return false;
        }

        // Disable read-only
        bool disable_readonly()
        {
            if(wait_for_ready() == true)
            {
                // Set SRP and clear [BP3, BP2,] BP1 and BP0
                const uint8_t status_block_protect = (1 << STATUS_REGISTER_PROTECT_BIT);

                enable_write();

                // Unprotect status register and Select flash
                m_wp = 1;
                m_cs = 0;

                m_spi_master.transfer(CMD_STATUS_REGISTER_WRITE);   // Send command
                m_spi_master.transfer(status_block_protect);        // Write status block protect

                // Deselect flash and Protect status register
                m_cs = 1;
                m_wp = 0;

                return true;
            }

            return false;
        }

        // Send the address bytes
        void send_address(const uint32_t address)
        {
            m_spi_master.transfer(static_cast<uint8_t>(address >> 16));
            m_spi_master.transfer(static_cast<uint8_t>(address >>  8));
            m_spi_master.transfer(static_cast<uint8_t>(address      ));
        }

        // Perform a page program
        bool program_page(const uint32_t address, const uint8_t* data_array)
        {
            if(wait_for_ready() == true)
            {
                enable_write();

                //m_wp = 1;
                m_cs = 0;

                m_spi_master.transfer(CMD_PAGE_PROGRAM);

                send_address(address);

                for(std::size_t byte = 0; byte < m_page_size; byte++)
                {
                    m_spi_master.transfer(*data_array++);
                }

                m_cs = 1;
                //m_wp = 0;

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

        const bool        m_readonly;

        const std::size_t m_sector_size { 4096 };
        const std::size_t m_page_size { 256 };

        std::size_t       m_sectors { 0 };
        //std::size_t       m_pages { 0 };

        uint8_t           m_status_block_protect { 0 };
};

} // namespace private_spi_nor_flash




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

// Class initialization
// NOTE: should be called by the user
void SpiNorFlash_initialize(hal::SpiMaster& spi_master, const hal::Pin::Name cs, const hal::Pin::Name wp, const bool readonly);




// ----------------------------------------------------------------------------
// FATFS DISK I/O FUNCTIONS
// ----------------------------------------------------------------------------

// Read the flash configuration
// NOTE: to be used inside disk_initialize function
bool SpiNorFlash_read_configuration();

// Check if flash is read-only
// NOTE: to be used inside disk_initialize function
bool SpiNorFlash_is_readonly();

// Read a single or multiple sectors from memory flash
// NOTE: to be used inside disk_read function
bool SpiNorFlash_read_sector(uint32_t starting_sector, uint8_t* data_array, std::size_t sector_count);

// Write a single or multiple sectors to memory flash
// NOTES: to be used inside disk_write function
//        a reading and comparison is done after each sector written, returning its result
bool SpiNorFlash_write_sector(uint32_t starting_sector, const uint8_t* data_array, std::size_t sector_count);

// Control device specific features and miscellaneous functions other than generic read/write
// NOTE: to be used inside disk_ioctl function
bool SpiNorFlash_control(const uint8_t code, void* data);




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_SPI_NOR_FLASH_HPP

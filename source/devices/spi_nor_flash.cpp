// ----------------------------------------------------------------------------
// @file    spi_nor_flash.cpp
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

#include "xarmlib_config.hpp"

#if (XARMLIB_ENABLE_FATFS == 1)

#include "devices/spi_nor_flash.hpp"

namespace xarmlib
{




// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
// ----------------------------------------------------------------------------

// Global flash interface
static std::unique_ptr<private_spi_nor_flash::SpiNorFlash> g_spi_nor_flash_ptr = nullptr;




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

// Class initialization
// NOTE: should be called by the user
void SpiNorFlash_initialize(hal::SpiMaster& spi_master, const hal::Pin::Name cs, const hal::Pin::Name wp, const bool readonly)
{
    assert(g_spi_nor_flash_ptr == nullptr);

    g_spi_nor_flash_ptr = std::make_unique<private_spi_nor_flash::SpiNorFlash>(spi_master, cs, wp, readonly);

    assert(g_spi_nor_flash_ptr != nullptr);

    //g_spi_nor_flash_ptr->erase_chip();
}




// ----------------------------------------------------------------------------
// FATFS DISK I/O FUNCTIONS
// ----------------------------------------------------------------------------

// Read the flash configuration
// NOTE: to be used inside disk_initialize function
bool SpiNorFlash_read_configuration()
{
    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->read_configuration();
}

// Check if flash is read-only
// NOTE: to be used inside disk_initialize function
bool SpiNorFlash_is_readonly()
{
    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->is_readonly();
}

// Read a single or multiple sectors from memory flash
// NOTE: to be used inside disk_read function
bool SpiNorFlash_read_sector(uint32_t starting_sector, uint8_t* data_array, std::size_t sector_count)
{
    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->read_sector(starting_sector, data_array, sector_count);
}

// Write a single or multiple sectors to memory flash
// NOTES: to be used inside disk_write function
//        a reading and comparison is done after each sector written, returning its result
bool SpiNorFlash_write_sector(uint32_t starting_sector, const uint8_t* data_array, std::size_t sector_count)
{
    assert(g_spi_nor_flash_ptr != nullptr);

    if(g_spi_nor_flash_ptr->erase_sector(starting_sector, sector_count) == true)
    {
        if(g_spi_nor_flash_ptr->program_sector(starting_sector, data_array, sector_count) == true)
        {
            return g_spi_nor_flash_ptr->compare_sector(starting_sector, data_array, sector_count);
        }
    }

    return false;
}

// Control device specific features and miscellaneous functions other than generic read/write
// NOTE: to be used inside disk_ioctl function
bool SpiNorFlash_control(const uint8_t code, void* data)
{
    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->control(code, data);
}




} // namespace xarmlib

#endif // (XARMLIB_ENABLE_FATFS == 1)

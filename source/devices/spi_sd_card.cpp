// ----------------------------------------------------------------------------
// @file    spi_sd_card.cpp
// @brief   SPI SD card class.
// @note    Exclusive to use in FatFs.
// @date    24 May 2019
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

#include "devices/spi_sd_card.hpp"

namespace xarmlib
{




// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
// ----------------------------------------------------------------------------

// Global SD card interface
static std::unique_ptr<private_spi_sd_card::SpiSdCard> g_spi_sd_card_ptr = nullptr;




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

// Class initialization
// NOTE: should be called by the user
void SpiSdCard_initialize(hal::SpiMaster& spi_master, const hal::Pin::Name cs)
{
    assert(g_spi_sd_card_ptr == nullptr);

    g_spi_sd_card_ptr = std::make_unique<private_spi_sd_card::SpiSdCard>(spi_master, cs);

    assert(g_spi_sd_card_ptr != nullptr);
}




// ----------------------------------------------------------------------------
// FATFS DISK I/O FUNCTIONS
// ----------------------------------------------------------------------------

// Second stage card initialization
// NOTE: to be used inside disk_initialize function
bool SpiSdCard_start()
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->start();
}

// Read the card configuration
// NOTE: to be used inside disk_initialize function
bool SpiSdCard_read_configuration()
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->read_configuration();
}

// Read a single or multiple sectors from memory card
// NOTE: to be used inside disk_read function
bool SpiSdCard_read_sector(uint32_t starting_sector, uint8_t* data_array, std::size_t sector_count)
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->read_sector(starting_sector, data_array, sector_count);
}

// Write a single or multiple sectors to memory card
// NOTE: to be used inside disk_write function
bool SpiSdCard_write_sector(uint32_t starting_sector, const uint8_t* data_array, std::size_t sector_count)
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->write_sector(starting_sector, data_array, sector_count);
}

// Control device specific features and miscellaneous functions other than generic read/write
// NOTE: to be used inside disk_ioctl function
bool SpiSdCard_control(const uint8_t code, void* data)
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->control(code, data);
}




} // namespace xarmlib

#endif // (XARMLIB_ENABLE_FATFS == 1)

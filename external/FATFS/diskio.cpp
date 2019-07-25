// ----------------------------------------------------------------------------
// @file    diskio.cpp
// @brief   Low level disk I/O functions for FatFs.
// @date    25 July 2019
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

#include "diskio.h"

namespace xarmlib
{




// ----------------------------------------------------------------------------
// SPI SD CARD DISK I/O IMPLEMENTATION
// ----------------------------------------------------------------------------

// Global SD card interface
static std::unique_ptr<SpiSdCard> g_spi_sd_card_ptr = nullptr;




// SPI SD card class initialization
// NOTE: should be called by the user case FatFs will be used
bool SpiSdCard_fatfs_initialize(hal::SpiMaster& spi_master, const hal::Pin::Name cs)
{
    assert(g_spi_sd_card_ptr == nullptr);

    g_spi_sd_card_ptr = std::make_unique<SpiSdCard>(spi_master, cs);

    assert(g_spi_sd_card_ptr != nullptr);

    if(g_spi_sd_card_ptr->start() == true)
    {
        return g_spi_sd_card_ptr->read_configuration();
    }

    return false;
}




// Check if memory card is initialized
// NOTE: to be used inside disk_initialize function
bool SpiSdCard_fatfs_is_initialized()
{
    return (g_spi_sd_card_ptr != nullptr);
}

// Read a single or multiple sectors from memory card
// NOTE: to be used inside disk_read function
bool SpiSdCard_fatfs_read_sector(uint32_t starting_sector, uint8_t* data_array, std::size_t sector_count)
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->read_sector(starting_sector, data_array, sector_count);
}

// Write a single or multiple sectors to memory card
// NOTE: to be used inside disk_write function
bool SpiSdCard_fatfs_write_sector(uint32_t starting_sector, const uint8_t* data_array, std::size_t sector_count)
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->write_sector(starting_sector, data_array, sector_count);
}

// Control device specific features and miscellaneous functions other than generic read/write
// NOTE: to be used inside disk_ioctl function
bool SpiSdCard_fatfs_control(const uint8_t code, void* data)
{
    assert(g_spi_sd_card_ptr != nullptr);

    return g_spi_sd_card_ptr->control(code, data);
}




#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)

// ----------------------------------------------------------------------------
// SPI NOR FLASH DISK I/O IMPLEMENTATION
// ----------------------------------------------------------------------------

// Global flash interface
static std::unique_ptr<SpiNorFlash> g_spi_nor_flash_ptr = nullptr;




// SPI NOR flash class initialization
// NOTE: should be called by the user case FatFs will be used
bool SpiNorFlash_fatfs_initialize(hal::SpiMaster& spi_master, const hal::Pin::Name cs, const hal::Pin::Name wp, const bool readonly)
{
    assert(g_spi_nor_flash_ptr == nullptr);

    g_spi_nor_flash_ptr = std::make_unique<SpiNorFlash>(spi_master, cs, wp, readonly);

    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->read_configuration();

    //g_spi_nor_flash_ptr->erase_chip();
}




// Check if flash is initialized
// NOTE: to be used inside disk_initialize function
bool SpiNorFlash_fatfs_is_initialized()
{
    return (g_spi_nor_flash_ptr != nullptr);
}

// Check if flash is read-only
// NOTE: to be used inside disk_initialize function
bool SpiNorFlash_fatfs_is_readonly()
{
    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->is_readonly();
}

// Read a single or multiple sectors from memory flash
// NOTE: to be used inside disk_read function
bool SpiNorFlash_fatfs_read_sector(uint32_t starting_sector, uint8_t* data_array, std::size_t sector_count)
{
    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->read_sector(starting_sector, data_array, sector_count);
}

// Write a single or multiple sectors to memory flash
// NOTES: to be used inside disk_write function
//        a reading and comparison is done after each sector written, returning its result
bool SpiNorFlash_fatfs_write_sector(uint32_t starting_sector, const uint8_t* data_array, std::size_t sector_count)
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
bool SpiNorFlash_fatfs_control(const uint8_t code, void* data)
{
    assert(g_spi_nor_flash_ptr != nullptr);

    return g_spi_nor_flash_ptr->control(code, data);
}

#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)




// ----------------------------------------------------------------------------
// FATFS DISK I/O IMPLEMENTATION
// ---------------------------------------------------------------------------

extern "C"
{

// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
// ----------------------------------------------------------------------------

// Disk status
#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
static DSTATUS g_disk_status[2] = { STA_NOINIT, STA_NOINIT };
#else
static DSTATUS g_disk_status[1] = { STA_NOINIT };
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

// -------- INITIALIZE A DRIVE ------------------------------------------------

DSTATUS disk_initialize(BYTE pdrv /* Physical drive number (0..) */)
{
    switch(pdrv)
    {
        case SDCARD:
            if(SpiSdCard_fatfs_is_initialized() == true)
            {
                // Start up OK!
                g_disk_status[SDCARD] &= ~STA_NOINIT;
            }
            return g_disk_status[SDCARD];

#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
        case FLASH:
            if(SpiNorFlash_fatfs_is_initialized() == true)
            {
                // Start up OK!
                g_disk_status[FLASH] &= ~STA_NOINIT;

                if (SpiNorFlash_fatfs_is_readonly() == true)
                {
                    // Write protected
                    g_disk_status[FLASH] |= STA_PROTECT;
                }
                else
                {
                    // Writable
                    g_disk_status[FLASH] &= ~STA_PROTECT;
                }
            }
            return g_disk_status[FLASH];
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    }

    // Drive not supported
    return STA_NOINIT;
}




// -------- GET DISK STATUS ---------------------------------------------------

DSTATUS disk_status(BYTE pdrv /* Physical drive number (0..) */)
{
    switch(pdrv)
    {
        case SDCARD: return g_disk_status[SDCARD];
#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
        case FLASH:  return g_disk_status[FLASH];
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    }

    // Drive not supported
    return STA_NOINIT;
}




// -------- READ SECTOR(S) ----------------------------------------------------

DRESULT disk_read(BYTE  pdrv,   /* Physical drive number (0..) */
                  BYTE *buff,   /* Data buffer to store read data */
                  DWORD sector, /* Sector address (LBA) */
                  UINT  count   /* Number of sectors to read (1..128) */)
{
#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    if(pdrv > 1 || count == 0)
#else
    if(pdrv > 0 || count == 0)
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    {
        return RES_PARERR;
    }

    if(g_disk_status[pdrv] & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch(pdrv)
    {
        case SDCARD:
            if(SpiSdCard_fatfs_read_sector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;

#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
        case FLASH:
            if(SpiNorFlash_fatfs_read_sector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    }

    return RES_ERROR;
}




#if (FF_FS_READONLY == 0)

// -------- WRITE SECTOR(S) ---------------------------------------------------

DRESULT disk_write(      BYTE  pdrv,   /* Physical drive number (0..) */
                   const BYTE *buff,   /* Data to be written */
                         DWORD sector, /* Sector address (LBA) */
                         UINT  count   /* Number of sectors to write (1..128) */)
{
#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    if(pdrv > 1 || count == 0)
#else
    if(pdrv > 0 || count == 0)
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    {
        return RES_PARERR;
    }

    if(g_disk_status[pdrv] & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    if(g_disk_status[pdrv] & STA_PROTECT)
    {
        return RES_WRPRT;
    }

    switch(pdrv)
    {
        case SDCARD:
            if(SpiSdCard_fatfs_write_sector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;

#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
        case FLASH:
            if(SpiNorFlash_fatfs_write_sector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    }

    return RES_ERROR;
}

#endif // (FF_FS_READONLY == 0)




// -------- MISCELLANEOUS FUNCTIONS -------------------------------------------

DRESULT disk_ioctl(BYTE  pdrv, /* Physical drive number (0..) */
                   BYTE  cmd,  /* Control code */
                   void *buff  /* Buffer to send/receive control data */)
{
#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    if(pdrv > 1)
#else
    if(pdrv > 0)
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    {
        return RES_PARERR;
    }

    if(g_disk_status[pdrv] & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch(pdrv)
    {
        case SDCARD:
            if(SpiSdCard_fatfs_control(cmd, buff) == true)
            {
                return RES_OK;
            }
            break;

#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)
        case FLASH:
            if(SpiNorFlash_fatfs_control(cmd, buff) == true)
            {
                if(cmd == CTRL_PROTECT)
                {
                    const bool protect = *(bool *)buff;

                    if(protect == true)
                    {
                        // Write protected
                        g_disk_status[FLASH] |= STA_PROTECT;
                    }
                    else
                    {
                        // Writable
                        g_disk_status[FLASH] &= ~STA_PROTECT;
                    }
                }

                return RES_OK;
            }
            break;
#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)
    }

    return RES_ERROR;
}




#if (FF_FS_NORTC == 0)

// User provided RTC function for FatFs module (prototype @ ff.h)
DWORD get_fattime(void)
{
    /* rtc_time_t now;

    // Get current time
    Rtc_GetFullTime(&now);

    // Pack date and time into a DWORD variable
    return ((DWORD)(now.YEAR - 1980) << 25)
         | ((DWORD) now.MONTH        << 21)
         | ((DWORD) now.DOM          << 16)
         | ((DWORD) now.HOUR         << 11)
         | ((DWORD) now.MIN          <<  5)
         | ((DWORD) now.SEC          >>  1); */

    return 0;
}

#endif  // (FF_FS_NORTC == 0)




} // extern "C"

} // namespace xarmlib

#endif // (XARMLIB_ENABLE_FATFS == 1)

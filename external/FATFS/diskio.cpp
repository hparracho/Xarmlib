// ----------------------------------------------------------------------------
// @file    diskio.cpp
// @brief   Low level disk I/O functions for FatFs.
// @date    29 May 2019
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

#include "diskio.h"
#include "devices/spi_sd_card.hpp"

namespace xarmlib
{

extern "C"
{





// ----------------------------------------------------------------------------
// GLOBAL VARIABLES
// ----------------------------------------------------------------------------

// Disk status
/*static DSTATUS g_disk_status[2] = {STA_NOINIT, STA_NOINIT};*/
static DSTATUS g_disk_status[1] = { STA_NOINIT };




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

// -------- INITIALIZE A DRIVE ------------------------------------------------

DSTATUS disk_initialize(BYTE pdrv /* Physical drive number (0..) */)
{
    switch(pdrv)
    {
        /*case FLASH:
            if(SpiFlashS25_FatFsIsInitialized() == true)
            {
                // Start up OK!
                g_disk_status[FLASH] &= ~STA_NOINIT;

                if (SpiFlashS25_FatFsIsReadOnly() == true)
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
            else if(SpiFlashAt45_IsInitialized() == true)
            {
                // Start up OK!
                g_disk_status[FLASH] &= ~STA_NOINIT;

                // @TODO: IMPLEMENTAR VERIFICAÇÃO DO WRITE PROTECT!!
            }
            return g_disk_status[FLASH];*/

        case SDCARD:
            if(SpiSdCard_start() == true && SpiSdCard_read_configuration() == true)
            {
                // Start up OK!
                g_disk_status[SDCARD] &= ~STA_NOINIT;
            }
            return g_disk_status[SDCARD];
    }

    // Drive not supported
    return STA_NOINIT;
}




// -------- GET DISK STATUS ---------------------------------------------------

DSTATUS disk_status(BYTE pdrv /* Physical drive number (0..) */)
{
    switch(pdrv)
    {
        /*case FLASH:  return g_disk_status[FLASH];*/
        case SDCARD: return g_disk_status[SDCARD];
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
    if(pdrv > /*1*/0 || count == 0)
    {
        return RES_PARERR;
    }

    if(g_disk_status[pdrv] & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch(pdrv)
    {
        /*case FLASH:
            if(SpiFlashS25_FatFsReadSector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            else if(SpiFlashAt45_ReadSector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;*/

        case SDCARD:
            if(SpiSdCard_read_sector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;
    }

    return RES_ERROR;
}




#if FF_FS_READONLY == 0

// -------- WRITE SECTOR(S) ---------------------------------------------------

DRESULT disk_write(      BYTE  pdrv,   /* Physical drive number (0..) */
                   const BYTE *buff,   /* Data to be written */
                         DWORD sector, /* Sector address (LBA) */
                         UINT  count   /* Number of sectors to write (1..128) */)
{
    if(pdrv > /*1*/0 || count == 0)
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
        /*case FLASH:
            if(SpiFlashS25_FatFsWriteSector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            else if(SpiFlashAt45_WriteSector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;*/

        case SDCARD:
            if(SpiSdCard_write_sector(sector, buff, count) == true)
            {
                return RES_OK;
            }
            break;
    }

    return RES_ERROR;
}

#endif




// -------- MISCELLANEOUS FUNCTIONS -------------------------------------------

DRESULT disk_ioctl(BYTE  pdrv, /* Physical drive number (0..) */
                   BYTE  cmd,  /* Control code */
                   void *buff  /* Buffer to send/receive control data */)
{
    if(pdrv > /*1*/0)
    {
        return RES_PARERR;
    }

    if(g_disk_status[pdrv] & STA_NOINIT)
    {
        return RES_NOTRDY;
    }

    switch(pdrv)
    {
        /*case FLASH:
            if(SpiFlashS25_FatFsControl(cmd, buff) == true)
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
            else if(SpiFlashAt45_Control(cmd, buff) == true)
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
            break;*/

        case SDCARD:
            if(SpiSdCard_control(cmd, buff) == true)
            {
                return RES_OK;
            }
            break;
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

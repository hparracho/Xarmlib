// ----------------------------------------------------------------------------
// @file    diskio.h
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

#ifndef __XARMLIB_EXTERNAL_FATFS_DISKIO_H
#define __XARMLIB_EXTERNAL_FATFS_DISKIO_H

#include "ff.h" // Obtains integer types

#ifdef __cplusplus
extern "C" {
#endif




// ----------------------------------------------------------------------------
// PUBLIC DEFINITIONS
// ----------------------------------------------------------------------------

// Definitions of physical drive number for each media
#define SDCARD      0
#define FLASH       1




// Status of Disk Functions
typedef BYTE    DSTATUS;




// Results of Disk Functions
typedef enum
{
    RES_OK = 0,     /* 0: Successful */
    RES_ERROR,      /* 1: R/W Error */
    RES_WRPRT,      /* 2: Write Protected */
    RES_NOTRDY,     /* 3: Not Ready */
    RES_PARERR      /* 4: Invalid Parameter */
} DRESULT;




// Disk Status Bits (DSTATUS)
#define STA_NOINIT      0x01    /* Drive not initialized */
#define STA_NODISK      0x02    /* No medium in the drive */
#define STA_PROTECT     0x04    /* Write protected */




// Command code for disk_ioctrl function

// Generic command (Used by FatFs)
#define CTRL_SYNC           0   /* Complete pending write process (needed at FF_FS_READONLY == 0) */
#define GET_SECTOR_COUNT    1   /* Get media size (needed at FF_USE_MKFS == 1) */
#define GET_SECTOR_SIZE     2   /* Get sector size (needed at FF_MAX_SS != FF_MIN_SS) */
#define GET_BLOCK_SIZE      3   /* Get erase block size (needed at FF_USE_MKFS == 1) */
#define CTRL_TRIM           4   /* Inform device that the data on the block of sectors is no longer used (needed at FF_USE_TRIM == 1) */

// Generic command (Not used by FatFs)
//#define CTRL_POWER          5   /* Get/Set power status */
//#define CTRL_LOCK           6   /* Lock/Unlock media removal */
//#define CTRL_EJECT          7   /* Eject media */
//#define CTRL_FORMAT         8   /* Create physical format on the media */

// (Added to the template)
#define CTRL_PROTECT        9   /* Enable/Disable write protect */

// MMC/SDC specific ioctl command
#define MMC_GET_TYPE        10  /* Get card type */
#define MMC_GET_CSD         11  /* Get CSD */
#define MMC_GET_CID         12  /* Get CID */
#define MMC_GET_OCR         13  /* Get OCR */
#define MMC_GET_SDSTAT      14  /* Get SD status */




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

DSTATUS disk_initialize(BYTE pdrv);
DSTATUS disk_status    (BYTE pdrv);
DRESULT disk_read      (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT disk_write     (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT disk_ioctl     (BYTE pdrv, BYTE cmd, void* buff);




#ifdef __cplusplus
}
#endif

#endif // __XARMLIB_EXTERNAL_FATFS_DISKIO_H

// ----------------------------------------------------------------------------
// @file    ff_util.cpp
// @brief   Utility functions for FatFs.
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

#include "ff_util.hpp"
#include "diskio.h"

namespace xarmlib
{

extern "C"
{




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

#if (FF_FS_READONLY == 0)

DRESULT f_readonly(BYTE drive, /* Physical drive number */
                   bool protect)
{
    return disk_ioctl(drive, CTRL_PROTECT, &protect);
}




FRESULT f_copy(const TCHAR *path_dst, const TCHAR *path_src)
{
    // File objects
    FIL fp_src, fp_dst;

    // Open the source file
    if(f_open(&fp_src, path_src, FA_OPEN_EXISTING | FA_READ) != FR_OK)
    {
        return FR_DISK_ERR;
    }

    // Open/create the destination file
    if(f_open(&fp_dst, path_dst, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
        // Close the source file
        f_close(&fp_src);

        return FR_DISK_ERR;
    }

    uint8_t buffer[512];

    UINT bytes_read, bytes_write;

    // Copy the source file to destination file 512 in 512 bytes
    do
    {
        if(f_read(&fp_src, buffer, 512, &bytes_read) != FR_OK)
        {
            // Close both files
            f_close(&fp_src);
            f_close(&fp_dst);

            // Delete destination file
            f_unlink(path_dst);

            return FR_DISK_ERR;
        }
        if((f_write(&fp_dst, buffer, bytes_read, &bytes_write) != FR_OK) || (bytes_read != bytes_write))
        {
            // Close both files
            f_close(&fp_src);
            f_close(&fp_dst);

            // Delete destination file
            f_unlink(path_dst);

            return FR_DISK_ERR;
        }
    } while(bytes_read == 512);

    // Close both files
    f_close(&fp_src);
    f_close(&fp_dst);

    return FR_OK;
}

#endif  // (FF_FS_READONLY == 0)




FRESULT f_file_exists(const TCHAR *path)
{
    FIL file;

    if(f_open(&file, path, FA_OPEN_EXISTING | FA_READ) == FR_OK)
    {
        f_close(&file);
        return FR_OK;
    }

    return FR_NO_FILE;

    /*FILINFO fno;

    if(f_stat(path, &fno) == FR_OK && (fno.fattrib & AM_DIR) == 0)
    {
        return FR_OK;
    }

    return FR_NO_FILE;*/
}




} // extern "C"

} // namespace xarmlib

#endif // (XARMLIB_ENABLE_FATFS == 1)

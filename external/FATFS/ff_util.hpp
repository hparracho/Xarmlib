// ----------------------------------------------------------------------------
// @file    ff_util.hpp
// @brief   Utility functions for FatFs.
// @date    19 May 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_EXTERNAL_FATFS_FF_UTIL_HPP
#define __XARMLIB_EXTERNAL_FATFS_FF_UTIL_HPP

#include "diskio.h"

namespace xarmlib
{

extern "C"
{




// ----------------------------------------------------------------------------
// PUBLIC FUNCTIONS
// ----------------------------------------------------------------------------

#if (FF_FS_READONLY == 0)

DRESULT f_readonly(BYTE drive, /* Physical drive number */ bool protect);

FRESULT f_copy(const TCHAR *path_dst, const TCHAR *path_src);

#endif  // (FF_FS_READONLY == 0)




FRESULT f_file_exists(const TCHAR *path);




} // extern "C"

} // namespace xarmlib

#endif // __XARMLIB_EXTERNAL_FATFS_FF_UTIL_HPP

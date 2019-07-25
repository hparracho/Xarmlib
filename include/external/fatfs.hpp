// ----------------------------------------------------------------------------
// @file    fatfs.hpp
// @brief   Generic FAT Filesystem Module header file. This should be the only
//          header file included when FatFs functionality is required by the
//          application. When setting XARMLIB_ENABLE_FATFS == 1 this is auto-
//          matically included by xarmlib_config.hpp.
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

#ifndef __XARMLIB_EXTERNAL_FATFS_HPP
#define __XARMLIB_EXTERNAL_FATFS_HPP

#include "ffconf.h"
#include "diskio.h"
#include "devices/spi_sd_card.hpp"

namespace xarmlib
{

// SPI SD card class initialization
// NOTE: should be called by the user case FatFs will be used
bool SpiSdCard_fatfs_initialize(hal::SpiMaster& spi_master, const hal::Pin::Name cs);

} // namespace xarmlib

#if defined(FF_VOLUMES) && (FF_VOLUMES > 1)

#include "devices/spi_nor_flash.hpp"

namespace xarmlib
{

// SPI NOR flash class initialization
// NOTE: should be called by the user case FatFs will be used
bool SpiNorFlash_fatfs_initialize(hal::SpiMaster& spi_master, const hal::Pin::Name cs, const hal::Pin::Name wp, const bool readonly);

} // namespace xarmlib

#endif // defined(FF_VOLUMES) && (FF_VOLUMES > 1)

#include "ff.h"
#include "ff_util.hpp"

#endif // __XARMLIB_EXTERNAL_FATFS_HPP

// ----------------------------------------------------------------------------
// @file    UHS_host.h
// @brief   UHS host definitions.
// @notes   Based on UHS30 UHS_host.h file with few changes
// @date    3 June 2020
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

#ifndef _UHS_host_h_
#define _UHS_host_h_

// WARNING: Do not change the order of includes, or stuff will break!
//#include <inttypes.h>
//#include <stddef.h>
//#include <stdio.h>
//#include <stdint.h>

#include "UHS_macros.h"

// None of these should ever be directly included by a driver, or a user's sketch.
//#include "UHS_USB_IDs.h"
//#include "UHS_settings.h"
#include "UHS_debug.h"
#include "UHS_usb_ch9.h"
#include "UHS_UsbCore.h"
#include "UHS_address.h"
#include "UHS_usbhost.h"
//#include "UHS_printhex.h"
//#include "UHS_message.h"

// Load system components as required
#if defined(LOAD_USB_HOST_SYSTEM) && !defined(USB_HOST_SYSTEM_LOADED)
//#include "UHS_util_INLINE.h"
#include "UHS_host_INLINE.h"
//#include "UHS_printf_HELPER.h"

#if defined(LOAD_MAX3421E)
#include "MAX3421E/spi_max3421e.h"
#endif

// Load USB drivers and multiplexers

#if defined(LOAD_UHS_HUB)
#include "UHS_HUB/UHS_HUB.h"
#endif

#if defined(LOAD_UHS_BULK_STORAGE)
#include "UHS_BULK_STORAGE/UHS_BULK_STORAGE.h"
#endif

// Add HID
#if defined(LOAD_UHS_HID)
#include "UHS_HID/UHS_HID.h"
#endif

#endif // System code loaded

#endif // _UHS_host_h_

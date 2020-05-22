// ----------------------------------------------------------------------------
// @file    usb_host.hpp
// @brief   USB Host header file to use in the library. This should be the only
//          header file included when USB Host functionality is required.
// @date    4 May 2020
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

#ifndef __XARMLIB_EXTERNAL_USB_HOST_HPP
#define __XARMLIB_EXTERNAL_USB_HOST_HPP

// LIBRARIES THAT WILL BE USED

// Load the USB Host System core
#define LOAD_USB_HOST_SYSTEM

// Load MAX3421E
#define LOAD_MAX3421E

// USB hub
//#define LOAD_UHS_HUB

// Bulk Storage
//#define LOAD_UHS_BULK_STORAGE

// HID
//#define LOAD_UHS_HID

// Patch printf so we can use it
//#define LOAD_UHS_PRINTF_HELPER

// DEBUG
#define ENABLE_UHS_DEBUGGING             0
#define DEBUG_PRINTF_EXTRA_HUGE          0
#define DEBUG_PRINTF_EXTRA_HUGE_UHS_HOST 0
#define DEBUG_PRINTF_EXTRA_HUGE_USB_HID  0
#define DEBUG_PRINTF_EXTRA_HUGE_USB_HUB  0

// OPTIONS
// Where to redirect debugging, also used for the program output
//#define USB_HOST_SERIAL xarmlib::hal::Uart

// INCLUDES
#include "UHS_host.h"

#endif // __XARMLIB_EXTERNAL_USB_HOST_HPP

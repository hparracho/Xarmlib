// ----------------------------------------------------------------------------
// @file    usb_host.hpp
// @brief   USB Host header file to use in the library. This should be the only
//          header file included when USB Host functionality is required.
// @date    5 June 2020
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
#define LOAD_UHS_HID
#define LOAD_UHS_HIDRAWBOOT_MOUSE
#define UHS_DEVICE_WINDOWS_USB_SPEC_VIOLATION_DESCRIPTOR_DEVICE 0

// DEBUG
#define DEBUG_USB_HOST

// Maximum number of USB interface drivers
#define UHS_HOST_MAX_INTERFACE_DRIVERS  2

// INCLUDES
#include "UHS_host.h"

#endif // __XARMLIB_EXTERNAL_USB_HOST_HPP

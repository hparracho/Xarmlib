// ----------------------------------------------------------------------------
// @file    Usb.h
// @brief   USB class.
// @notes   USB_Host_Shield_2.0 Usb.h file suitable for Xarmlib
// @date    8 June 2020
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

#ifndef _usb_h_
#define _usb_h_

// WARNING: Do not change the order of includes, or stuff will break!
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>

// None of these should ever be included by a driver, or a user's sketch.
#include "settings.h"
#include "debug.h"
#include "printhex.h"
#include "message.h"
//#include "hexdump.h"
#include "sink_parser.h"
#include "max3421e.h"
#include "address.h"
//#include "avrpins.h"
#include "usb_ch9.h"
#include "spi_max3421e.h"// "usbhost.h"
#include "UsbCore.h"
#include "parsetools.h"
#include "confdescparser.h"

#endif // _usb_h_

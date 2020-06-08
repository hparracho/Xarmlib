// ----------------------------------------------------------------------------
// @file    settings.h
// @brief   settings.
// @notes   USB_Host_Shield_2.0 settings.h file with some cleanups
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

#ifndef USB_HOST_SHIELD_SETTINGS_H
#define USB_HOST_SHIELD_SETTINGS_H
#include "macros.h"


////////////////////////////////////////////////////////////////////////////////
// DEBUGGING
////////////////////////////////////////////////////////////////////////////////

/* Set this to 1 to activate serial debugging */
#define ENABLE_UHS_DEBUGGING 0

////////////////////////////////////////////////////////////////////////////////
// MASS STORAGE
////////////////////////////////////////////////////////////////////////////////
// <<<<<<<<<<<<<<<< IMPORTANT >>>>>>>>>>>>>>>
// Set this to 1 to support single LUN devices, and save RAM. -- I.E. thumb drives.
// Each LUN needs ~13 bytes to be able to track the state of each unit.
#ifndef MASS_MAX_SUPPORTED_LUN
#define MASS_MAX_SUPPORTED_LUN 8
#endif

////////////////////////////////////////////////////////////////////////////////
// AUTOMATIC Settings
////////////////////////////////////////////////////////////////////////////////

// No user serviceable parts below this line.
// DO NOT change anything below here unless you are a developer!

//#include "version_helper.h"

#if !defined(DEBUG_USB_HOST) && ENABLE_UHS_DEBUGGING
#define DEBUG_USB_HOST
#endif


#define PROGMEM

#define PSTR(s) (s)

#define pgm_read_byte(addr) (*reinterpret_cast<const uint8_t*>(addr))
#define pgm_read_word(addr) (*reinterpret_cast<const uint16_t*>(addr))


#endif /* SETTINGS_H */

// ----------------------------------------------------------------------------
// @file    lpc408x_7x_crp.cpp
// @brief   CRP (Code Read Protect) word definition.
// @date    19 October 2018
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
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

#include "core/target_specs.hpp"

#ifdef __LPC408X_7X__

#include "xarmlib_config.hpp"
#include "targets/LPC408x_7x/lpc408x_7x_section_macros.hpp"

extern "C"
{




#define CRP_NO_CRP              0xFFFFFFFF

// Disables SWD debugging & JTAG, leaves ISP with with reads and writes
// enabled. You will need UART connectivity and FlashMagic (flashmagictool.com)
// to reverse this. Don't even try this without these tools; most likely the
// SWD flash programming will not even complete.
// Allows reads and writes only to RAM above 0x10000200 and flash other than
// sector 0 (the first 4 kB). Full erase also allowed- again only through UART
// and FlashMagic (NO JTAG/SWD)
#define CRP_CRP1                0x12345678

// Disables SWD debugging & JTAG, leaves UART ISP with only full erase
// enabled. You must have UART access and FlashMagic before setting this
// option.
// Don't even try this without these tools; most likely the SWD flash
// programming will not even complete.
#define CRP_CRP2                0x87654321

/************************************************************/
/**** DANGER CRP3 WILL LOCK PART TO ALL READS and WRITES ****/
#define CRP_CRP3_CONSUME_PART   0x43218765
/************************************************************/

#if (XARMLIB_CONFIG_CRP_SETTING == CRP_NO_CRP)
#define CURRENT_CRP_SETTING     CRP_NO_CRP
#elif (XARMLIB_CONFIG_CRP_SETTING == CRP_CRP1)
#define CURRENT_CRP_SETTING     CRP_CRP1
#elif (XARMLIB_CONFIG_CRP_SETTING == CRP_CRP2)
#define CURRENT_CRP_SETTING     CRP_CRP2
#elif (XARMLIB_CONFIG_CRP_SETTING == CRP_CRP3_CONSUME_PART)
#define CURRENT_CRP_SETTING     CRP_CRP3_CONSUME_PART
#endif

#ifndef CURRENT_CRP_SETTING
#define CURRENT_CRP_SETTING     CRP_NO_CRP
#endif




// ----------------------------------------------------------------------------
// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// ----------------------------------------------------------------------------
__CRP const unsigned int CRP_WORD = CURRENT_CRP_SETTING;




} // extern "C"

#endif // __LPC408X_7X__

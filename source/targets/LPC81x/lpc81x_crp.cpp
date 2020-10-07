// ----------------------------------------------------------------------------
// @file    lpc81x_crp.cpp
// @brief   CRP (Code Read Protect) word definition.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#include "xarmlib_config.hpp"
#include "core/target_specs.hpp"

#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_section_macros.hpp"




extern "C"
{

#define CRP_NO_CRP              0xFFFFFFFFU

// Disables UART and USB In System Programming (reads and writes)
// Leaves SWD debugging, with reads and writes, enabled
#define CRP_NO_ISP              0x4E697370U

// Disables SWD debugging & JTAG, leaves ISP with with reads and writes
// enabled. You will need UART connectivity and FlashMagic (flashmagictool.com)
// to reverse this. Don't even try this without these tools; most likely the
// SWD flash programming will not even complete.
// Allows reads and writes only to RAM above 0x10000300 and flash other than
// sector 0 (the first 4 kB). Full erase also allowed- again only through UART
// and FlashMagic (NO JTAG/SWD)
#define CRP_CRP1                0x12345678U

// Disables SWD debugging & JTAG, leaves UART ISP with only full erase
// enabled. You must have UART access and FlashMagic before setting this
// option.
// Don't even try this without these tools; most likely the SWD flash
// programming will not even complete.
#define CRP_CRP2                0x87654321U

/************************************************************/
/**** DANGER CRP3 WILL LOCK PART TO ALL READS and WRITES ****/
#define CRP_CRP3_CONSUME_PART   0x43218765U
/************************************************************/

#if (XARMLIB_CONFIG_CRP_SETTING == CRP_NO_CRP)
#define CURRENT_CRP_SETTING     CRP_NO_CRP
#elif (XARMLIB_CONFIG_CRP_SETTING == CRP_NO_ISP)
#define CURRENT_CRP_SETTING     CRP_NO_ISP
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




#endif // defined(__LPC81X__)

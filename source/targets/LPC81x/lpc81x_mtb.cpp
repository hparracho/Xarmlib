// ----------------------------------------------------------------------------
// @file    lpc81x_mtb.cpp
// @brief   Optionally defines an array to be used as a buffer for Micro
//          Trace Buffer (MTB) instruction trace on Cortex-M0+ parts.
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




extern "C"
{

// ----------------------------------------------------------------------------
//
// Symbols controlling behavior of this code...
//
// XARMLIB_ENABLE_MTB
//      If this symbol is defined, then the
//      buffer array for the MTB will be created.
//
// XARMLIB_CONFIG_MTB_BUFFER_SIZE
//      Symbol specifying the sizer of the buffer array for the MTB.
//      This must be a power of 2 in size, and fit into the available
//      RAM. The MTB buffer will also be aligned to its 'size'
//      boundary and be placed at the start of a RAM bank (which
//      should ensure minimal or zero padding due to alignment).
//
// XARMLIB_CONFIG_MTB_RAM_BANK
//      Allows MTB Buffer to be placed into specific RAM
//      bank. When this is not defined, the "default"
//      (first if there are several) RAM bank is used.
//
// ----------------------------------------------------------------------------
//
// __MTB_BUFFER(512)
//      Allocates a buffer of 512 bytes in the default ram memory region
//
// __MTB_BUFFER_EXT(1024, RAM2)
//      Allocates a buffer of 1024 bytes in the 2nd (RAM2) ram memory region
//
// ----------------------------------------------------------------------------

// Allow MTB to be removed by setting a define
#if (XARMLIB_ENABLE_MTB == 1)

// Allow for MTB buffer size being set by define set via
// command line. Otherwise provide small default buffer.
#ifndef XARMLIB_CONFIG_MTB_BUFFER_SIZE
#define XARMLIB_CONFIG_MTB_BUFFER_SIZE      (128)
#endif

// Check that buffer size requested is > 0 bytes in size
#if (XARMLIB_CONFIG_MTB_BUFFER_SIZE > 0)

#include "targets/LPC81x/lpc81x_section_macros.hpp"

#define __MTB_BUFFER_EXT(size, bank)         \
             __SECTION(mtb, bank)            \
             __attribute__ ((aligned(size))) \
             char __mtb_buffer__[size]

#define __MTB_BUFFER(size) __MTB_BUFFER_EXT(size, RAM)


// Check if MTB buffer is to be placed in specific RAM bank
#ifdef XARMLIB_CONFIG_MTB_RAM_BANK
// Place MTB buffer into explicit bank of RAM
__MTB_BUFFER_EXT(XARMLIB_CONFIG_MTB_BUFFER_SIZE, XARMLIB_CONFIG_MTB_RAM_BANK);
#else
// Place MTB buffer into 'default' bank of RAM
__MTB_BUFFER(XARMLIB_CONFIG_MTB_BUFFER_SIZE);
#endif

#endif // (XARMLIB_CONFIG_MTB_BUFFER_SIZE > 0)

#endif // (XARMLIB_ENABLE_MTB == 1)

} // extern "C




#endif // defined(__LPC81X__)

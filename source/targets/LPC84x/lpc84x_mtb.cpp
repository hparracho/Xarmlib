// ----------------------------------------------------------------------------
// @file    lpc84x_mtb.cpp
// @brief   Optionally defines an array to be used as a buffer for Micro
//          Trace Buffer (MTB) instruction trace on Cortex-M0+ parts.
// @date    18 May 2018
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

#include "system/target"

#ifdef __LPC84X__

#include "xarmlib_config.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




// ----------------------------------------------------------------------------
//
// Symbols controlling behavior of this code...
//
// __MTB_DISABLE
//      If this symbol is defined, then the buffer
//      array for the MTB will not be created.
//
// __MTB_BUFFER_SIZE
//      Symbol specifying the sizer of the buffer array for the MTB.
//      This must be a power of 2 in size, and fit into the available
//      RAM. The MTB buffer will also be aligned to its 'size'
//      boundary and be placed at the start of a RAM bank (which
//      should ensure minimal or zero padding due to alignment).
//
// __MTB_RAM_BANK
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
#ifndef __MTB_DISABLE

// Allow for MTB buffer size being set by define set via
// command line. Otherwise provide small default buffer.
#ifndef __MTB_BUFFER_SIZE
#define __MTB_BUFFER_SIZE 128
#endif

// Check that buffer size requested is > 0 bytes in size
#if (__MTB_BUFFER_SIZE > 0)

#include "targets/LPC84x/lpc84x_section_macros.h"

#define __MTB_BUFFER_EXT(size, bank)         \
             __SECTION(mtb, bank)            \
             __attribute__ ((aligned(size))) \
             char __mtb_buffer__[size]

#define __MTB_BUFFER(size) __MTB_BUFFER_EXT(size, RAM)


// Check if MTB buffer is to be placed in specific RAM bank
#ifdef __MTB_RAM_BANK
// Place MTB buffer into explicit bank of RAM
__MTB_BUFFER_EXT(__MTB_BUFFER_SIZE, __MTB_RAM_BANK);
#else
// Place MTB buffer into 'default' bank of RAM
__MTB_BUFFER(__MTB_BUFFER_SIZE);
#endif

#endif // (__MTB_BUFFER_SIZE > 0)

#endif // !__MTB_DISABLE




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __LPC84X__

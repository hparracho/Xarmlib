// ----------------------------------------------------------------------------
// @file    xarmlib_config.hpp
// @brief   Xarmlib configuration file for FPIO32S-V1-blinky example.
// @date    21 May 2018
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

#ifndef __XARMLIB_CONFIG_HPP
#define __XARMLIB_CONFIG_HPP

#include "xarmlib.hpp"

namespace xarmlib
{




// ----------------------------------------------------------------------------
// SYSTEM DEFINITIONS
// ----------------------------------------------------------------------------

constexpr System::Clock XARMLIB_SYSTEM_CLOCK { System::Clock::OSC_30MHZ };




// CRP (Code Read Protect) word definition
#define CONFIG_CRP_SETTING_NO_CRP       (1)

// Buffer for Micro Trace Buffer (MTB) instruction trace on Cortex-M0+ parts
#ifdef NDEBUG
#define __MTB_DISABLE
#else
#define __MTB_BUFFER_SIZE               (256)
#endif

// Uncomment the next line to disable Heap memory allocation functionality
//#define CPP_NO_HEAP




// ----------------------------------------------------------------------------
// DEFINITIONS
// ----------------------------------------------------------------------------

// Define maximum number of input pins to be debounced
constexpr std::size_t XARMLIB_CONFIG_DEBOUNCER_INPUT_COUNT_MAX { 0 };




} // namespace xarmlib

#endif // __XARMLIB_CONFIG_HPP

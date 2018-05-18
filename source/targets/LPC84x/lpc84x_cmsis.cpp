// ----------------------------------------------------------------------------
// @file    lpc84x_cmsis.cpp
// @brief   CMSIS Core Peripheral Access Layer source file for NXP LPC84x MCUs.
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

#include "targets/LPC84x/lpc84x_syscon_clock.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{

extern "C"
{




// CMSIS system core clock variable
uint32_t SystemCoreClock;




// Update system core clock frequency
// NOTE: This function is called in startup functions but should be
//       called every time the system has a clock frequency change.
void SystemCoreClockUpdate()
{
    // Store the clock frequency in the SystemCoreClock global RAM location
    SystemCoreClock = Clock::get_system_clock_frequency();
}




} // extern "C"

} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __LPC84X__

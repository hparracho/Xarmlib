// ----------------------------------------------------------------------------
// @file    lpc84x_romdivide.h
// @brief   Patch the AEABI integer divide functions to use MCU's romdivide library.
// @date    7 March 2018
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
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




#ifndef __XARMLIB_TARGETS_LPC84X_ROMDIVIDE_H
#define __XARMLIB_TARGETS_LPC84X_ROMDIVIDE_H

#ifdef __LPC84X__

#ifdef __USE_ROMDIVIDE

// Patch the AEABI integer divide functions to use MCU's romdivide library.
extern "C" void ROMDIVIDE_PatchAeabiIntegerDivide(void);

#endif // __USE_ROMDIVIDE

#endif // __LPC84X__

#endif // __XARMLIB_TARGETS_LPC84X_ROMDIVIDE_H

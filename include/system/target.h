// ----------------------------------------------------------------------------
// @file    target.h
// @brief   System level target configuration header file.
// @date    10 May 2018
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

#ifndef __XARMLIB_SYSTEM_TARGET_H
#define __XARMLIB_SYSTEM_TARGET_H




#if defined (LPC845M301JBD64)
#   define __LPC845__
#   define __LPC84X__
#   define __LPC84X_PINS__                  (64)
#   define __LPC84X_GPIOS__                 (54)
#elif defined (LPC845M301JBD48) || defined (LPC845M301JHI48)
#   define __LPC845__
#   define __LPC84X__
#   define __LPC84X_PINS__                  (48)
#   define __LPC84X_GPIOS__                 (42)
#elif defined (LPC845M301JHI33)
#   define __LPC845__
#   define __LPC84X__
#   define __LPC84X_PINS__                  (33)
#   define __LPC84X_GPIOS__                 (29)
#elif defined (LPC844M201JBD64)
#   define __LPC844__
#   define __LPC84X__
#   define __LPC84X_PINS__                  (64)
#   define __LPC84X_GPIOS__                 (54)
#elif defined (LPC844M201JBD48) || defined (LPC844M201JHI48)
#   define __LPC844__
#   define __LPC84X__
#   define __LPC84X_PINS__                  (48)
#   define __LPC84X_GPIOS__                 (42)
#elif defined (LPC844M201JHI33)
#   define __LPC844__
#   define __LPC84X__
#   define __LPC84X_PINS__                  (33)
#   define __LPC84X_GPIOS__                 (29)
#else
#   error "Target MCU not defined!"
#endif




#if defined __LPC84X__
#   define __TARGET_TIMER_TYPE_IS_MRT__
#endif




#endif // __XARMLIB_SYSTEM_TARGET_H

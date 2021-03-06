// ----------------------------------------------------------------------------
// @file    freertos.hpp
// @brief   FreeRTOS header file to use in the library. This should be the only
//          header file included when FreeRTOS usage is required.
// @date    28 September 2020
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

#ifndef __XARMLIB_EXTERNAL_FREERTOS_HPP
#define __XARMLIB_EXTERNAL_FREERTOS_HPP

#include "xarmlib_config.hpp"

#if defined(XARMLIB_ENABLE_FREERTOS) && (XARMLIB_ENABLE_FREERTOS == 1)

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#endif // defined(XARMLIB_ENABLE_FREERTOS) && (XARMLIB_ENABLE_FREERTOS == 1)

#endif // __XARMLIB_EXTERNAL_FREERTOS_HPP

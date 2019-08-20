// ----------------------------------------------------------------------------
// @file    kv4x_cmsis.hpp
// @brief   CMSIS Core Peripheral Access Layer header file for Kinetis KV4x MCUs.
// @date    31 October 2018
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

#ifndef __XARMLIB_TARGETS_KV4X_CMSIS_HPP
#define __XARMLIB_TARGETS_KV4X_CMSIS_HPP

#include "targets/KV4x/kv4x_specs.hpp"

#if defined (__KV46__)
#include "fsl_mkv46f16_cmsis.h"
#elif defined (__KV44__)
#include "fsl_mkv44f16_cmsis.h"
#elif defined (__KV42__)
#include "fsl_mkv42f16_cmsis.h"
#endif

#endif // __XARMLIB_TARGETS_KV4X_CMSIS_HPP

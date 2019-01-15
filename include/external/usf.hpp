// ----------------------------------------------------------------------------
// @file    usf.hpp
// @brief   Micro String Format (USF) configuration and main header file
//          to use in the library. This should be the only header file
//          included when USF functionality is required.
// @date    8 January 2019
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

#ifndef __XARMLIB_EXTERNAL_USF_HPP
#define __XARMLIB_EXTERNAL_USF_HPP

// ----------------------------------------------------------------------------
// usflib configuration options
// ----------------------------------------------------------------------------

// Configuration of floating point support.
// USF_DISABLE_FLOAT_SUPPORT           : disables the support of floating point types (it will save considerable code size)

// Configuration of format output string termination option.
// USF_DISABLE_STRING_TERMINATION      : disables the null termination of the format output string

// Configuration of possible behavior when a condition is violated.
// USF_TERMINATE_ON_CONTRACT_VIOLATION : std::terminate() will be called (default)
// USF_ABORT_ON_CONTRACT_VIOLATION     : std::abort() will be called (more suitable for embedded platforms, maybe?)
// USF_THROW_ON_CONTRACT_VIOLATION     : an exception will be thrown

// ----------------------------------------------------------------------------

#define USF_ABORT_ON_CONTRACT_VIOLATION

#include "usf/usf.hpp"

#endif // __XARMLIB_EXTERNAL_USF_HPP

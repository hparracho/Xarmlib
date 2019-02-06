// ----------------------------------------------------------------------------
// @file    span.hpp
// @brief   std::span implementation for C++11 and later (SPAN) configuration
//          and main header file to use in the library. This should be the only
//          header file included when SPAN functionality is required.
// @date    6 February 2019
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

#ifndef __XARMLIB_EXTERNAL_SPAN_HPP
#define __XARMLIB_EXTERNAL_SPAN_HPP

#if defined(__has_include) && __has_include(<span>)
#  include <span>
#else
#  define TCB_SPAN_NAMESPACE_NAME std
#  define TCB_SPAN_NO_EXCEPTIONS
#  define TCB_SPAN_STD_COMPLIANT_MODE
#  define TCB_SPAN_TERMINATE_ON_CONTRACT_VIOLATION
#  include "tcb/span.hpp"
#endif

#endif // __XARMLIB_EXTERNAL_SPAN_HPP

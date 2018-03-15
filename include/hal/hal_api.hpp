// ----------------------------------------------------------------------------
// @file    hal_api.hpp
// @brief   HAL API main header file.
// @date    15 March 2018
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
//
// Emanuel Pinto(emanuelangelopinto@gmail.com) is an official contributor of
// this library and some of the following code is based on his original work.
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

#ifndef __XARMLIB_HAL_API_HPP
#define __XARMLIB_HAL_API_HPP




#if defined __LPC84X__
#include "targets/LPC84x/lpc84x_watchdog.hpp"

namespace xarmlib
{

namespace hal = lpc84x;

} // namespace xarmlib

#elif defined __OHER_TARGET__
// Other target include files

namespace xarmlib
{

namespace hal = other_target;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_API_HPP

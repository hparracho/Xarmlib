// ----------------------------------------------------------------------------
// @file    non_copyable.hpp
// @brief   Non copyable helper class. Classes which are not value type should
//          inherit privately from this class to avoid generation of invalid
//          copy constructor or copy assignment operator.
// @date    6 July 2018
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

#ifndef __XARMLIB_CORE_NON_COPYABLE_HPP
#define __XARMLIB_CORE_NON_COPYABLE_HPP

namespace xarmlib
{




template<typename Type>
class NonCopyable
{
        // The template parameter 'Type' is the type that should be made non copyable.
        // It prevent cases where the empty base optimization cannot be applied and
        // therefore ensure zero cost while using this class.
    
    protected:

        // Prevent the use of this class as a final one (without being derived)
        NonCopyable()
        {}

        ~NonCopyable()
        {}

    private:

        // NonCopyable copy constructor
        NonCopyable(const NonCopyable&) = delete;

        // NonCopyable copy assignment operator
        NonCopyable& operator = (const NonCopyable&) = delete;
};




} // namespace xarmlib

#endif // __XARMLIB_CORE_NON_COPYABLE_HPP

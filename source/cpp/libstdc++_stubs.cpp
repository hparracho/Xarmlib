// ----------------------------------------------------------------------------
// @file    libstdc++_stubs.cpp
// @brief   Minimal implementations of the verbose terminate handler for
//          exceptions and pure virtual functions call handler to avoid
//          references to the heavy implementations in the standard C++ library.
// @date    21 June 2018
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

// Forward declaration of abort function defined in 'newlib_stubs.cpp' file.
extern "C" __attribute__ ((weak, noreturn))
void abort(void);




// ----------------------------------------------------------------------------
// This function is called when an uncaught C++ exception is encountered. The
// default version within the C++ library prints the name of the uncaught
// exception, but to do so it must demangle its name - which causes a large
// amount of code to be pulled in. The below minimal implementation can reduce
// code size noticeably. Note that this function should not return.
// ----------------------------------------------------------------------------
namespace __gnu_cxx
{

[[noreturn]] void __verbose_terminate_handler()
{
    abort();
}

} // namespace __gnu_cxx




// ----------------------------------------------------------------------------
// This function is referenced whenever the code contains pure virtual
// functions. This is a placeholder used while constructing an object, and
// the function should never actually get called. The below minimal
// implementation removes all the dependencies from original function and can
// reduce code size noticeably. Note that this function should not return.
// ----------------------------------------------------------------------------
extern "C" [[noreturn]] void __cxa_pure_virtual()
{
    abort();
}




// ----------------------------------------------------------------------------
// Handle for the DSO (Dynamic Shared Object)
void* __dso_handle __attribute__ ((weak));

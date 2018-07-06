// ----------------------------------------------------------------------------
// @file    heap_operators.cpp
// @brief   Minimal implementations of the new/delete operators. Optional null
//          stubs for malloc/free (only used if symbol CPP_NO_HEAP is defined).
// @date    28 March 2018
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

#include <cstdlib>




#ifndef CPP_NO_HEAP

void* operator new(std::size_t size) noexcept
{
    return malloc(size);
}

void* operator new[](std::size_t size) noexcept
{
    return malloc(size);
}

void operator delete(void* ptr) noexcept
{
    free(ptr);
}

void operator delete(void* ptr, std::size_t size __attribute__((unused))) noexcept
{
    free(ptr);
}

void operator delete[](void* ptr) noexcept
{
    free(ptr);
}

void operator delete[](void* ptr, std::size_t size __attribute__((unused))) noexcept
{
    free(ptr);
}




// Default placement versions of operator new.
inline void* operator new(std::size_t size __attribute__((unused)),
                          void* ptr) noexcept
{
    return ptr;
}

inline void* operator new[](std::size_t size __attribute__((unused)),
                            void* ptr) noexcept
{
    return ptr;
}

// Default placement versions of operator delete.
inline void operator delete(void* ptr __attribute__((unused)),
                            void* place __attribute__((unused))) noexcept
{}

inline void operator delete[](void* ptr __attribute__((unused)),
                              void* place __attribute__((unused))) noexcept
{}

#else // CPP_NO_HEAP

extern "C" void* malloc(size_t size __attribute__((unused)))
{
    return nullptr;
}

extern "C" void free(void* ptr __attribute__((unused)))
{}

#endif // CPP_NO_HEAP

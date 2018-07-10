// ----------------------------------------------------------------------------
// @file    newlib_stubs.cpp
// @brief   Support files for GNU libc. These functions will replace or
//          extend some of the newlib functionality.
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

#include "system/cmsis"

extern "C"
{




#if !defined (MCUXPRESSO_MANAGED_LINKER_SCRIPTS) && !defined (CPP_NO_HEAP)

#include <sys/types.h>
#include <errno.h>

// ----------------------------------------------------------------------------
// The function below is taken from the µOS++ IIIe project.
// (https://github.com/micro-os-plus)
// Copyright (c) 2016 Liviu Ionescu.

// A custom _sbrk() function to match the settings defined by the linker script.
caddr_t _sbrk(int incr)
{
    extern char __heap_begin;   // Defined in the linker script
    extern char __heap_limit;   // Defined in the linker script

    static char* current_heap_end;
    char* current_block_address;

    if(current_heap_end == 0)
    {
        current_heap_end = &__heap_begin;
    }

    current_block_address = current_heap_end;

    // Need to align heap to word boundary, else will get hard
    // faults on Cortex-M0. So we assume that heap starts on word
    // boundary, hence make sure we always add a multiple of 4 to it.
    incr = (incr + 3) & (~3); // Align value to 4
    if(current_heap_end + incr > &__heap_limit)
    {
        // Some of the libstdc++-v3 tests rely upon detecting
        // out of memory errors, so do not abort here.
#if 0
        extern void abort(void);

        _write(1, "_sbrk: Heap and stack collision\n", 32);

        abort();
#else
        // Heap has overflowed
        errno = ENOMEM;
        return (caddr_t)-1;
#endif
    }

    current_heap_end += incr;

    return (caddr_t)current_block_address;
}
// ----------------------------------------------------------------------------
#endif // !MCUXPRESSO_MANAGED_LINKER_SCRITS && !CPP_NO_HEAP




__attribute__ ((weak, noreturn))
void _exit(int code __attribute__ ((unused)))
{
#ifdef NDEBUG
    NVIC_SystemReset();
#else
    while(1)
    {}
#endif
}




__attribute__ ((weak, noreturn))
void abort(void)
{
    _exit(1);
}




} // extern "C"

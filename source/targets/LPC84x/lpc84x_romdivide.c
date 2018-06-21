// ----------------------------------------------------------------------------
// @file    lpc84x_romdivide.cpp
// @brief   Patch the AEABI integer divide functions to use MCU's romdivide library.
// @date    18 May 2018
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

#include "system/target"

#ifdef __LPC84X__

#include <targets/LPC84x/lpc84x_cmsis.h>




// Variables to store addresses of idiv and udiv functions within MCU ROM
uint32_t* __romdiv_idiv;
uint32_t* __romdiv_uidiv;




// Patch the AEABI integer divide functions to use MCU's romdivide library.
void ROMDIVIDE_PatchAeabiIntegerDivide(void)
{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
	// ISO C forbids conversion of function pointer to object pointer type [-Wpedantic]

    // Get addresses of integer divide routines in ROM
    // These address are then used by the code in aeabi_romdiv_patch.s
    __romdiv_idiv  = (uint32_t*)LPC_ROM_DIV_API->idiv;
    __romdiv_uidiv = (uint32_t*)LPC_ROM_DIV_API->uidiv;

#pragma GCC diagnostic pop
}




#endif // __LPC84X__

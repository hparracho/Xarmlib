/*
// ----------------------------------------------------------------------------
// @file    lpc4076.ld
// @brief   Linker Script for NXP LPC4076 MCU.
// @date    19 October 2018
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
*/




MEMORY
{
    /* Define each memory region */
    Flash (rx)  : ORIGIN = 0x0,        LENGTH = 0x40000 /* 256K bytes */
    RAM   (rwx) : ORIGIN = 0x10000000, LENGTH = 0x10000 /* 64K bytes  */
    RAM2  (rwx) : ORIGIN = 0x20000000, LENGTH = 0x4000  /* 16K bytes  */
}




/* Define a symbol for the base and top of each memory region */
__flash_base = 0x0;                  /* Flash */   
__flash_top  = 0x0 + 0x40000;        /* 256K bytes */  
__ram_base   = 0x10000000;           /* RAM */  
__ram_top    = 0x10000000 + 0x10000; /* 64K bytes */  
__ram2_base  = 0x20000000;           /* RAM2 */  
__ram2_top   = 0x20000000 + 0x4000;  /* 16K bytes */  




/* Define the required Stack and Heap sizes */
__stack_size    = 0x400;                /* 1K bytes */
__heap_size     = 0x400;                /* 1K bytes */

/* IAP commands, which results in a flash write/erase operation, use 32 bytes
 * of space in the top portion of the on-chip RAM for execution. The user
 * program should not use this space if IAP flash programming is permitted
 * in the application.
 */
__stack_offset  = 32;                   /* Stack offset : 0 or 32 */




INCLUDE "lpc408x_7x_common.ld"

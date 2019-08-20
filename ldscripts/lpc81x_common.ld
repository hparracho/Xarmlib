/*
// ----------------------------------------------------------------------------
// @file    lpc81x_common.ld
// @brief   Linker Script common part for NXP LPC81x MCUs.
// @date    16 August 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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




GROUP
(
    "libgcc.a"
    "libc_nano.a"
    "libstdc++_nano.a"
    "libm.a"
)




ENTRY(Reset_Handler)




SECTIONS
{
    /* Main TEXT section */
    .text : ALIGN(4)
    {
        FILL(0xFF)
        __vectors_start__ = ABSOLUTE(.);
        KEEP(*(.isr_vector))

        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        __bss_section_table_end = .;
        __section_table_end = .;
        /* End of Global Section Table */

        *(.after_vectors*)

        /* Code Read Protection data */
        . = 0x000002FC ;
        PROVIDE(__CRP_WORD_START__ = .);
        KEEP(*(.crp))
        PROVIDE(__CRP_WORD_END__ = .);
        ASSERT(!(__CRP_WORD_START__ == __CRP_WORD_END__), "Linker CRP Enabled, but no CRP_WORD provided within application");
        /* End of Code Read Protection */
    } > Flash

    .text : ALIGN(4)
    {
        *(.text*)
        *(.rodata .rodata.* .constdata .constdata.*)
        . = ALIGN(4);

        /* C++ constructors */
        . = ALIGN(4);
        KEEP(*(.init))

        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        . = ALIGN(4);
    } > Flash

    /* Used for exception handling/unwind - some Newlib
     * functions (in common with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > Flash
    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > Flash
    __exidx_end = .;

    /* Some NXP parts have drivers built into the ROM. These drivers need
     * some internal memory that is *always* at the start of the first RAM
     * block. This is not configurable, so if the driver is used, then the
     * application must not use that RAM.
     */
    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RAM

    /* Default MTB section */
    .mtb_buffer_default (NOLOAD) : ALIGN(4)
    {
       KEEP(*(.mtb*))
    } > RAM

    /* Main DATA section (RAM) */
    .data : ALIGN(4)
    {
       FILL(0xFF)
       _data = . ;
       *(vtable)
       *(.ramfunc*)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
    } > RAM AT > Flash

    /* Main BSS section */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        /*PROVIDE(end = .);*/
    } > RAM

    /* Default NOINIT section */
    .noinit (NOLOAD) : ALIGN(4)
    {
        _noinit = .;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RAM

    /* Reserve space and place Heap within memory map */
    .heap : ALIGN(4)
    {
        __heap_begin = .;
        . += __heap_size;
        . = ALIGN(4);
        __heap_limit = .;
    } > RAM

     /* Reserve space in memory for Stack */
    .heap2stackfill  :
    {
        . += __stack_size;
    } > RAM

    /* Locate actual Stack in memory map */
    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size - __stack_offset : ALIGN(4)
    {
        __stack_base = .;
        . = ALIGN(4);
        __stack_top = . + __stack_size;
    } > RAM

    /* ## Create checksum value (used in startup) ## */
    PROVIDE(__valid_user_code_checksum = 0 - (__stack_top + (Reset_Handler     + 1)
                                                          + (NMI_Handler       + 1)
                                                          + (HardFault_Handler + 1)));
}

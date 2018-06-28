// ----------------------------------------------------------------------------
// @file    mcu_startup.cpp
// @brief   MCU bare-metal startup code.
// @date    10 June 2018
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

// ----------------------------------------------------------------------------
// The code below is *HEAVILY* based on the Cortex-M Startup project,
// part of the µOS++ IIIe project. (https://github.com/micro-os-plus)
// Copyright (c) 2016 Liviu Ionescu.
// ----------------------------------------------------------------------------

#include <cstdint>

extern "C"
{




// ----------------------------------------------------------------------------
// Copy the specified DATA section from flash to SRAM.
// ----------------------------------------------------------------------------
inline __attribute__((always_inline))
static void mcu_initialize_data(uint32_t* from, uint32_t* region_addr, const uint32_t region_size)
{
    // Iterate and copy word by word.
    // It is assumed that the pointers are word aligned.
    const uint32_t* region_end = region_addr + (region_size / sizeof(uint32_t));

    while(region_addr < region_end)
    {
        *region_addr++ = *from++;
    }
}




// ----------------------------------------------------------------------------
// Zero fill the specified BSS section.
// ----------------------------------------------------------------------------
inline __attribute__((always_inline))
static void mcu_initialize_bss(uint32_t* region_addr, const uint32_t region_size)
{
    // Iterate and clear word by word.
    // It is assumed that the pointers are word aligned.
    const uint32_t* region_end = region_addr + (region_size / sizeof(uint32_t));

    while(region_addr < region_end)
    {
        *region_addr++ = 0;
    }
}




// ----------------------------------------------------------------------------
// Iterate over all the preinit/init routines (mainly static constructors).
// ----------------------------------------------------------------------------
inline __attribute__((always_inline))
static void mcu_cpp_init_array()
{
    // These magic symbols are provided by the Linker Script.
    extern void (*__preinit_array_start[])(void) __attribute__((weak));
    extern void (*__preinit_array_end[])  (void) __attribute__((weak));
    extern void (*__init_array_start[])   (void) __attribute__((weak));
    extern void (*__init_array_end[])     (void) __attribute__((weak));

    int32_t count = __preinit_array_end - __preinit_array_start;
    for(int32_t i = 0; i < count; i++)
    {
        __preinit_array_start[i]();
    }

    // If the application needs to run the code in the .init section,
    // please use the startup files, since this requires the code in
    // crti.o and crtn.o to add the function prologue/epilogue.
    //_init(); // DO NOT ENABE THIS!

    count = __init_array_end - __init_array_start;
    for(int32_t i = 0; i < count; i++)
    {
        __init_array_start[i]();
    }
}




// ----------------------------------------------------------------------------
// This is the place where the Cortex-M core will go immediately
// after reset (the 'Reset_Handler' calls this function).
// To reach this location, the reset stack must point to a valid
// internal RAM area.
// Debugging new startup configurations usually begins with placing
// a breakpoint at 'mcu_startup()', and stepping through the routine.
// ----------------------------------------------------------------------------
__attribute__ ((section(".after_vectors"), noreturn))
void mcu_startup()
{
    // ------------------------------------------------------------------------
    // After Reset the Cortex-M processor is in Thread mode,
    // priority is Privileged, and the Stack is set to Main.

    // Initialize hardware right after reset, to switch clock to higher
    // frequency and have the rest of the initializations run faster.
    //
    // Mandatory on platforms that start with the watchdog
    // enabled and require an early sequence to disable it.
    //
    // Also useful on platforms with external RAM, that need
    // to be initialized before filling the BSS section.
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    // The following symbols are constructs generated by the linker, indicating
    // the location of various points in the "Global Section Table". It contains
    // the load address, execution address and length of each RW data section
    // and the execution and length of each BSS (zero initialized) section.
    // ------------------------------------------------------------------------
    extern uint32_t __data_section_table;
    extern uint32_t __data_section_table_end;
    extern uint32_t __bss_section_table;
    extern uint32_t __bss_section_table_end;

    // ------------------------------------------------------------------------
    // Forward declaration of main and hardware initialization functions.
    // Should be implemented by each target with its specific initializations.
    // ------------------------------------------------------------------------
    extern int main(void);
    extern void mcu_startup_initialize_hardware_early();
    extern void mcu_startup_initialize_hardware();

    mcu_startup_initialize_hardware_early();

    // Copy the data sections from flash to SRAM
    for(uint32_t* p = &__data_section_table; p < &__data_section_table_end; )
    {
        uint32_t* from        = reinterpret_cast<uint32_t*>(*p++);
        uint32_t* region_addr = reinterpret_cast<uint32_t*>(*p++);
        uint32_t  region_size = *p++;

        mcu_initialize_data(from, region_addr, region_size);
    }

    // Zero fill all BSS sections
    for(uint32_t *p = &__bss_section_table; p < &__bss_section_table_end; )
    {
        uint32_t* region_addr = reinterpret_cast<uint32_t*>(*p++);
        uint32_t  region_size = *p++;

        mcu_initialize_bss(region_addr, region_size);
    }

    // Hook to continue the initializations. Usually compute and store the
    // clock frequency in the global CMSIS variable, configure IOs, etc...
    mcu_startup_initialize_hardware();

    // Call the standard library initialization (mandatory for C++ to
    // execute the constructors for the static objects).
    mcu_cpp_init_array();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
    // ISO C++ forbids taking address of function '::main' [-Wpedantic]

    // Call the main entry point
    main();
#pragma GCC diagnostic pop

    // 'main()' shouldn't return, but if it does,
    // we'll just enter an infinite loop.
    while(true)
    {}
}




} // extern "C"

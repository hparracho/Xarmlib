// ----------------------------------------------------------------------------
// @file    lpc81x_timer.cpp
// @brief   NXP LPC81x Timer (MRT) class.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#include "core/os_support.hpp"
#include "core/target_specs.hpp"

#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_timer.hpp"




using namespace xarmlib;

// ----------------------------------------------------------------------------
// IRQ HANDLER
// ----------------------------------------------------------------------------

extern "C" void MRT_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::Timer::irq_handler();

    Os::yield_from_isr(yield);
}




#endif // defined(__LPC81X__)

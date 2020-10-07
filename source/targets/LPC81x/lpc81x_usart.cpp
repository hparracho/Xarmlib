// ----------------------------------------------------------------------------
// @file    lpc81x_usart.cpp
// @brief   NXP LPC81x USART class.
// @notes   Synchronous mode not implemented.
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

#include "core/target_specs.hpp"

#if defined(__LPC81X__)

#include "core/os_support.hpp"
#include "targets/LPC81x/lpc81x_usart.hpp"




using namespace xarmlib;

// --------------------------------------------------------------------
// IRQ HANDLERS
// --------------------------------------------------------------------

extern "C" void USART0_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::Usart::irq_handler(0);

    Os::yield_from_isr(yield);
}




extern "C" void USART1_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::Usart::irq_handler(1);

    Os::yield_from_isr(yield);
}




#if (TARGET_USART_COUNT == 3)

extern "C" void USART2_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::Usart::irq_handler(2);

    Os::yield_from_isr(yield);
}

#endif




#endif // defined(__LPC81X__)

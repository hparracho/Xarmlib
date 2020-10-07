// ----------------------------------------------------------------------------
// @file    lpc81x_pin_int.cpp
// @brief   NXP LPC81x pin interrupt class.
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

#include "targets/LPC81x/lpc81x_pin_int.hpp"




using namespace xarmlib;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void PININT0_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(0);

    Os::yield_from_isr(yield);
}




extern "C" void PININT1_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(1);

    Os::yield_from_isr(yield);
}




extern "C" void PININT2_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(2);

    Os::yield_from_isr(yield);
}




extern "C" void PININT3_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(3);

    Os::yield_from_isr(yield);
}




extern "C" void PININT4_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(4);

    Os::yield_from_isr(yield);
}




extern "C" void PININT5_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(5);

    Os::yield_from_isr(yield);
}




extern "C" void PININT6_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(6);

    Os::yield_from_isr(yield);
}




extern "C" void PININT7_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::PinInt::irq_handler(7);

    Os::yield_from_isr(yield);
}




#endif // defined(__LPC81X__)

// ----------------------------------------------------------------------------
// @file    lpc81x_spi.cpp
// @brief   NXP LPC81x SPI class.
// @date    9 October 2020
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

#include "targets/LPC81x/lpc81x_spi.hpp"




using namespace xarmlib;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void SPI0_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::Spi::irq_handler(0);

    Os::yield_from_isr(yield);
}




#if (TARGET_SPI_COUNT == 2)

extern "C" void SPI1_IRQHandler(void)
{
    const int32_t yield = targets::lpc81x::Spi::irq_handler(1);

    Os::yield_from_isr(yield);
}

#endif




#endif // defined(__LPC81X__)

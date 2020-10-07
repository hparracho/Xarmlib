// ----------------------------------------------------------------------------
// @file    lpc81x_spi.cpp
// @brief   NXP LPC81x SPI class.
// @date    28 September 2020
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
#include "targets/LPC81x/lpc81x_spi.hpp"




using namespace xarmlib;

// ----------------------------------------------------------------------------
// IRQ HANDLERS
// ----------------------------------------------------------------------------

extern "C" void SPI0_IRQHandler(void)
{
    const int32_t yield = Spi::irq_handler(targets::lpc81x::Spi::Name::spi0);

    Os::yield_from_isr(yield);
}




#if (TARGET_SPI_COUNT == 2)

extern "C" void SPI1_IRQHandler(void)
{
    const int32_t yield = Spi::irq_handler(targets::lpc81x::Spi::Name::spi1);

    Os::yield_from_isr(yield);
}

#endif // (TARGET_SPI_COUNT == 2)




#endif // defined(__LPC81X__)

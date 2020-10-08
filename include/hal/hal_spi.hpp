// ----------------------------------------------------------------------------
// @file    hal_spi.hpp
// @brief   SPI HAL class specialized for the current target.
// @date    8 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_HAL_SPI_HPP
#define XARMLIB_HAL_SPI_HPP

#include "core/target_specs.hpp"




#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_spi.hpp"

namespace xarmlib::hal
{
using Spi = SpiBase<targets::lpc81x::Spi, targets::lpc81x::SpiTraits>;
}

#endif




#endif // XARMLIB_HAL_SPI_HPP

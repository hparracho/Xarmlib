// ----------------------------------------------------------------------------
// @file    target_gpio.hpp
// @brief   Final GPIO class.
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

#ifndef XARMLIB_CORE_TARGET_GPIO_HPP
#define XARMLIB_CORE_TARGET_GPIO_HPP

#include "core/target_specs.hpp"




#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_gpio.hpp"

namespace xarmlib
{
using Gpio = targets::lpc81x::Gpio;
}

#endif

#endif // XARMLIB_CORE_TARGET_GPIO_HPP

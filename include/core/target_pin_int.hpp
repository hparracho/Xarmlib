// ----------------------------------------------------------------------------
// @file    target_gpio.hpp
// @brief   Final Pin Interrupt class.
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

#ifndef XARMLIB_CORE_TARGET_PIN_INT_HPP
#define XARMLIB_CORE_TARGET_PIN_INT_HPP

#include "core/target_specs.hpp"




#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_pin_int.hpp"

namespace xarmlib
{
using PinInt = targets::lpc81x::PinInt;
}

#endif

#endif // XARMLIB_CORE_TARGET_PIN_INT_HPP

// ----------------------------------------------------------------------------
// @file    hal_port.hpp
// @brief   Port HAL class specialized for the current target.
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

#ifndef XARMLIB_HAL_PORT_HPP
#define XARMLIB_HAL_PORT_HPP

#include "core/target_specs.hpp"




#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_port.hpp"

namespace xarmlib::hal
{
using Port = PortBase<targets::lpc81x::Port>;
}

#endif




#endif // XARMLIB_HAL_PORT_HPP

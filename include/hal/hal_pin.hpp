// ----------------------------------------------------------------------------
// @file    hal_pin.hpp
// @brief   Pin HAL interface class.
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

#ifndef XARMLIB_HAL_PIN_HPP
#define XARMLIB_HAL_PIN_HPP

#include "core/target_specs.hpp"




namespace xarmlib::hal
{

class Pin
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using Name = xarmlib::targets::PinName;
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_PIN_HPP

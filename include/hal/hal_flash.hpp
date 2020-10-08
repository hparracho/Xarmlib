// ----------------------------------------------------------------------------
// @file    hal_flash.hpp
// @brief   Flash HAL classes specialized for the current target.
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

#ifndef XARMLIB_HAL_FLASH_HPP
#define XARMLIB_HAL_FLASH_HPP

#include "hal/hal_flash_base.hpp"




#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_flash_boot.hpp"
#include "targets/LPC81x/lpc81x_flash_iap.hpp"

namespace xarmlib::hal
{
using FlashBoot = FlashIapBase<targets::lpc81x::FlashBoot>;
using FlashIap  = FlashIapBase<targets::lpc81x::FlashIap>;
}

#endif




#endif // XARMLIB_HAL_FLASH_HPP

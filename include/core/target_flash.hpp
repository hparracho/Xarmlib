// ----------------------------------------------------------------------------
// @file    target_flash.hpp
// @brief   Final flash classes.
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

#ifndef XARMLIB_CORE_TARGET_FLASH_HPP
#define XARMLIB_CORE_TARGET_FLASH_HPP

#include "core/target_specs.hpp"




#if defined(__LPC81X__)

#include "targets/LPC81x/lpc81x_flash_boot.hpp"
#include "targets/LPC81x/lpc81x_flash_iap.hpp"

namespace xarmlib
{
using FlashBoot = targets::lpc81x::FlashBoot;
using FlashIap  = targets::lpc81x::FlashIap;
}

#endif

#endif // XARMLIB_CORE_TARGET_FLASH_HPP

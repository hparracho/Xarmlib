// ----------------------------------------------------------------------------
// @file    hal_flash_base.hpp
// @brief   Flash HAL interface classes.
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

#ifndef XARMLIB_HAL_FLASH_BASE_HPP
#define XARMLIB_HAL_FLASH_BASE_HPP

#include <span>




namespace xarmlib::hal
{

template <typename Driver>
class FlashBootBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // NOTE: the application should destruct all objects instantiated by itself before
    static void boot_application(const int32_t flash_address) { Driver::boot_application(flash_address); }
};




template <typename Driver>
class FlashIapBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Write to flash
    // @flash_address: Destination flash address where data is to be written.
    // @ buffer:       Buffer span containing the data to be written.
    // NOTE:           See target specific notes
    static bool write(const int32_t flash_address, const std::span<const uint8_t> buffer) { return Driver::write(flash_address, buffer); }
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_FLASH_BASE_HPP

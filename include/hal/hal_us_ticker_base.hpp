// ----------------------------------------------------------------------------
// @file    hal_us_ticker_base.hpp
// @brief   Microsecond ticker HAL interface class.
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

#ifndef XARMLIB_HAL_US_TICKER_BASE_HPP
#define XARMLIB_HAL_US_TICKER_BASE_HPP




namespace xarmlib::hal
{

template <typename Driver>
class UsTickerBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static uint32_t read()
    {
        return Driver::read();
    }
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_US_TICKER_BASE_HPP

// ----------------------------------------------------------------------------
// @file    hal_system_base.hpp
// @brief   HAL system level configuration class.
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

#ifndef XARMLIB_HAL_SYSTEM_BASE_HPP
#define XARMLIB_HAL_SYSTEM_BASE_HPP




namespace xarmlib::hal
{

template <typename Traits>
class SystemBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using Clock = typename Traits::Clock;

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static constexpr int32_t get_irc_frequency()
    {
        return Traits::k_irc_freq;
    }

    static constexpr int32_t get_xtal_frequency()
    {
        return Traits::k_xtal_freq;
    }

    static constexpr int32_t get_clkin_frequency()
    {
        return Traits::k_clkin_freq;
    }

    static constexpr int32_t get_core_clock_frequency(const Clock clock)
    {
        return Traits::get_core_clock_frequency(clock);
    }

    static constexpr int32_t get_main_clock_frequency(const Clock clock)
    {
        return Traits::get_main_clock_frequency(clock);
    }
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_SYSTEM_BASE_HPP

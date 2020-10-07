// ----------------------------------------------------------------------------
// @file    os_support.hpp
// @brief   Operating System (FreeRTOS / baremetal) support functions.
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

#ifndef XARMLIB_CORE_OS_SUPPORT_HPP
#define XARMLIB_CORE_OS_SUPPORT_HPP

#include "xarmlib_config.hpp"




namespace xarmlib
{

class Os
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // Provides context switch from an ISR handler (using FreeRTOS or baremetal)
    static void yield_from_isr([[maybe_unused]] const int32_t yield) noexcept
    {
#if (XARMLIB_ENABLE_FREERTOS == 1)
        portYIELD_FROM_ISR(yield);
#endif
    }
};

} // namespace xarmlib




#endif // XARMLIB_CORE_OS_SUPPORT_HPP

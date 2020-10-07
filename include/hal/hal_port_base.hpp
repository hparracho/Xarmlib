// ----------------------------------------------------------------------------
// @file    hal_port_base.hpp
// @brief   Port HAL interface class.
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

#ifndef XARMLIB_HAL_PORT_BASE_HPP
#define XARMLIB_HAL_PORT_BASE_HPP

#include "core/target_specs.hpp"




namespace xarmlib::hal
{

template <typename Driver>
class PortBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using Name = xarmlib::targets::PortName;

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static void set_direction  (const Name port, const uint32_t mask) { Driver::set_direction(port, mask); }
    static void clear_direction(const Name port, const uint32_t mask) { Driver::clear_direction(port, mask); }

    static uint32_t read_direction(const Name port)                      { return Driver::read_direction(port); }
    static uint32_t read_direction(const Name port, const uint32_t mask) { return Driver::read_direction(port, mask); }

    static void write_direction(const Name port, const uint32_t mask, const uint32_t value) { Driver::write_direction(port, mask, value); }

    static void set  (const Name port, const uint32_t mask) { Driver::set(port, mask); }
    static void clear(const Name port, const uint32_t mask) { Driver::clear(port, mask); }

    static uint32_t read(const Name port)                      { return Driver::read(port); }
    static uint32_t read(const Name port, const uint32_t mask) { return Driver::read(port, mask); }

    static void write(const Name port, const uint32_t mask, const uint32_t value) { Driver::write(port, mask, value); }
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_PORT_BASE_HPP

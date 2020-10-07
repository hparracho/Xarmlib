// ----------------------------------------------------------------------------
// @file    lpc81x_port.hpp
// @brief   NXP LPC81x port class.
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

#ifndef XARMLIB_TARGETS_LPC81X_PORT_HPP
#define XARMLIB_TARGETS_LPC81X_PORT_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_specs.hpp"
#include "hal/hal_port_base.hpp"




namespace xarmlib::targets::lpc81x
{

class Port : public hal::PortBase<Port>
{
public:

    using Name = xarmlib::targets::PortName;

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS - HAL IMPLEMENTATION
    // ------------------------------------------------------------------------

    static void set_direction(const Name port, const uint32_t mask)
    {
        // NOTE: Ones configure as outputs
        LPC_GPIO->DIR[static_cast<std::size_t>(port)] |= mask;
    }

    static void clear_direction(const Name port, const uint32_t mask)
    {
        // NOTE: Zeros configure as inputs
        LPC_GPIO->DIR[static_cast<std::size_t>(port)] &= ~mask;
    }

    static uint32_t read_direction(const Name port)
    {
        return LPC_GPIO->DIR[static_cast<std::size_t>(port)];
    }

    static uint32_t read_direction(const Name port, const uint32_t mask)
    {
        return LPC_GPIO->DIR[static_cast<std::size_t>(port)] & mask;
    }

    static void write_direction(const Name port, const uint32_t mask, const uint32_t value)
    {
        LPC_GPIO->DIR[static_cast<std::size_t>(port)] = (LPC_GPIO->DIR[static_cast<std::size_t>(port)] & ~mask) | (value & mask);
    }

    static void set(const Name port, const uint32_t mask)
    {
        LPC_GPIO->SET[static_cast<std::size_t>(port)] = mask;
    }

    static void clear(const Name port, const uint32_t mask)
    {
        LPC_GPIO->CLR[static_cast<std::size_t>(port)] = mask;
    }

    static uint32_t read(const Name port)
    {
        return LPC_GPIO->PIN[static_cast<std::size_t>(port)];
    }

    static uint32_t read(const Name port, const uint32_t mask)
    {
        return LPC_GPIO->PIN[static_cast<std::size_t>(port)] & mask;
    }

    static void write(const Name port, const uint32_t mask, const uint32_t value)
    {
        LPC_GPIO->PIN[static_cast<std::size_t>(port)] = (LPC_GPIO->PIN[static_cast<std::size_t>(port)] & ~mask) | (value & mask);
    }

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static void write_mask(const Name port, const uint32_t mask)
    {
        // NOTE: Zeroes in these registers enable reading and writing.
        //       Ones disable writing and result in zeros in corresponding
        //       positions when reading.
        LPC_GPIO->MASK[static_cast<std::size_t>(port)] = mask;
    }

    static uint32_t read_masked(const Name port)
    {
        return LPC_GPIO->MPIN[static_cast<std::size_t>(port)];
    }

    static void write_masked(const Name port, const uint32_t value)
    {
        LPC_GPIO->MPIN[static_cast<std::size_t>(port)] = value;
    }
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_PORT_HPP

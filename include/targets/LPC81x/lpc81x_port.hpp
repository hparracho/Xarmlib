// ----------------------------------------------------------------------------
// @file    lpc81x_port.hpp
// @brief   NXP LPC81x port class.
// @date    28 February 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
//
// See README.md file for additional credits and acknowledgments.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// ----------------------------------------------------------------------------

#ifndef __XARMLIB_TARGETS_LPC81X_PORT_HPP
#define __XARMLIB_TARGETS_LPC81X_PORT_HPP

#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc81x
{




class PortDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Port names (all packages have 1 port)
        enum class Name
        {
            port0 = 0
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void set_direction(const Name port)
        {
            // NOTE: Ones configure as outputs
            LPC_GPIO->DIR[static_cast<std::size_t>(port)] = 0xFFFFFFFF;
        }

        static void clear_direction(const Name port)
        {
            // NOTE: Zeros configure as inputs
            LPC_GPIO->DIR[static_cast<std::size_t>(port)] = 0;
        }

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

        static void set_direction(const PinDriver::Name pin)
        {
            assert(pin >= PinDriver::Name::nc);

            // NOTE: Ones configure as outputs
            LPC_GPIO->DIR0 |= 1UL << static_cast<uint32_t>(pin);
        }

        static void clear_direction(const PinDriver::Name pin)
        {
            assert(pin >= PinDriver::Name::nc);

            // NOTE: Zeros configure as inputs
            LPC_GPIO->DIR0 &= ~(1UL << static_cast<uint32_t>(pin));
        }

        static void write_direction(const Name port, const uint32_t mask, const uint32_t value)
        {
            LPC_GPIO->DIR[static_cast<std::size_t>(port)] = (LPC_GPIO->DIR[static_cast<std::size_t>(port)] & ~mask) | (value & mask);
        }

        static void set_mask(const Name port)
        {
            // NOTE: Zeroes in these registers enable reading and writing.
            //       Ones disable writing and result in zeros in corresponding
            //       positions when reading.
            LPC_GPIO->MASK[static_cast<std::size_t>(port)] = 0xFFFFFFFF;
        }

        static void clear_mask(const Name port)
        {
            // NOTE: Zeroes in these registers enable reading and writing.
            //       Ones disable writing and result in zeros in corresponding
            //       positions when reading.
            LPC_GPIO->MASK[static_cast<std::size_t>(port)] = 0;
        }

        static void set_mask(const PinDriver::Name pin)
        {
            assert(pin >= PinDriver::Name::nc);

            // NOTE: Zeroes in these registers enable reading and writing.
            //       Ones disable writing and result in zeros in corresponding
            //       positions when reading.
            LPC_GPIO->MASK0 |= 1UL << static_cast<uint32_t>(pin);
        }

        static void clear_mask(const PinDriver::Name pin)
        {
            assert(pin >= PinDriver::Name::nc);

            // NOTE: Zeroes in these registers enable reading and writing.
            //       Ones disable writing and result in zeros in corresponding
            //       positions when reading.
            LPC_GPIO->MASK0 &= ~(1UL << static_cast<uint32_t>(pin));
        }

        static uint32_t read(const Name port)
        {
            return LPC_GPIO->PIN[static_cast<std::size_t>(port)];
        }

        static uint32_t read(const Name port, const uint32_t mask)
        {
            return LPC_GPIO->PIN[static_cast<std::size_t>(port)] & mask;
        }

        static uint32_t read_masked(const Name port)
        {
            return LPC_GPIO->MPIN[static_cast<std::size_t>(port)];
        }

        static void write(const Name port, const uint32_t value)
        {
            LPC_GPIO->PIN[static_cast<std::size_t>(port)] = value;
        }

        static void write(const Name port, const uint32_t mask, const uint32_t value)
        {
            LPC_GPIO->PIN[static_cast<std::size_t>(port)] = (LPC_GPIO->PIN[static_cast<std::size_t>(port)] & ~mask) | (value & mask);
        }

        static void write_masked(const Name port, const uint32_t value)
        {
            LPC_GPIO->MPIN[static_cast<std::size_t>(port)] = value;
        }
};




} // namespace lpc81x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC81X_PORT_HPP

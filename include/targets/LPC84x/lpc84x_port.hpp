// ----------------------------------------------------------------------------
// @file    lpc84x_port.hpp
// @brief   NXP LPC84x port class.
// @date    21 June 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_PORT_HPP
#define __XARMLIB_TARGETS_LPC84X_PORT_HPP

#include "system/target"
#include "targets/LPC84x/lpc84x_cmsis.hpp"
#include "targets/LPC84x/lpc84x_pin.hpp"

namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




// HVQFN48 and LQFP48 packages have 2 ports
// HVQFN33 package only has 1 port




class Port
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Port names according to the target package
        enum class Name
        {
            PORT0 = 0,
#if (__LPC84X_PINS__ > 33)
            PORT1
#endif
        };

        // Number of ports available according to the target package
#if (__LPC84X_PINS__ == 33)
        static const std::size_t COUNT = 1;
#else
        static const std::size_t COUNT = 2;
#endif

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

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

        static void set_mask(const Pin::Name pin)
        {
            assert(pin >= Pin::Name::NC);

            // NOTE: Zeroes in these registers enable reading and writing.
            //       Ones disable writing and result in zeros in corresponding
            //       positions when reading.

            if(static_cast<uint32_t>(pin) < 32)
            {
                LPC_GPIO->MASK0 |= 1UL << static_cast<uint32_t>(pin);
            }
            else
            {
                LPC_GPIO->MASK1 |= 1UL << (static_cast<uint32_t>(pin) - 32);
            }
        }

        static void clear_mask(const Pin::Name pin)
        {
            assert(pin >= Pin::Name::NC);

            // NOTE: Zeroes in these registers enable reading and writing.
            //       Ones disable writing and result in zeros in corresponding
            //       positions when reading.

            if(static_cast<uint32_t>(pin) < 32)
            {
                LPC_GPIO->MASK0 &= ~(1UL << static_cast<uint32_t>(pin));
            }
            else
            {
                LPC_GPIO->MASK1 &= ~(1UL << (static_cast<uint32_t>(pin) - 32));
            }
        }

        static uint32_t read(const Name port)
        {
            return LPC_GPIO->PIN[static_cast<std::size_t>(port)];
        }

        static uint32_t read_masked(const Name port)
        {
            return LPC_GPIO->MPIN[static_cast<std::size_t>(port)];
        }

        static void write(const Name port, const uint32_t value)
        {
            LPC_GPIO->PIN[static_cast<std::size_t>(port)] = value;
        }

        static void write_masked(const Name port, const uint32_t value)
        {
            LPC_GPIO->MPIN[static_cast<std::size_t>(port)] = value;
        }
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_PORT_HPP

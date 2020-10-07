// ----------------------------------------------------------------------------
// @file    hal_port.hpp
// @brief   Port HAL interface class.
// @date    20 September 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_HAL_PORT_HPP
#define __XARMLIB_HAL_PORT_HPP

namespace xarmlib
{
namespace hal
{




template <typename PortDriver>
class PortBase : protected PortDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Name = typename PortDriver::Name;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static void set_direction  (const Name port) { PortDriver::set_direction(port); }
        static void clear_direction(const Name port) { PortDriver::clear_direction(port); }

        static void set_direction  (const Name port, const uint32_t mask) { PortDriver::set_direction(port, mask); }
        static void clear_direction(const Name port, const uint32_t mask) { PortDriver::clear_direction(port, mask); }

        static void set_direction  (const hal::Pin::Name pin) { PortDriver::set_direction(pin); }
        static void clear_direction(const hal::Pin::Name pin) { PortDriver::clear_direction(pin); }

        static void write_direction(const Name port, const uint32_t mask, const uint32_t value) { PortDriver::write_direction(port, mask, value); }

        static uint32_t read(const Name port)                      { return PortDriver::read(port); }
        static uint32_t read(const Name port, const uint32_t mask) { return PortDriver::read(port, mask); }

        static void write(const Name port, const uint32_t value)                      { PortDriver::write(port, value); }
        static void write(const Name port, const uint32_t mask, const uint32_t value) { PortDriver::write(port, mask, value); }

        static void set  (const Name port, const uint32_t mask) { PortDriver::set(port, mask); }
        static void clear(const Name port, const uint32_t mask) { PortDriver::clear(port, mask); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_port.hpp"

namespace xarmlib
{
namespace hal
{

using Port = PortBase<targets::kv5x::PortDriver>;

} // namespace hal

using Port = hal::Port;

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_port.hpp"

namespace xarmlib
{
namespace hal
{

using Port = PortBase<targets::kv4x::PortDriver>;

} // namespace hal

using Port = hal::Port;

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_port.hpp"

namespace xarmlib
{
namespace hal
{

using Port = PortBase<targets::lpc84x::PortDriver>;

} // namespace hal

class Port : public hal::Port
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal = hal::Port;

        static void set_mask(const Name port) { Hal::set_mask(port); }
        static void clear_mask(const Name port) { Hal::clear_mask(port); }

        static void set_mask(const hal::Pin::Name pin) { Hal::set_mask(pin); }
        static void clear_mask(const hal::Pin::Name pin) { Hal::clear_mask(pin); }

        static uint32_t read_masked(const Name port) { return Hal::read_masked(port); }
        static void write_masked(const Name port, const uint32_t value) { Hal::write_masked(port, value); }
};

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_port.hpp"

namespace xarmlib
{
namespace hal
{

using Port = PortBase<targets::lpc81x::PortDriver>;

} // namespace hal

class Port : public hal::Port
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using Hal = hal::Port;

        static void set_mask  (const Name port) { Hal::set_mask(port); }
        static void clear_mask(const Name port) { Hal::clear_mask(port); }

        static void set_mask  (const hal::Pin::Name pin) { Hal::set_mask(pin); }
        static void clear_mask(const hal::Pin::Name pin) { Hal::clear_mask(pin); }

        static uint32_t read_masked (const Name port)                       { return Hal::read_masked(port); }
        static void     write_masked(const Name port, const uint32_t value) { Hal::write_masked(port, value); }
};

} // namespace xarmlib

#elif defined __OTHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Port = PortBase<targets::other_target::PortDriver>;

} // namespace hal

using Port = hal::Port;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_PORT_HPP

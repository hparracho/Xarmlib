// ----------------------------------------------------------------------------
// @file    hal_port.hpp
// @brief   Port HAL interface class.
// @date    29 November 2018
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

#ifndef __XARMLIB_HAL_PORT_HPP
#define __XARMLIB_HAL_PORT_HPP

namespace xarmlib
{
namespace hal
{




template <typename TargetPortDriver>
class PortHal : protected TargetPortDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Name = typename TargetPortDriver::Name;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static inline void set_direction  (const Name port) { TargetPortDriver::set_direction(port); }
        static inline void clear_direction(const Name port) { TargetPortDriver::clear_direction(port); }

        static inline void set_direction  (const Name port, const uint32_t mask) { TargetPortDriver::set_direction(port, mask); }
        static inline void clear_direction(const Name port, const uint32_t mask) { TargetPortDriver::clear_direction(port, mask); }

        static inline void set_direction  (const xarmlib::PinHal::Name pin) { TargetPortDriver::set_direction(pin); }
        static inline void clear_direction(const xarmlib::PinHal::Name pin) { TargetPortDriver::clear_direction(pin); }

        static inline void write_direction(const Name port, const uint32_t mask, const uint32_t value) { TargetPortDriver::write_direction(port, mask, value); }

        static inline uint32_t read(const Name port)                      { return TargetPortDriver::read(port); }
        static inline uint32_t read(const Name port, const uint32_t mask) { return TargetPortDriver::read(port, mask); }

        static inline void write(const Name port, const uint32_t value)                      { TargetPortDriver::write(port, value); }
        static inline void write(const Name port, const uint32_t mask, const uint32_t value) { TargetPortDriver::write(port, mask, value); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_port.hpp"

namespace xarmlib
{
using PortHal = hal::PortHal<targets::kv4x::PortDriver>;
//using Port = PortHal;
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_port.hpp"

namespace xarmlib
{
using PortHal = hal::PortHal<targets::lpc84x::PortDriver>;

//class Port : public PortHal
//{
//    public:
//
//        // --------------------------------------------------------------------
//        // PUBLIC MEMBER FUNCTIONS
//        // --------------------------------------------------------------------
//
//        static inline void set_mask(const Name port) { PortHal::set_mask(port); }
//        static inline void clear_mask(const Name port) { PortHal::clear_mask(port); }
//
//        static inline void set_mask(const PinHal::Name pin) { PortHal::set_mask(pin); }
//        static inline void clear_mask(const PinHal::Name pin) { PortHal::clear_mask(pin); }
//
//        static inline uint32_t read_masked(const Name port) { return PortHal::read_masked(port); }
//        static inline void write_masked(const Name port, const uint32_t value) { PortHal::write_masked(port, value); }
//};
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_port.hpp"

namespace xarmlib
{
using PortHal = hal::PortHal<targets::lpc81x::PortDriver>;

//class Port : public PortHal
//{
//    public:
//
//        // --------------------------------------------------------------------
//        // PUBLIC MEMBER FUNCTIONS
//        // --------------------------------------------------------------------
//
//        static inline void set_mask  (const Name port) { PortHal::set_mask(port); }
//        static inline void clear_mask(const Name port) { PortHal::clear_mask(port); }
//
//        static inline void set_mask  (const PinHal::Name pin) { PortHal::set_mask(pin); }
//        static inline void clear_mask(const PinHal::Name pin) { PortHal::clear_mask(pin); }
//
//        static inline uint32_t read_masked (const Name port)                       { return PortHal::read_masked(port); }
//        static inline void     write_masked(const Name port, const uint32_t value) { PortHal::write_masked(port, value); }
//};
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using PortHal = hal::PortHal<targets::other_target::PortDriver>;
using Port = PortHal;
}

#endif




#endif // __XARMLIB_HAL_PORT_HPP

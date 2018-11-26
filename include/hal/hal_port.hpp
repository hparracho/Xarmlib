// ----------------------------------------------------------------------------
// @file    hal_port.hpp
// @brief   Port HAL interface class.
// @date    26 November 2018
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




template <class TargetPort>
class Port : public TargetPort
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Name = typename TargetPort::Name;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        using TargetPort::set_direction;
        using TargetPort::clear_direction;
        using TargetPort::write_direction;

        using TargetPort::read;

        using TargetPort::write;
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_port.hpp"

namespace xarmlib
{
using Port = hal::Port<targets::kv4x::Port>;
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_port.hpp"

namespace xarmlib
{
using Port = hal::Port<targets::lpc84x::Port>;
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_port.hpp"

namespace xarmlib
{
using Port = hal::Port<targets::lpc81x::Port>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Port = hal::Port<targets::other_target::Port>;
}

#endif




#endif // __XARMLIB_HAL_PORT_HPP

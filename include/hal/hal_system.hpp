// ----------------------------------------------------------------------------
// @file    hal_system.hpp
// @brief   HAL system level configuration class.
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

#ifndef __XARMLIB_HAL_SYSTEM_HPP
#define __XARMLIB_HAL_SYSTEM_HPP

namespace xarmlib
{
namespace hal
{




template <typename TargetSystemDriver>
class SystemHal : protected TargetSystemDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Clock = typename TargetSystemDriver::Clock;
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_system.hpp"

namespace xarmlib
{
using SystemHal = hal::SystemHal<targets::kv4x::SystemDriver>;
//using System = SystemHal;
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_system.hpp"

namespace xarmlib
{
using SystemHal = hal::SystemHal<targets::lpc84x::SystemDriver>;

class System : public SystemHal
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Swd = typename SystemHal::Swd;
};
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_system.hpp"

namespace xarmlib
{
using SystemHal = hal::SystemHal<targets::lpc81x::SystemDriver>;
//using System = SystemHal;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using SystemHal = hal::SystemHal<targets::other_target::SystemDriver>;
using System = SystemHal;
}

#endif




#endif // __XARMLIB_HAL_SYSTEM_HPP

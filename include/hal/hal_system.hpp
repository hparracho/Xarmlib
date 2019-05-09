// ----------------------------------------------------------------------------
// @file    hal_system.hpp
// @brief   HAL system level configuration class.
// @date    9 May 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2019 Helder Parracho (hparracho@gmail.com)
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




template <typename SystemDriver>
class SystemBase : protected SystemDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Clock = typename SystemDriver::Clock;
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_system.hpp"

namespace xarmlib
{
namespace hal
{

using System = SystemBase<targets::kv4x::SystemDriver>;

} // namespace hal

using System = hal::System;

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_system.hpp"

namespace xarmlib
{
namespace hal
{

using System = SystemBase<targets::lpc84x::SystemDriver>;

} // namespace hal

class System : public hal::System
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Hal = hal::Spi;

        using Swd = typename Hal::Swd;
};

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_system.hpp"

namespace xarmlib
{
namespace hal
{

using System = SystemBase<targets::lpc81x::SystemDriver>;

} // namespace hal

using System = hal::System;

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using System = SystemBase<targets::other_target::SystemDriver>;

} // namespace hal

using System = hal::System;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_SYSTEM_HPP

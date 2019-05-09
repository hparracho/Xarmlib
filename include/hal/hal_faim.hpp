// ----------------------------------------------------------------------------
// @file    hal_faim.hpp
// @brief   Fast Initialization Memory (FAIM) HAL interface class.
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

#ifndef __XARMLIB_HAL_FAIM_HPP
#define __XARMLIB_HAL_FAIM_HPP

#include <cstdint>

namespace xarmlib
{
namespace hal
{




template <typename FaimDriver>
class FaimBase : protected FaimDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using PinConfig = typename FaimDriver::PinConfig;

        template<std::size_t Size>
        using PinConfigArray = typename FaimDriver::template PinConfigArray<Size>;
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_faim.hpp"

namespace xarmlib
{
namespace hal
{

using Faim = FaimBase<targets::lpc84x::FaimDriver>;

} // namespace hal

using Faim = hal::Faim;

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Faim = FaimBase<targets::other_target::FaimDriver>;

} // namespace hal

using Faim = hal::Faim;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_FAIM_HPP

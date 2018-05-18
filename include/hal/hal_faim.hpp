// ----------------------------------------------------------------------------
// @file    hal_faim.hpp
// @brief   Fast Initialization Memory (FAIM) HAL interface class.
// @date    18 May 2018
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

#ifndef __XARMLIB_HAL_FAIM_HPP
#define __XARMLIB_HAL_FAIM_HPP

#include <cstdint>

#include "system/target"

namespace xarmlib
{
namespace hal
{




template <class TargetFaim>
class Faim
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        using PinConfig = typename TargetFaim::PinConfig;

        template<std::size_t SIZE>
        using PinConfigArray = typename TargetFaim::template PinConfigArray<SIZE>;
};




} // namespace hal
} // namespace xarmlib




#if defined __LPC84X__

#include "targets/LPC84x/lpc84x_faim.hpp"

namespace xarmlib
{
using Faim = hal::Faim<targets::lpc84x::Faim>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Faim = hal::Faim<targets::other_target::Faim>;
}

#endif




#endif // __XARMLIB_HAL_FAIM_HPP

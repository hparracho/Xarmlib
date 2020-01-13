// ----------------------------------------------------------------------------
// @file    hal_pin.hpp
// @brief   Pin HAL interface class.
// @date    13 January 2020
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

#ifndef __XARMLIB_HAL_PIN_HPP
#define __XARMLIB_HAL_PIN_HPP

namespace xarmlib
{
namespace hal
{




template <typename PinDriver>
class PinBase : protected PinDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Name = typename PinDriver::Name;
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_pin.hpp"

namespace xarmlib
{
namespace hal
{

using Pin = PinBase<targets::kv5x::PinDriver>;

} // namespace hal

using Pin = hal::Pin;

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_pin.hpp"

namespace xarmlib
{
namespace hal
{

using Pin = PinBase<targets::kv4x::PinDriver>;

} // namespace hal

using Pin = hal::Pin;

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_pin.hpp"

namespace xarmlib
{
namespace hal
{

using Pin = PinBase<targets::lpc84x::PinDriver>;

} // namespace hal

using Pin = hal::Pin;

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_pin.hpp"

namespace xarmlib
{
namespace hal
{

using Pin = PinBase<targets::lpc81x::PinDriver>;

} // namespace hal

using Pin = hal::Pin;

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Pin = PinBase<targets::other_target::PinDriver>;

} // namespace hal

using Pin = hal::Pin;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_PIN_HPP

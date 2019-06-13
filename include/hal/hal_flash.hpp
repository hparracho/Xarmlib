// ----------------------------------------------------------------------------
// @file    hal_flash.hpp
// @brief   Flash HAL interface class.
// @date    21 May 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_HAL_FLASH_HPP
#define __XARMLIB_HAL_FLASH_HPP

#include "external/span.hpp"

namespace xarmlib
{
namespace hal
{




template <typename FlashIapDriver, typename FlashBootDriver>
class FlashBase : protected FlashIapDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write to flash
        // @flash_address: Destination flash address where data is to be written.
        // @ buffer:       Buffer span containing the data to be written.
        // NOTE:           See target specific notes
        static bool write_flash(const int32_t flash_address, const std::span<const uint8_t> buffer) { return FlashIapDriver::write_flash(flash_address, buffer); }

        // NOTE: the application should destruct all objects instantiated by itself before
        static void boot_application(const int32_t flash_address) { FlashBootDriver::boot_application(flash_address); }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_flash_iap.hpp"
#include "targets/KV4x/kv4x_flash_boot.hpp"

namespace xarmlib
{
namespace hal
{

using Flash = FlashBase<targets::kv4x::FlashIapDriver, targets::kv4x::FlashBootDriver>;

} // namespace hal

using Flash = hal::Flash;

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_flash_iap.hpp"
#include "targets/LPC84x/lpc84x_flash_boot.hpp"

namespace xarmlib
{
namespace hal
{

using Flash = FlashBase<targets::lpc84x::FlashIapDriver, targets::lpc84x::FlashBootDriver>;

} // namespace hal

class Flash : public hal::Flash
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write a flash page
        // @flash_page: Destination flash page where data is to be written.
        // @ buffer:    Buffer span containing the data to be written.
        // NOTE:        Buffer size must be 64 bytes (page size).
        static bool write_flash_page(const int32_t flash_page, const std::span<const uint8_t> buffer) { return hal::Flash::write_flash_page(flash_page, buffer); }
};

} // namespace xarmlib

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_flash_iap.hpp"
#include "targets/LPC81x/lpc81x_flash_boot.hpp"

namespace xarmlib
{
namespace hal
{

using Flash = FlashBase<targets::lpc81x::FlashIapDriver, targets::lpc81x::FlashBootDriver>;

} // namespace hal

class Flash : public hal::Flash
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Write a flash page
        // @flash_page: Destination flash page where data is to be written.
        // @ buffer:    Buffer span containing the data to be written.
        // NOTE:        Buffer size must be 64 bytes (page size).
        static bool write_flash_page(const int32_t flash_page, const std::span<const uint8_t> buffer) { return hal::Flash::write_flash_page(flash_page, buffer); }
};

} // namespace xarmlib

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using Flash = FlashBase<targets::other_target::FlashIapDriver, targets::other_target::FlashBootDriver>;

} // namespace hal

using Flash = hal::Flash;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_FLASH_HPP

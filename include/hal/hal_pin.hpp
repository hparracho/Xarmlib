// ----------------------------------------------------------------------------
// @file    hal_pin.hpp
// @brief   Pin HAL interface class.
// @date    20 November 2018
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

#ifndef __XARMLIB_HAL_PIN_HPP
#define __XARMLIB_HAL_PIN_HPP

namespace xarmlib
{
namespace hal
{




template <class TargetPin>
class Pin
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using Name         = typename TargetPin::Name;
        using FunctionMode = typename TargetPin::FunctionMode;

        //TMP
        using SlewRate        = typename TargetPin::SlewRate;
        using PassiveFilter   = typename TargetPin::PassiveFilter;
        using OpenDrain       = typename TargetPin::OpenDrain;
        using DriveStrength   = typename TargetPin::DriveStrength;
        using PinMuxControl   = typename TargetPin::PinMuxControl;
        using LockRegister    = typename TargetPin::LockRegister;

        static void set_mode(const Name pin_name, const FunctionMode    function_mode,
                                                  const PinMuxControl   pin_mux_control,
                                                  const SlewRate        slew_rate      = SlewRate::kPORT_FastSlewRate,
                                                  const PassiveFilter   passive_filter = PassiveFilter::kPORT_PassiveFilterDisable,
                                                  const OpenDrain       open_drain     = OpenDrain::kPORT_OpenDrainDisable,
                                                  const DriveStrength   drive_strength = DriveStrength::kPORT_LowDriveStrength,
                                                  const LockRegister    lock_register  = LockRegister::kPORT_UnlockRegister)
        {
            TargetPin::set_mode(pin_name, function_mode, pin_mux_control, slew_rate, passive_filter, open_drain, drive_strength, lock_register);
        }

        static void set_mode(const Name pin_name, const PinMuxControl   pin_mux_control = PinMuxControl::kPORT_MuxAlt7, // I2C0 configuration
                                                  const SlewRate        slew_rate       = SlewRate::kPORT_FastSlewRate,
                                                  const PassiveFilter   passive_filter  = PassiveFilter::kPORT_PassiveFilterDisable,
                                                  const DriveStrength   drive_strength  = DriveStrength::kPORT_LowDriveStrength,
                                                  const LockRegister    lock_register   = LockRegister::kPORT_UnlockRegister)
        {
            TargetPin::set_mode(pin_name, pin_mux_control, slew_rate, passive_filter, drive_strength, lock_register);
        }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_pin.hpp"

namespace xarmlib
{
using Pin = hal::Pin<targets::kv4x::Pin>;
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_pin.hpp"

namespace xarmlib
{
using Pin = hal::Pin<targets::lpc84x::Pin>;
}

#elif defined __LPC81X__

#include "targets/LPC81x/lpc81x_pin.hpp"

namespace xarmlib
{
using Pin = hal::Pin<targets::lpc81x::Pin>;
}

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using Pin = hal::Pin<targets::other_target::Pin>;
}

#endif




#endif // __XARMLIB_HAL_PIN_HPP

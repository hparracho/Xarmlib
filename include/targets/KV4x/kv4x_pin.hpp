// ----------------------------------------------------------------------------
// @file    kv4x_pin.hpp
// @brief   Kinetis KV4x pin class.
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

#ifndef __XARMLIB_TARGETS_KV4X_PIN_HPP
#define __XARMLIB_TARGETS_KV4X_PIN_HPP

#include "core/target_specs.hpp"
#include "fsl_port.h"

namespace xarmlib
{
namespace targets
{
namespace kv4x
{




class Pin
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Pin names according to the target package
        enum class Name
        {
            // The following pins are present in all packages
            PA_0  = 0,
            PA_1  = 1,
            PA_2  = 2,
            PA_3  = 3,
            PA_4  = 4,
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            PA_5  = 5,
            PA_12 = 12,
            PA_13 = 13,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            PA_14 = 14,
            PA_15 = 15,
            PA_16 = 16,
            PA_17 = 17,
#endif
            // The following pins are present in all packages
            PA_18 = 18,
            PA_19 = 19,

            PB_0  = 32,
            PB_1  = 33,
            PB_2  = 34,
            PB_3  = 35,
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            PB_9  = 41,
            PB_10 = 42,
            PB_11 = 43,
#endif
            // The following pins are present in all packages
            PB_16 = 48,
            PB_17 = 49,
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            PB_18 = 50,
            PB_19 = 51,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            PB_20 = 52,
            PB_21 = 53,
            PB_22 = 54,
            PB_23 = 55,
#endif

            // The following pins are present in all packages
            PC_0  = 64,
            PC_1  = 65,
            PC_2  = 66,
            PC_3  = 67,
            PC_4  = 68,
            PC_5  = 69,
            PC_6  = 70,
            PC_7  = 71,
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            PC_8  = 72,
            PC_9  = 73,
            PC_10 = 74,
            PC_11 = 75,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            PC_12 = 76,
            PC_13 = 77,
            PC_14 = 78,
            PC_15 = 79,
            PC_16 = 80,
            PC_17 = 81,
            PC_18 = 82,
#endif

            // The following pins are present in all packages
            PD_0  = 96,
            PD_1  = 97,
            PD_2  = 98,
            PD_3  = 99,
            PD_4  = 100,
            PD_5  = 101,
            PD_6  = 102,
            PD_7  = 103,

#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            PE_0  = 128,
            PE_1  = 129,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            PE_2  = 130,
            PE_3  = 131,
            PE_4  = 132,
            PE_5  = 133,
            PE_6  = 134,
#endif
            // The following pins are present in all packages
            PE_16 = 144,
            PE_17 = 145,
            PE_18 = 146,
            PE_19 = 147,
#if (TARGET_PACKAGE_PIN_COUNT == 48 || TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP48 / LQFP100 packages
            PE_20 = 148,
            PE_21 = 149,
#endif
            // The following pins are present in all packages
            PE_24 = 152,
            PE_25 = 153,
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pin is only present in
            // LQFP100 package
            PE_26 = 154,
#endif
            // The following pins are present in all packages
            PE_29 = 157,
            PE_30 = 158,

            // Not connected
            NC
        };

        using FunctionMode    = _port_pull;
        using SlewRate        = _port_slew_rate;
        using PassiveFilter   = _port_passive_filter_enable;
        using OpenDrain       = _port_open_drain_enable;
        using DriveStrength   = _port_drive_strength;
        using PinMuxControl   = _port_mux;
        using LockRegister    = _port_lock_register;
        //using InterruptConfig = _port_interrupt;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Set mode of normal pins
        static void set_mode(const Name pin_name, const FunctionMode    function_mode,
                                                  const PinMuxControl   pin_mux_control,
                                                  const SlewRate        slew_rate      = SlewRate::kPORT_FastSlewRate,
                                                  const PassiveFilter   passive_filter = PassiveFilter::kPORT_PassiveFilterDisable,
                                                  const OpenDrain       open_drain     = OpenDrain::kPORT_OpenDrainDisable,
                                                  const DriveStrength   drive_strength = DriveStrength::kPORT_LowDriveStrength,
                                                  const LockRegister    lock_register  = LockRegister::kPORT_UnlockRegister)
        {
#ifdef DEBUG
            // Exclude NC
            assert(pin_name != Name::NC);

            // Exclude true open-drain pins
            assert(pin_name != Name::PC_6 && pin_name != Name::PC_7);

            if(passive_filter == PassiveFilter::kPORT_PassiveFilterEnable)
            {
                // Only PA_4 has passive filter enable control
                assert(pin_name == Name::PA_4);
            }

            if(drive_strength == DriveStrength::kPORT_HighDriveStrength)
            {
                // Only PB_0, PB_1, PC_3, PC_4, PD_4, PD_5, PD_6 and PD_7 have drive strength enable control
                assert(pin_name == Name::PB_0
                    || pin_name == Name::PB_1
                    || pin_name == Name::PC_3
                    || pin_name == Name::PC_4
                    || pin_name == Name::PD_4
                    || pin_name == Name::PD_5
                    || pin_name == Name::PD_6
                    || pin_name == Name::PD_7);
            }
#endif

            const port_pin_config_t pin_config =
            {
                function_mode,
                slew_rate,
                passive_filter,
                open_drain,
                drive_strength,
                pin_mux_control,
                lock_register
            };

            const uint32_t port_index = get_port_index(pin_name);
            const uint32_t pin_bit    = get_pin_bit(pin_name);

            switch(port_index)
            {
                case 0: CLOCK_EnableClock(kCLOCK_PortA); PORT_SetPinConfig(PORTA, pin_bit, &pin_config); break;
                case 1: CLOCK_EnableClock(kCLOCK_PortB); PORT_SetPinConfig(PORTB, pin_bit, &pin_config); break;
                case 2: CLOCK_EnableClock(kCLOCK_PortC); PORT_SetPinConfig(PORTC, pin_bit, &pin_config); break;
                case 3: CLOCK_EnableClock(kCLOCK_PortD); PORT_SetPinConfig(PORTD, pin_bit, &pin_config); break;
                case 4: CLOCK_EnableClock(kCLOCK_PortE); PORT_SetPinConfig(PORTE, pin_bit, &pin_config); break;
            }
        }

        // Set mode of true open-drain pins (only available on PC_6 and PC_7)
        static void set_mode(const Name pin_name, const PinMuxControl   pin_mux_control,
                                                  const SlewRate        slew_rate      = SlewRate::kPORT_FastSlewRate,
                                                  const PassiveFilter   passive_filter = PassiveFilter::kPORT_PassiveFilterDisable,
                                                  const DriveStrength   drive_strength = DriveStrength::kPORT_LowDriveStrength,
                                                  const LockRegister    lock_register  = LockRegister::kPORT_UnlockRegister)
        {
            // Available only on true open-drain pins
            assert(pin_name == Name::PC_6 || pin_name == Name::PC_7);

            const port_pin_config_t pin_config =
            {
                FunctionMode::kPORT_PullDisable,
                slew_rate,
                passive_filter,
                OpenDrain::kPORT_OpenDrainDisable,
                drive_strength,
                pin_mux_control,
                lock_register
            };

            CLOCK_EnableClock(kCLOCK_PortC);
            PORT_SetPinConfig(PORTC, get_pin_bit(pin_name), &pin_config);
        }

        static constexpr uint32_t get_port_index(const Name pin_name)
        {
            return (static_cast<uint32_t>(pin_name) >> 5);      // (pin_name / 32)
        }

        static constexpr uint32_t get_pin_bit(const Name pin_name)
        {
            return (static_cast<uint32_t>(pin_name) & 0x1F);    // (pin_name % 32)
        }
};




} // namespace kv4x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV4X_PIN_HPP

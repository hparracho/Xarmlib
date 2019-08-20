// ----------------------------------------------------------------------------
// @file    kv4x_pin.hpp
// @brief   Kinetis KV4x pin class.
// @date    12 December 2018
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




class PinDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        // Pin names according to the target package
        enum class Name
        {
            // The following pins are present in all packages
            pa_0  = 0,
            pa_1  = 1,
            pa_2  = 2,
            pa_3  = 3,
            pa_4  = 4,
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            pa_5  = 5,
            pa_12 = 12,
            pa_13 = 13,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            pa_14 = 14,
            pa_15 = 15,
            pa_16 = 16,
            pa_17 = 17,
#endif
            // The following pins are present in all packages
            pa_18 = 18,
            pa_19 = 19,

            pb_0  = 32,
            pb_1  = 33,
            pb_2  = 34,
            pb_3  = 35,
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            pb_9  = 41,
            pb_10 = 42,
            pb_11 = 43,
#endif
            // The following pins are present in all packages
            pb_16 = 48,
            pb_17 = 49,
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            pb_18 = 50,
            pb_19 = 51,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            pb_20 = 52,
            pb_21 = 53,
            pb_22 = 54,
            pb_23 = 55,
#endif

            // The following pins are present in all packages
            pc_0  = 64,
            pc_1  = 65,
            pc_2  = 66,
            pc_3  = 67,
            pc_4  = 68,
            pc_5  = 69,
            pc_6  = 70,
            pc_7  = 71,
#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            pc_8  = 72,
            pc_9  = 73,
            pc_10 = 74,
            pc_11 = 75,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            pc_12 = 76,
            pc_13 = 77,
            pc_14 = 78,
            pc_15 = 79,
            pc_16 = 80,
            pc_17 = 81,
            pc_18 = 82,
#endif

            // The following pins are present in all packages
            pd_0  = 96,
            pd_1  = 97,
            pd_2  = 98,
            pd_3  = 99,
            pd_4  = 100,
            pd_5  = 101,
            pd_6  = 102,
            pd_7  = 103,

#if (TARGET_PACKAGE_PIN_COUNT >= 64)
            // The following pins are only present in
            // LQFP64 / LQFP100 packages
            pe_0  = 128,
            pe_1  = 129,
#endif
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP100 package
            pe_2  = 130,
            pe_3  = 131,
            pe_4  = 132,
            pe_5  = 133,
            pe_6  = 134,
#endif
            // The following pins are present in all packages
            pe_16 = 144,
            pe_17 = 145,
            pe_18 = 146,
            pe_19 = 147,
#if (TARGET_PACKAGE_PIN_COUNT == 48 || TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pins are only present in
            // LQFP48 / LQFP100 packages
            pe_20 = 148,
            pe_21 = 149,
#endif
            // The following pins are present in all packages
            pe_24 = 152,
            pe_25 = 153,
#if (TARGET_PACKAGE_PIN_COUNT == 100)
            // The following pin is only present in
            // LQFP100 package
            pe_26 = 154,
#endif
            // The following pins are present in all packages
            pe_29 = 157,
            pe_30 = 158,

            // Not connected
            nc
        };

        // Pull select and enable
        enum class FunctionMode
        {
            hiz       = 0,
            pull_down = 2,
            pull_up
        };

        // Slew rate enable
        enum class SlewRate
        {
            fast = 0,
            slow
        };

        // Passive filter enable
        enum class PassiveFilter
        {
            disable = 0,
            enable
        };

        // Open drain enable
        enum class OpenDrain
        {
            disable = 0,
            enable
        };

        // Drive strength enable
        enum class DriveStrength
        {
            low = 0,
            high
        };

        // Pin mux control
        enum class PinMux
        {
            pin_disabled_or_analog = 0,
            gpio,
            // Chip-specific
            alt2,
            alt3,
            alt4,
            alt5,
            alt6,
            alt7
        };

        // Lock register (Pin Control Register (PCR) fields [15:0])
        enum class LockRegister
        {
            unlock = 0,
            lock
        };

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // Set mode of normal pins
        static void set_mode(const Name pin_name, const FunctionMode  function_mode,
                                                  const PinMux        pin_mux        = PinMux::gpio,
                                                  const SlewRate      slew_rate      = SlewRate::fast,
                                                  const PassiveFilter passive_filter = PassiveFilter::disable,
                                                  const OpenDrain     open_drain     = OpenDrain::disable,
                                                  const DriveStrength drive_strength = DriveStrength::low,
                                                  const LockRegister  lock_register  = LockRegister::unlock)
        {
            // Exclude NC
            assert(pin_name != Name::nc);

            // Exclude true open-drain pins
            assert(pin_name != Name::pc_6 && pin_name != Name::pc_7);

            // Only PA_4 has passive filter enable control
            assert(passive_filter == PassiveFilter::disable || (passive_filter == PassiveFilter::enable && pin_name == Name::pa_4));

            // Only PB_0, PB_1, PC_3, PC_4, PD_4, PD_5, PD_6 and PD_7 have drive strength enable control
            assert(drive_strength == DriveStrength::low || (drive_strength == DriveStrength::high && (pin_name == Name::pb_0
                                                                                                   || pin_name == Name::pb_1
                                                                                                   || pin_name == Name::pc_3
                                                                                                   || pin_name == Name::pc_4
                                                                                                   || pin_name == Name::pd_4
                                                                                                   || pin_name == Name::pd_5
                                                                                                   || pin_name == Name::pd_6
                                                                                                   || pin_name == Name::pd_7)));

            const port_pin_config_t pin_config =
            {
                static_cast<uint16_t>(function_mode),
                static_cast<uint16_t>(slew_rate),
                static_cast<uint16_t>(passive_filter),
                static_cast<uint16_t>(open_drain),
                static_cast<uint16_t>(drive_strength),
                static_cast<uint16_t>(pin_mux),
                static_cast<uint16_t>(lock_register)
            };

            const uint32_t port_index = get_port_index(pin_name);
            const uint32_t pin_bit    = get_pin_bit(pin_name);

            switch(port_index)
            {
                case 0: CLOCK_EnableClock(kCLOCK_PortA); assert((PORTA->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinConfig(PORTA, pin_bit, &pin_config); break;
                case 1: CLOCK_EnableClock(kCLOCK_PortB); assert((PORTB->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinConfig(PORTB, pin_bit, &pin_config); break;
                case 2: CLOCK_EnableClock(kCLOCK_PortC); assert((PORTC->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinConfig(PORTC, pin_bit, &pin_config); break;
                case 3: CLOCK_EnableClock(kCLOCK_PortD); assert((PORTD->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinConfig(PORTD, pin_bit, &pin_config); break;
                case 4: CLOCK_EnableClock(kCLOCK_PortE); assert((PORTE->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinConfig(PORTE, pin_bit, &pin_config); break;
            }
        }

        // Set mode of true open-drain pins (only available on PC_6 and PC_7)
        static void set_mode(const Name pin_name, const PinMux        pin_mux        = PinMux::gpio,
                                                  const SlewRate      slew_rate      = SlewRate::fast,
                                                  const PassiveFilter passive_filter = PassiveFilter::disable,
                                                  const DriveStrength drive_strength = DriveStrength::low,
                                                  const LockRegister  lock_register  = LockRegister::unlock)
        {
            // Available only on true open-drain pins
            assert(pin_name == Name::pc_6 || pin_name == Name::pc_7);

            const port_pin_config_t pin_config =
            {
                static_cast<uint16_t>(FunctionMode::hiz),
                static_cast<uint16_t>(slew_rate),
                static_cast<uint16_t>(passive_filter),
                static_cast<uint16_t>(OpenDrain::disable),
                static_cast<uint16_t>(drive_strength),
                static_cast<uint16_t>(pin_mux),
                static_cast<uint16_t>(lock_register)
            };

            CLOCK_EnableClock(kCLOCK_PortC);

            const uint32_t pin_bit = get_pin_bit(pin_name);

            assert((PORTC->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0);

            PORT_SetPinConfig(PORTC, pin_bit, &pin_config);
        }

        static void set_pin_mux(const Name pin_name, const PinMux pin_mux)
        {
            // Exclude NC
            assert(pin_name != Name::nc);

            const uint32_t port_index = get_port_index(pin_name);
            const uint32_t pin_bit    = get_pin_bit(pin_name);

            switch(port_index)
            {
                case 0: CLOCK_EnableClock(kCLOCK_PortA); assert((PORTA->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinMux(PORTA, pin_bit, static_cast<port_mux_t>(pin_mux)); break;
                case 1: CLOCK_EnableClock(kCLOCK_PortB); assert((PORTB->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinMux(PORTB, pin_bit, static_cast<port_mux_t>(pin_mux)); break;
                case 2: CLOCK_EnableClock(kCLOCK_PortC); assert((PORTC->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinMux(PORTC, pin_bit, static_cast<port_mux_t>(pin_mux)); break;
                case 3: CLOCK_EnableClock(kCLOCK_PortD); assert((PORTD->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinMux(PORTD, pin_bit, static_cast<port_mux_t>(pin_mux)); break;
                case 4: CLOCK_EnableClock(kCLOCK_PortE); assert((PORTE->PCR[pin_bit] & PORT_PCR_LK_MASK) == 0); PORT_SetPinMux(PORTE, pin_bit, static_cast<port_mux_t>(pin_mux)); break;
            }
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

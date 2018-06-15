// ----------------------------------------------------------------------------
// @file    api_digital_in_bus.hpp
// @brief   API digital input bus class.
// @date    15 June 2018
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

#ifndef __XARMLIB_API_DIGITAL_IN_BUS_HPP
#define __XARMLIB_API_DIGITAL_IN_BUS_HPP

#include "hal/hal_gpio.hpp"

namespace xarmlib
{




template <std::size_t BusWidth>
class DigitalInBus
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        DigitalInBus(const Pin::Name           (&pin_names)[BusWidth],
                     const Gpio::InputMode       input_mode,
                     const Gpio::InputFilter     input_filter     = Gpio::InputFilter::BYPASS,
                     const Gpio::InputInvert     input_invert     = Gpio::InputInvert::NORMAL,
                     const Gpio::InputHysteresis input_hysteresis = Gpio::InputHysteresis::ENABLE)
            : m_bus  { make_array(pin_names,
                                  input_mode,
                                  input_filter,
                                  input_invert,
                                  input_hysteresis,
                                  std::make_index_sequence<BusWidth>()) }
        {}

        DigitalInBus(const Pin::Name                  (&pin_names)[BusWidth],
                     const Gpio::InputModeTrueOpenDrain input_mode,
                     const Gpio::InputFilter            input_filter = Gpio::InputFilter::BYPASS,
                     const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
            : m_bus  { make_array(pin_names,
                                  input_mode,
                                  input_filter,
                                  input_invert,
                                  std::make_index_sequence<BusWidth>()) }
        {}

        // -------- READ ------------------------------------------------------

        uint32_t read() const
        {
            uint32_t value = 0;

            for(std::size_t pin = 0; pin < BusWidth; pin++)
            {
                value |= m_bus[pin].read() << pin;
            }

            return value;
        }

        operator uint32_t () const
        {
            return read();
        }

        // Read negated value operator
        uint32_t operator ! () const
        {
            return !read();
        }

        // -------- MASK ------------------------------------------------------

        constexpr uint32_t get_mask() const
        {
            return static_cast<uint32_t>((1ULL << BusWidth) - 1);
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template<std::size_t ...Pin>
        static constexpr std::array<Gpio, BusWidth> make_array(const Pin::Name           (&pin_names)[BusWidth],
                                                               const Gpio::InputMode       input_mode,
                                                               const Gpio::InputFilter     input_filter,
                                                               const Gpio::InputInvert     input_invert,
                                                               const Gpio::InputHysteresis input_hysteresis,
                                                               std::index_sequence<Pin...>)
        {
            static_assert(BusWidth > 0 && BusWidth <= 32, "Invalid bus width.");

            return { Gpio(pin_names[Pin], input_mode, input_filter, input_invert, input_hysteresis) ... };
        }

        template<std::size_t ...Pin>
        static constexpr std::array<Gpio, BusWidth> make_array(const Pin::Name                  (&pin_names)[BusWidth],
                                                               const Gpio::InputModeTrueOpenDrain input_mode,
                                                               const Gpio::InputFilter            input_filter,
                                                               const Gpio::InputInvert            input_invert,
                                                               std::index_sequence<Pin...>)
        {
            static_assert(BusWidth > 0 && BusWidth <= 32, "Invalid bus width.");

            return { Gpio(pin_names[Pin], input_mode, input_filter, input_invert) ... };
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::array<Gpio, BusWidth> m_bus;
};




} // namespace xarmlib

#endif // __XARMLIB_API_DIGITAL_IN_BUS_HPP

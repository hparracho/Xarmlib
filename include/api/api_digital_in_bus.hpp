// ----------------------------------------------------------------------------
// @file    api_digital_in_bus.hpp
// @brief   API digital input bus class.
// @date    18 June 2018
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

#include <type_traits>

#include "hal/hal_gpio.hpp"
#include "api/api_pin_bus.hpp"

namespace xarmlib
{




template <Pin::Name... pins>
class DigitalInBus
{
        using Type = typename std::conditional<sizeof...(pins) <= 32, uint32_t, uint64_t>::type;

    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        //template <Pin::Name... pins>
        constexpr DigitalInBus(const PinBus<pins...>&      pin_bus,
                               const Gpio::InputMode       input_mode,
                               const Gpio::InputFilter     input_filter     = Gpio::InputFilter::BYPASS,
                               const Gpio::InputInvert     input_invert     = Gpio::InputInvert::NORMAL,
                               const Gpio::InputHysteresis input_hysteresis = Gpio::InputHysteresis::ENABLE)
            : m_bus { make_array(pin_bus,
                                 input_mode,
                                 input_filter,
                                 input_invert,
                                 input_hysteresis,
                                 std::make_index_sequence<sizeof...(pins)>()) }
        {}

        constexpr DigitalInBus(const PinBus<pins...>&             pin_bus,
                               const Gpio::InputModeTrueOpenDrain input_mode,
                               const Gpio::InputFilter            input_filter = Gpio::InputFilter::BYPASS,
                               const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL)
            : m_bus { make_array(pin_bus,
                                 input_mode,
                                 input_filter,
                                 input_invert,
                                 std::make_index_sequence<sizeof...(pins)>()) }
        {}

        // -------- READ ------------------------------------------------------

        Type read() const
        {
            Type value = 0;

            for(std::size_t pin = 0; pin < get_width(); pin++)
            {
                value |= static_cast<Type>(m_bus[pin].read()) << pin;
            }

            return value;
        }

        operator Type () const
        {
            return read();
        }

        // Read negated value operator
        Type operator ! () const
        {
            return !read();
        }

        // -------- BUS WIDTH / MASK ------------------------------------------------------

        constexpr std::size_t get_width() const
        {
            return sizeof...(pins);
        }

        constexpr Type get_mask() const
        {
            Type mask = 0;

            for(std::size_t bit = 0; bit < get_width(); bit++)
            {
                mask |= static_cast<Type>(1) << bit;
            }

            return mask;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template<std::size_t... index>
        static constexpr std::array<Gpio, sizeof...(pins)> make_array(const PinBus<pins...>&      pin_bus,
                                                                      const Gpio::InputMode       input_mode,
                                                                      const Gpio::InputFilter     input_filter,
                                                                      const Gpio::InputInvert     input_invert,
                                                                      const Gpio::InputHysteresis input_hysteresis,
                                                                      std::index_sequence<index...>)
        {
            return { Gpio(pin_bus.get_pin_name(index), input_mode, input_filter, input_invert, input_hysteresis)... };
        }

        template<std::size_t... index>
        static constexpr std::array<Gpio, sizeof...(pins)> make_array(const PinBus<pins...>&             pin_bus,
                                                                      const Gpio::InputModeTrueOpenDrain input_mode,
                                                                      const Gpio::InputFilter            input_filter,
                                                                      const Gpio::InputInvert            input_invert,
                                                                      std::index_sequence<index...>)
        {
            return { Gpio(pin_bus.get_pin_name(index), input_mode, input_filter, input_invert)... };
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::array<Gpio, sizeof...(pins)> m_bus;
};




} // namespace xarmlib

#endif // __XARMLIB_API_DIGITAL_IN_BUS_HPP

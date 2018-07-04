// ----------------------------------------------------------------------------
// @file    api_digital_in_bus.hpp
// @brief   API digital input bus class.
// @date    3 July 2018
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

#include "api/api_pin_bus.hpp"
#include "hal/hal_gpio.hpp"
#include "system/dynarray"
#include "system/non_copyable"

#include <memory>

namespace xarmlib
{




class DigitalInBus : private NonCopyable<DigitalInBus>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        //template <Pin::Name... pins>
        DigitalInBus(const PinBus&               pin_bus,
                     const Gpio::InputMode       input_mode,
                     const Gpio::InputFilter     input_filter     = Gpio::InputFilter::BYPASS,
                     const Gpio::InputInvert     input_invert     = Gpio::InputInvert::NORMAL,
                     const Gpio::InputHysteresis input_hysteresis = Gpio::InputHysteresis::ENABLE) : m_bus(pin_bus.size())
        {
            assert(pin_bus.size() <= 32);

            std::size_t index = 0;
            for(auto pin : pin_bus)
            {
                m_bus[index++] = std::make_unique<Gpio>(pin, input_mode, input_filter, input_invert, input_hysteresis);
            }
        }

        DigitalInBus(const PinBus&                      pin_bus,
                     const Gpio::InputModeTrueOpenDrain input_mode,
                     const Gpio::InputFilter            input_filter = Gpio::InputFilter::BYPASS,
                     const Gpio::InputInvert            input_invert = Gpio::InputInvert::NORMAL) : m_bus(pin_bus.size())
        {
            assert(pin_bus.size() <= 32);

            std::size_t index = 0;
            for(auto pin : pin_bus)
            {
                m_bus[index++] = std::make_unique<Gpio>(pin, input_mode, input_filter, input_invert);
            }
        }

        // -------- READ ------------------------------------------------------

        uint32_t read() const
        {
            uint32_t value = 0;

            for(std::size_t pin = 0; pin < get_width(); ++pin)
            {
                value |= m_bus[pin]->read() << pin;
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

        // -------- BUS WIDTH / MASK ------------------------------------------------------

        std::size_t get_width() const
        {
            return m_bus.size();
        }

        uint32_t get_mask() const
        {
            return static_cast<uint32_t>((1UL << get_width()) - 1);
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        dynarray<std::unique_ptr<Gpio>> m_bus;
};




#if 0 // DEPRECATED 20180703
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
#endif // DEPRECATED 20180703




} // namespace xarmlib

#endif // __XARMLIB_API_DIGITAL_IN_BUS_HPP

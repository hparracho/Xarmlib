// ----------------------------------------------------------------------------
// @file    api_digital_in_bus.hpp
// @brief   API digital input bus class.
// @date    14 January 2020
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

#ifndef __XARMLIB_API_DIGITAL_IN_BUS_HPP
#define __XARMLIB_API_DIGITAL_IN_BUS_HPP

#include "api/api_pin_bus.hpp"
#include "hal/hal_gpio.hpp"
#include "core/non_copyable.hpp"

#include <dynarray>
#include <memory>

namespace xarmlib
{




#if 1 // HP BRANCH
template <std::size_t Size>
class DigitalInBus : private NonCopyable<DigitalInBus<Size>>
{
    using PinNameBus = PinBus<PinNameTraits, Size>;

    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        constexpr DigitalInBus(const PinNameBus& pin_bus, const hal::Gpio::InputModeConfig& config)
        {
            for(std::size_t index = 0; index < Size; ++index)
            {
                m_bus[index] = std::make_unique<hal::Gpio>(pin_bus[index], config);
            }
        }

#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        DigitalInBus(const PinNameBus& pin_bus, const hal::Gpio::InputModeTrueOpenDrainConfig& config)
        {
            for(std::size_t index = 0; index < Size; ++index)
            {
                m_bus[index] = std::make_unique<hal::Gpio>(pin_bus[index], config);
            }
        }
#endif

        // -------- READ ------------------------------------------------------

        uint32_t read() const
        {
            uint32_t value = 0;

            for(std::size_t pin = 0; pin < m_bus.size(); ++pin)
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

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::array<std::unique_ptr<hal::Gpio>, Size> m_bus;
};
#endif // HP_BRANCH




#if 0 // DEPRECATED 20200912
class DigitalInBus : private NonCopyable<DigitalInBus>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        DigitalInBus(const PinNameBus& pin_name_bus, const hal::Gpio::InputModeConfig& config) : m_bus(pin_name_bus.get_size())
        {
            std::size_t index = 0;
            for(auto pin_name : pin_name_bus)
            {
                m_bus[index++] = std::make_unique<hal::Gpio>(pin_name, config);
            }
        }

#if defined(TARGET_PORT_HAS_TRUE_OPEN_DRAIN) && TARGET_PORT_HAS_TRUE_OPEN_DRAIN
        DigitalInBus(const PinNameBus& pin_name_bus, const hal::Gpio::InputModeTrueOpenDrainConfig& config) : m_bus(pin_name_bus.get_size())
        {
            std::size_t index = 0;
            for(auto pin_name : pin_name_bus)
            {
                m_bus[index++] = std::make_unique<hal::Gpio>(pin_name, config);
            }
        }
#endif

        // -------- READ ------------------------------------------------------

        uint32_t read() const
        {
            uint32_t value = 0;

            for(std::size_t pin = 0; pin < m_bus.size(); ++pin)
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

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        std::dynarray<std::unique_ptr<hal::Gpio>> m_bus;
};
#endif // DEPRECATED 20200912




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

        // -------- BUS WIDTH / MASK ------------------------------------------

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

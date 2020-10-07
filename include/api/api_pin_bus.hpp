// ----------------------------------------------------------------------------
// @file    api_pin_bus.hpp
// @brief   API pin bus class.
// @date    30 September 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_API_PIN_BUS_HPP
#define XARMLIB_API_PIN_BUS_HPP

//#include "hal/hal_pin.hpp"
#include "core/target_pin.hpp"

#include <type_traits>




namespace xarmlib
{

#if 1 // HP_BRANCH
struct PinNameTraits
{
    using Type = Pin::Name;

    static constexpr bool valid(const Type pin) noexcept
    {
        return (pin != Pin::Name::nc);
    }

};

struct PinIndexTraits
{
    using Type = uint8_t;

    static constexpr bool valid(const int pin) noexcept
    {
        return (pin >= 0 && pin <= 255);
    }
};




template <typename PinTraits, std::size_t Size>
class PinBus
{
    static_assert(Size <= 32);

    using Type = typename PinTraits::Type;

    static constexpr auto make_valid_list(const Type (&pin_list)[Size]) noexcept
    {
        std::array<Type, Size> valid_list{};

        for(std::size_t index = 0; index < Size; ++index)
        {
            if(!PinTraits::valid(pin_list[index])) std::terminate();
            valid_list[index] = pin_list[index];
        }

        return valid_list;
    }

public:

    constexpr PinBus(const Type (&pin_list)[Size]) noexcept
        : m_pin_list {make_valid_list(pin_list)} {}

    constexpr std::size_t size() const noexcept
    {
        return m_pin_list.size();
    }

    constexpr uint32_t mask() const noexcept
    {
        return static_cast<uint32_t>((1UL << size()) - 1);
    }

    constexpr const Type* begin() const noexcept
    {
        return m_pin_list.cbegin();
    }

    constexpr const Type* end() const noexcept
    {
        return m_pin_list.end();
    }

    constexpr const Type operator[](const std::size_t index) const
    {
        return m_pin_list[index];
    }

    private:

    const std::array<Type, Size> m_pin_list;
};

// PinBus template parameter deduction guides
template <typename PinTraits = PinNameTraits, std::size_t Size>
PinBus(const Pin::Name (&)[Size]) -> PinBus<PinTraits, Size>;

template <typename PinTraits = PinIndexTraits, std::size_t Size>
PinBus(const int (&)[Size]) -> PinBus<PinTraits, Size>;
#endif // HP_BRANCH




#if 0 // DEPRECATED 20200912
template <class Type, class Enable = void>
class PinBus;

template <class Type>
class PinBus<Type, typename std::enable_if<std::is_same<Type, hal::Pin::Name>::value == true
                                        || std::is_same<Type, int8_t        >::value == true>::type>
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        PinBus() = delete;

        constexpr PinBus(const std::initializer_list<Type> pin_list) : m_pin_list (pin_list)
        {
#ifndef NDEBUG
            assert(pin_list.size() <= 32);

            for(const auto pin : pin_list)
            {
                if constexpr(std::is_same<Type, hal::Pin::Name>::value == true)
                {
                    assert(pin != hal::Pin::Name::nc);
                }

                if constexpr(std::is_same<Type, int8_t>::value == true)
                {
                    assert(pin >= 0);
                }
            }
#endif
        }

        constexpr std::size_t get_size() const
        {
            return m_pin_list.size();
        }

        constexpr uint32_t get_mask() const
        {
            return static_cast<uint32_t>((1UL << get_size()) - 1);
        }

        constexpr const Type* begin() const
        {
            return m_pin_list.begin();
        }

        constexpr const Type* end() const
        {
            return m_pin_list.end();
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const std::initializer_list<Type> m_pin_list;
};




using PinNameBus  = PinBus<hal::Pin::Name>;
using PinIndexBus = PinBus<int8_t>;
#endif // DEPRECATED 20200912




#if 0 // DEPRECATED 20180703
template <Pin::Name... pins>
class PinBus
{
    public:

        constexpr PinBus() : m_pin_names { pins... }
        {}

        constexpr std::size_t get_count() const
        {
            return sizeof...(pins);
        }

        constexpr const std::array<Pin::Name, sizeof...(pins)>& get_array() const
        {
            return m_pin_names;
        }

        constexpr Pin::Name get_pin_name(const std::size_t index) const
        {
            assert(index < get_count());

            return m_pin_names[index];
        }

    private:

        const std::array<Pin::Name, sizeof...(pins)> m_pin_names;
};
#endif // DEPRECATED 20180703

} // namespace xarmlib




#endif // XARMLIB_API_PIN_BUS_HPP

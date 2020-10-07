// ----------------------------------------------------------------------------
// @file    bitmask.hpp
// @brief   Type-safe bitmask implementation.
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

#ifndef XARMLIB_CORE_BITMASK_HPP
#define XARMLIB_CORE_BITMASK_HPP

#include <type_traits>




namespace xarmlib
{

template <typename EnumType>
class Bitmask
{
    // Ensure that bitmask can only take enums.
    static_assert(std::is_enum_v<EnumType> == true);

    // The type used to store the bitmask should be the same as the enum's underlying type.
    using UnderlyingType = typename std::underlying_type_t<EnumType>;

public:

    // Default constructor creates a bitmask with no options selected.
    constexpr Bitmask() : m_mask {0}
    {}

    // Constructor that safely initializes the bitmask from an integer value
    explicit constexpr Bitmask(UnderlyingType value) : m_mask {value & static_cast<UnderlyingType>(EnumType::bitmask)}
    {}

    // Creates a bitmask with just one bit set. This constructor is intentionally
    // non-explicit to allow passing an enum option to a function taking a bitmask.
    // Example: FunctionExpectingBitmask(Options::Opt1)
    constexpr Bitmask(EnumType bit) : m_mask {static_cast<UnderlyingType>(bit)}
    {}

    constexpr Bitmask operator | (const Bitmask& rhs) const { return Bitmask{m_mask | rhs.m_mask}; }
    constexpr Bitmask operator & (const Bitmask& rhs) const { return Bitmask{m_mask & rhs.m_mask}; }
    constexpr Bitmask operator ^ (const Bitmask& rhs) const { return Bitmask{m_mask ^ rhs.m_mask}; }

    constexpr Bitmask& operator |= (const Bitmask& rhs) { m_mask |= static_cast<UnderlyingType>(rhs.m_mask); return (*this); }
    constexpr Bitmask& operator &= (const Bitmask& rhs) { m_mask &= static_cast<UnderlyingType>(rhs.m_mask); return (*this); }
    constexpr Bitmask& operator ^= (const Bitmask& rhs) { m_mask ^= static_cast<UnderlyingType>(rhs.m_mask); return (*this); }

    constexpr Bitmask operator | (EnumType bit) const { return Bitmask{m_mask | static_cast<UnderlyingType>(bit)}; }
    constexpr Bitmask operator & (EnumType bit) const { return Bitmask{m_mask & static_cast<UnderlyingType>(bit)}; }
    constexpr Bitmask operator ^ (EnumType bit) const { return Bitmask{m_mask ^ static_cast<UnderlyingType>(bit)}; }

    constexpr Bitmask& operator |= (EnumType bit) { m_mask |= static_cast<UnderlyingType>(bit); return (*this); }
    constexpr Bitmask& operator &= (EnumType bit) { m_mask &= static_cast<UnderlyingType>(bit); return (*this); }
    constexpr Bitmask& operator ^= (EnumType bit) { m_mask ^= static_cast<UnderlyingType>(bit); return (*this); }

    constexpr Bitmask operator ~ () const { return Bitmask{~m_mask & static_cast<UnderlyingType>(EnumType::bitmask)}; }

    constexpr bool operator ! () const { return (m_mask == 0); }

    constexpr operator UnderlyingType() const { return m_mask; }

    constexpr void clear() { m_mask = 0; }

private:

    UnderlyingType m_mask {0};
};




template <class EnumType>
constexpr Bitmask<EnumType> operator | (EnumType lhs, EnumType rhs) { return Bitmask<EnumType>{lhs} | rhs; }

template <class EnumType>
constexpr Bitmask<EnumType> operator & (EnumType lhs, EnumType rhs) { return Bitmask<EnumType>{lhs} & rhs; }

} // namespace xarmlib




#endif // XARMLIB_CORE_BITMASK_HPP

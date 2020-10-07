// ----------------------------------------------------------------------------
// @file    chrono.hpp
// @brief   Helpers, additions and variations of std::chrono.
// @date    1 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_CORE_CHRONO_HPP
#define XARMLIB_CORE_CHRONO_HPP

#include <cassert>
#include <chrono>




namespace xarmlib
{
namespace chrono
{

// Type traits helper
template<typename T>
struct is_duration
{
    static constexpr bool value = false;
};

template<typename Rep, typename Period>
struct is_duration<std::chrono::duration<Rep, Period>>
{
    static constexpr bool value = true;
};




// 32-bit microsecond duration type. Alternative to the standard
// std::chrono::microseconds which is signed 64-bit. The use of
// an unsigned value allows wrapping.
using microseconds_u32 = std::chrono::duration<std::uint32_t, std::micro>;

// 32-bit millisecond duration type. Alternative to the standard
// std::chrono::milliseconds which is signed 64-bit. The use of
// an unsigned value allows wrapping.
using milliseconds_u32 = std::chrono::duration<std::uint32_t, std::milli>;



} // namespace chrono




inline namespace literals
{
inline namespace chrono_literals_u32
{

constexpr chrono::microseconds_u32 operator ""_us(unsigned long long x)
{
    chrono::microseconds_u32::rep val = static_cast<chrono::microseconds_u32::rep>(x);
    assert(static_cast<uint32_t>(val) == x);
    return chrono::microseconds_u32(val);
}

constexpr chrono::milliseconds_u32 operator ""_ms(unsigned long long x)
{
    chrono::milliseconds_u32::rep val = static_cast<chrono::milliseconds_u32::rep>(x);
    assert(static_cast<uint32_t>(val) == x);
    return chrono::milliseconds_u32(val);
}

} // inline namespace chrono_literals_u32

} // inline namespace literals

namespace chrono
{

using namespace chrono_literals_u32;

} // namespace chrono

} // namespace xarmlib




#endif // XARMLIB_CORE_CHRONO_HPP

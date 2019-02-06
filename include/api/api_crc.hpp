// ----------------------------------------------------------------------------
// @file    api_crc.hpp
// @brief   Generic CRC class (usable when a hardware implementation is not
//          available). Specializations for some common types of 8, 16 and
//          32 bits. Formulas taken from:
//          https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
// @date    6 February 2019
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

#ifndef __XARMLIB_API_CRC_HPP
#define __XARMLIB_API_CRC_HPP

#include "external/span.hpp"

#include <array>
#include <stdint.h>

namespace xarmlib
{




template <typename Type,
          Type     Polynomial,
          Type     InitialRemainder,
          Type     FinalXorValue,
          bool     ReflectInput,
          bool     ReflectOutput>
class Crc
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        static constexpr Type calculate(const std::span<const uint8_t> buffer)
        {
            Type remainder = InitialRemainder;

            for(auto& elem : buffer)
            {
                const uint8_t data = static_cast<uint8_t>(reflect_input(elem) ^ (remainder >> (WIDTH - 8)));
                remainder = static_cast<Type>(m_lookup_table[data] ^ (remainder << 8));
            }

            return reflect_output(remainder) ^ FinalXorValue;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        // CRC type width in bits
        enum Width : uint8_t
        {
            WIDTH = 8 * sizeof(Type)
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        template<bool enabled = ReflectInput>
        static constexpr Type reflect_input(typename std::enable_if<enabled, Type>::type data)
        {
            return reflect(data, 8);
        }

        template<bool enabled = ReflectInput>
        static constexpr Type reflect_input(typename std::enable_if<!enabled, Type>::type data)
        {
            return data;
        }

        template<bool enabled = ReflectOutput>
        static constexpr Type reflect_output(typename std::enable_if<enabled, Type>::type data)
        {
            return reflect(data, WIDTH);
        }

        template<bool enabled = ReflectOutput>
        static constexpr Type reflect_output(typename std::enable_if<!enabled, Type>::type data)
        {
            return data;
        }

        static constexpr Type reflect(Type data, const int32_t num_bits)
        {
            Type reflection = 0;

            // Reflect the data about the center bit.
            for(int32_t bit = 0; bit < num_bits; ++bit)
            {
                // If the LSB bit is set, set the reflection of it.
                if((data & 0x01) != 0)
                {
                    reflection |= static_cast<Type>((1 << ((num_bits - 1) - bit)));
                }

                data = static_cast<Type>(data >> 1);
            }

            return reflection;
        }

        static constexpr Type modulo2(const std::size_t dividend)
        {
            //static_assert(dividend >= 0 && dividend <= 255);

            // Start with the dividend followed by zeros
            Type remainder = static_cast<Type>(dividend << (WIDTH - 8));

            // Perform modulo-2 division (a bit at a time)
            for(int32_t bit = 8; bit > 0; --bit)
            {
                // Try to divide the current data bit
                if ((remainder & (1 << (WIDTH - 1))) != 0)
                {
                    remainder = static_cast<Type>((remainder << 1) ^ Polynomial);
                }
                else
                {
                    remainder = static_cast<Type>(remainder << 1);
                }
            }

            return remainder;
        }

        static constexpr auto build_lookup_table()
        {
            std::array<Type, 256> result {0};

            for(std::size_t i = 0; i < result.size(); ++i)
            {
                result[i] = modulo2(i);
            }

            return result;
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        static constexpr std::array<Type, 256> m_lookup_table { build_lookup_table() };
};




// Static definition
template <typename Type,
          Type     Polynomial,
          Type     InitialRemainder,
          Type     FinalXorValue,
          bool     ReflectInput,
          bool     ReflectOutput>
constexpr std::array<Type, 256> Crc<Type,
                                    Polynomial,
                                    InitialRemainder,
                                    FinalXorValue,
                                    ReflectInput,
                                    ReflectOutput>::m_lookup_table;




// Catalog of parameterized CRC algorithms
// http://reveng.sourceforge.net/crc-catalogue/all.htm

// --------------------------------------------------------------------
// Most commonly used CRC types definition
// --------------------------------------------------------------------

using Crc32 = Crc<uint32_t, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true>;

using Crc16Modbus = Crc<uint16_t, 0x8005, 0xFFFF, 0x0000, true,  true>;
using Crc16Xmodem = Crc<uint16_t, 0x1021, 0x0000, 0x0000, false, false>;

using Crc8    = Crc<uint8_t, 0x07, 0x00, 0x00, false, false>;
using Crc8Itu = Crc<uint8_t, 0x07, 0x00, 0x55, false, false>;




} // namespace xarmlib

#endif // __XARMLIB_API_CRC_HPP

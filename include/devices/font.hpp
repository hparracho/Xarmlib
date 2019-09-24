// ----------------------------------------------------------------------------
// @file    font.hpp
// @brief   Generic Font class.
// @date    16 September 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_DEVICES_FONT_HPP
#define __XARMLIB_DEVICES_FONT_HPP

#include <cassert>
#include <cstdint>

namespace xarmlib
{




class Font
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC DEFINITIONS
        // --------------------------------------------------------------------

        struct Config
        {
                  uint8_t  char_width;
                  uint8_t  char_height;
                  uint8_t  char_first;
                  uint8_t  char_last;
                  uint8_t  char_size;
            const uint8_t* data;
        };


        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        Font(const Config& config) : m_config { config }
        {}

        uint8_t get_char_width()  { return m_config.char_width; }
        uint8_t get_char_height() { return m_config.char_height; }
        uint8_t get_char_size()   { return m_config.char_size; }

        const uint8_t* get_data(const std::size_t char_index) const
        {
            assert(char_index >= m_config.char_first && char_index <= m_config.char_last);

            return &m_config.data[(char_index - m_config.char_first) * m_config.char_size];
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        const Config& m_config;
};




} // namespace xarmlib

#endif // __XARMLIB_DEVICES_FONT_HPP

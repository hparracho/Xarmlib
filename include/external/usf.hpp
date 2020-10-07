// ----------------------------------------------------------------------------
// @file    usf.hpp
// @brief   Micro String Format (USF) configuration and main header file
//          to use in the library. This should be the only header file
//          included when USF functionality is required.
// @date    7 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_EXTERNAL_USF_HPP
#define XARMLIB_EXTERNAL_USF_HPP

#include "xarmlib_config.hpp"




// ----------------------------------------------------------------------------
// usflib configuration options
// ----------------------------------------------------------------------------

// Configuration of floating point support.
// USF_DISABLE_FLOAT_SUPPORT           : disables the support of floating point types (it will save considerable code size)

// Configuration of format output string termination option.
// USF_DISABLE_STRING_TERMINATION      : disables the null termination of the format output string

// Configuration of possible behavior when a condition is violated.
// USF_TERMINATE_ON_CONTRACT_VIOLATION : std::terminate() will be called (default)
// USF_ABORT_ON_CONTRACT_VIOLATION     : std::abort() will be called (more suitable for embedded platforms, maybe?)
// USF_THROW_ON_CONTRACT_VIOLATION     : an exception will be thrown

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// NOTE: Options should be specified in 'xarmlib_config.hpp' header file.
// ----------------------------------------------------------------------------

#include "../../external/USFLIB/include/usf/usf.hpp"




namespace usf
{

// Helper function to print a formatted string to any object that implements
// a 'putc()' member function. The 'str' variable serves as a writing buffer.
// Optimally the buffer should be static initialized and used across all
// function calls. Returns a string span to the written buffer.
template <typename Writer, typename... Args>
StringSpan print_to(Writer& writer, StringSpan str, StringView fmt, Args&&... args)
{
    auto ss = format_to(str, fmt, args...);

    for(const auto ch : ss)
    {
        writer.putc(ch);
    }

    return ss;
}

} // namespace usf

#endif // XARMLIB_EXTERNAL_USF_HPP

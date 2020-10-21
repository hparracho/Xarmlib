// ----------------------------------------------------------------------------
// @file    non_copyable.hpp
// @brief   Non copyable helper class. Classes which are not value type should
//          inherit privately from this class to avoid generation of invalid
//          copy constructor or copy assignment operator.
// @date    21 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_CORE_NON_COPYABLE_HPP
#define XARMLIB_CORE_NON_COPYABLE_HPP




namespace xarmlib
{

template<typename Type>
class NonCopyable
{
    // The template parameter 'Type' is the type that should be made non copyable.
    // It prevent cases where the empty base optimization cannot be applied and
    // therefore ensure zero cost while using this class.

protected:

    // Prevent the use of this class as a final one (without being derived)
    NonCopyable()
    {}

    ~NonCopyable()
    {}

private:

    // NonCopyable copy constructor
    NonCopyable(const NonCopyable&) = delete;

    // NonCopyable copy assignment operator
    NonCopyable& operator = (const NonCopyable&) = delete;
};

} // namespace xarmlib




#endif // XARMLIB_CORE_NON_COPYABLE_HPP

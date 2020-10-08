// ----------------------------------------------------------------------------
// @file    scoped_lock.hpp
// @brief   Class for owning a lock of Lockable object for the duration of a
//          scoped block.
// @date    8 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_CORE_SCOPED_LOCK_HPP
#define XARMLIB_CORE_SCOPED_LOCK_HPP

#include "core/mutex.hpp"




namespace xarmlib
{

class ScopedLock : private NonCopyable<ScopedLock>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    ScopedLock(Lockable& lockable): m_lockable {lockable}
    {
        m_lockable.lock();
    }

    ~ScopedLock()
    {
        m_lockable.unlock();
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    Lockable& m_lockable;
};

} // namespace xarmlib




#endif // XARMLIB_CORE_SCOPED_LOCK_HPP

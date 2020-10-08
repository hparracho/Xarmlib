// ----------------------------------------------------------------------------
// @file    mutex.hpp
// @brief   Mutex class used to protect access to a shared resource. Also
//          specifies a dummy class for when the mutex functionality is not
//          required (e.g., but not limited to baremetal without OS).
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

#ifndef XARMLIB_CORE_MUTEX_HPP
#define XARMLIB_CORE_MUTEX_HPP

#include "xarmlib_config.hpp"

#include "core/non_copyable.hpp"




namespace xarmlib
{

class Lockable : private NonCopyable<Lockable>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    Lockable() {}

    virtual ~Lockable() {}

    // Wait until a mutex becomes available
    virtual void lock() {}

    // Unlock a previously locked mutex
    virtual void unlock() {}
};

using MutexDummy = Lockable;




#if (XARMLIB_ENABLE_FREERTOS == 1)

// NOTE: Mutex class cannot be used inside ISRs

class Mutex : private Lockable
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    Mutex() : m_rtos_mutex {xSemaphoreCreateMutex()}
    {
        assert(m_rtos_mutex != nullptr);
    }

    ~Mutex() final
    {
        vSemaphoreDelete(m_rtos_mutex);
    }

    // Wait until a mutex becomes available
    void lock() final
    {
        xSemaphoreTake(m_rtos_mutex, portMAX_DELAY);
    }

    // Unlock a previously locked mutex
    void unlock() final
    {
        xSemaphoreGive(m_rtos_mutex);
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    SemaphoreHandle_t m_rtos_mutex;
};

#else

using Mutex = Lockable;

#endif

} // namespace xarmlib




#endif // XARMLIB_CORE_MUTEX_HPP

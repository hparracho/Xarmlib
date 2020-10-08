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

class MutexBase : private NonCopyable<MutexBase>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    MutexBase() {}

    virtual ~MutexBase() {}

    // Wait until a mutex becomes available
    virtual void lock() {}

    // Wait until a mutex becomes available from an IRQ
    virtual void lock_from_isr([[maybe_unused]] int32_t& yield) {}

    // Unlock a previously locked mutex
    virtual void unlock() {}

    // Unlock a previously locked mutex from an ISR
    virtual void unlock_from_isr([[maybe_unused]] int32_t& yield) {}
};

using DummyMutex = MutexBase;




#if (XARMLIB_ENABLE_FREERTOS == 1)
class Mutex : private MutexBase
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    Mutex()
    {
        m_rtos_mutex = xSemaphoreCreateMutex();
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

    // Wait until a mutex becomes available from an IRQ
    void lock_from_isr(int32_t& yield) final
    {
        xSemaphoreTakeFromISR(m_rtos_mutex, &yield);
    }

    // Unlock a previously locked mutex
    void unlock() final
    {
        xSemaphoreGive(m_rtos_mutex);
    }

    // Unlock a previously locked mutex from an ISR
    void unlock_from_isr(int32_t& yield) final
    {
        xSemaphoreGiveFromISR(m_rtos_mutex, &yield);
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    SemaphoreHandle_t m_rtos_mutex {nullptr};
};
#else

using Mutex = DummyMutex;

#endif

} // namespace xarmlib




#endif // XARMLIB_CORE_MUTEX_HPP

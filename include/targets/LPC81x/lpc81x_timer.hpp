// ----------------------------------------------------------------------------
// @file    lpc81x_timer.hpp
// @brief   NXP LPC81x Timer (MRT) class.
// @date    6 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_TARGETS_LPC81X_TIMER_HPP
#define XARMLIB_TARGETS_LPC81X_TIMER_HPP

#include "xarmlib_config.hpp"

#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "hal/hal_timer_base.hpp"




// Forward declaration of IRQ handler
extern "C" void MRT_IRQHandler(void);




namespace xarmlib::targets::lpc81x
{

class Timer : public hal::TimerBase<Timer>
{
    // Give IRQ handler C function access to private member functions
    friend void ::MRT_IRQHandler(void);

public:

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTOR / DESTRUCTOR --------------------------------------

    Timer() : hal::TimerBase<Timer>(*this)
    {
        // Enable MRT if this is the first timer peripheral instantiation
        if(m_ref_counter.get_use_count() == 1)
        {
            SysClock::enable(SysClock::Peripheral::mrt);
            Power::reset(Power::ResetPeripheral::mrt);
        }

        // Set pointer to the next available channel structure
        m_channel = &LPC_MRT->CHANNEL[m_ref_counter.get_this_index()];

        set_interval(0);
    }

#if !defined(XARMLIB_DISABLE_DESTRUCTORS) || (XARMLIB_DISABLE_DESTRUCTORS == 0)
    ~Timer()
    {
        set_interval(0);

        // Disable MRT if this is the last timer peripheral destruction
        if(m_ref_counter.get_use_count() == 1)
        {
            SysClock::disable(SysClock::Peripheral::mrt);
        }
    }
#endif

    // -------- START / STOP --------------------------------------------------

    // Start the timer with the supplied interval and mode
    void start(const chrono::microseconds_u32& rate_us, const Mode mode = Mode::free_running)
    {
        assert(rate_us.count() >= get_min_rate_us());
        assert(rate_us.count() <= get_max_rate_us());

        m_interval = convert_us_to_interval(rate_us.count());

        set_mode(mode);
        set_interval(m_interval);
    }

    // Stop a previously started timer
    void stop()
    {
        set_interval(0);
    }

    // Reload a previously set interval and re-start the timer
    void reload()
    {
        // Ensure interval is set
        assert(m_interval != 0);
        set_interval(m_interval);
    }

    // Check if the timer is running or stopped
    bool is_running() const
    {
        return ((m_channel->STAT & stat_run) != 0);
    }

    // -------- INTERRUPTS ----------------------------------------------------

    // Enable the interrupt
    void enable_interrupt() { m_channel->CTRL |= ctrl_inten; }

    // Disable the interrupt
    void disable_interrupt() { m_channel->CTRL &= ~ctrl_inten; }

    // Check if the interrupt is enabled
    bool is_interrupt_enabled() const { return (m_channel->CTRL & ctrl_inten) != 0; }

    // Check if the interrupt is pending
    bool is_interrupt_pending() const { return (m_channel->STAT & stat_intflag) != 0; }

    // Clear the interrupt pending flag
    void clear_interrupts_pending() { m_channel->STAT |= stat_intflag; }

    // -------- IRQ -----------------------------------------------------------

    // Get the IRQ type associated with this peripheral
    constexpr static IRQn_Type get_irq_name()
    {
        return MRT_IRQn;
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE DEFINITIONS
    // ------------------------------------------------------------------------

    // MRT Time Interval Register (INTVAL) bits
    enum INTVAL : uint32_t
    {
        intval_load         = (1UL << 31)
    };

    // MRT Control Register (CTRL) bits and masks
    enum CTRL : uint32_t
    {
        ctrl_inten          = (1UL << 0),
        ctrl_mode_repeat    = (0UL << 1),   // Free running
        ctrl_mode_one_shot  = (1UL << 1),   // Single shot
        ctrl_mode_mask      = (3UL << 1)
    };

    // MRT Status Register (STAT) bits
    enum STAT : uint32_t
    {
        stat_intflag        = (1UL << 0),
        stat_run            = (1UL << 1)
    };

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    void set_mode(const Mode mode)
    {
        uint32_t ctrl_mode = 0;

        switch(mode)
        {
            case Mode::free_running: ctrl_mode = ctrl_mode_repeat;   break;
            case Mode::single_shot:  ctrl_mode = ctrl_mode_one_shot; break;
        }

        m_channel->CTRL = (m_channel->CTRL & ~ctrl_mode_mask) | ctrl_mode;
    }

    // Set timer time interval value
    // NOTE: Setting INTVAL_LOAD bit in timer time interval register
    //       causes the time interval value to load immediately, otherwise
    //       the time interval value will be loaded in next timer cycle.
    void set_interval(const uint32_t timer_interval)
    {
        m_channel->INTVAL = timer_interval | intval_load;
    }

    // Get timer interval value (ready to load into INTVAL register) based on supplied rate in microseconds
    static uint32_t convert_us_to_interval(const uint64_t rate_us)
    {
        return static_cast<uint32_t>(rate_us * SystemCoreClock / 1000000UL);
    }

    // Get the minimum allowed rate in microseconds
    static uint32_t get_min_rate_us()
    {
        const uint64_t min_interval = 0x01;

        const auto min_rate_us = static_cast<uint32_t>(min_interval * 1000000UL / SystemCoreClock);

        return (min_rate_us > 0) ? min_rate_us : 1;
    }

    // Get the maximum allowed rate in microseconds
    static uint32_t get_max_rate_us()
    {
        const uint64_t max_interval = 0x7FFFFFFF;

        return static_cast<uint32_t>(max_interval * 1000000UL / SystemCoreClock);
    }

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    LPC_MRT_CHANNEL_T*  m_channel  {nullptr};   // Pointer to the individual MRT channel structure
    uint32_t            m_interval {0};         // Last loaded interval value
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_TIMER_HPP

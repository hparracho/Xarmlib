// ----------------------------------------------------------------------------
// @file    hal_peripheral_shared_irq.hpp
// @brief   HAL interface class for peripherals that have a shared IRQ handler
//          and a single interrupt.
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

#ifndef XARMLIB_HAL_PERIPHERAL_SHARED_IRQ_HPP
#define XARMLIB_HAL_PERIPHERAL_SHARED_IRQ_HPP

#include "xarmlib_config.hpp"

#include "core/delegate.hpp"
#include "core/non_copyable.hpp"
#include "hal/hal_peripheral_ref_counter.hpp"




namespace xarmlib::hal
{

template <typename Driver, std::size_t PeripheralCount, uint32_t FixedMask = 0>
class PeripheralSharedIrq : NonCopyable<PeripheralSharedIrq<Driver, PeripheralCount, FixedMask>>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    // IRQ handler definitions
    using IrqHandlerType = int32_t();
    using IrqHandler     = Delegate<IrqHandlerType>;

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- INTERRUPTS ----------------------------------------------------

    // Enable the interrupt
    void enable_interrupt() { static_cast<Driver*>(this)->enable_interrupt(); }

    // Disable the interrupt
    void disable_interrupt() { static_cast<Driver*>(this)->disable_interrupt(); }

    // Check if the interrupt is enabled
    bool is_interrupt_enabled() const { return static_cast<const Driver*>(this)->is_interrupt_enabled(); }

    // Check if the interrupt is pending
    bool is_interrupt_pending() const { return static_cast<const Driver*>(this)->is_interrupt_pending(); }

    // Clear the interrupt pending flag
    void clear_interrupt_pending() { static_cast<Driver*>(this)->clear_interrupt_pending(); }

    // -------- IRQ -----------------------------------------------------------

    // Check if the shared IRQ is enabled
    static bool is_shared_irq_enabled()
    {
        return NVIC_GetEnableIRQ(Driver::get_irq_name());
    }

    // Set the shared IRQ priority
    static void set_shared_irq_priority(const uint32_t irq_priority)
    {
        NVIC_SetPriority(Driver::get_irq_name(), irq_priority);
    }

    // Get the shared IRQ priority
    static uint32_t get_shared_irq_priority()
    {
        return NVIC_GetPriority(Driver::get_irq_name());
    }

    // Assign an IRQ handler and automatically enable the shared IRQ interrupts
    void assign_irq_handler(const IrqHandler& irq_handler)
    {
        assert(irq_handler != nullptr);
        m_irq_handler = irq_handler;
        s_irq_handlers_mask |= 1UL << m_ref_counter.get_this_index();

        NVIC_EnableIRQ(Driver::get_irq_name());
    }

    // Remove an IRQ handler and automatically disable the shared IRQ interrupts
    void remove_irq_handler()
    {
        s_irq_handlers_mask &= ~(1UL << m_ref_counter.get_this_index());

        if(s_irq_handlers_mask == 0)
        {
            NVIC_DisableIRQ(Driver::get_irq_name());
        }

        m_irq_handler = nullptr;
    }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTOR / DESTRUCTOR --------------------------------------

    PeripheralSharedIrq(Driver& driver_ref) : m_ref_counter {driver_ref}
    {}

#if !defined(XARMLIB_DISABLE_DESTRUCTORS) || (XARMLIB_DISABLE_DESTRUCTORS == 0)
    ~PeripheralSharedIrq()
    {
        disable_interrupt();
        clear_interrupt_pending();
        remove_irq_handler();
    }
#endif

    // -------- IRQ HANDLER ---------------------------------------------------

    // Shared IRQ handler for all channels of this peripheral called directly by the interrupt C function
    // NOTE: Returns yield flag for FreeRTOS
    static int32_t irq_handler()
    {
        int32_t yield = 0;  // Used by FreeRTOS

        for(std::size_t index = 0; index < PeripheralCount; ++index)
        {
            auto* const peripheral = RefCounter::get_pointer(index);

            if(peripheral != nullptr)
            {
                if(peripheral->is_interrupt_pending())
                {
                    if(peripheral->m_irq_handler != nullptr)
                    {
                        yield = peripheral->m_irq_handler();
                    }
                }

                peripheral->clear_interrupt_pending();
            }
        }

        return yield;
    }

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER VARIABLES
    // ------------------------------------------------------------------------

    // Reference counter alias
    using RefCounter = PeripheralRefCounter<Driver, PeripheralCount, FixedMask>;

    RefCounter              m_ref_counter;          // Peripheral reference counter
    IrqHandler              m_irq_handler;          // User defined IRQ handler
    inline static uint32_t  s_irq_handlers_mask {0};
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_PERIPHERAL_SHARED_IRQ_HPP

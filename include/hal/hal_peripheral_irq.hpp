// ----------------------------------------------------------------------------
// @file    hal_peripheral_irq.hpp
// @brief   HAL interface class for peripherals that have an individual IRQ
//          handler and a single interrupt.
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

#ifndef XARMLIB_HAL_PERIPHERAL_IRQ_HPP
#define XARMLIB_HAL_PERIPHERAL_IRQ_HPP

#include "xarmlib_config.hpp"

#include "core/delegate.hpp"
#include "core/non_copyable.hpp"
#include "hal/hal_peripheral_ref_counter.hpp"




namespace xarmlib::hal
{

template <typename Driver, std::size_t PeripheralCount, uint32_t FixedMask = 0>
class PeripheralIrq : NonCopyable<PeripheralIrq<Driver, PeripheralCount, FixedMask>>
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

    // Check is the interrupt is enabled
    bool is_interrupt_enabled() const { return static_cast<const Driver*>(this)->is_interrupt_enabled(); }

    // Check is the interrupt is pending
    bool is_interrupt_pending() const { return static_cast<const Driver*>(this)->is_interrupt_pending(); }

    // Clear the interrupt pending flag
    void clear_interrupt_pending() { static_cast<Driver*>(this)->clear_interrupt_pending(); }

    // -------- IRQ -----------------------------------------------------------

    // Check if the IRQ is enabled
    bool is_irq_enabled() const
    {
        return NVIC_GetEnableIRQ(static_cast<const Driver*>(this)->get_irq_name());
    }

    // Set the IRQ priority
    void set_irq_priority(const uint32_t irq_priority) const
    {
        NVIC_SetPriority(static_cast<const Driver*>(this)->get_irq_name(), irq_priority);
    }

    // Get the IRQ priority
    uint32_t get_irq_priority() const
    {
        return NVIC_GetPriority(static_cast<const Driver*>(this)->get_irq_name());
    }

    // Assign an IRQ handler and automatically enable the IRQ interrupts
    void assign_irq_handler(const IrqHandler& irq_handler)
    {
        assert(irq_handler != nullptr);
        m_irq_handler = irq_handler;
        NVIC_EnableIRQ(static_cast<const Driver*>(this)->get_irq_name());
    }

    // Remove an IRQ handler and automatically disable the IRQ interrupts
    void remove_irq_handler()
    {
        NVIC_DisableIRQ(static_cast<const Driver*>(this)->get_irq_name());
        m_irq_handler = nullptr;
    }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTOR / DESTRUCTOR --------------------------------------

    PeripheralIrq(Driver& driver_ref) : m_ref_counter {driver_ref}
    {}

#if (XARMLIB_DISABLE_EXPENSIVE_PERIPHERAL_DESTRUCTORS != 1)
    ~PeripheralIrq()
    {
        disable_interrupt();
        clear_interrupt_pending();
        remove_irq_handler();
    }
#endif

    // -------- IRQ HANDLER ---------------------------------------------------

    // IRQ handler called directly by the interrupt C functions
    // NOTE: Returns yield flag for FreeRTOS
    static int32_t irq_handler(const std::size_t index)
    {
        int32_t yield = 0;  // Used by FreeRTOS

        auto* const peripheral = RefCounter::get_pointer(index);

        if(peripheral != nullptr)
        {
            if(peripheral->m_irq_handler != nullptr)
            {
                yield = peripheral->m_irq_handler();
            }

            peripheral->clear_interrupt_pending();
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
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_PERIPHERAL_IRQ_HPP

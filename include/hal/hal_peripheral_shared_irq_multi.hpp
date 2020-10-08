// ----------------------------------------------------------------------------
// @file    hal_peripheral_shared_irq_multi.hpp
// @brief   HAL interface class for peripherals that have a shared IRQ handler
//          and multiple interrupts.
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

#ifndef XARMLIB_HAL_PERIPHERAL_SHARED_IRQ_MULTI_HPP
#define XARMLIB_HAL_PERIPHERAL_SHARED_IRQ_MULTI_HPP

#include "xarmlib_config.hpp"

#include "core/bitmask.hpp"
#include "core/delegate.hpp"
#include "core/non_copyable.hpp"
#include "hal/hal_peripheral_ref_counter.hpp"




namespace xarmlib::hal
{

template <typename Driver, typename Interrupt, std::size_t PeripheralCount, uint32_t FixedMask = 0>
class PeripheralSharedIrqMulti : NonCopyable<PeripheralSharedIrqMulti<Driver, Interrupt, PeripheralCount, FixedMask>>
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

    // Enable the interrupts specified in the bitmask
    void enable_interrupts(const Bitmask<Interrupt> bitmask) { static_cast<Driver*>(this)->enable_interrupts(bitmask); }

    // Disable the interrupts specified in the bitmask
    void disable_interrupts(const Bitmask<Interrupt> bitmask) { static_cast<Driver*>(this)->disable_interrupts(bitmask); }

    // Get all the interrupts that are enabled
    Bitmask<Interrupt> get_interrupts_enabled() const { return static_cast<const Driver*>(this)->get_interrupts_enabled(); }

    // Get all the interrupts that are pending
    Bitmask<Interrupt> get_interrupts_pending() const { return static_cast<const Driver*>(this)->get_interrupts_pending(); }

    // Clear all the interrupts pending flags
    void clear_interrupts_pending() { static_cast<Driver*>(this)->clear_interrupts_pending(); }

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

    PeripheralSharedIrqMulti(Driver& driver_ref) : m_ref_counter {driver_ref}
    {}

#if (XARMLIB_DISABLE_DESTRUCTORS != 1)
    ~PeripheralSharedIrqMulti()
    {
        disable_interrupts(Interrupt::bitmask); // Disable all
        clear_interrupts_pending();
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
                if(peripheral->get_interrupts_pending() != 0)
                {
                    if(peripheral->m_irq_handler != nullptr)
                    {
                        yield = peripheral->m_irq_handler();
                    }
                }

                peripheral->clear_interrupts_pending();
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




#endif // XARMLIB_HAL_PERIPHERAL_SHARED_IRQ_MULTI_HPP

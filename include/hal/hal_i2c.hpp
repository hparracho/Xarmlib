// ----------------------------------------------------------------------------
// @file    hal_i2c.hpp
// @brief   I2C Master HAL interface class.
// @note    Slave mode is not implemented.
// @date    18 April 2019
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2019 Helder Parracho (hparracho@gmail.com)
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

#ifndef __XARMLIB_HAL_I2C_HPP
#define __XARMLIB_HAL_I2C_HPP

#include "external/span.hpp"
#include "hal/hal_pin.hpp"

namespace xarmlib
{
namespace hal
{




template <typename TargetI2cDriver>
class I2cMasterHal : protected TargetI2cDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using MasterConfig     = typename TargetI2cDriver::MasterConfig;

        using TransferStatus   = typename TargetI2cDriver::TransferStatus;

        using Status           = typename TargetI2cDriver::Status;
        using StatusBitmask    = typename TargetI2cDriver::StatusBitmask;
        using Interrupt        = typename TargetI2cDriver::Interrupt;
        using InterruptBitmask = typename TargetI2cDriver::InterruptBitmask;

        using IrqHandler       = typename TargetI2cDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        I2cMasterHal(const xarmlib::PinHal::Name master_sda,
                     const xarmlib::PinHal::Name master_scl,
                     const MasterConfig&         master_config) : TargetI2cDriver(master_sda, master_scl, master_config)
        {}

        // -------- ENABLE / DISABLE ------------------------------------------

        inline void enable()           { TargetI2cDriver::master_enable(); }
        inline void disable()          { TargetI2cDriver::master_disable(); }
        inline bool is_enabled() const { return TargetI2cDriver::is_master_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        inline StatusBitmask get_status() const                        { return TargetI2cDriver::get_status(); }
        inline void          clear_status(const StatusBitmask bitmask) { TargetI2cDriver::clear_status(bitmask); }

        // -------- INTERRUPTS ------------------------------------------------

        inline void             enable_interrupts (const InterruptBitmask bitmask) { TargetI2cDriver::enable_interrupts(bitmask); }
        inline void             disable_interrupts(const InterruptBitmask bitmask) { TargetI2cDriver::disable_interrupts(bitmask); }
        inline InterruptBitmask get_interrupts_enabled() const                     { return TargetI2cDriver::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        inline void enable_irq()     { TargetI2cDriver::enable_irq(); }
        inline void disable_irq()    { TargetI2cDriver::disable_irq(); }
        inline bool is_irq_enabled() { return TargetI2cDriver::is_irq_enabled(); }

        inline void set_irq_priority(const int32_t irq_priority) { TargetI2cDriver::set_irq_priority(irq_priority); }

        inline void assign_irq_handler(const IrqHandler& irq_handler) { TargetI2cDriver::assign_irq_handler(irq_handler); }
        inline void remove_irq_handler()                              { TargetI2cDriver::remove_irq_handler(); }

        // -------- TRANSFER --------------------------------------------------

        // NOTES: - 7-bit slave address.
        //        - these methods does not return until the transfer succeeds
        //          or fails due to arbitration lost or any error.

        // Polling receive transaction
        TransferStatus read(const uint8_t slave_address, const std::span<uint8_t> rx_buffer)
        {
            std::array<uint8_t, 0> tx_buffer{};

            return TargetI2cDriver::master_transfer(slave_address, tx_buffer, rx_buffer);
        }

        // Polling send transaction
        TransferStatus write(const uint8_t slave_address, const std::span<uint8_t> tx_buffer)
        {
            std::array<uint8_t, 0> rx_buffer{};

            return TargetI2cDriver::master_transfer(slave_address, tx_buffer, rx_buffer);
        }

        // Polling transfer
        TransferStatus write_read(const uint8_t slave_address, const std::span<uint8_t> command, const std::span<uint8_t> rx_buffer)
        {
            return TargetI2cDriver::master_transfer(slave_address, command, rx_buffer);
        }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV4X__

#include "targets/KV4x/kv4x_i2c.hpp"

namespace xarmlib
{
using I2cMasterHal = hal::I2cMasterHal<targets::kv4x::I2cDriver>;
//using I2cMaster = I2cMasterHal;
}

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_i2c.hpp"

namespace xarmlib
{
using I2cMasterHal = hal::I2cMasterHal<targets::lpc84x::I2cDriver>;
//using I2cMaster = I2cMasterHal;
}

#elif defined __LPC81X__

/*#include "targets/LPC81x/lpc81x_i2c.hpp"

namespace xarmlib
{
using I2cMasterHal = hal::I2cMasterHal<targets::lpc81x::I2cDriver>;
//using I2cMaster = I2cMasterHal;
}*/

#elif defined __OHER_TARGET__

// Other target include files

namespace xarmlib
{
using I2cMasterHal = hal::I2cMasterHal<targets::other_target::I2cDriver>;
using I2cMaster = I2cMasterHal;
}

#endif




#endif // __XARMLIB_HAL_I2C_HPP

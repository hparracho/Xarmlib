// ----------------------------------------------------------------------------
// @file    hal_i2c.hpp
// @brief   I2C Master HAL interface class.
// @note    Slave mode is not implemented.
// @date    20 September 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
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




template <typename I2cDriver>
class I2cMasterBase : protected I2cDriver
{
    public:

        // --------------------------------------------------------------------
        // PUBLIC TYPE ALIASES
        // --------------------------------------------------------------------

        using MasterConfig     = typename I2cDriver::MasterConfig;

        using TransferStatus   = typename I2cDriver::TransferStatus;

        using Status           = typename I2cDriver::Status;
        using StatusBitmask    = typename I2cDriver::StatusBitmask;
        using Interrupt        = typename I2cDriver::Interrupt;
        using InterruptBitmask = typename I2cDriver::InterruptBitmask;

        using IrqHandler       = typename I2cDriver::IrqHandler;

        // --------------------------------------------------------------------
        // PUBLIC MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        I2cMasterBase(const hal::Pin::Name master_sda,
                      const hal::Pin::Name master_scl,
                      const MasterConfig&  master_config) : I2cDriver(master_sda, master_scl, master_config)
        {}

        // -------- ENABLE / DISABLE ------------------------------------------

        void enable()           { I2cDriver::master_enable(); }
        void disable()          { I2cDriver::master_disable(); }
        bool is_enabled() const { return I2cDriver::is_master_enabled(); }

        // -------- STATUS FLAGS ----------------------------------------------

        StatusBitmask get_status() const                        { return I2cDriver::get_status(); }
        void          clear_status(const StatusBitmask bitmask) { I2cDriver::clear_status(bitmask); }

        // -------- INTERRUPTS ------------------------------------------------

        void             enable_interrupts (const InterruptBitmask bitmask) { I2cDriver::enable_interrupts(bitmask); }
        void             disable_interrupts(const InterruptBitmask bitmask) { I2cDriver::disable_interrupts(bitmask); }
        InterruptBitmask get_interrupts_enabled() const                     { return I2cDriver::get_interrupts_enabled(); }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()     { I2cDriver::enable_irq(); }
        void disable_irq()    { I2cDriver::disable_irq(); }
        bool is_irq_enabled() { return I2cDriver::is_irq_enabled(); }

        void set_irq_priority(const int32_t irq_priority) { I2cDriver::set_irq_priority(irq_priority); }

        void assign_irq_handler(const IrqHandler& irq_handler) { I2cDriver::assign_irq_handler(irq_handler); }
        void remove_irq_handler()                              { I2cDriver::remove_irq_handler(); }

        // -------- TRANSFER --------------------------------------------------

        // NOTES: - 7-bit slave address.
        //        - these methods does not return until the transfer succeeds
        //          or fails due to arbitration lost or any error.

        // Polling receive transaction
        TransferStatus read(const uint8_t slave_address, const std::span<uint8_t> rx_buffer)
        {
            std::array<uint8_t, 0> tx_buffer{};

            return I2cDriver::master_transfer(slave_address, tx_buffer, rx_buffer);
        }

        // Polling send transaction
        TransferStatus write(const uint8_t slave_address, const std::span<uint8_t> tx_buffer)
        {
            std::array<uint8_t, 0> rx_buffer{};

            return I2cDriver::master_transfer(slave_address, tx_buffer, rx_buffer);
        }

        // Polling transfer
        TransferStatus write_read(const uint8_t slave_address, const std::span<uint8_t> command, const std::span<uint8_t> rx_buffer)
        {
            return I2cDriver::master_transfer(slave_address, command, rx_buffer);
        }
};




} // namespace hal
} // namespace xarmlib




#include "core/target_specs.hpp"

#if defined __KV5X__

#include "targets/KV5x/kv5x_i2c.hpp"

namespace xarmlib
{
namespace hal
{

using I2cMaster = I2cMasterBase<targets::kv5x::I2cDriver>;

} // namespace hal

using I2cMaster = hal::I2cMaster;

} // namespace xarmlib

#elif defined __KV4X__

#include "targets/KV4x/kv4x_i2c.hpp"

namespace xarmlib
{
namespace hal
{

using I2cMaster = I2cMasterBase<targets::kv4x::I2cDriver>;

} // namespace hal

using I2cMaster = hal::I2cMaster;

} // namespace xarmlib

#elif defined __LPC84X__

#include "targets/LPC84x/lpc84x_i2c.hpp"

namespace xarmlib
{
namespace hal
{

using I2cMaster = I2cMasterBase<targets::lpc84x::I2cDriver>;

} // namespace hal

using I2cMaster = hal::I2cMaster;

} // namespace xarmlib

#elif defined __LPC81X__

/*
#include "targets/LPC81x/lpc81x_i2c.hpp"

namespace xarmlib
{
namespace hal
{

using I2cMaster = I2cMasterBase<targets::lpc81x::I2cDriver>;

} // namespace hal

using I2cMaster = hal::I2cMaster;

} // namespace xarmlib
*/

#elif defined __OTHER_TARGET__

// Other target include files

namespace xarmlib
{
namespace hal
{

using I2cMaster = I2cMasterBase<targets::other_target::I2cDriver>;

} // namespace hal

using I2cMaster = hal::I2cMaster;

} // namespace xarmlib

#endif




#endif // __XARMLIB_HAL_I2C_HPP

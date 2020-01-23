// ----------------------------------------------------------------------------
// @file    kv5x_i2c.hpp
// @brief   Kinetis KV5x I2C class.
// @note    Only master mode is implemented.
// @date    23 January 2020
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

#ifndef __XARMLIB_TARGETS_KV5X_I2C_HPP
#define __XARMLIB_TARGETS_KV5X_I2C_HPP

#include "external/bitmask.hpp"
#include "fsl_i2c.h"
#include "targets/KV5x/kv5x_pin.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of IRQ handlers for all KV5x packages
extern "C" void I2C0_IRQHandler(void);
extern "C" void I2C1_IRQHandler(void);




namespace xarmlib
{
namespace targets
{
namespace kv5x
{




namespace private_i2c
{

// I2C status flags
enum class Status
{
    tx_complete         = I2C_S_TCF_MASK,           // Transfer complete flag
    address_match       = I2C_S_IAAS_MASK,          // Address match flag
    bus_busy            = I2C_S_BUSY_MASK,          // Bus busy flag
    arbitration_lost    = I2C_S_ARBL_MASK,          // Arbitration lost flag
    range_address_match = I2C_S_RAM_MASK,           // Range address match flag
    transfer_direction  = I2C_S_SRW_MASK,           // Transfer direction flag
                                                    // NOTE: 0 -> master writing to slave
                                                    //       1 -> master reading from slave
    int_pending         = I2C_S_IICIF_MASK,         // Interrupt pending flag
    rx_nack             = I2C_S_RXAK_MASK,          // Receive NACK flag
    stop_detect         = I2C_FLT_STOPF_MASK  << 8, // Stop detect flag
    start_detect        = I2C_FLT_STARTF_MASK << 8, // Start detect flag
    clear_all_bitmask   = arbitration_lost
                        | int_pending
                        | stop_detect
                        | start_detect,
    bitmask             = tx_complete
                        | address_match
                        | bus_busy
                        | arbitration_lost
                        | range_address_match
                        | transfer_direction
                        | int_pending
                        | rx_nack
                        | stop_detect
                        | start_detect
};

// I2C interrupt sources
enum class Interrupt
{
    global               = I2C_C1_IICIE_MASK,   // Global interrupt
    start_or_stop_detect = I2C_FLT_SSIE_MASK,   // Start or stop detect interrupt
    bitmask              = global
                         | start_or_stop_detect
};

BITMASK_DEFINE_VALUE_MASK(Status,    static_cast<uint32_t>(Status::bitmask))
BITMASK_DEFINE_VALUE_MASK(Interrupt, static_cast<uint32_t>(Interrupt::bitmask))

} // namespace private_i2c




static constexpr uint32_t TARGET_I2C_MASK = (1UL << TARGET_I2C_COUNT) - 1;

class I2cDriver : private PeripheralRefCounter<I2cDriver, TARGET_I2C_COUNT, TARGET_I2C_MASK>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTION DECLARATION
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::I2C0_IRQHandler(void);
        friend void ::I2C1_IRQHandler(void);

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralI2c = PeripheralRefCounter<I2cDriver, TARGET_I2C_COUNT, TARGET_I2C_MASK>;

        struct MasterConfig
        {
            int32_t baudrate = 100000;
        };

        // Transfer status return codes
        enum class TransferStatus
        {
            ok               = 0,   // Transfer success
            busy             = 1,   // Bus is busy with current transfer
            /*idle             = 2,   // Bus is idle */
            nack_data        = 3,   // NACK received during transfer
            arbitration_lost = 4,   // Arbitration lost during transfer
            /*timeout          = 5,   // Timeout poling status flags */
            nack_address     = 6,   // NACK received during the address probe
            error                   // Unknown error condition
        };

        // Type safe accessor to status flags
        using Status        = private_i2c::Status;
        using StatusBitmask = bitmask::bitmask<Status>;

        // Type safe accessor to interrupt sources
        using Interrupt        = private_i2c::Interrupt;
        using InterruptBitmask = bitmask::bitmask<Interrupt>;

        // IRQ handler definition
        using IrqHandlerType = int32_t();
        using IrqHandler     = Delegate<IrqHandlerType>;

        // --------------------------------------------------------------------
        // PROTECTED MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONSTRUCTOR / DESTRUCTOR ----------------------------------

        // Master mode constructor
        I2cDriver(const PinDriver::Name master_sda,
                  const PinDriver::Name master_scl,
                  const MasterConfig&   master_config) : PeripheralI2c(*this, get_peripheral_index(master_sda, master_scl))
       {
            const auto pin_config = get_pin_config(master_sda, master_scl);

            PinDriver::set_mode(master_sda, PinDriver::FunctionMode::hiz, pin_config.pin_mux, PinDriver::SlewRate::fast, PinDriver::PassiveFilter::disable, PinDriver::OpenDrain::enable);
            PinDriver::set_mode(master_scl, PinDriver::FunctionMode::hiz, pin_config.pin_mux, PinDriver::SlewRate::fast, PinDriver::PassiveFilter::disable, PinDriver::OpenDrain::enable);

            m_i2c_name = pin_config.i2c_name;

            switch(m_i2c_name)
            {
                case Name::i2c0: m_i2c_base = I2C0; break;
                case Name::i2c1: m_i2c_base = I2C1; break;
            };

            initialize(master_config);

            disable_irq();

            /* FSL I2C_MasterInit function already do this
            // Clear all status bits
            clear_status(Status::clear_all_bitmask); */
        }

        // Slave mode constructor ...

        ~I2cDriver()
        {
            disable_irq();

            // Clear all status bits
            clear_status(Status::clear_all_bitmask);

            // NOTE: I2C_MasterDeinit() and I2C_SlaveDeinit() contents are the same
            I2C_MasterDeinit(m_i2c_base);
        }

        // -------- TRANSFER --------------------------------------------------

        // Master polling transfer (write and/or read)
        // - Only write:     tx_buffer.size >  0 && rx_buffer.size == 0
        // - Only read:      tx_buffer.size == 0 && rx_buffer.size >  0
        // - Write and read: tx_buffer.size >  0 && rx_buffer.size >  0 (in this case the tx_buffer is the command / sub-address (NOTE: max 4 bytes))
        // NOTES: - 7-bit slave address.
        //        - this method does not return until the transfer succeeds
        //          or fails due to arbitration lost or any error.
        TransferStatus master_transfer(const uint8_t slave_address, const std::span<uint8_t> tx_buffer, const std::span<uint8_t> rx_buffer)
        {
            const i2c_direction_t direction = (rx_buffer.size() == 0) ? kI2C_Write : kI2C_Read;

            uint32_t subaddress = 0;
            uint8_t  subaddress_size = 0;

            // Fill subaddress (transferred MSB first) if it's a read operation with command
            if ((tx_buffer.size() > 0) && (direction == kI2C_Read))
            {
                // NOTE: subaddress variable is an uint32_t type!
                assert(tx_buffer.size() <= 4);

                for(std::size_t byte_count = 0; byte_count < tx_buffer.size(); byte_count++)
                {
                    subaddress = (subaddress << 8) | tx_buffer[byte_count];
                }

                subaddress_size = tx_buffer.size();
            }

            i2c_master_transfer_t i2c_master_transfer =
            {
                kI2C_TransferDefaultFlag,   // A transfer starts with a start signal, stops with a stop signal
                slave_address,
                direction,
                subaddress,
                subaddress_size,
                (direction == kI2C_Write) ? tx_buffer.data() : rx_buffer.data(),
                (direction == kI2C_Write) ? tx_buffer.size() : rx_buffer.size()
            };

            const int32_t result = I2C_MasterTransferBlocking(m_i2c_base, &i2c_master_transfer);

            switch(result)
            {
                case kStatus_Success:             return TransferStatus::ok;
                case kStatus_I2C_Busy:            return TransferStatus::busy;
                case kStatus_I2C_Nak:             return TransferStatus::nack_data;
                case kStatus_I2C_ArbitrationLost: return TransferStatus::arbitration_lost;
                case kStatus_I2C_Addr_Nak:        return TransferStatus::nack_address;
                default:                          return TransferStatus::error;
            }
        }

        // -------- MASTER ENABLE / DISABLE -----------------------------------

        void master_enable()
        {
            enable();
        }

        void master_disable()
        {
            disable();
        }

        bool is_master_enabled() const
        {
            return is_master_enabled();
        }

        // -------- STATUS FLAGS ----------------------------------------------

        StatusBitmask get_status() const
        {
            // NOTE: I2C_MasterGetStatusFlags() and I2C_SlaveGetStatusFlags() are the same
            return static_cast<Status>(I2C_MasterGetStatusFlags(m_i2c_base));
        }

        void clear_status(const StatusBitmask bitmask)
        {
            // NOTE: I2C_MasterClearStatusFlags() and I2C_SlaveClearStatusFlags() are the same
            I2C_MasterClearStatusFlags(m_i2c_base, (bitmask & Status::clear_all_bitmask).bits());
        }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_interrupts(const InterruptBitmask bitmask)
        {
            I2C_EnableInterrupts(m_i2c_base, bitmask.bits());
        }

        void disable_interrupts(const InterruptBitmask bitmask)
        {
            I2C_DisableInterrupts(m_i2c_base, bitmask.bits());
        }

        InterruptBitmask get_interrupts_enabled() const
        {
            const uint8_t mask = (m_i2c_base->C1 & I2C_C1_IICIE_MASK) | (m_i2c_base->FLT & I2C_FLT_SSIE_MASK);

            return static_cast<Interrupt>(mask);
        }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()
        {
            switch(m_i2c_name)
            {
                case Name::i2c0: NVIC_EnableIRQ(I2C0_IRQn); break;
                case Name::i2c1: NVIC_EnableIRQ(I2C1_IRQn); break;
                default:                                    break;
            }
        }

        void disable_irq()
        {
            switch(m_i2c_name)
            {
                case Name::i2c0: NVIC_DisableIRQ(I2C0_IRQn); break;
                case Name::i2c1: NVIC_DisableIRQ(I2C1_IRQn); break;
                default:                                     break;
            }
        }

        bool is_irq_enabled()
        {
            switch(m_i2c_name)
            {
                case Name::i2c0: return (NVIC_GetEnableIRQ(I2C0_IRQn) != 0); break;
                case Name::i2c1: return (NVIC_GetEnableIRQ(I2C1_IRQn) != 0); break;
                default:         return false;
            }
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            switch(m_i2c_name)
            {
                case Name::i2c0: NVIC_SetPriority(I2C0_IRQn, irq_priority); break;
                case Name::i2c1: NVIC_SetPriority(I2C1_IRQn, irq_priority); break;
                default:                                                    break;
            }
        }

        void assign_irq_handler(const IrqHandler& irq_handler)
        {
            assert(irq_handler != nullptr);

            m_irq_handler = irq_handler;
        }

        void remove_irq_handler()
        {
            m_irq_handler = nullptr;
        }

    private:

        // --------------------------------------------------------------------
        // PRIVATE DEFINITIONS
        // --------------------------------------------------------------------

        enum class Name
        {
            i2c0 = 0,
            i2c1
        };

        struct PinConfig
        {
            Name              i2c_name;
            PinDriver::PinMux pin_mux;
        };

        // Pin map type
        using PinMap = std::tuple<PinDriver::Name, PinDriver::Name, PinConfig>;

        // Pin map array type
        template <std::size_t Size>
        using PinMapArray = std::array<PinMap, Size>;

        static constexpr std::size_t m_pin_map_array_size { (TARGET_PACKAGE_PIN_COUNT == 144) ? 13 : 11 };

        static constexpr PinMapArray<m_pin_map_array_size> m_pin_map_array
        { { //                   SDA                     SCL      PinConfig
              { PinDriver::Name::pe_0,  PinDriver::Name::pe_1,  { Name::i2c1, PinDriver::PinMux::alt6 } },
              { PinDriver::Name::pe_18, PinDriver::Name::pe_19, { Name::i2c0, PinDriver::PinMux::alt4 } },
              { PinDriver::Name::pe_25, PinDriver::Name::pe_24, { Name::i2c0, PinDriver::PinMux::alt5 } },
#if (TARGET_PACKAGE_PIN_COUNT == 144)
              { PinDriver::Name::pa_11, PinDriver::Name::pa_12, { Name::i2c0, PinDriver::PinMux::alt8 } },
#endif
              { PinDriver::Name::pa_13, PinDriver::Name::pa_14, { Name::i2c1, PinDriver::PinMux::alt8 } },
              { PinDriver::Name::pb_1,  PinDriver::Name::pb_0,  { Name::i2c0, PinDriver::PinMux::alt2 } },
              { PinDriver::Name::pb_3,  PinDriver::Name::pb_2,  { Name::i2c0, PinDriver::PinMux::alt2 } },
              { PinDriver::Name::pc_7,  PinDriver::Name::pc_6,  { Name::i2c0, PinDriver::PinMux::alt7 } },
              { PinDriver::Name::pc_11, PinDriver::Name::pc_10, { Name::i2c1, PinDriver::PinMux::alt2 } },
              { PinDriver::Name::pc_15, PinDriver::Name::pc_14, { Name::i2c1, PinDriver::PinMux::alt2 } },
              { PinDriver::Name::pc_15, PinDriver::Name::pc_14, { Name::i2c0, PinDriver::PinMux::alt3 } },
              { PinDriver::Name::pd_3,  PinDriver::Name::pd_2,  { Name::i2c0, PinDriver::PinMux::alt7 } },
#if (TARGET_PACKAGE_PIN_COUNT == 144)
              { PinDriver::Name::pd_9,  PinDriver::Name::pd_8,  { Name::i2c1, PinDriver::PinMux::alt2 } }
#endif
        } };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- CONFIGURATION / INITIALIZATION ----------------------------

        // Return the peripheral index according to the pins (constructor helper function):
        static constexpr int32_t get_peripheral_index(const PinDriver::Name sda, const PinDriver::Name scl)
        {
            const auto pin_config = get_pin_config(sda, scl);

            return static_cast<int32_t>(pin_config.i2c_name);
        }

        // Get the pin config struct if the specified SDA and SCL are I2C pins
        static constexpr PinConfig get_pin_config(const PinDriver::Name sda, const PinDriver::Name scl)
        {
            std::size_t index = 0;

            for(; index < m_pin_map_array.size(); ++index)
            {
                const auto pin_map = m_pin_map_array[index];

                if(std::get<0>(pin_map) == sda && std::get<1>(pin_map) == scl)
                {
                    return std::get<2>(pin_map);
                }
            }

            // Assert SDA and SCL are I2C pins
            assert(index < m_pin_map_array.size());

            return { Name::i2c0, PinDriver::PinMux::pin_disabled_or_analog };
        }

        // NOTE: implemented on the CPP file because it uses parameters from
        //       the library configuration file (xarmlib_config.h).
        void initialize(const MasterConfig& master_config);

        // -------- ENABLE / DISABLE ------------------------------------------

        // Enable the peripheral operation
        void enable()
        {
            I2C_Enable(m_i2c_base, true);
        }

        // Disable the peripheral operation
        void disable()
        {
            I2C_Enable(m_i2c_base, false);
        }

        // Get the enable state
        bool is_enabled() const
        {
            return ((m_i2c_base->C1 & I2C_C1_IICEN_MASK) != 0);
        }

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            int32_t yield = 0;  // User in FreeRTOS

            if(m_irq_handler != nullptr)
            {
                yield = m_irq_handler();
            }

            return yield;
        }

        // IRQ handler called directly by the interrupt C functions
        // NOTE: Returns yield flag for FreeRTOS
        static int32_t irq_handler(const Name name)
        {
            const auto index = static_cast<std::size_t>(name);

            return I2cDriver::get_reference(index).irq_handler();
        }

        // --------------------------------------------------------------------
        // PRIVATE MEMBER VARIABLES
        // --------------------------------------------------------------------

        Name       m_i2c_name;
        I2C_Type*  m_i2c_base { nullptr };  // Pointer to the CMSIS I2C structure
        IrqHandler m_irq_handler;           // User defined IRQ handler
};




} // namespace kv5x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_KV5X_I2C_HPP

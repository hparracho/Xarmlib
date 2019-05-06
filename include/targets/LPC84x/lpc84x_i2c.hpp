// ----------------------------------------------------------------------------
// @file    lpc84x_i2c.hpp
// @brief   NXP LPC84x I2C class.
// @note    Only master mode is implemented.
// @date    6 May 2019
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

#ifndef __XARMLIB_TARGETS_LPC84X_I2C_HPP
#define __XARMLIB_TARGETS_LPC84X_I2C_HPP

#include "external/bitmask.hpp"
#include "targets/LPC84x/lpc84x_pin.hpp"
#include "targets/LPC84x/lpc84x_swm.hpp"
#include "targets/LPC84x/lpc84x_syscon_clock.hpp"
#include "targets/LPC84x/lpc84x_syscon_power.hpp"
#include "core/delegate.hpp"
#include "core/peripheral_ref_counter.hpp"




// Forward declaration of IRQ handlers for both LPC844 and LPC845
extern "C" void I2C0_IRQHandler(void);
extern "C" void I2C1_IRQHandler(void);

#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
// Forward declaration of additional IRQ handlers for LPC845
extern "C" void I2C2_IRQHandler(void);
extern "C" void I2C3_IRQHandler(void);
#endif




namespace xarmlib
{
namespace targets
{
namespace lpc84x
{




namespace private_i2c
{

// I2C Status Register (STAT) bits
enum class Status
{
    master_pending            = (1 << 0),   // Master Pending Status Bit
                                            // NOTE: the master state code reflects the master
                                            //       state when the master_pending bit is set
    master_state_mask         = (7 << 1),   // Master State Code Mask
    /*master_state_idle         = (0 << 1),   // Master Idle State Code
    master_state_rx_ready     = (1 << 1),   // Master Receive Ready State Code
    master_state_tx_ready     = (2 << 1),   // Master Transmit Ready State Code
    master_state_nack_address = (3 << 1),   // Master NACK by slave on address State Code
    master_state_nack_data    = (4 << 1),   // Master NACK by slave on data State Code */
    master_arbitration_lost   = (1 << 4),   // Master Arbitration Lost Bit
    master_start_stop_error   = (1 << 6),   // Master Start Stop Error Bit
    slave_pending             = (1 << 8),   // Slave Pending Status Bit
    slave_state_mask          = (3 << 9),   // Slave State Code Mask
    /*slave_state_address       = (0 << 9),   // Slave address State Code
    slave_state_rx            = (1 << 9),   // Slave receive State Code
    slave_state_tx            = (2 << 9),   // Slave transmit State Code */
    slave_not_stretching      = (1 << 11),  // Slave not stretching Clock Bit
    slave_address_match       = (3 << 12),  // Slave Address Match Index
    slave_selected            = (1 << 14),  // Slave Selected Bit
    slave_deselect            = (1 << 15),  // Slave Deselect Bit
    monitor_ready             = (1 << 16),  // Monitor Ready Bit
    monitor_overflow          = (1 << 17),  // Monitor Overflow Flag
    monitor_active            = (1 << 18),  // Monitor Active Flag
    monitor_idle              = (1 << 19),  // Monitor Idle Flag
    event_timeout             = (1 << 24),  // Event Timeout Interrupt Flag
    scl_timeout               = (1 << 25),  // SCL Timeout Interrupt Flag
    clear_all_bitmask         = master_arbitration_lost
                              | master_start_stop_error
                              | slave_deselect
                              | monitor_overflow
                              | monitor_idle
                              | event_timeout
                              | scl_timeout,
    bitmask                   = master_pending
                              | master_state_mask
                              | master_arbitration_lost
                              | master_start_stop_error
                              | slave_pending
                              | slave_state_mask
                              | slave_not_stretching
                              | slave_address_match
                              | slave_selected
                              | slave_deselect
                              | monitor_ready
                              | monitor_overflow
                              | monitor_active
                              | monitor_idle
                              | event_timeout
                              | scl_timeout
};

// I2C Interrupt Register (INTENSET and INTENCLR) bits
enum class Interrupt
{
    master_pending          = (1 << 0),     // Master Pending Interrupt Bit
    master_arbitration_lost = (1 << 4),     // Master Arbitration Lost Interrupt Bit
    master_start_stop_error = (1 << 6),     // Master Start Stop Error Interrupt Bit
    slave_pending           = (1 << 8),     // Slave Pending Interrupt Bit
    slave_not_stretching    = (1 << 11),    // Slave not stretching Clock Interrupt Bit
    slave_deselect          = (1 << 15),    // Slave Deselect Interrupt Bit
    monitor_ready           = (1 << 16),    // Monitor Ready Interrupt Bit
    monitor_overflow        = (1 << 17),    // Monitor Overflow Interrupt Bit
    monitor_idle            = (1 << 19),    // Monitor Idle Interrupt Bit
    event_timeout           = (1 << 24),    // Event Timeout Interrupt Bit
    scl_timeout             = (1 << 25),    // SCL Timeout Interrupt Bit
    bitmask                 = master_pending
                            | master_arbitration_lost
                            | master_start_stop_error
                            | slave_pending
                            | slave_not_stretching
                            | slave_deselect
                            | monitor_ready
                            | monitor_overflow
                            | monitor_idle
                            | event_timeout
                            | scl_timeout
};

BITMASK_DEFINE_VALUE_MASK(Status,    static_cast<uint32_t>(Status::bitmask))
BITMASK_DEFINE_VALUE_MASK(Interrupt, static_cast<uint32_t>(Interrupt::bitmask))

} // namespace private_i2c




static constexpr uint32_t TARGET_I2C0_MASK  =  1; // Fixed mask
static constexpr  int32_t TARGET_I2C0_INDEX =  0; // Fixed index
static constexpr  int32_t TARGET_I2C_INDEX  = -1; // Automatic index given by PeripheralRefCounter

class I2cDriver : private PeripheralRefCounter<I2cDriver, TARGET_I2C_COUNT, TARGET_I2C0_MASK>
{
        // --------------------------------------------------------------------
        // FRIEND FUNCTIONS DECLARATIONS
        // --------------------------------------------------------------------

        // Friend IRQ handler C functions to give access to private IRQ handler member function
        friend void ::I2C0_IRQHandler(void);
        friend void ::I2C1_IRQHandler(void);
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
        friend void ::I2C2_IRQHandler(void);
        friend void ::I2C3_IRQHandler(void);
#endif

    protected:

        // --------------------------------------------------------------------
        // PROTECTED DEFINITIONS
        // --------------------------------------------------------------------

        // Base class alias
        using PeripheralI2c = PeripheralRefCounter<I2cDriver, TARGET_I2C_COUNT, TARGET_I2C0_MASK>;

        struct MasterConfig
        {
            int32_t max_frequency = 100000;
        };

        // Transfer status return codes
        enum class TransferStatus
        {
            ok = 0,             // Request was executed successfully
            error,              // Unknown error condition
            nack_address,       // No acknowledgment received from slave during address phase
            bus_error,          // Bus error
            nack_data,          // No acknowledgment received from slave during data phase
            arbitration_lost,   // Arbitration lost
            busy                // Transmitter is busy
        };

        // Type safe accessor to STAT register
        using Status        = private_i2c::Status;
        using StatusBitmask = bitmask::bitmask<Status>;

        // Type safe accessor to INTENSET / INTENCLR registers
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
        // NOTE: p0_11 and p0_10 are true open drain pins
        I2cDriver(const PinDriver::Name master_sda,
                  const PinDriver::Name master_scl,
                  const MasterConfig&   master_config) : PeripheralI2c(*this, get_peripheral_index(master_sda, master_scl))
        {
            assert(master_config.max_frequency > 0);

            const Name name = static_cast<Name>(get_index());

            if(name == Name::i2c0)
            {
                assert(master_config.max_frequency <= 1000000);

                // p0_11 and p0_10 are fixed true open drain pins
                initialize(master_config.max_frequency);
            }
            else
            {
                assert(master_config.max_frequency <= 400000);

                initialize(master_sda, master_scl);
            }

            // Set supplied maximum frequency
            set_frequency(master_config.max_frequency);
        }

        // Slave mode constructor ...

        ~I2cDriver()
        {
            master_disable();

            const Name name = static_cast<Name>(get_index());

            // Disable peripheral clock sources and interrupts
            switch(name)
            {
                case Name::i2c0: ClockDriver::disable(ClockDriver::Peripheral::i2c0); NVIC_DisableIRQ(I2C0_IRQn); break;
                case Name::i2c1: ClockDriver::disable(ClockDriver::Peripheral::i2c1); NVIC_DisableIRQ(I2C1_IRQn); break;
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
                case Name::i2c2: ClockDriver::disable(ClockDriver::Peripheral::i2c2); NVIC_DisableIRQ(I2C2_IRQn); break;
                case Name::i2c3: ClockDriver::disable(ClockDriver::Peripheral::i2c3); NVIC_DisableIRQ(I2C3_IRQn); break;
#endif
                default:                                                                                          break;
            }
        }

        // -------- TRANSFER --------------------------------------------------

        // Master polling transfer (write and/or read)
        // - Only write:     tx_buffer.size >  0 && rx_buffer.size == 0
        // - Only read:      tx_buffer.size == 0 && rx_buffer.size >  0
        // - Write and read: tx_buffer.size >  0 && rx_buffer.size >  0 (in this case the tx_buffer is the command / sub-address)
        // NOTES: - 7-bit slave address.
        //        - this method does not return until the transfer succeeds
        //          or fails due to arbitration lost or any error.
        TransferStatus master_transfer(const uint8_t slave_address, const std::span<uint8_t> tx_buffer, const std::span<uint8_t> rx_buffer)
        {
            // Clear controller state
            clear_status(Status::master_arbitration_lost | Status::master_start_stop_error);

            const TransferDirection direction = (tx_buffer.size() == 0) ? TransferDirection::master_read : TransferDirection::master_write;

            assert(direction == TransferDirection::master_read && rx_buffer.size() > 0);

            std::ptrdiff_t buffer_count = 0;

            // Write slave address and RW bit to data register
            master_write_data(static_cast<uint8_t>((slave_address << 1)) | static_cast<uint8_t>(direction));

            // Enter to master transmitter mode
            send_start();

            TransferStatus result = TransferStatus::busy;

            while(result == TransferStatus::busy)
            {
                // wait for status change
                while(is_master_pending() == false);

                const StatusBitmask status = get_status();

                if(status & Status::master_arbitration_lost)
                {
                    result = TransferStatus::arbitration_lost;
                    clear_status(Status::master_arbitration_lost);
                }
                else if(status & Status::master_start_stop_error)
                {
                    result = TransferStatus::bus_error;
                    clear_status(Status::master_start_stop_error);
                }
                else if(status & Status::master_pending)
                {
                    // Branch based on master state code
                    switch(get_master_state())
                    {
                        case MasterState::idle: break; // Master job is done
                        case MasterState::rx_ready:

                            rx_buffer[buffer_count++] = master_read_data();

                            if(buffer_count < rx_buffer.size())
                            {
                                // Set Continue if there is more data to read
                                master_continue();
                            }
                            else
                            {
                                result = TransferStatus::ok;

                                // No data to read send Stop
                                send_stop();
                            }
                            break;

                        case MasterState::tx_ready:

                            if(buffer_count < tx_buffer.size())
                            {
                                // If Tx data available transmit data and continue
                                master_write_data(tx_buffer[buffer_count++]);

                                master_continue();
                            }
                            else
                            {
                                // If receive queued after transmit then initiate master receive transfer
                                if(rx_buffer.size() > 0)
                                {
                                    buffer_count = 0;

                                    // Write slave address and RW bit to data register
                                    master_write_data(static_cast<uint8_t>((slave_address << 1)) | static_cast<uint8_t>(TransferDirection::master_read));

                                    // Enter to Master Transmitter mode
                                    send_start();
                                }
                                else
                                {
                                    // If no receive queued then set transfer status as OK
                                    result = TransferStatus::ok;

                                    send_stop();
                                }
                            }
                            break;

                        case MasterState::nack_address: result = TransferStatus::nack_address; send_stop(); break;
                        case MasterState::nack_data:    result = TransferStatus::nack_data;    send_stop(); break;
                        default:                        result = TransferStatus::error;                     break; // Default case should not occur
                    }
                }
                else
                {
                    // Default case should not occur
                    result = TransferStatus::error;
                }
            }

            return result;
        }

        // -------- MASTER ENABLE / DISABLE -----------------------------------

        void master_enable()
        {
            m_i2c->CFG |= cfg_master_enable;
        }

        void master_disable()
        {
            m_i2c->CFG &= ~cfg_master_enable;
        }

        bool is_master_enabled() const
        {
            return (m_i2c->CFG & cfg_master_enable) != 0;
        }

        // -------- STATUS FLAGS ----------------------------------------------

        StatusBitmask get_status() const
        {
            return static_cast<Status>(m_i2c->STAT);
        }

        void clear_status(const StatusBitmask bitmask)
        {
            m_i2c->STAT = (bitmask & Status::clear_all_bitmask).bits();
        }

        // -------- INTERRUPTS ------------------------------------------------

        void enable_interrupts(const InterruptBitmask bitmask)
        {
            m_i2c->INTENSET = bitmask.bits();
        }

        void disable_interrupts(const InterruptBitmask bitmask)
        {
            m_i2c->INTENCLR = bitmask.bits();
        }

        InterruptBitmask get_interrupts_enabled() const
        {
            return static_cast<Interrupt>(m_i2c->INTSTAT);
        }

        // -------- IRQ / IRQ HANDLER -----------------------------------------

        void enable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::i2c0: NVIC_EnableIRQ(I2C0_IRQn); break;
                case Name::i2c1: NVIC_EnableIRQ(I2C1_IRQn); break;
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
                case Name::i2c2: NVIC_EnableIRQ(I2C2_IRQn); break;
                case Name::i2c3: NVIC_EnableIRQ(I2C3_IRQn); break;
#endif
                default:                                    break;
            }
        }

        void disable_irq()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::i2c0: NVIC_DisableIRQ(I2C0_IRQn); break;
                case Name::i2c1: NVIC_DisableIRQ(I2C1_IRQn); break;
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
                case Name::i2c2: NVIC_DisableIRQ(I2C2_IRQn); break;
                case Name::i2c3: NVIC_DisableIRQ(I2C3_IRQn); break;
#endif
                default:                                     break;
            }
        }

        bool is_irq_enabled()
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::i2c0: return (NVIC_GetEnableIRQ(I2C0_IRQn) != 0); break;
                case Name::i2c1: return (NVIC_GetEnableIRQ(I2C1_IRQn) != 0); break;
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
                case Name::i2c2: return (NVIC_GetEnableIRQ(I2C2_IRQn) != 0); break;
                case Name::i2c3: return (NVIC_GetEnableIRQ(I2C3_IRQn) != 0); break;
#endif
                default:         return false;                               break;
            }
        }

        void set_irq_priority(const int32_t irq_priority)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::i2c0: NVIC_SetPriority(I2C0_IRQn, irq_priority); break;
                case Name::i2c1: NVIC_SetPriority(I2C1_IRQn, irq_priority); break;
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
                case Name::i2c2: NVIC_SetPriority(I2C2_IRQn, irq_priority); break;
                case Name::i2c3: NVIC_SetPriority(I2C3_IRQn, irq_priority); break;
#endif
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

        // I2C peripheral names selection
        enum class Name
        {
            i2c0 = 0,
            i2c1,
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
            i2c2,
            i2c3
#endif
        };

        // Transfer direction selection
        enum class TransferDirection
        {
            master_write = 0,   // Master transmits to the slave
            master_read         // Master receives from the slave
        };

        // Master state code
        enum class MasterState
        {
            idle         = (0 << 1),    // Master Idle State Code
            rx_ready     = (1 << 1),    // Master Receive Ready State Code
            tx_ready     = (2 << 1),    // Master Transmit Ready State Code
            nack_address = (3 << 1),    // Master NACK by slave on address State Code
            nack_data    = (4 << 1)     // Master NACK by slave on data State Code
        };

        // Slave state code
        /*enum class SlaveState
        {
            address       = (0 << 9),   // Slave address State Code
            rx            = (1 << 9),   // Slave receive State Code
            tx            = (2 << 9)    // Slave transmit State Code
        }; */

        // I2C Configuration Register (CFG) bits
        enum CFG : uint32_t
        {
            cfg_master_enable            = (1 << 0),    // Master Enable/Disable Bit
            cfg_slave_enable             = (1 << 1),    // Slave Enable/Disable Bit
            cfg_monitor_enable           = (1 << 2),    // Monitor Enable/Disable Bit
            cfg_timeout_enable           = (1 << 3),    // Timeout Enable/Disable Bit
            cfg_monitor_clock_stretching = (1 << 4)     // Monitor Clock Stretching Bit
        };

        // I2C Master Control Register (MSTCTL) bits
        enum MSTCTL : uint32_t
        {
            mstctl_master_continue = (1 << 0),  // Master Continue Bit
            mstctl_master_start    = (1 << 1),  // Master Start Control Bit
            mstctl_master_stop     = (1 << 2),  // Master Stop Control Bit
            mstctl_master_dma      = (1 << 3)   // Master DMA Enable Bit
        };

        // --------------------------------------------------------------------
        // PRIVATE MEMBER FUNCTIONS
        // --------------------------------------------------------------------

        // -------- INITIALIZATION / CONFIGURATION ----------------------------

        // Return the peripheral index according to the pins (constructor helper function):
        // I2C0 (true open drain pins)    ->  0
        // I2C1/I2C2/I2C3 (standard pins) -> -1 (automatic index given by the PeripheralRefCounter)
        static constexpr int32_t get_peripheral_index(const PinDriver::Name sda, const PinDriver::Name scl)
        {
            if(sda == PinDriver::Name::p0_11 && scl == PinDriver::Name::p0_10)
            {
                return TARGET_I2C0_INDEX;
            }
            else
            {
                assert(sda != PinDriver::Name::p0_11 && scl != PinDriver::Name::p0_10);

                return TARGET_I2C_INDEX;
            }
        }

        // Initialize I2C0 peripheral structure and the fixed true open drain pins (constructor helper function)
        void initialize(const int32_t max_frequency)
        {
            // Set pointer to the I2C0 structure
            m_i2c = LPC_I2C0;

            ClockDriver::enable(ClockDriver::Peripheral::i2c0);
            PowerDriver::reset(PowerDriver::ResetPeripheral::i2c0);

            ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::i2c0, ClockDriver::PeripheralClockSource::main_clk);

            SwmDriver::enable(SwmDriver::PinFixed::i2c0_sda);
            SwmDriver::enable(SwmDriver::PinFixed::i2c0_scl);

            const PinDriver::I2cMode i2c_mode = (max_frequency <= 400000) ? PinDriver::I2cMode::standard_fast_i2c : PinDriver::I2cMode::fast_plus_i2c;

            PinDriver::set_mode(PinDriver::Name::p0_11, i2c_mode, PinDriver::InputFilter::bypass, PinDriver::InputInvert::normal);
            PinDriver::set_mode(PinDriver::Name::p0_10, i2c_mode, PinDriver::InputFilter::bypass, PinDriver::InputInvert::normal);
        }

        // Initialize I2C1/I2C2/I2C3 peripheral structure and the standard pins (constructor helper function)
        void initialize(const PinDriver::Name sda, const PinDriver::Name scl)
        {
            const Name name = static_cast<Name>(get_index());

            switch(name)
            {
                case Name::i2c1:
                {
                    // Set pointer to the available I2C structure
                    m_i2c = LPC_I2C1;

                    ClockDriver::enable(ClockDriver::Peripheral::i2c1);
                    PowerDriver::reset(PowerDriver::ResetPeripheral::i2c1);

                    ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::i2c1, ClockDriver::PeripheralClockSource::main_clk);

                    SwmDriver::assign(SwmDriver::PinMovable::i2c1_sda_io, sda);
                    SwmDriver::assign(SwmDriver::PinMovable::i2c1_scl_io, scl);

                }   break;
#if (TARGET_I2C_COUNT == 4) /* __LPC845__ */
                case Name::i2c2:
                {
                    // Set pointer to the available I2C structure
                    m_i2c = LPC_I2C2;

                    ClockDriver::enable(ClockDriver::Peripheral::i2c2);
                    PowerDriver::reset(PowerDriver::ResetPeripheral::i2c2);

                    ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::i2c2, ClockDriver::PeripheralClockSource::main_clk);

                    SwmDriver::assign(SwmDriver::PinMovable::i2c2_sda_io, sda);
                    SwmDriver::assign(SwmDriver::PinMovable::i2c2_scl_io, scl);

                }   break;

                case Name::i2c3:
                {
                    // Set pointer to the available I2C structure
                    m_i2c = LPC_I2C3;

                    ClockDriver::enable(ClockDriver::Peripheral::i2c3);
                    PowerDriver::reset(PowerDriver::ResetPeripheral::i2c3);

                    ClockDriver::set_peripheral_clock_source(ClockDriver::PeripheralClockSelect::i2c3, ClockDriver::PeripheralClockSource::main_clk);

                    SwmDriver::assign(SwmDriver::PinMovable::i2c3_sda_io, sda);
                    SwmDriver::assign(SwmDriver::PinMovable::i2c3_scl_io, scl);

                }   break;
#endif

                default: break;
            }

            PinDriver::set_mode(sda, PinDriver::FunctionMode::hiz, PinDriver::OpenDrain::enable);
            PinDriver::set_mode(scl, PinDriver::FunctionMode::hiz, PinDriver::OpenDrain::enable);
        }

        // Set I2C maximum frequency (constructor helper function)
        // NOTE: due to the nature of the I2C bus, it will set a base frequency
        //       that represents the fastest that the I2C bus could operate if
        //       nothing slows it down
        void set_frequency(const int32_t max_frequency)
        {
            const int32_t clock_freq = ClockDriver::get_main_clock_frequency();

            // NOTES:
            // SCL time (SCL high time + SCL low time): scl      = clock_freq / (clkdiv * max_frequency)
            // Nominal SCL rate:                        scl_freq = clock_freq / (clkdiv * (SCL high time + SCL low time))
            // clkdiv value: [1 - 65536]
            // SCL high time value: [2 - 9]
            // SCL low  time value: [2 - 9]

            const int32_t scl_min = 4;

            assert(max_frequency >= (clock_freq / (18 * 65536)) &&  max_frequency <= (clock_freq / scl_min));

            int32_t clkdiv = (clock_freq / (scl_min * max_frequency));

            while(clkdiv >= 1)
            {
                // Find the SCL integer value
                if((clock_freq % (clkdiv * max_frequency)) == 0)
                {
                    break;
                }

                clkdiv--;
            }

            // Configure the I2C clock divider
            m_i2c->CLKDIV = (clkdiv - 1) & 0xFFFF;

            // Calculate the SCL integer value
            const uint32_t scl = clock_freq / (clkdiv * max_frequency);

            // Set HIGH and LOW duty cycle
            const uint32_t scl_high = scl >> 1;
            const uint32_t scl_low  = scl - scl_high;

            m_i2c->MSTTIME = (((scl_high - 2) & 0x07) << 4) | ((scl_low - 2) & 0x07);
        }

        // -------- TRANSFER --------------------------------------------------

        // Send a START or Repeat-START signal on I2C bus
        void send_start()
        {
            m_i2c->MSTCTL = mstctl_master_start;
        }

        // Send a STOP signal on I2C bus
        void send_stop()
        {
            m_i2c->MSTCTL = mstctl_master_stop;
        }

        // Inform the Master function to continue to the next operation
        void master_continue()
        {
            m_i2c->MSTCTL = mstctl_master_continue;
        }

        // Read data that has been received in master mode
        uint8_t master_read_data() const
        {
            return static_cast<uint8_t>(m_i2c->MSTDAT);
        }

        // Write data to be transmitted in master mode
        void master_write_data(const uint8_t value)
        {
            m_i2c->MSTDAT = static_cast<uint32_t>(value);
        }

        // -------- STATUS FLAGS ----------------------------------------------

        bool        is_master_pending() const { return (get_status() & Status::master_pending) != 0; }
        MasterState get_master_state()  const { return static_cast<MasterState>((get_status() & Status::master_state_mask).bits()); }

        // -------- PRIVATE IRQ HANDLERS --------------------------------------

        // IRQ handler private implementation (call user IRQ handler)
        int32_t irq_handler()
        {
            if(m_irq_handler != nullptr)
            {
                return m_irq_handler();
            }

            return 0;
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

        LPC_I2C_T* m_i2c { nullptr };   // Pointer to the CMSIS I2C structure
        IrqHandler m_irq_handler;       // User defined IRQ handler
};




} // namespace lpc84x
} // namespace targets
} // namespace xarmlib

#endif // __XARMLIB_TARGETS_LPC84X_I2C_HPP

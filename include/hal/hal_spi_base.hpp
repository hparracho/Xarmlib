// ----------------------------------------------------------------------------
// @file    hal_spi_base.hpp
// @brief   SPI HAL interface classes (SpiMaster / SpiSlave).
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

#ifndef XARMLIB_HAL_SPI_BASE_HPP
#define XARMLIB_HAL_SPI_BASE_HPP

#include "api/api_high_res_clock.hpp"
#include "core/mutex.hpp"
#include "core/target_specs.hpp"
#include "hal/hal_peripheral_irq_multi.hpp"

#include <span>




namespace xarmlib::hal
{

template <typename Driver, typename Interrupt>
using SpiPeripheral = PeripheralIrqMulti<Driver, Interrupt, TARGET_SPI_COUNT>;

template <typename Driver, typename Traits>
class SpiBase : public SpiPeripheral<Driver, typename Traits::Interrupt>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using Status    = typename Traits::Status;
    using Interrupt = typename Traits::Interrupt;

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONFIGURATION -------------------------------------------------

    int32_t get_frequency() const
    { return static_cast<const Driver*>(this)->get_frequency(); }

    void set_frequency(const int32_t max_frequency)
    { static_cast<Driver*>(this)->set_frequency(max_frequency); }

    // -------- ENABLE / DISABLE ----------------------------------------------

    void enable()
    { static_cast<Driver*>(this)->enable(); }

    void disable()
    { static_cast<Driver*>(this)->disable(); }

    bool is_enabled() const
    { return static_cast<const Driver*>(this)->is_enabled(); }

    bool is_master() const
    { return static_cast<const Driver*>(this)->is_master(); }

    // -------- STATUS FLAGS --------------------------------------------------

    bool is_rx_ready() const
    { return static_cast<const Driver*>(this)->is_rx_ready(); }

    bool is_tx_ready() const
    { return static_cast<const Driver*>(this)->is_tx_ready(); }

    Bitmask<Status> get_status() const
    { return static_cast<const Driver*>(this)->get_status(); }

    void clear_status(const Bitmask<Status> bitmask)
    { static_cast<Driver*>(this)->clear_status(bitmask); }

    // -------- READ / WRITE --------------------------------------------------

    // Read data as soon as possible with infinite timeout
    uint32_t read() const
    {
        while(is_rx_ready() == false);

        return static_cast<const Driver*>(this)->read_data();
    }

    // Read data as soon as possible with timeout
    bool read(uint32_t& data, const chrono::microseconds_u32 timeout_us) const
    {
        const auto start = HighResClock::now();

        while(HighResClock::is_timeout(start, timeout_us) == false)
        {
            if(is_rx_ready())
            {
                data = static_cast<const Driver*>(this)->read_data();
                return true;
            }
        }

        return false;
    }

    // Write data as soon as possible with infinite timeout
    void write(const uint32_t data)
    {
        while(is_tx_ready() == false);

        static_cast<Driver*>(this)->write_data(data);
    }

    // Write data as soon as possible with timeout (returns true if successful or false if failed)
    bool write(const uint32_t data, const chrono::microseconds_u32 timeout_us)
    {
        const auto start = HighResClock::now();

        while(HighResClock::is_timeout(start, timeout_us) == false)
        {
            if(is_tx_ready())
            {
                static_cast<Driver*>(this)->write_data(data);
                return true;
            }
        }

        return false;
    }

    // -------- TRANSFER ------------------------------------------------------

    // Transfer (write data and wait reply) with infinite timeout.
    // Return the data read value.
    uint32_t transfer(const uint32_t data)
    {
        write(data);
        return read();
    }

    // Transfer a buffer (write data and wait reply) with infinite timeout. The
    // read values will be placed on the same buffer, destroying the original data.
    void transfer(const std::span<uint8_t> buffer)
    {
        for(auto& data : buffer)
        {
            data = static_cast<uint8_t>(transfer(data));
        }
    }

    // Transfer a buffer (write data and wait reply) with infinite timeout. Send
    // the data on the 'tx_buffer' and put the received data on the 'rx_buffer'.
    void transfer(const std::span<const uint8_t> tx_buffer, const std::span<uint8_t> rx_buffer)
    {
        assert(tx_buffer.size() == rx_buffer.size());

        for(std::size_t index = 0; index < tx_buffer.size(); ++index)
        {
            rx_buffer[index] = static_cast<uint8_t>(transfer(tx_buffer[index]));
        }
    }

    // -------- MUTEX ---------------------------------------------------------

    Lockable& get_mutex()
    {
        return *m_mutex;
    }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    SpiBase(Driver& driver, const bool enable_mutex)
        : SpiPeripheral<Driver, typename Traits::Interrupt>(driver),
          m_mutex {make_mutex(enable_mutex)}
    {}

private:

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    static std::unique_ptr<Lockable> make_mutex(const bool enable_mutex)
    {
        std::unique_ptr<Lockable> mutex = (enable_mutex) ? std::make_unique<Mutex>()
                                                         : std::make_unique<MutexDummy>();

        assert(mutex != nullptr);

        return mutex;
    }

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    // Pointer to the mutex that guards the shared bus access (dummy or not)
    std::unique_ptr<Lockable> m_mutex;
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_SPI_BASE_HPP

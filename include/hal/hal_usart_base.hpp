// ----------------------------------------------------------------------------
// @file    hal_usart_base.hpp
// @brief   UART HAL interface class.
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

#ifndef XARMLIB_HAL_USART_BASE_HPP
#define XARMLIB_HAL_USART_BASE_HPP

#include "api/api_high_res_clock.hpp"
#include "core/target_specs.hpp"
#include "hal/hal_peripheral_irq_multi.hpp"

#include <span>




namespace xarmlib::hal
{

template <typename Driver, typename Interrupt>
using UsartPeripheral = PeripheralIrqMulti<Driver, Interrupt, TARGET_USART_COUNT>;

template <typename Driver, typename Traits>
class UsartBase : public UsartPeripheral<Driver, typename Traits::Interrupt>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using DataBits  = typename Traits::DataBits;
    using StopBits  = typename Traits::StopBits;
    using Parity    = typename Traits::Parity;
    using Config    = typename Traits::Config;

    using Status    = typename Traits::Status;
    using Interrupt = typename Traits::Interrupt;

    // --------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // --------------------------------------------------------------------

    // -------- FORMAT / BAUDRATE ---------------------------------------------

    void set_format(const DataBits data_bits, const StopBits stop_bits, const Parity parity)
    { static_cast<Driver*>(this)->set_format(data_bits, stop_bits, parity); }

    void set_data_bits(const DataBits data_bits)
    { static_cast<Driver*>(this)->set_data_bits(data_bits); }

    void set_stop_bits(const StopBits stop_bits)
    { static_cast<Driver*>(this)->set_stop_bits(stop_bits); }

    void set_parity(const Parity parity)
    { static_cast<Driver*>(this)->set_parity(parity); }

    void set_baudrate(const int32_t baudrate)
    { static_cast<Driver*>(this)->set_baudrate( baudrate); }

    // -------- ENABLE / DISABLE ----------------------------------------------

    void enable()
    { static_cast<Driver*>(this)->enable(); }

    void disable()
    { static_cast<Driver*>(this)->disable(); }

    bool is_enabled() const
    { return static_cast<const Driver*>(this)->is_enabled(); }

    // -------- STATUS FLAGS --------------------------------------------------

    bool is_rx_ready() const
    { return static_cast<const Driver*>(this)->is_rx_ready(); }

    bool is_rx_idle() const
    { return static_cast<const Driver*>(this)->is_rx_idle(); }

    bool is_tx_ready() const
    { return static_cast<const Driver*>(this)->is_tx_ready(); }

    bool is_tx_idle() const
    { return static_cast<const Driver*>(this)->is_tx_idle(); }

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

    // Read a byte buffer as soon as possible with timeout (returns the number of actual read bytes)
    std::size_t read(const std::span<uint8_t> buffer, const chrono::microseconds_u32 timeout_us) const
    {
        const auto start = HighResClock::now();
        std::size_t count = 0;

        while(count < buffer.size() && HighResClock::is_timeout(start, timeout_us) == false)
        {
            if(is_rx_ready())
            {
                buffer[count] = static_cast<const Driver*>(this)->read_data();
                count++;
            }
        }

        return count;
    }

    // Write data as soon as possible with infinite timeout
    void write(const uint32_t value)
    {
        while(is_tx_ready() == false);

        static_cast<Driver*>(this)->write_data(value);
    }

    // Write data as soon as possible with timeout (returns true if successful or false if failed)
    bool write(const uint32_t value, const chrono::microseconds_u32 timeout_us)
    {
        const auto start = HighResClock::now();

        while(HighResClock::is_timeout(start, timeout_us) == false)
        {
            if(is_tx_ready())
            {
                static_cast<Driver*>(this)->write_data(value);
                return true;
            }
        }

        return false;
    }

    // Write a byte buffer as soon as possible with timeout (returns the number of actual written bytes)
    std::size_t write(const std::span<const uint8_t> buffer, const chrono::microseconds_u32 timeout_us)
    {
        const auto start = HighResClock::now();
        std::size_t count = 0;

        while(count < buffer.size() && HighResClock::is_timeout(start, timeout_us) == false)
        {
            if(is_tx_ready())
            {
                static_cast<Driver*>(this)->write_data(buffer[count]);
                count++;
            }
        }

        return count;
    }

    // Write a C string (null terminated) as soon as possible with infinite timeout
    void write(const char* str)
    {
        while(*str != '\0')
        {
            if(is_tx_ready())
            {
                static_cast<Driver*>(this)->write_data(*str++);
            }
        }
    }

    // Write a C string (null terminated) with timeout (returns the number of actual written characters)
    std::size_t write(const char* str, const chrono::microseconds_u32 timeout_us)
    {
        const auto start = HighResClock::now();

        const char *ptr = str;

        while(*ptr != '\0' && HighResClock::is_timeout(start, timeout_us) == false)
        {
            if(is_tx_ready())
            {
                static_cast<Driver*>(this)->write_data(*ptr++);
            }
        }

        return (ptr - str);
    }

    void putc(char ch)
    {
        write(ch);
    }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    UsartBase(Driver& driver_ref) : UsartPeripheral<Driver, typename Traits::Interrupt>(driver_ref)
    {}
};

} // namespace xarmlib::hal




#endif // XARMLIB_HAL_USART_BASE_HPP

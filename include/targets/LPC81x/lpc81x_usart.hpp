// ----------------------------------------------------------------------------
// @file    lpc81x_usart.hpp
// @brief   NXP LPC81x USART class (synchronous mode not implemented).
// @date    7 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_TARGETS_LPC81X_USART_HPP
#define XARMLIB_TARGETS_LPC81X_USART_HPP

#include "xarmlib_config.hpp"

#include "core/bitmask.hpp"
#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_pin.hpp"
#include "targets/LPC81x/lpc81x_swm.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "hal/hal_usart_base.hpp"

#include <cmath>




// Forward declaration of IRQ handlers
extern "C" void USART0_IRQHandler(void);
extern "C" void USART1_IRQHandler(void);
extern "C" void USART2_IRQHandler(void);




namespace xarmlib::targets::lpc81x
{

struct UsartTraits
{
    // Data length selection (defined to map the CFG register directly)
    enum class DataBits
    {
        bits_7  = (0UL << 2),   // USART 7 bit data mode
        bits_8  = (1UL << 2),   // USART 8 bit data mode
        bits_9  = (2UL << 2)    // USART 9 bit data mode
    };

    // Stop bits selection (defined to map the CFG register directly)
    enum class StopBits
    {
        bits_1  = (0UL << 6),   // USART 1 stop bit
        bits_2  = (1UL << 6)    // USART 2 stop bits
    };

    // Parity selection (defined to map the CFG register directly)
    enum class Parity
    {
        none    = (0UL << 4),   // USART no parity
        even    = (2UL << 4),   // USART even parity select
        odd     = (3UL << 4)    // USART odd parity select
    };

    struct Config
    {
        int32_t  baudrate  = 9600;
        DataBits data_bits = DataBits::bits_8;
        StopBits stop_bits = StopBits::bits_1;
        Parity   parity    = Parity::none;
    };

    // USART Status register (STAT) bits
    enum class Status
    {
        rx_ready            = (1UL << 0),   // Receiver ready
        rx_idle             = (1UL << 1),   // Receiver idle
        tx_ready            = (1UL << 2),   // Transmitter ready for data
        tx_idle             = (1UL << 3),   // Transmitter idle
        cts                 = (1UL << 4),   // Status of CTS signal
        cts_delta           = (1UL << 5),   // Change in CTS state
        tx_disabled         = (1UL << 6),   // Transmitter disabled
        rx_overrun          = (1UL << 8),   // Overrun Error interrupt flag
        rx_break            = (1UL << 10),  // Received break
        rx_break_delta      = (1UL << 11),  // Change in receive break detection
        start               = (1UL << 12),  // Start detected
        frame_error         = (1UL << 13),  // Framing Error interrupt flag
        parity_error        = (1UL << 14),  // Parity Error interrupt flag
        rx_noise            = (1UL << 15),  // Received Noise interrupt flag
        clear_all_bitmask   = 0xF920UL,     // Clear all bitmask (1111'1001'0010'0000)
        bitmask             = 0xFD7FUL      // Full bitmask (1111'1101'0111'1111)
    };

    // USART Interrupt Enable Get, Set or Clear Register (INTSTAT / INTENSET / INTENCLR) bits
    enum class Interrupt
    {
        rx_ready            = (1UL << 0),   // Receiver ready
        tx_ready            = (1UL << 2),   // Transmitter ready for data
        cts_delta           = (1UL << 5),   // Change in CTS state
        tx_disabled         = (1UL << 6),   // Transmitter disabled
        rx_overrun          = (1UL << 8),   // Overrun Error interrupt flag
        rx_break_delta      = (1UL << 11),  // Change in receive break detection
        start               = (1UL << 12),  // Start detected
        frame_error         = (1UL << 13),  // Framing Error interrupt flag
        parity_errort       = (1UL << 14),  // Parity Error interrupt flag
        rx_noise            = (1UL << 15),  // Received Noise interrupt flag
        clear_all_bitmask   = 0xF920UL,     // Clear all bitmask (1111'1001'0010'0000)
        bitmask             = 0xF965UL      // Full bitmask (1111'1001'0110'0101)
    };
};




class Usart : public hal::UsartBase<Usart, UsartTraits>
{
    // Give IRQ handler C functions access to private member functions
    friend void ::USART0_IRQHandler(void);
    friend void ::USART1_IRQHandler(void);
    friend void ::USART2_IRQHandler(void);

    // Give HAL access to private member functions
    friend hal::UsartBase<Usart, UsartTraits>;

public:

    // --------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // --------------------------------------------------------------------

    using DataBits  = typename UsartTraits::DataBits;
    using StopBits  = typename UsartTraits::StopBits;
    using Parity    = typename UsartTraits::Parity;
    using Config    = typename UsartTraits::Config;

    using Status    = typename UsartTraits::Status;
    using Interrupt = typename UsartTraits::Interrupt;

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTOR / DESTRUCTOR --------------------------------------

    Usart(const Pin::Name txd, const Pin::Name rxd, const Config& config = Config{})
        : hal::UsartBase<Usart, UsartTraits>(*this)
    {
        // Configure USART clock if this is the first USART peripheral instantiation
        if(m_ref_counter.get_use_count() == 1)
        {
            // Set USART clock divider to 1 for USART FRG clock-in to be equal to the main clock
            SysClock::set_usart_clock_divider(1);

            initialize_usart_frg();
        }

        const auto uart_index = m_ref_counter.get_this_index();

        SysClock::enable(static_cast<SysClock::Peripheral>(static_cast<std::size_t>(SysClock::Peripheral::usart0) + uart_index));
        Power::reset(static_cast<Power::ResetPeripheral>(static_cast<std::size_t>(Power::ResetPeripheral::usart0) + uart_index));

        switch(static_cast<Name>(uart_index))
        {
            case Name::usart0: m_usart = LPC_USART0;
                               Swm::assign(Swm::PinMovable::u0_rxd_i, rxd);
                               Swm::assign(Swm::PinMovable::u0_txd_o, txd);
                               break;

            case Name::usart1: m_usart = LPC_USART1;
                               Swm::assign(Swm::PinMovable::u1_rxd_i, rxd);
                               Swm::assign(Swm::PinMovable::u1_txd_o, txd);
                               break;
#if (TARGET_USART_COUNT == 3)
            case Name::usart2: m_usart = LPC_USART2;
                               Swm::assign(Swm::PinMovable::u2_rxd_i, rxd);
                               Swm::assign(Swm::PinMovable::u2_txd_o, txd);
                               break;
#endif
        };

        Pin::set_mode(txd, Pin::FunctionMode::hiz);
        Pin::set_mode(rxd, Pin::FunctionMode::pull_up);

        // No continuous break, no address detect, no Tx disable and no continuous clock generation.
        m_usart->CTL = 0;

        set_format(config.data_bits, config.stop_bits, config.parity);
        set_baudrate(config.baudrate);

        // Clear all status bits
        clear_status(Status::clear_all_bitmask);
    }

#if !defined(XARMLIB_DISABLE_DESTRUCTORS) || (XARMLIB_DISABLE_DESTRUCTORS == 0)
    ~Usart()
    {
        // Disable peripheral
        disable();

        SysClock::disable(static_cast<SysClock::Peripheral>(static_cast<std::size_t>(SysClock::Peripheral::usart0) + m_ref_counter.get_this_index()));

        // Disable USART clock if this the last USART peripheral deleted
        if(m_ref_counter.get_use_count() == 1)
        {
            SysClock::set_usart_clock_divider(0);
        }
    }
#endif

    // -------- FORMAT / BAUDRATE ---------------------------------------------

    void set_format(const DataBits data_bits, const StopBits stop_bits, const Parity parity)
    {
        set_data_bits(data_bits);
        set_stop_bits(stop_bits);
        set_parity(parity);
    }

    void set_data_bits(const DataBits data_bits)
    {
        const bool enabled = is_enabled();

        if(enabled) { disable(); }

        // Set new data bits
        m_usart->CFG = (m_usart->CFG & ~cfg_datalen_bitmask) | static_cast<uint32_t>(data_bits);

        // If previously enabled, re-enable.
        if(enabled) { enable(); }
    }

    void set_stop_bits(const StopBits stop_bits)
    {
        const bool enabled = is_enabled();

        if(enabled) { disable(); }

        // Set new stop bits
        m_usart->CFG = (m_usart->CFG & ~cfg_stoplen_bitmask) | static_cast<uint32_t>(stop_bits);

        // If previously enabled, re-enable.
        if(enabled) { enable(); }
    }

    void set_parity(const Parity parity)
    {
        const bool enabled = is_enabled();

        if(enabled) { disable(); }

        // Set new parity
        m_usart->CFG = (m_usart->CFG & ~cfg_parity_bitmask) | static_cast<uint32_t>(parity);

        // If previously enabled, re-enable.
        if(enabled) { enable(); }
    }

    void set_baudrate(const int32_t baudrate)
    {
        assert(baudrate > 0);

        const bool enabled = is_enabled();

        if(enabled) { disable(); }

        const int32_t div = get_baudrate_generator_div(baudrate);
        assert(div >= 1 && div <= 65536);

        // Set baudrate generator register
        m_usart->BRG = div - 1;

        // If previously enabled, re-enable.
        if(enabled) { enable(); }
    }

    // -------- ENABLE / DISABLE ----------------------------------------------

    // Enable peripheral
    void enable() { m_usart->CFG |= cfg_enable; }

    // Disable peripheral (wait until both RX and TX are idle)
    void disable()
    {
        wait_until_idle();
        m_usart->CFG &= ~cfg_enable;
    }

    // Check if peripheral is enabled
    bool is_enabled() const { return (m_usart->CFG & cfg_enable) != 0; }

    // -------- STATUS FLAGS --------------------------------------------------

    bool is_rx_ready() const { return (get_status() & Status::rx_ready) != 0; }
    bool is_rx_idle () const { return (get_status() & Status::rx_idle ) != 0; }
    bool is_tx_ready() const { return (get_status() & Status::tx_ready) != 0; }
    bool is_tx_idle () const { return (get_status() & Status::tx_idle ) != 0; }

    // Check if both RX and TX are idle
    bool is_idle() const {return (get_status() & Status::rx_idle & Status::tx_idle) != 0; }

    // Wait until both RX and TX are idle
    void wait_until_idle() const { while(is_idle() == false) {} }

    Bitmask<Status> get_status() const
    {
        return Bitmask<Status>(m_usart->STAT);
    }

    void clear_status(const Bitmask<Status> bitmask)
    {
        m_usart->STAT = (bitmask & Status::clear_all_bitmask);
    }

    // -------- INTERRUPTS ----------------------------------------------------

    void enable_interrupts(const Bitmask<Interrupt> bitmask)
    {
        m_usart->INTENSET = bitmask;
    }

    void disable_interrupts(const Bitmask<Interrupt> bitmask)
    {
        m_usart->INTENCLR = bitmask;
    }

    Bitmask<Interrupt> get_interrupts_enabled() const
    {
        return Bitmask<Interrupt>(m_usart->INTSTAT);
    }

    Bitmask<Interrupt> get_interrupts_pending() const
    {
        return Bitmask<Interrupt>(m_usart->STAT);
    }

    void clear_interrupts_pending()
    {
        m_usart->STAT = static_cast<uint32_t>(Interrupt::clear_all_bitmask);
    }

    // -------- IRQ -----------------------------------------------------------

    // Get the IRQ name associated with this peripheral
    IRQn_Type get_irq_name() const
    {
        return static_cast<IRQn_Type>(USART0_IRQn + m_ref_counter.get_this_index());
    }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- NAMES ---------------------------------------------------------

    SysClock::Peripheral get_clock_name() const
    {
        const auto usart0 = static_cast<std::size_t>(SysClock::Peripheral::usart0);

        return static_cast<SysClock::Peripheral>(usart0 + m_ref_counter.get_this_index());
    }

    Power::ResetPeripheral get_power_name() const
    {
        const auto usart0 = static_cast<std::size_t>(Power::ResetPeripheral::usart0);

        return static_cast<Power::ResetPeripheral>(usart0 + m_ref_counter.get_this_index());
    }

    // -------- RAW READ / WRITE ----------------------------------------------

    // Read data that has been received
    uint32_t read_data() const
    {
        // Strip off undefined reserved bits, keep 9 lower bits.
        return m_usart->RXDAT & 0x000001FF;
    }

    // Write data to be transmitted
    void write_data(const uint32_t value)
    {
        // Strip off undefined reserved bits, keep 9 lower bits.
        m_usart->TXDAT = value & 0x000001FF;
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE DEFINITIONS
    // ------------------------------------------------------------------------

    // USART peripheral names selection
    enum class Name
    {
        usart0 = 0,
        usart1,
#if (TARGET_USART_COUNT == 3)
        usart2
#endif
    };

    // USART Configuration Register (CFG) bits and masks
    enum CFG : uint32_t
    {
        cfg_enable          = (1 << 0),     // USART enable
        cfg_datalen_bitmask = (3 << 2),     // USART data mode bitmask
        cfg_stoplen_bitmask = (1 << 6),     // USART stop bits bitmask
        cfg_parity_bitmask  = (3 << 4)      // USART parity bitmask
    };

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- USART FRG CONFIGURATION ---------------------------------------

    // NOTE: USART FRG clock-in is equal to the main clock due to the USART clock divider was previously set to 1.

    // Return the maximum USART frequency that can be used to generate a
    // standard baudrate frequency for the supplied main clock frequency.
    static constexpr int32_t get_max_standard_frequency(const int32_t main_clk_freq)
    {
        // 58.982400MHz is the maximum attainable frequency from a 60MHz clock source
        int32_t max_standard_freq = 58982400;

        while(max_standard_freq > main_clk_freq)
        {
            max_standard_freq /= 2;
        }

        return max_standard_freq;
    }

    // Return the FRG MUL value for the supplied USART and main clock frequencies
    static constexpr uint8_t get_frg_mul(const int32_t usart_freq, const int64_t main_clk_freq)
    {
        const auto mul = std::round((main_clk_freq * 256 / usart_freq) - 256);

        return static_cast<uint8_t>(mul);
    }

    // Configure the USART FRG that is shared by all USART peripherals
    void initialize_usart_frg()
    {
        constexpr int32_t main_clk_freq = System::get_main_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK);
        constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

        constexpr uint8_t mul = get_frg_mul(usart_freq, main_clk_freq);
        constexpr uint8_t div = 0xFF; // Fixed value to use with the fractional baudrate generator

        // Set the USART FRG fractional divider
        SysClock::set_usart_frg_clock_divider(mul, div);
    }

    // Return the USART frequency divider (baudrate generator divider) to obtain the supplied baudrate frequency
    static constexpr int32_t get_baudrate_generator_div(const int32_t baudrate)
    {
        constexpr int32_t main_clk_freq = System::get_main_clock_frequency(XARMLIB_CONFIG_SYSTEM_CLOCK);
        constexpr int32_t usart_freq = get_max_standard_frequency(main_clk_freq);

        return usart_freq / 16 / baudrate;
    }

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    LPC_USART_T*    m_usart {nullptr};  // Pointer to the CMSIS USART structure
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_USART_HPP

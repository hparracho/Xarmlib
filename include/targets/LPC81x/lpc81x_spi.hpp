// ----------------------------------------------------------------------------
// @file    lpc81x_spi.hpp
// @brief   NXP LPC81x SPI class.
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

#ifndef XARMLIB_TARGETS_LPC81X_SPI_HPP
#define XARMLIB_TARGETS_LPC81X_SPI_HPP

#include "xarmlib_config.hpp"

#include "external/bitmask.hpp"
#include "core/mutex.hpp"
#include "targets/LPC81x/lpc81x_cmsis.hpp"
#include "targets/LPC81x/lpc81x_pin.hpp"
#include "targets/LPC81x/lpc81x_swm.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "hal/hal_spi_base.hpp"




// Forward declaration of IRQ handlers
extern "C" void SPI0_IRQHandler(void);
extern "C" void SPI1_IRQHandler(void);




namespace xarmlib::targets::lpc81x
{

struct SpiTraits
{
    // Numbered operating mode selection (defined to map the CFG register directly)
    enum class Mode
    {
        //          CPOL   |     CPHA
        mode0 = (0UL << 5) | (0UL << 4),    // CPOL = 0 | CPHA = 0
        mode1 = (0UL << 5) | (1UL << 4),    // CPOL = 0 | CPHA = 1
        mode2 = (1UL << 5) | (0UL << 4),    // CPOL = 1 | CPHA = 0
        mode3 = (1UL << 5) | (1UL << 4)     // CPOL = 1 | CPHA = 1
    };

    // Data length selection (defined to map the TXCTL register directly)
    enum class DataBits
    {
        bit_1   = (0x00 << 24),
        bits_2  = (0x01 << 24),
        bits_3  = (0x02 << 24),
        bits_4  = (0x03 << 24),
        bits_5  = (0x04 << 24),
        bits_6  = (0x05 << 24),
        bits_7  = (0x06 << 24),
        bits_8  = (0x07 << 24),
        bits_9  = (0x08 << 24),
        bits_10 = (0x09 << 24),
        bits_11 = (0x0a << 24),
        bits_12 = (0x0b << 24),
        bits_13 = (0x0c << 24),
        bits_14 = (0x0d << 24),
        bits_15 = (0x0e << 24),
        bits_16 = (0x0f << 24)
    };

    // Data order selection (defined to map the CFG register directly)
    enum class DataOrder
    {
        msb_first = (0UL << 3),
        lsb_first = (1UL << 3)
    };

    // Slave select polarity selection (defined to map the CFG register directly)
    enum class SselPolarity
    {
        low  = (0UL << 8),
        high = (1UL << 8)
    };

    // Loopback mode selection (defined to map the CFG register directly)
    enum class LoopbackMode
    {
        disabled = (0UL << 7),
        enabled  = (1UL << 7)
    };

    struct MasterConfig
    {
        int32_t      max_frequency = 1000000;
        Mode         mode          = Mode::mode0;
        DataBits     data_bits     = DataBits::bits_8;
        DataOrder    data_order    = DataOrder::msb_first;
        LoopbackMode loopback_mode = LoopbackMode::disabled;
    };

    struct SlaveConfig
    {
        Mode         mode          = Mode::mode0;
        DataBits     data_bits     = DataBits::bits_8;
        DataOrder    data_order    = DataOrder::msb_first;
        SselPolarity ssel_polarity = SselPolarity::low;
        LoopbackMode loopback_mode = LoopbackMode::disabled;
    };

    // SPI Status register (STAT) bits
    // NOTE: Used to read all the available status register bits. Some of then are
    //       read-only. The mask 'clear_all_bitmask' have all the bits of the status
    //       register that can be cleared by writing 1 (W1). The status register
    //       contains all the possible interrupt and additional status flags.
    enum class Status
    {
        rx_ready          = (1UL << 0), // RO - Receiver Ready flag
        tx_ready          = (1UL << 1), // RO - Transmitter Ready flag
        rx_overrun        = (1UL << 2), // W1 - Receiver Overrun interrupt flag (applies only to slave mode)
        tx_underrun       = (1UL << 3), // W1 - Transmitter Underrun interrupt flag (applies only to slave mode)
        ssel_assert       = (1UL << 4), // W1 - Slave Select Assert flag
        ssel_deassert     = (1UL << 5), // W1 - Slave Select Deassert flag
        stalled           = (1UL << 6), // RO - Stalled status flag
        end_transfer      = (1UL << 7), // W1 - End Transfer control bit
        master_idle       = (1UL << 8), // RO - Master idle status flag
        clear_all_bitmask = 0x00BCUL,   // W1 - Clear all bitmask (0000'0000'1011'1100)
        bitmask           = 0x01FFUL    //      Full bitmask      (0000'0001'1111'1111)
    };

    // SPI Interrupt enumeration
    // NOTE: Used to enable (INTENSET), disable (INTENCLR) and get enabled (INTSTAT) interrupts.
    enum class Interrupt
    {
        rx_ready          = (1UL << 0), // Receiver data is available
        tx_ready          = (1UL << 1), // Transmitter holding register is available
        rx_overrun        = (1UL << 2), // Receiver overrun occurs (applies only to slave mode)
        tx_underrun       = (1UL << 3), // Transmitter underrun occurs (applies only to slave mode)
        ssel_assert       = (1UL << 4), // Slave Select is asserted
        ssel_deassert     = (1UL << 5), // Slave Select is deasserted
        bitmask           = 0x003FUL    // Full bitmask (0000'0000'0011'1111)
    };
};




class Spi : public hal::SpiBase<Spi, SpiTraits>
{
    // Give IRQ handler C functions access to private member functions
    friend void ::SPI0_IRQHandler(void);
    friend void ::SPI1_IRQHandler(void);

    // Give HAL access to private member functions
    friend hal::SpiBase<Spi, SpiTraits>;

public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using Mode         = typename SpiTraits::Mode;
    using DataBits     = typename SpiTraits::DataBits;
    using DataOrder    = typename SpiTraits::DataOrder;
    using SselPolarity = typename SpiTraits::SselPolarity;
    using LoopbackMode = typename SpiTraits::LoopbackMode;

    using MasterConfig = typename SpiTraits::MasterConfig;
    using SlaveConfig  = typename SpiTraits::SlaveConfig;

    using Status       = typename SpiTraits::Status;
    using Interrupt    = typename SpiTraits::Interrupt;

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTOR / DESTRUCTOR --------------------------------------

    // Master mode constructor
    Spi(const Pin::Name mosi, const Pin::Name miso, const Pin::Name sck,
        const MasterConfig& config = MasterConfig{}, const bool enable_mutex = false)
        : hal::SpiBase<Spi, SpiTraits>(*this, enable_mutex)
   {
        // Initialize peripheral structure and pins
        initialize(mosi, miso, sck, Pin::Name::nc);

        // Configure data format and operating modes
        set_configuration(Operation::master, config.mode, config.data_bits,
                          config.data_order, SselPolarity::low, config.loopback_mode);

        // Set supplied maximum frequency
        set_frequency(config.max_frequency);
    }

    // Slave mode constructor
    Spi(const Pin::Name mosi, const Pin::Name miso, const Pin::Name sck, const Pin::Name ssel,
        const SlaveConfig& config = SlaveConfig{}) : hal::SpiBase<Spi, SpiTraits>(*this, false)
    {
        assert(ssel != Pin::Name::nc);

        // Initialize peripheral structure and pins
        initialize(mosi, miso, sck, ssel);

        // Configure data format and operating modes
        set_configuration(Operation::slave, config.mode, config.data_bits,
                          config.data_order, config.ssel_polarity, config.loopback_mode);

        // In slave mode, the clock is taken from the SCK input and the SPI clock divider is not used!
        // set_frequency(...);
    }

#if (XARMLIB_DISABLE_DESTRUCTORS != 1)
    ~Spi()
    {
        // Disable peripheral
        disable();

        SysClock::disable(get_clock_name());
    }
#endif

    // -------- FREQUENCY -------------------------------------------------

    // Get the current frequency
    // NOTE: Function only available in master mode!
    int32_t get_frequency() const
    {
        assert(is_master());

        return m_frequency;
    }

    // Set the maximum frequency
    // NOTE: If the specified maximum frequency cannot be obtained it will
    //       set the closest frequency that is below the target frequency.
    void set_frequency(const int32_t max_frequency)
    {
        assert(is_master());

        const int32_t clock_freq = SysClock::get_system_clock_frequency();

        assert(max_frequency >= (clock_freq / 65536) &&  max_frequency <= clock_freq);

        // Integer ceiling of clock_freq / max_frequency
        const int32_t divval = (clock_freq / max_frequency) + ((clock_freq % max_frequency) != 0);

        const bool enabled = is_enabled();

        if(enabled) { disable(); }

        // Configure the SPI clock divider
        // NOTE: DIVVAL is -1 encoded such that the value 0 results in PCLK/1, the
        //       value 1 results in PCLK/2, up to the maximum possible divide value
        //       of 0xFFFF, which results in PCLK/65536.
        m_spi->DIV = (divval - 1) & 0xFFFF;

        m_frequency = max_frequency;

        // If previously enabled, re-enable.
        if(enabled) { enable(); }
    }

    // -------- ENABLE / DISABLE ----------------------------------------------

    // Enable peripheral
    void enable() { m_spi->CFG |= cfg_enable; }

    // Disable peripheral
    void disable()
    {
        if(is_master()) { wait_until_master_idle(); }

        m_spi->CFG &= ~cfg_enable;
    }

    // Check if peripheral is enabled
    bool is_enabled() const { return (m_spi->CFG & cfg_enable) != 0; }

    // Check if peripheral is configured as master
    bool is_master() const { return (m_spi->CFG & cfg_master) != 0; }

    // -------- STATUS FLAGS --------------------------------------------------

    bool is_rx_ready() const { return (get_status() & Status::rx_ready) != 0; }
    bool is_tx_ready() const { return (get_status() & Status::tx_ready) != 0; }

    // Check master idle status
    bool is_master_idle() const {return (get_status() & Status::master_idle) != 0; }

    // Wait until master is idle
    void wait_until_master_idle() const { while(is_master_idle() == false) {} }

    Bitmask<Status> get_status() const
    {
        return Bitmask<Status>(m_spi->STAT);
    }

    void clear_status(const Bitmask<Status> bitmask)
    {
        m_spi->STAT = (bitmask & Status::clear_all_bitmask);
    }

    // -------- INTERRUPTS ----------------------------------------------------

    void enable_interrupts(const Bitmask<Interrupt> bitmask)
    {
        m_spi->INTENSET = bitmask;
    }

    void disable_interrupts(const Bitmask<Interrupt> bitmask)
    {
        m_spi->INTENCLR = bitmask;
    }

    Bitmask<Interrupt> get_interrupts_enabled() const
    {
        return Bitmask<Interrupt>(m_spi->INTSTAT);
    }

    Bitmask<Interrupt> get_interrupts_pending() const
    {
        return Bitmask<Interrupt>(m_spi->STAT);
    }

    void clear_interrupts_pending()
    {
        m_spi->STAT = Bitmask<Interrupt>(Interrupt::bitmask);
    }

    // -------- IRQ -----------------------------------------------------------

    // Get the IRQ name associated with this peripheral
    IRQn_Type get_irq_name() const
    {
        return static_cast<IRQn_Type>(SPI0_IRQn + m_ref_counter.get_this_index());
    }

protected:

    // ------------------------------------------------------------------------
    // PROTECTED MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- NAMES ---------------------------------------------------------

    SysClock::Peripheral get_clock_name() const
    {
        const auto spi0 = static_cast<std::size_t>(SysClock::Peripheral::spi0);

        return static_cast<SysClock::Peripheral>(spi0 + m_ref_counter.get_this_index());
    }

    Power::ResetPeripheral get_power_name() const
    {
        const auto spi0 = static_cast<std::size_t>(SysClock::Peripheral::spi0);

        return static_cast<Power::ResetPeripheral>(spi0 + m_ref_counter.get_this_index());
    }

    // -------- RAW READ / WRITE ----------------------------------------------

    // Read data that has been received
    uint32_t read_data() const
    {
        // Strip off undefined reserved bits, keep 16 lower bits.
        return m_spi->RXDAT & 0x0000FFFF;
    }

    // Write data to be transmitted
    void write_data(const uint32_t value)
    {
        // Strip off undefined reserved bits, keep 16 lower bits.
        m_spi->TXDAT = value & 0x0000FFFF;
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE DEFINITIONS
    // ------------------------------------------------------------------------

    // SPI peripheral names selection
    enum class Name
    {
        spi0 = 0,
#if (TARGET_SPI_COUNT == 2)
        spi1
#endif
    };

    // SPI Configuration Register (CFG) bits
    enum CFG : uint32_t
    {
        cfg_enable = (1UL << 0),
        cfg_master = (1UL << 2),
        cfg_lsbf   = (1UL << 3),
        cfg_cpha   = (1UL << 4),
        cfg_cpol   = (1UL << 5),
        cfg_loop   = (1UL << 7),
        cfg_spol   = (1UL << 8)
    };

    // Master / slave operation selection (defined to map the CFG register directly)
    enum class Operation
    {
        slave  = (0UL << 2),
        master = (1UL << 2)
    };

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- INITIALIZATION / CONFIGURATION --------------------------------

    // Initialize SPI peripheral structure and pins (constructor helper function)
    void initialize(const Pin::Name mosi,
                    const Pin::Name miso,
                    const Pin::Name sck,
                    const Pin::Name ssel)
    {
        SysClock::enable(get_clock_name());
        Power::reset(get_power_name());

        switch(static_cast<Name>(m_ref_counter.get_this_index()))
        {
            case Name::spi0:
            {
                m_spi = LPC_SPI0;
                Swm::assign(Swm::PinMovable::spi0_mosi_io, mosi);
                Swm::assign(Swm::PinMovable::spi0_miso_io, miso);
                Swm::assign(Swm::PinMovable::spi0_sck_io, sck);

                if(ssel != Pin::Name::nc)
                {
                    Swm::assign(Swm::PinMovable::spi0_ssel_io, ssel);
                }
            }   break;

#if (TARGET_SPI_COUNT == 2)
            case Name::spi1:
            {
                m_spi = LPC_SPI1;
                SwmDriver::assign(SwmDriver::PinMovable::spi1_mosi_io, mosi);
                SwmDriver::assign(SwmDriver::PinMovable::spi1_miso_io, miso);
                SwmDriver::assign(SwmDriver::PinMovable::spi1_sck_io, sck);

                if(ssel != Pin::Name::nc)
                {
                    Swm::assign(Swm::PinMovable::spi1_ssel_io, ssel);
                }
            }   break;
#endif
        }
    }

    // Configure SPI data format and operating modes (constructor helper function)
    void set_configuration(const Operation    operation,
                           const Mode         mode,
                           const DataBits     data_bits,
                           const DataOrder    data_order,
                           const SselPolarity ssel_polarity,
                           const LoopbackMode loopback_mode)
    {
        // Set the MASTER, LSBF, CPHA, CPOL, LOOP and SPOL0 bits
        m_spi->CFG = static_cast<uint32_t>(operation    )
                   | static_cast<uint32_t>(data_order   )
                   | static_cast<uint32_t>(mode         )
                   | static_cast<uint32_t>(loopback_mode)
                   | static_cast<uint32_t>(ssel_polarity);

        // Set the LEN bits
        m_spi->TXCTL = static_cast<uint32_t>(data_bits);

        // Configure the SPI delay register (DLY)
        // Pre-delay = 0 clocks / post-delay = 0 clocks / frame-delay = 0 clocks / transfer-delay = 0 clocks
        m_spi->DLY = 0x0000;

        disable_interrupts(Interrupt::bitmask);
    }

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    LPC_SPI_T*  m_spi {nullptr};    // Pointer to the CMSIS SPI structure
    int32_t     m_frequency {0};    // User defined frequency
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_SPI_HPP

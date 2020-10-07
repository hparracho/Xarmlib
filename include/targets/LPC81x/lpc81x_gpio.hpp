// ----------------------------------------------------------------------------
// @file    lpc81x_gpio.hpp
// @brief   NXP LPC81x GPIO class.
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

#ifndef XARMLIB_TARGETS_LPC81X_GPIO_HPP
#define XARMLIB_TARGETS_LPC81X_GPIO_HPP

#include "targets/LPC81x/lpc81x_pin.hpp"
#include "targets/LPC81x/lpc81x_specs.hpp"
#include "targets/LPC81x/lpc81x_syscon_clock.hpp"
#include "targets/LPC81x/lpc81x_syscon_power.hpp"
#include "hal/hal_gpio_base.hpp"




namespace xarmlib::targets::lpc81x
{

struct GpioTraits
{
    // Pin type alias
    using InputFilter     = Pin::InputFilter;
    using InputInvert     = Pin::InputInvert;
    using InputHysteresis = Pin::InputHysteresis;

    // Input pin modes
    enum class InputMode
    {
        hiz = 0,
        pull_down,
        pull_up,
        repeater
    };

    struct InputModeConfig
    {
        InputMode       input_mode       = InputMode::pull_up;
        InputFilter     input_filter     = InputFilter::bypass;
        InputInvert     input_invert     = InputInvert::normal;
        InputHysteresis input_hysteresis = InputHysteresis::enable;
    };

    // Output pin modes
    enum class OutputMode
    {
        push_pull_low = 0,
        push_pull_high,
        open_drain_low,
        open_drain_hiz
    };

    struct OutputModeConfig
    {
        OutputMode output_mode = OutputMode::push_pull_high;
    };

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    struct InputModeTrueOpenDrainConfig
    {
        // Input mode: HIZ
        InputFilter input_filter = InputFilter::bypass;
        InputInvert input_invert = InputInvert::normal;
    };

    // True open-drain output pin modes
    enum class OutputModeTrueOpenDrain
    {
        low = 0,
        hiz
    };

    struct OutputModeTrueOpenDrainConfig
    {
        OutputModeTrueOpenDrain output_mode = OutputModeTrueOpenDrain::hiz;
    };
#endif
};




class Gpio : public hal::GpioBase<Gpio, GpioTraits>
{
public:

    // ------------------------------------------------------------------------
    // PUBLIC DEFINITIONS
    // ------------------------------------------------------------------------

    using InputFilter     = typename GpioTraits::InputFilter;
    using InputInvert     = typename GpioTraits::InputInvert;
    using InputHysteresis = typename GpioTraits::InputHysteresis;

    using InputMode        = typename GpioTraits::InputMode;
    using InputModeConfig  = typename GpioTraits::InputModeConfig;

    using OutputMode       = typename GpioTraits::OutputMode;
    using OutputModeConfig = typename GpioTraits::OutputModeConfig;

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    using InputModeTrueOpenDrainConfig  = typename GpioTraits::InputModeTrueOpenDrainConfig;

    using OutputModeTrueOpenDrain       = typename GpioTraits::OutputModeTrueOpenDrain;
    using OutputModeTrueOpenDrainConfig = typename GpioTraits::OutputModeTrueOpenDrainConfig;
#endif

    // ------------------------------------------------------------------------
    // PUBLIC MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    // -------- CONSTRUCTORS --------------------------------------------------

    // Default constructor (assign a NC pin)
    Gpio() = default;

    // Normal input pin constructor (not available on pins P0_10 and P0_11)
    Gpio(const Pin::Name pin_name, const InputModeConfig& config)
        : hal::GpioBase<Gpio, GpioTraits>(pin_name)
    {
        if(pin_name != Pin::Name::nc)
        {
            config_pin();
            set_mode(config);
        }
    }

    // Normal output pin constructor (not available on pins P0_10 and P0_11)
    Gpio(const Pin::Name pin_name, const OutputModeConfig& config)
        : hal::GpioBase<Gpio, GpioTraits>(pin_name)
    {
        if(pin_name != Pin::Name::nc)
        {
            config_pin();
            set_mode(config);
        }
    }

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    // True open-drain input pin constructor (only available on pins P0_10 and P0_11)
    Gpio(const Pin::Name pin_name, const InputModeTrueOpenDrainConfig& config)
        : hal::GpioBase<Gpio, GpioTraits>(pin_name)
    {
        if(pin_name != Pin::Name::nc)
        {
            config_pin();
            set_mode(config);
        }
    }

    // True open-drain output pin constructor (only available on pins P0_10 and P0_11)
    Gpio(const Pin::Name pin_name, const OutputModeTrueOpenDrainConfig& config)
        : hal::GpioBase<Gpio, GpioTraits>(pin_name)
    {
        if(pin_name != Pin::Name::nc)
        {
            config_pin();
            set_mode(config);
        }
    }
#endif

    // -------- CONFIGURATION -------------------------------------------------

    // Set normal input pin mode
    void set_mode(const InputModeConfig& config)
    {
        // Exclude NC
        assert(m_pin_name != Pin::Name::nc);

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
        // Exclude open-drain pins
        assert(m_pin_name != Pin::Name::p0_10 && m_pin_name != Pin::Name::p0_11);
#endif

        Pin::FunctionMode function_mode;

        switch(config.input_mode)
        {
            case InputMode::hiz:       function_mode = Pin::FunctionMode::hiz;       break;
            case InputMode::pull_down: function_mode = Pin::FunctionMode::pull_down; break;
            case InputMode::repeater:  function_mode = Pin::FunctionMode::repeater;  break;
            case InputMode::pull_up:
            default:                   function_mode = Pin::FunctionMode::pull_up;   break;
        }

        write(0);
        set_direction(Direction::input);

        Pin::set_mode(m_pin_name, function_mode,
                              Pin::OpenDrain::disable,
                              config.input_filter,
                              config.input_invert,
                              config.input_hysteresis);
    }

    // Set normal output pin mode
    void set_mode(const OutputModeConfig& config)
    {
        // Exclude NC
        assert(m_pin_name != Pin::Name::nc);

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
        // Exclude open-drain pins
        assert(m_pin_name != Pin::Name::p0_10 && m_pin_name != Pin::Name::p0_11);
#endif

        uint32_t       pin_value;
        Pin::OpenDrain open_drain;

        switch(config.output_mode)
        {
            case OutputMode::push_pull_low:  pin_value = 0; open_drain = Pin::OpenDrain::disable; break;
            case OutputMode::open_drain_low: pin_value = 0; open_drain = Pin::OpenDrain::enable;  break;
            case OutputMode::open_drain_hiz: pin_value = 1; open_drain = Pin::OpenDrain::enable;  break;
            case OutputMode::push_pull_high:
            default:                         pin_value = 1; open_drain = Pin::OpenDrain::disable; break;
        }

        write(pin_value);
        set_direction(Direction::output);

        Pin::set_mode(m_pin_name, Pin::FunctionMode::hiz,
                                  open_drain,
                                  Pin::InputFilter::bypass,
                                  Pin::InputInvert::normal,
                                  Pin::InputHysteresis::enable);
    }

#if (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)
    // Set true open-drain input pin mode (only available on P0_10 and P0_11)
    void set_mode(const InputModeTrueOpenDrainConfig& config)
    {
        // Available only on open-drain pins
        assert(m_pin_name == Pin::Name::p0_10 || m_pin_name == Pin::Name::p0_11);

        write(0);
        set_direction(Direction::input);

        Pin::set_mode(m_pin_name, Pin::I2cMode::standard_gpio, config.input_filter, config.input_invert);
    }

    // Set true open-drain output pin mode (only available on P0_10 and P0_11)
    void set_mode(const OutputModeTrueOpenDrainConfig config)
    {
        // Available only on open-drain pins
        assert(m_pin_name == Pin::Name::p0_10 || m_pin_name == Pin::Name::p0_11);

        write((config.output_mode == OutputModeTrueOpenDrain::hiz) ? 1 : 0);
        set_direction(Direction::output);

        Pin::set_mode(m_pin_name, Pin::I2cMode::standard_gpio, Pin::InputFilter::bypass, Pin::InputInvert::normal);
    }
#endif // (TARGET_HAS_TRUE_OPEN_DRAIN_PINS == 1)

    // -------- READ / WRITE --------------------------------------------------

    uint32_t read() const
    {
        if(m_reg_w != nullptr)
        {
            return (*m_reg_w != 0) ? 1 : 0;
        }

        return 0;
    }

    void write(const uint32_t value)
    {
        if(m_reg_w != nullptr)
        {
            *m_reg_w = value;
        }
    }

    // -------- INPUT FILTER CLOCK DIVIDER SELECTION --------------------------

    // Input filter clock divider alias
    using InputFilterClockDivider = SysClock::IoconClockDividerSelect;

    // Set the value of the supplied IOCON clock divider (used by input filters)
    // NOTE: The input filter source (where the divider is applied) is the MAIN clock
    static void set_input_filter_clock_divider(const InputFilterClockDivider clock_div, const uint8_t div)
    {
        SysClock::set_iocon_clock_divider(clock_div, div);
    }

private:

    // ------------------------------------------------------------------------
    // PRIVATE DEFINITIONS
    // ------------------------------------------------------------------------

    // Pin direction
    enum class Direction
    {
        input = 0,
        output
    };

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER FUNCTIONS
    // ------------------------------------------------------------------------

    void config_pin()
    {
        m_reg_w = &LPC_GPIO->W0[static_cast<uint32_t>(m_pin_name)];

        if(SysClock::is_enabled(SysClock::Peripheral::gpio) == false)
        {
            // Enable GPIO
            SysClock::enable(SysClock::Peripheral::gpio);
            Power::reset(Power::ResetPeripheral::gpio);
        }
    }

    void set_direction(const Direction direction)
    {
        const uint32_t pin_mask = 1UL << static_cast<uint32_t>(m_pin_name);

        switch(direction)
        {
            case Direction::input:  LPC_GPIO->DIR0 &= ~pin_mask; break;
            case Direction::output: LPC_GPIO->DIR0 |=  pin_mask; break;
        }
    }

    // ------------------------------------------------------------------------
    // PRIVATE MEMBER VARIABLES
    // ------------------------------------------------------------------------

    __IO uint32_t* m_reg_w {nullptr};
};

} // namespace xarmlib::targets::lpc81x




#endif // XARMLIB_TARGETS_LPC81X_GPIO_HPP

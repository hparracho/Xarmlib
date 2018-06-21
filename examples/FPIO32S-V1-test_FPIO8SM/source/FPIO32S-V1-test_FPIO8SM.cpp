// ----------------------------------------------------------------------------
// @file    FPIO32S-V1-test_FPIO8SM.cpp
// @brief   FPIO32S-V1 board Test FPIO8SM example application.
// @date    21 June 2018
// ----------------------------------------------------------------------------
//
// Xarmlib 0.1.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018 Helder Parracho (hparracho@gmail.com)
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

#include <stdio.h>
#include <stdarg.h>

#include "xarmlib_config.hpp"

using namespace std::chrono_literals;
using namespace xarmlib;




std::size_t read_module_count()
{
    // Dip switch                                                 SW1              SW2               SW3               SW4
    constexpr std::array<Pin::Name, 4> INPUTS_SW = { Pin::Name::P0_15, Pin::Name::P0_1, Pin::Name::P0_11, Pin::Name::P0_10 };

    uint32_t sw_value = 0;

    // Reads the number of modules assigned by dip switch
    for(std::size_t i = 0; i < INPUTS_SW.size(); i++)
    {
        if(INPUTS_SW[i] == Pin::Name::P0_10 || INPUTS_SW[i] == Pin::Name::P0_11)
        {
            DigitalIn sw(INPUTS_SW[i], DigitalIn::InputModeTrueOpenDrain::HIZ);

            sw_value |= sw << i;
        }
        else
        {
            DigitalIn sw(INPUTS_SW[i], DigitalIn::InputMode::PULL_UP);

            sw_value |= sw << i;
        }
    }

    return (~sw_value) & 0x0F;
}




void print(Usart& usart, const char *format, ...)
{
    uint8_t buffer[32] { '\0' };

    va_list arg_list;
    va_start(arg_list, format);

    vsnprintf(reinterpret_cast<char *>(buffer), 32, format, arg_list);

    // do something with the error

    va_end(arg_list);

    const gsl::span<const uint8_t> usart_buffer { buffer };
    usart.write_buffer(usart_buffer, 100ms);
}




void test_FPIO8SM(Usart& usart, SpiMaster& spi_master, const std::size_t module_count)
{
    SpiIoModule<4> spi_io_module(spi_master, Pin::Name::P0_7, Pin::Name::P0_16);

    spi_io_module.enable();

    print(usart, "Testing %d FPIO8SM...\n", module_count);

    SpiIoModule<4>::Type pins_mask;

    switch(module_count)
    {
        case 1: pins_mask = 0xFF;        break;
        case 2: pins_mask = 0xFFFF;      break;
        case 3: pins_mask = 0xFFFFFF;    break;
        case 4:
        default: pins_mask = 0xFFFFFFFF; break;
    }

    SpiIoModule<4>::Type pin_init_failed  = 0;
    SpiIoModule<4>::Type pin_set_failed   = 0;
    SpiIoModule<4>::Type pin_clear_failed = 0;

    for(std::size_t pin = 0; pin < (module_count * 8); pin++)
    {
        const SpiIoModule<4>::Type output_clear = pins_mask;
        const SpiIoModule<4>::Type output_set   = pins_mask & ~(1 << pin);

        // Sets pin and verifies if pin was initially clear
        if((spi_io_module.transfer(output_set, module_count) & pins_mask) != output_clear)
        {
            pin_init_failed |= 1 << pin;
        }
        UsTicker::wait(10ms);

        // Clears pin and verifies if pin was set
        if((spi_io_module.transfer(output_clear, module_count) & pins_mask) != output_set)
        {
            pin_set_failed |= 1 << pin;
        }
        UsTicker::wait(10ms);

        // Clears pin again to verify if pin was clear
        if((spi_io_module.transfer(output_clear, module_count) & pins_mask) != output_clear)
        {
            pin_clear_failed |= 1 << pin;
        }
        UsTicker::wait(10ms);
    }

    if(pin_init_failed == 0 && pin_set_failed == 0 && pin_clear_failed == 0)
    {
        print(usart, "%d FPIO8SM tested OK.\n", module_count);
    }
    else
    {
        int8_t pin = 1;

        while(pin_init_failed > 0)
        {
            if(pin_init_failed & 1)
            {
                print(usart, "X%d pin initial read FAILED\n", pin);
            }
            pin++;
            pin_init_failed >>= 1;
        }

        pin = 1;

        while(pin_set_failed > 0)
        {
            if(pin_set_failed & 1)
            {
                print(usart, "X%d pin set read FAILED\n", pin);
            }
            pin++;
            pin_set_failed >>= 1;
        }

        pin = 1;

        while(pin_clear_failed > 0)
        {
            if(pin_clear_failed & 1)
            {
                print(usart, "X%d pin clear read FAILED\n", pin);
            }
            pin++;
            pin_clear_failed >>= 1;
        }
    }
}




int main(void)
{
    // ------------ USART -----------------------------------------------------

    // RS485 USART output enable
    DigitalOut usart_oe(Pin::Name::P0_4, DigitalOut::OutputMode::PUSH_PULL_HIGH);

    // RS485 USART
    Usart usart(Pin::Name::P0_13, Pin::Name::P0_17, 115200);

    usart.enable();

    // ------------ READ AND VALIDATE MODULE COUNT ----------------------------

    const std::size_t module_count = read_module_count();

    if(module_count < 1 || module_count > 8)
    {
        print(usart, "ERROR! Number of modules defined must be 1 to 8!\n");

        while(true);
    }
    else
    {
        print(usart, "NOTE: SPI bus must be closed!\n");
    }

    // ------------ SPI -------------------------------------------------------

    SpiMaster spi_master(Pin::Name::P0_0, Pin::Name::P0_14, Pin::Name::P0_6, 500000);

    // NOTE: Optional since it will be enabled by SpiIoModule class
    //spi_master.enable();

    // ------------ TEST IO MODULES -------------------------------------------

    test_FPIO8SM(usart, spi_master, module_count);

    while(1)
    {}

    return 0;
}

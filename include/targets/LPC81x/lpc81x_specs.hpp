// ----------------------------------------------------------------------------
// @file    lpc81x_specs.hpp
// @brief   NXP LPC81x specification definitions.
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

#ifndef XARMLIB_TARGETS_LPC81X_SPECS_HPP
#define XARMLIB_TARGETS_LPC81X_SPECS_HPP

#ifndef TARGET_DEFINED

#include <cstdint>




#if defined (LPC812M101JDH20)
#   define __LPC812__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (256)
#   define TARGET_PACKAGE_PIN_COUNT         (20)
#   define TARGET_GPIO_COUNT                (18)
#   define TARGET_SPI_COUNT                 (2)
#   define TARGET_USART_COUNT               (3)
#   define TARGET_HAS_TRUE_OPEN_DRAIN_PINS  (1)
#elif defined (LPC812M101JD20)
#   define __LPC812__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (256)
#   define TARGET_PACKAGE_PIN_COUNT         (20)
#   define TARGET_GPIO_COUNT                (18)
#   define TARGET_SPI_COUNT                 (1)
#   define TARGET_USART_COUNT               (2)
#   define TARGET_HAS_TRUE_OPEN_DRAIN_PINS  (1)
#elif defined (LPC812M101JDH16) || defined (LPC812M101JTB16)
#   define __LPC812__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (256)
#   define TARGET_PACKAGE_PIN_COUNT         (16)
#   define TARGET_GPIO_COUNT                (14)
#   define TARGET_SPI_COUNT                 (2)
#   define TARGET_USART_COUNT               (3)
#   define TARGET_HAS_TRUE_OPEN_DRAIN_PINS  (1)
#elif defined (LPC811M001JDH16)
#   define __LPC811__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (128)
#   define TARGET_PACKAGE_PIN_COUNT         (16)
#   define TARGET_GPIO_COUNT                (14)
#   define TARGET_SPI_COUNT                 (1)
#   define TARGET_USART_COUNT               (2)
#   define TARGET_HAS_TRUE_OPEN_DRAIN_PINS  (1)
#elif defined (LPC810M021FN8)
#   define __LPC810__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (64)
#   define TARGET_PACKAGE_PIN_COUNT         (8)
#   define TARGET_GPIO_COUNT                (6)
#   define TARGET_SPI_COUNT                 (1)
#   define TARGET_USART_COUNT               (2)
#   define TARGET_HAS_TRUE_OPEN_DRAIN_PINS  (0)
#endif




#if defined (__LPC81X__)
#   define TARGET_DEFINED                   (__LPC81X__)
#   define TARGET_FLASH_PAGE_SIZE           (64)
#   define TARGET_FLASH_SIZE                (TARGET_FLASH_PAGE_COUNT * TARGET_FLASH_PAGE_SIZE)
#   define TARGET_PORT_COUNT                (1)
#   define TARGET_PIN_INTERRUPT_COUNT       (8)
#   define TARGET_TIMER_COUNT               (4)
#   define TARGET_HAS_MRT_TIMER             (1)
#endif




namespace xarmlib::targets
{

// Pin names according to the target package
enum class PinName : uint16_t
{
    // The following pins are present in all packages
    p0_0 = 0,
    p0_1,
    p0_2,
    p0_3,
    p0_4,
    p0_5,

#if (TARGET_PACKAGE_PIN_COUNT >= 16)
    // The following pins are only present in
    // TSSOP16 / XSON16 / SO20 / TSSOP20 packages
    p0_6,
    p0_7,
    p0_8,
    p0_9,
    p0_10,
    p0_11,
    p0_12,
    p0_13,
#endif // (TARGET_PACKAGE_PIN_COUNT >= 16)

#if (TARGET_PACKAGE_PIN_COUNT == 20)
    // The following pins are only present in
    // SO20 / TSSOP20 packages
    p0_14,
    p0_15,
    p0_16,
    p0_17,
#endif // (TARGET_PACKAGE_PIN_COUNT == 20)

    // Not connected
    nc
};




// Port names (all packages have 1 port)
enum class PortName : uint16_t
{
    port0 = 0
};

} // xarmlib::targets




#endif // TARGET_DEFINED

#endif // XARMLIB_TARGETS_LPC81X_SPECS_HPP

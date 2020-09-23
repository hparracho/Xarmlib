// ----------------------------------------------------------------------------
// @file    lpc81x_specs.hpp
// @brief   NXP LPC81x specification definitions.
// @date    14 September 2020
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

#ifndef __XARMLIB_TARGETS_LPC81X_SPECS_HPP
#define __XARMLIB_TARGETS_LPC81X_SPECS_HPP

#ifndef TARGET_DEFINED




#if defined (LPC812M101JDH20)
#   define __LPC812__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (256)
#   define TARGET_PACKAGE_PIN_COUNT         (20)
#   define TARGET_GPIO_COUNT                (18)
#   define TARGET_SPI_COUNT                 (2)
#   define TARGET_USART_COUNT               (3)
#elif defined (LPC812M101JD20)
#   define __LPC812__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (256)
#   define TARGET_PACKAGE_PIN_COUNT         (20)
#   define TARGET_GPIO_COUNT                (18)
#   define TARGET_SPI_COUNT                 (1)
#   define TARGET_USART_COUNT               (2)
#elif defined (LPC812M101JDH16) || defined (LPC812M101JTB16)
#   define __LPC812__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (256)
#   define TARGET_PACKAGE_PIN_COUNT         (16)
#   define TARGET_GPIO_COUNT                (14)
#   define TARGET_SPI_COUNT                 (2)
#   define TARGET_USART_COUNT               (3)
#elif defined (LPC811M001JDH16)
#   define __LPC811__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (128)
#   define TARGET_PACKAGE_PIN_COUNT         (16)
#   define TARGET_GPIO_COUNT                (14)
#   define TARGET_SPI_COUNT                 (1)
#   define TARGET_USART_COUNT               (2)
#elif defined (LPC810M021FN8)
#   define __LPC810__
#   define __LPC81X__
#   define TARGET_FLASH_PAGE_COUNT          (64)
#   define TARGET_PACKAGE_PIN_COUNT         (8)
#   define TARGET_GPIO_COUNT                (6)
#   define TARGET_SPI_COUNT                 (1)
#   define TARGET_USART_COUNT               (2)
#endif




#if defined (__LPC81X__)
#   define TARGET_DEFINED                   (__LPC81X__)
#   define TARGET_FLASH_PAGE_SIZE           (64)
#   define TARGET_FLASH_SIZE                (TARGET_FLASH_PAGE_COUNT * TARGET_FLASH_PAGE_SIZE)
#   define TARGET_PORT_HAS_TRUE_OPEN_DRAIN  (1)
#   define TARGET_PORT_COUNT                (1)
#   define TARGET_PIN_INTERRUPT_COUNT       (8)
#   define TARGET_TIMER_COUNT               (4)
#   define TARGET_TIMER_TYPE_MRT
#endif




#endif // TARGET_DEFINED

#endif // __XARMLIB_TARGETS_LPC81X_SPECS_HPP

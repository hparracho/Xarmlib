// ----------------------------------------------------------------------------
// @file    lpc408x_7x_specs.hpp
// @brief   NXP LPC408x_7x specification definitions.
// @date    19 October 2018
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

#ifndef __XARMLIB_TARGETS_LPC408X_7X_SPECS_HPP
#define __XARMLIB_TARGETS_LPC408X_7X_SPECS_HPP

#ifndef TARGET_DEFINED




#if defined (LPC4088FBD208) || defined (LPC4088FET208)
#   define __LPC4088__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (208)
#   define TARGET_GPIO_COUNT                (165)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC4088FET180)
#   define __LPC4088__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (180)
#   define TARGET_GPIO_COUNT                (141)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC4088FBD144)
#   define __LPC4088__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (144)
#   define TARGET_GPIO_COUNT                (109)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)

#elif defined (LPC4078FBD208) || defined (LPC4078FET208)
#   define __LPC4078__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (208)
#   define TARGET_GPIO_COUNT                (165)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC4078FET180)
#   define __LPC4078__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (180)
#   define TARGET_GPIO_COUNT                (141)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC4078FBD144)
#   define __LPC4078__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (144)
#   define TARGET_GPIO_COUNT                (109)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC4078FBD100)
#   define __LPC4078__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (100)
#   define TARGET_GPIO_COUNT                (71)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC4078FBD80)
#   define __LPC4078__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 512)
#   define TARGET_PACKAGE_PIN_COUNT         (80)
#   define TARGET_GPIO_COUNT                (52)
#   define TARGET_PORT_COUNT                (4)
#   define TARGET_USART_COUNT               (5)

#elif defined (LPC4076FET180)
#   define __LPC4076__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 256)
#   define TARGET_PACKAGE_PIN_COUNT         (180)
#   define TARGET_GPIO_COUNT                (141)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC4076FBD144)
#   define __LPC4076__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 256)
#   define TARGET_PACKAGE_PIN_COUNT         (144)
#   define TARGET_GPIO_COUNT                (109)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (5)

#elif defined (LPC4074FBD144)
#   define __LPC4074__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 128)
#   define TARGET_PACKAGE_PIN_COUNT         (144)
#   define TARGET_GPIO_COUNT                (109)
#   define TARGET_PORT_COUNT                (6)
#   define TARGET_USART_COUNT               (4)
#elif defined (LPC4074FBD80)
#   define __LPC4074__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 128)
#   define TARGET_PACKAGE_PIN_COUNT         (80)
#   define TARGET_GPIO_COUNT                (52)
#   define TARGET_PORT_COUNT                (4)
#   define TARGET_USART_COUNT               (4)

#elif defined (LPC4072FET80) || defined (LPC4072FBD80)
#   define __LPC4072__
#   define __LPC408X_7X__
#   define TARGET_FLASH_SIZE                (1024 * 64)
#   define TARGET_PACKAGE_PIN_COUNT         (80)
#   define TARGET_GPIO_COUNT                (52)
#   define TARGET_PORT_COUNT                (4)
#   define TARGET_USART_COUNT               (4)
#endif




#if defined (__LPC408X_7X__)
#   define TARGET_DEFINED                   (__LPC408X_7X__)
#   define TARGET_SPI_COUNT                 (3)
#   define TARGET_CAN_COUNT                 (2)
#   define TARGET_TIMER_COUNT               (4)
#endif




#endif // TARGET_DEFINED

#endif // __XARMLIB_TARGETS_LPC408X_7X_SPECS_HPP

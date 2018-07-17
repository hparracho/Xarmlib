// ----------------------------------------------------------------------------
// @file    lpc84x_specs.hpp
// @brief   NXP LPC84x specification definitions.
// @date    16 July 2018
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

#ifndef __XARMLIB_TARGETS_LPC84X_SPECS_HPP
#define __XARMLIB_TARGETS_LPC84X_SPECS_HPP

#ifndef TARGET_DEFINED




#if defined (LPC845M301JBD64)
#   define __LPC845__
#   define __LPC84X__
#   define TARGET_PACKAGE_PIN_COUNT         (64)
#   define TARGET_GPIO_COUNT                (54)
#   define TARGET_PORT_COUNT                (2)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC845M301JBD48) || defined (LPC845M301JHI48)
#   define __LPC845__
#   define __LPC84X__
#   define TARGET_PACKAGE_PIN_COUNT         (48)
#   define TARGET_GPIO_COUNT                (42)
#   define TARGET_PORT_COUNT                (2)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC845M301JHI33)
#   define __LPC845__
#   define __LPC84X__
#   define TARGET_PACKAGE_PIN_COUNT         (33)
#   define TARGET_GPIO_COUNT                (29)
#   define TARGET_PORT_COUNT                (1)
#   define TARGET_USART_COUNT               (5)
#elif defined (LPC844M201JBD64)
#   define __LPC844__
#   define __LPC84X__
#   define TARGET_PACKAGE_PIN_COUNT         (64)
#   define TARGET_GPIO_COUNT                (54)
#   define TARGET_PORT_COUNT                (2)
#   define TARGET_USART_COUNT               (2)
#elif defined (LPC844M201JBD48) || defined (LPC844M201JHI48)
#   define __LPC844__
#   define __LPC84X__
#   define TARGET_PACKAGE_PIN_COUNT         (48)
#   define TARGET_GPIO_COUNT                (42)
#   define TARGET_PORT_COUNT                (2)
#   define TARGET_USART_COUNT               (2)
#elif defined (LPC844M201JHI33)
#   define __LPC844__
#   define __LPC84X__
#   define TARGET_PACKAGE_PIN_COUNT         (33)
#   define TARGET_GPIO_COUNT                (29)
#   define TARGET_PORT_COUNT                (1)
#   define TARGET_USART_COUNT               (2)
#endif




#if defined (__LPC84X__)
#   define TARGET_DEFINED                   (__LPC84X__)
#   define TARGET_FLASH_PAGE_COUNT          (1024)
#   define TARGET_FLASH_PAGE_SIZE           (64)
#   define TARGET_FLASH_SIZE                (TARGET_FLASH_PAGE_COUNT * TARGET_FLASH_PAGE_SIZE)
#   define TARGET_SPI_COUNT                 (2)
#   define TARGET_TIMER_COUNT               (4)
#   define TARGET_TIMER_TYPE_MRT
#endif




#endif // TARGET_DEFINED

#endif // __XARMLIB_TARGETS_LPC84X_SPECS_HPP

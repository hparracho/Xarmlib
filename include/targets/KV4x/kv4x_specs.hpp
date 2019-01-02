// ----------------------------------------------------------------------------
// @file    kv4x_specs.hpp
// @brief   Kinetis KV4x specification definitions.
// @date    2 January 2019
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

#ifndef __XARMLIB_TARGETS_KV4X_SPECS_HPP
#define __XARMLIB_TARGETS_KV4X_SPECS_HPP

#ifndef TARGET_DEFINED




#if defined (CPU_MKV46F256VLL16) || defined (CPU_MKV46F128VLL16)
#   define __KV46__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (100)
#   define TARGET_GPIO_COUNT                (74)
#elif defined (CPU_MKV46F256VLH16) || defined (CPU_MKV46F128VLH16)
#   define __KV46__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (64)
#   define TARGET_GPIO_COUNT                (48)

#elif defined (CPU_MKV44F256VLL16) || defined (CPU_MKV44F128VLL16)
#   define __KV44__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (100)
#   define TARGET_GPIO_COUNT                (74)
#elif defined (CPU_MKV44F256VLH16) || defined (CPU_MKV44F128VLH16) || defined (CPU_MKV44F64VLH16)
#   define __KV44__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (64)
#   define TARGET_GPIO_COUNT                (48)
#elif defined (CPU_MKV44F128VLF16) || defined (CPU_MKV44F64VLF16)
#   define __KV44__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (48)
#   define TARGET_GPIO_COUNT                (39)

#elif defined (CPU_MKV42F256VLL16) || defined (CPU_MKV42F128VLL16)
#   define __KV42__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (100)
#   define TARGET_GPIO_COUNT                (74)
#elif defined (CPU_MKV42F256VLH16) || defined (CPU_MKV42F128VLH16) || defined (CPU_MKV42F64VLH16)
#   define __KV42__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (64)
#   define TARGET_GPIO_COUNT                (48)
#elif defined (CPU_MKV42F128VLF16) || defined (CPU_MKV42F64VLF16)
#   define __KV42__
#   define __KV4X__
#   define TARGET_PACKAGE_PIN_COUNT         (48)
#   define TARGET_GPIO_COUNT                (39)
#endif




#if defined (__KV4X__)
#   define TARGET_DEFINED                   (__KV4X__)
#   define TARGET_PORT_COUNT                (FSL_FEATURE_SOC_GPIO_COUNT)
#   define TARGET_SPI_COUNT                 (FSL_FEATURE_SOC_DSPI_COUNT)
#   define TARGET_UART_COUNT                (FSL_FEATURE_SOC_UART_COUNT)
#   define TARGET_TIMER_COUNT               (FSL_FEATURE_PIT_TIMER_COUNT)
#   define TARGET_TIMER16_COUNT             (FSL_FEATURE_SOC_LPTMR_COUNT)
#   define TARGET_ENC_COUNT                 (FSL_FEATURE_SOC_ENC_COUNT)
#   define TARGET_PWM_COUNT                 (FSL_FEATURE_SOC_PWM_COUNT)
#   define TARGET_FTM_COUNT                 (FSL_FEATURE_SOC_FTM_COUNT)
#   define TARGET_DAC_COUNT                 (FSL_FEATURE_SOC_DAC_COUNT)
#endif




#endif // TARGET_DEFINED

#endif // __XARMLIB_TARGETS_KV4X_SPECS_HPP

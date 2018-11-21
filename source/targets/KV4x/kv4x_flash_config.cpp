// ----------------------------------------------------------------------------
// @file    kv4x_flash_config.cpp
// @brief   Flash Configuration block: 16-byte flash configuration field that
//          stores default protection settings (loaded on reset) and security
//          information that allows the MCU to restrict access to the Flash
//          Memory module.
//          Placed at address 0x400 by the linker script.
// @notes   Flash is not secure, flash backdoor is enabled/disabled are the
//          only options implemented for now.
//          For more details:
//          https://www.nxp.com/docs/en/application-note/AN4507.pdf
// @date    5 November 2018
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

#include "core/target_specs.hpp"

#ifdef __KV4X__

#include "xarmlib_config.hpp"

extern "C"
{




// Unsecured state: all flash commands are available to the programming
// interfaces (JTAG), as well as user code execution of Flash Controller
// commands.
#define FLASH_CONFIG_FLASH_NOT_SECURE           0xFFFFFFFE

// Secured without backdoor key state: programmer interfaces are only allowed
// to launch mass erase operations and have no access to memory locations.
#define FLASH_CONFIG_FLASH_BACKDOOR_DISABLED    0xFFFFFFFF

// Secured with backdoor key state: the backdoor key option allows for a means
// of temporarily disabling flash security if the correct 64-bit key value is
// provided when executing a flash verify backdoor access key command.
#define FLASH_CONFIG_FLASH_BACKDOOR_ENABLED     0xFFFFFFBF


#if (XARMLIB_CONFIG_FLASH_CONFIG_SETTING == FLASH_CONFIG_FLASH_NOT_SECURE)
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_NOT_SECURE
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       0xFFFFFFFFFFFFFFFFULL
#elif (XARMLIB_CONFIG_FLASH_CONFIG_SETTING == FLASH_CONFIG_FLASH_BACKDOOR_DISABLED)
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_BACKDOOR_DISABLED
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       0xFFFFFFFFFFFFFFFFULL
#elif (XARMLIB_CONFIG_FLASH_CONFIG_SETTING == FLASH_CONFIG_FLASH_BACKDOOR_ENABLED)
#ifdef XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY
#if ((XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY != 0ULL) \
  && (XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY != 0xFFFFFFFFFFFFFFFFULL))
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_BACKDOOR_ENABLED
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       (uint64_t)XARMLIB_CONFIG_FLASH_CONFIG_BACKDOOR_KEY
#else
#   error "Backdoor Key invalid!"
#endif
#else
#   error "Backdoor Key not defined!"
#endif
#endif

#ifndef CURRENT_FLASH_CONFIG_SETTING
#define CURRENT_FLASH_CONFIG_SETTING            FLASH_CONFIG_FLASH_NOT_SECURE
#define CURRENT_FLASH_CONFIG_BACKDOOR_KEY       0xFFFFFFFFFFFFFFFFULL
#endif




// ----------------------------------------------------------------------------
// Flash Configuration block : 16-byte flash configuration field that stores
// default protection settings (loaded on reset) and security information that
// allows the MCU to restrict access to the Flash Memory module.
// Placed at address 0x400 by the linker script.
// ----------------------------------------------------------------------------
__attribute__ ((used, section(".FlashConfig")))
const struct
{
    unsigned int word1;
    unsigned int word2;
    unsigned int word3;
    unsigned int word4;
} FlashConfig = { (uint32_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY),
                  (uint32_t)(CURRENT_FLASH_CONFIG_BACKDOOR_KEY >> 32),
                  0xFFFFFFFF,
                  CURRENT_FLASH_CONFIG_SETTING };




} // extern "C"

#endif // __KV4X__

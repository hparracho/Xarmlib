// ----------------------------------------------------------------------------
// @file    fsl_device_registers.h
// @brief   Freescale drivers generic header include file.
// @date    27 August 2020
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

// ----------------------------------------------------------------------------
// The CPU macro should be declared in the project or makefile.
// ----------------------------------------------------------------------------

#ifndef __XARMLIB_EXTERNAL_FSL_DEVICE_REGISTERS_H
#define __XARMLIB_EXTERNAL_FSL_DEVICE_REGISTERS_H




#if defined (CPU_MKV58F1M0VLL24) || defined (CPU_MKV58F1M0VLQ24) || defined (CPU_MKV58F1M0VMD24) || \
    defined (CPU_MKV58F512VLL24) || defined (CPU_MKV58F512VLQ24) || defined (CPU_MKV58F512VMD24)

// CMSIS-style register definitions
#include "MKV58F24.h"
// CPU specific feature definitions
#include "MKV58F24_features.h"

// Added by Emanuel Pinto
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341 (1)

#elif defined (CPU_MKV56F1M0VLL24) || defined (CPU_MKV56F1M0VLQ24) || defined (CPU_MKV56F1M0VMD24) || \
      defined (CPU_MKV56F512VLL24) || defined (CPU_MKV56F512VLQ24) || defined (CPU_MKV56F512VMD24)

// CMSIS-style register definitions
#include "MKV56F24.h"
// CPU specific feature definitions
#include "MKV56F24_features.h"

// Added by Emanuel Pinto
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341 (1)

#elif defined (CPU_MKV46F256VLL16) || defined (CPU_MKV46F256VLH16) || \
      defined (CPU_MKV46F128VLL16) || defined (CPU_MKV46F128VLH16)

// CMSIS-style register definitions
#include "MKV46F16.h"
// CPU specific feature definitions
#include "MKV46F16_features.h"

// Added by Emanuel Pinto
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341 (1)

#elif defined (CPU_MKV44F256VLL16) || defined (CPU_MKV44F256VLH16) || \
      defined (CPU_MKV44F128VLL16) || defined (CPU_MKV44F128VLH16) || defined (CPU_MKV44F128VLF16) || \
                                      defined (CPU_MKV44F64VLH16)  || defined (CPU_MKV44F64VLF16)

// CMSIS-style register definitions
#include "MKV44F16.h"
// CPU specific feature definitions
#include "MKV44F16_features.h"

// Added by Emanuel Pinto
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341 (1)

#elif defined (CPU_MKV42F256VLL16) || defined (CPU_MKV42F256VLH16) || \
      defined (CPU_MKV42F128VLL16) || defined (CPU_MKV42F128VLH16) || defined (CPU_MKV42F128VLF16) || \
                                      defined (CPU_MKV42F64VLH16)  || defined (CPU_MKV42F64VLF16)

// CMSIS-style register definitions
#include "MKV42F16.h"
// CPU specific feature definitions
#include "MKV42F16_features.h"

// Added by Emanuel Pinto
#define FSL_FEATURE_FLEXCAN_HAS_ERRATA_8341 (1)

#else
    #error "No valid CPU defined!"
#endif




#endif // __XARMLIB_EXTERNAL_FSL_DEVICE_REGISTERS_H

// ----------------------------------------------------------------------------
// @file    macros.h
// @brief   macros.
// @notes   USB_Host_Shield_2.0 macros.h file suitable for Xarmlib
// @date    8 June 2020
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

#if !defined(_usb_h_) || defined(MACROS_H)
#error "Never include macros.h directly; include Usb.h instead"
#else
#define MACROS_H

#include "hal/hal_us_ticker.hpp"

#define delay(ms) xarmlib::hal::UsTicker::wait(std::chrono::milliseconds(ms))
#define micros() std::chrono::duration_cast<std::chrono::microseconds>(xarmlib::hal::UsTicker::now()).count()
#define millis() std::chrono::duration_cast<std::chrono::milliseconds>(xarmlib::hal::UsTicker::now()).count()

////////////////////////////////////////////////////////////////////////////////
// HANDY MACROS
////////////////////////////////////////////////////////////////////////////////

#define VALUE_BETWEEN(v,l,h) (((v)>(l)) && ((v)<(h)))
#define VALUE_WITHIN(v,l,h) (((v)>=(l)) && ((v)<=(h)))
#define output_pgm_message(wa,fp,mp,el) wa = &mp, fp((char *)pgm_read_pointer(wa), el)
#define output_if_between(v,l,h,wa,fp,mp,el) if(VALUE_BETWEEN(v,l,h)) output_pgm_message(wa,fp,mp[v-(l+1)],el);

#define SWAP(a, b) (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b)))
#ifndef __BYTE_GRABBING_DEFINED__
#define __BYTE_GRABBING_DEFINED__ 1
#ifdef BROKEN_OPTIMIZER_LITTLE_ENDIAN
// Note: Use this if your compiler generates horrible assembler!
#define BGRAB0(__usi__)  (((uint8_t *)&(__usi__))[0])
#define BGRAB1(__usi__)  (((uint8_t *)&(__usi__))[1])
#define BGRAB2(__usi__)  (((uint8_t *)&(__usi__))[2])
#define BGRAB3(__usi__)  (((uint8_t *)&(__usi__))[3])
#define BGRAB4(__usi__)  (((uint8_t *)&(__usi__))[4])
#define BGRAB5(__usi__)  (((uint8_t *)&(__usi__))[5])
#define BGRAB6(__usi__)  (((uint8_t *)&(__usi__))[6])
#define BGRAB7(__usi__)  (((uint8_t *)&(__usi__))[7])
#else
// Note: The cast alone to uint8_t is actually enough.
// GCC throws out the "& 0xff", and the size is no different.
// Some compilers need it.
#define BGRAB0(__usi__)  ((uint8_t)((__usi__) & 0xff ))
#define BGRAB1(__usi__)  ((uint8_t)(((__usi__) >> 8) & 0xff))
#define BGRAB2(__usi__)  ((uint8_t)(((__usi__) >> 16) & 0xff))
#define BGRAB3(__usi__)  ((uint8_t)(((__usi__) >> 24) & 0xff))
#define BGRAB4(__usi__)  ((uint8_t)(((__usi__) >> 32) & 0xff))
#define BGRAB5(__usi__)  ((uint8_t)(((__usi__) >> 40) & 0xff))
#define BGRAB6(__usi__)  ((uint8_t)(((__usi__) >> 48) & 0xff))
#define BGRAB7(__usi__)  ((uint8_t)(((__usi__) >> 56) & 0xff))
#endif
#define BOVER1(__usi__)  ((uint16_t)(__usi__) << 8)
#define BOVER2(__usi__)  ((uint32_t)(__usi__) << 16)
#define BOVER3(__usi__)  ((uint32_t)(__usi__) << 24)
#define BOVER4(__usi__)  ((uint64_t)(__usi__) << 32)
#define BOVER5(__usi__)  ((uint64_t)(__usi__) << 40)
#define BOVER6(__usi__)  ((uint64_t)(__usi__) << 48)
#define BOVER7(__usi__)  ((uint64_t)(__usi__) << 56)

// These are the smallest and fastest ways I have found so far in pure C/C++.
#define BMAKE16(__usc1__,__usc0__) ((uint16_t)((uint16_t)(__usc0__) | (uint16_t)BOVER1(__usc1__)))
#define BMAKE32(__usc3__,__usc2__,__usc1__,__usc0__) ((uint32_t)((uint32_t)(__usc0__) | (uint32_t)BOVER1(__usc1__) | (uint32_t)BOVER2(__usc2__) | (uint32_t)BOVER3(__usc3__)))
#define BMAKE64(__usc7__,__usc6__,__usc5__,__usc4__,__usc3__,__usc2__,__usc1__,__usc0__) ((uint64_t)((uint64_t)__usc0__ | (uint64_t)BOVER1(__usc1__) | (uint64_t)BOVER2(__usc2__) | (uint64_t)BOVER3(__usc3__) | (uint64_t)BOVER4(__usc4__) | (uint64_t)BOVER5(__usc5__) | (uint64_t)BOVER6(__usc6__) | (uint64_t)BOVER1(__usc7__)))
#endif

/*
 * Debug macros: Strings are stored in progmem (flash) instead of RAM.
 */
#define USBTRACE(s) (Notify(PSTR(s), 0x80))
#define USBTRACE1(s,l) (Notify(PSTR(s), l))
#define USBTRACE2(s,r) (Notify(PSTR(s), 0x80), D_PrintHex((r), 0x80), Notify(PSTR("\r\n"), 0x80))
#define USBTRACE3(s,r,l) (Notify(PSTR(s), l), D_PrintHex((r), l), Notify(PSTR("\r\n"), l))


#endif /* MACROS_H */


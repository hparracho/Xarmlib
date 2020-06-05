// ----------------------------------------------------------------------------
// @file    UHS_HID.h
// @brief   UHS HID class.
// @notes   Based on UHS30 UHS_HID.h file with few changes
// @date    28 May 2020
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

#if !defined(__UHS_HID_H__)
#define __UHS_HID_H__
#include "UHS_host.h"

//#if !defined(DEBUG_PRINTF_EXTRA_HUGE_USB_HID)
//#define DEBUG_PRINTF_EXTRA_HUGE_USB_HID 0
//#endif
//#if DEBUG_PRINTF_EXTRA_HUGE
//#if DEBUG_PRINTF_EXTRA_HUGE_USB_HID
//#define HID_DEBUG(...) printf(__VA_ARGS__)
//#else
//#define HID_DEBUG(...) VOID0
//#endif
//#else
//#define HID_DEBUG(...) VOID0
//#endif

#ifndef AJK_NI
#define AJK_NI __attribute__((noinline))
#endif

enum UHS_HID_driver_t {
        UHS_HID_raw = 0
        , UHS_HID_keyboard
        , UHS_HID_mouse

};

class UHS_HID_base;

class UHS_HID_PROCESSOR {
public:
//        UHS_HID_PROCESSOR(void) {};
        virtual void onStart(UHS_HID_base *d);
        virtual void onPoll(UHS_HID_base *d, uint8_t *data, uint16_t length);
        virtual void onRelease(UHS_HID_base *d);
};


class UHS_HID : public UHS_USBInterface {
public:
        UHS_HID(UHS_USB_HOST_BASE *p, UHS_HID_PROCESSOR *hp);
        // Configure and internal methods, these should never be called by a user's sketch.
        volatile UHS_EpInfo epInfo[3];
        UHS_HID_base *hiddriver;
        UHS_HID_PROCESSOR *hidProcessor;
        uint8_t bSubClass;
        uint8_t bProtocol;
        uint8_t pollRate;
        const int epInterruptInIndex = 1;
        const int epInterruptOutIndex = 2;


        uint8_t Start(void);

        bool OKtoEnumerate(ENUMERATION_INFO *ei);
        uint8_t SetInterface(ENUMERATION_INFO *ei);
        void Poll(void);
        void DriverDefaults(void);
        void driverPoll(void);
        uint8_t AJK_NI GetAddress(void) {
                return bAddress;
        };

        bool AJK_NI Polling(void) {
                return bPollEnable;
        }

        void AJK_NI OnRelease(void);

        uint8_t AJK_NI SetIdle(uint8_t iface, uint8_t reportID, uint8_t duration);
        uint8_t AJK_NI SetProtocol(uint8_t iface, uint8_t protocol);
        uint8_t AJK_NI SetReport(uint8_t iface, uint8_t report_type, uint8_t report_id, uint16_t nbytes, uint8_t* dataptr);
        uint8_t AJK_NI ReportDescr(uint16_t wIndex, uint16_t nbytes, uint8_t *buffer);

};

class UHS_HID_base {
public:
        UHS_HID *parent;
        UHS_HID_driver_t driver;

        virtual void AJK_NI driverPoll(void){};
        virtual void AJK_NI driverStart() {
                parent->hidProcessor->onStart(this);
        }

        virtual void AJK_NI driverRelease() {
                parent->hidProcessor->onRelease(this);
        }

        uint8_t AJK_NI writeReport(uint16_t nbytes, uint8_t *dataptr) {
                uint8_t rv = UHS_HOST_ERROR_NOT_IMPLEMENTED;
                if(parent->epInfo[parent->epInterruptOutIndex].epAddr != 0) {
                        parent->pUsb->DisablePoll();
                        rv = parent->pUsb->outTransfer(parent->bAddress, parent->epInfo[parent->epInterruptOutIndex].epAddr, nbytes, dataptr);
                        parent->pUsb->EnablePoll();
                }
                return rv;
        };
        virtual ~UHS_HID_base(){};
};


class UHS_HID_RAW : public UHS_HID_base {
public:

        UHS_HID_RAW(UHS_HID *p) {
                parent = p;
                driver = UHS_HID_raw;
        }
        ~UHS_HID_RAW(){};
        void AJK_NI driverPoll() {
                uint8_t data[parent->epInfo[parent->epInterruptInIndex].maxPktSize];
                uint16_t length = parent->epInfo[parent->epInterruptInIndex].maxPktSize;
                uint8_t rv = parent->pUsb->inTransfer(parent->bAddress, parent->epInfo[parent->epInterruptInIndex].epAddr, &length, data);

                if(rv == 0) {
                        DBG("Length {:d} ", length);
                        parent->hidProcessor->onPoll(this, data, length);
                } else if(rv != UHS_HOST_ERROR_NAK) {
                        const uint8_t epAddr = parent->epInfo[parent->epInterruptInIndex].epAddr;
                        DBG("DP 0x{:X} A 0x{:X} EI 0x{:X} EA 0x{:X}\r\n", rv, parent->bAddress, parent->epInterruptInIndex, epAddr);
                }
        }

};

#if defined(LOAD_UHS_HID) && !defined(UHS_HID_LOADED)
#if defined(LOAD_UHS_HIDRAWBOOT_KEYBOARD)
#include "HIDBOOT/UHS_HIDRAWBOOT_KEYBOARD.h"
#endif
#if defined(LOAD_UHS_HIDRAWBOOT_MOUSE)
#include "HIDBOOT/UHS_HIDRAWBOOT_MOUSE.h"
#endif
#include "UHS_HID_INLINE.h"
#endif
#endif // __UHS_HID_H__

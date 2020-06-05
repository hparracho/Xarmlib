// ----------------------------------------------------------------------------
// @file    UHS_HIDRAWBOOT_MOUSE.h
// @brief   UHS_HID MOUSE class.
// @notes   Based on UHS30 UHS_HIDRAWBOOT_MOUSE.h file with few changes
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

// TO-DO: parse config.
// TO-DO: implement cooked interface.

#if !defined(__UHS_HIDBOOT_MOUSE_H__)
#define __UHS_HIDBOOT_MOUSE_H__

//#if !defined(DEBUG_PRINTF_EXTRA_HUGE_USB_HIDBOOT_MOUSE)
//#define DEBUG_PRINTF_EXTRA_HUGE_USB_HIDBOOT_MOUSE 0
//#endif
//
//#if DEBUG_PRINTF_EXTRA_HUGE
//#if DEBUG_PRINTF_EXTRA_HUGE_USB_HIDBOOT_MOUSE
//#define HIDBOOT_MOUSE_DEBUG(...) printf(__VA_ARGS__)
//#else
//#define HIDBOOT_MOUSE_DEBUG(...) VOID0
//#endif
//#else
//#define HIDBOOT_MOUSE_DEBUG(...) VOID0
//#endif

struct MOUSEINFO {
        uint8_t ReportID;
        struct {
                uint8_t bmLeftButton : 1;
                uint8_t bmRightButton : 1;
                uint8_t bmMiddleButton : 1;
                uint8_t bmButton4 : 1;
                uint8_t bmButton5 : 1;
                uint8_t bmDummy : 3;
        }__attribute__((packed));
        int8_t dX;
        int8_t dY;
        int8_t wheel1;
        int8_t wheel2;
        int8_t wheel3;
}__attribute__((packed));

class UHS_HIDBOOT_mouse : public UHS_HID_base {
public:
        MOUSEINFO mouse_data[2];

        UHS_HIDBOOT_mouse(UHS_HID *p) {
                parent = p;
                driver = UHS_HID_mouse;
        }

        ~UHS_HIDBOOT_mouse(){};
        void AJK_NI driverPoll() {
                uint8_t data[parent->epInfo[parent->epInterruptInIndex].maxPktSize];
                uint16_t length = parent->epInfo[parent->epInterruptInIndex].maxPktSize;
                uint8_t rv = parent->pUsb->inTransfer(parent->bAddress, parent->epInfo[parent->epInterruptInIndex].epAddr, &length, data);

                if(rv == 0 && length > 1) {
                        if(*data == 0x01U) {
                                // send only mouse events. Other IDs are "system controls"
                                parent->hidProcessor->onPoll(this, data, length);
                        }
                } else if(rv != UHS_HOST_ERROR_NAK) {
                        const uint8_t epAddr = parent->epInfo[parent->epInterruptInIndex].epAddr;
                        DBG("DP 0x{:X} A 0x{:X} EI 0x{:X} EA 0x{:X}\r\n", rv, parent->bAddress, parent->epInterruptInIndex, epAddr);
                }
        }

        void AJK_NI driverStart() {
                DBG("BOOT_MOUSE\r\n");
                {
                        uint16_t length = 128;
                        uint8_t buffer[length];
                        parent->ReportDescr(parent->bIface, length, buffer);
                }
                parent->SetProtocol(parent->bIface, 0);
                parent->SetIdle(parent->bIface, 0, 0);
                parent->hidProcessor->onStart(this);
        }

        void AJK_NI driverRelease() {
                parent->hidProcessor->onRelease(this);
        }
};

#endif // __UHS_HIDBOOT_MOUSE_H__

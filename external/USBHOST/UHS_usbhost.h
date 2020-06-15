// ----------------------------------------------------------------------------
// @file    UHS_usbhost.h
// @brief   UHS USB Host class base.
// @notes   Based on UHS30 UHS_usbhost.h file with some changes and cleanup
// @date    9 June 2020
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

#if !defined(_UHS_host_h_)
#error "Never include UHS_usbhost.h directly; include UHS_host.h instead"
#else
#if !defined(_USBHOST_H_)
#define _USBHOST_H_


class UHS_USBInterface; // forward class declaration


// enumerator to turn the VBUS on/off

typedef enum
{
    vbus_on = 0,
    vbus_off = 1
} VBUS_t;


// All host SEI use this base class
class UHS_USB_HOST_BASE
{
public:

    AddressPool addrPool;
    UHS_USBInterface* devConfig[UHS_HOST_MAX_INTERFACE_DRIVERS];
    volatile uint8_t usb_error{ 0 };
    volatile uint8_t usb_task_state{ UHS_USB_HOST_STATE_INITIALIZE }; //set up state machine
    volatile uint8_t usb_task_polling_disabled{ 0 };
    volatile uint8_t usb_host_speed{ 0 };
    volatile uint8_t hub_present{ 0 };

    UHS_USB_HOST_BASE()
    {
        for(uint16_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
        {
            devConfig[i] = nullptr;
        }
    }

    /////////////////////////////////////////////
    //
    // Virtual methods that interface to the SIE
    // Overriding each is mandatory.
    //
    /////////////////////////////////////////////

//    virtual void UHS_NI initialize() {}

    /**
     * Delay for x milliseconds
     * Override if your controller provides an SOF IRQ, which may involve
     * some sort of reentrant ISR or workaround with interrupts enabled.
     *
     * @param x how many milliseconds to delay
     * @return true if delay completed without a state change, false if delay aborted
     */
    virtual bool UHS_NI sof_delay(uint16_t ms)
    {
        if(!(usb_task_state & UHS_USB_HOST_STATE_MASK)) return false;

        const uint8_t current_state = usb_task_state;

        while(current_state == usb_task_state && ms--)
        {
#ifdef XARMLIB_ENABLE_FREERTOS
            vTaskDelay(pdMS_TO_TICKS(1));
#else
            xarmlib::hal::UsTicker::wait(std::chrono::milliseconds(1));
#endif
        }

        return (current_state == usb_task_state);
    }

    virtual void UHS_NI doHostReset() {}

    virtual void UHS_NI vbusPower(NOTUSED(VBUS_t state)) {}

    virtual uint8_t UHS_NI SetAddress(NOTUSED(uint8_t addr), NOTUSED(uint8_t ep), NOTUSED(UHS_EpInfo **ppep), NOTUSED(uint16_t &nak_limit))
    {
        return UHS_HOST_ERROR_NOT_IMPLEMENTED;
    }

    virtual uint8_t UHS_NI OutTransfer(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint16_t nak_limit), NOTUSED(uint16_t nbytes), NOTUSED(uint8_t *data))
    {
        return UHS_HOST_ERROR_NOT_IMPLEMENTED;
    }

    virtual uint8_t UHS_NI InTransfer(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint16_t nak_limit), NOTUSED(uint16_t *nbytesptr), NOTUSED(uint8_t *data))
    {
        return UHS_HOST_ERROR_NOT_IMPLEMENTED;
    }

    virtual UHS_EpInfo * UHS_NI ctrlReqOpen(NOTUSED(uint8_t addr), NOTUSED(uint64_t Request), NOTUSED(uint8_t* dataptr))
    {
        return nullptr;
    }

    virtual uint8_t UHS_NI ctrlReqClose(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint8_t bmReqType), NOTUSED(uint16_t left), NOTUSED(uint16_t nbytes), NOTUSED(uint8_t *dataptr))
    {
        return UHS_HOST_ERROR_NOT_IMPLEMENTED;
    }

    virtual uint8_t UHS_NI ctrlReqRead(NOTUSED(UHS_EpInfo *pep), NOTUSED(uint16_t *left), NOTUSED(uint16_t *read), NOTUSED(uint16_t nbytes), NOTUSED(uint8_t *dataptr))
    {
        return UHS_HOST_ERROR_NOT_IMPLEMENTED;
    }

    virtual uint8_t UHS_NI dispatchPkt(NOTUSED(uint8_t token), NOTUSED(uint8_t ep), NOTUSED(uint16_t nak_limit))
    {
        return UHS_HOST_ERROR_NOT_IMPLEMENTED;
    }

//    virtual uint8_t hwlPowerUp(void) {
//            /* This is for machine specific support to enable/power up the USB HW to operate*/
//            return UHS_HOST_ERROR_NOT_IMPLEMENTED;
//    };
//
//    virtual uint8_t hwPowerDown(void) {
//            /* This is for machine specific support to disable/powerdown the USB Hw */
//            return UHS_HOST_ERROR_NOT_IMPLEMENTED;
//    };

    virtual void IsHub(NOTUSED(bool p)) {}

    /////////////////////////////////////////////
    //
    // Built-ins, No need to override
    //
    /////////////////////////////////////////////

    inline void DisablePoll()
    {
        //noInterrupts();
        usb_task_polling_disabled++;
        //DDSB();
        //interrupts();
    }

    inline void EnablePoll()
    {
        //noInterrupts();
        usb_task_polling_disabled--;
        //DDSB();
        //interrupts();
    }

//        uint8_t UHS_NI seekInterface(ENUMERATION_INFO *ei, uint16_t inf, USB_CONFIGURATION_DESCRIPTOR *ucd);

    uint8_t UHS_NI EPClearHalt(uint8_t addr, uint8_t ep);

    uint8_t UHS_NI ctrlReq(uint8_t addr, uint64_t Request, uint16_t nbytes, uint8_t* dataptr);

    uint8_t UHS_NI getDevDescr(uint8_t addr, uint16_t nbytes, uint8_t* dataptr);

    uint8_t UHS_NI getConfDescr(uint8_t addr, uint16_t nbytes, uint8_t conf, uint8_t* dataptr);

    uint8_t UHS_NI setAddr(uint8_t oldaddr, uint8_t newaddr);

    uint8_t UHS_NI setConf(uint8_t addr, uint8_t conf_value);

//        uint8_t UHS_NI getStrDescr(uint8_t addr, uint16_t nbytes, uint8_t index, uint16_t langid, uint8_t* dataptr);
//
    void UHS_NI ReleaseDevice(uint8_t addr);

    uint8_t UHS_NI Configuring(uint8_t parent, uint8_t port, uint8_t speed);

    void UHS_NI DeviceDefaults(uint8_t maxep, UHS_USBInterface *device);

    UHS_EpInfo* UHS_NI getEpInfoEntry(uint8_t addr, uint8_t ep);

    uint8_t UHS_NI setEpInfoEntry(uint8_t addr, uint8_t iface, uint8_t epcount, volatile UHS_EpInfo* eprecord_ptr);

    inline uint8_t getUsbTaskState() { return usb_task_state; }

    AddressPool* GetAddressPool() { return &addrPool; }

    int UHS_NI RegisterDeviceClass(UHS_USBInterface *pdev)
    {
        for(uint8_t i = 0; i < UHS_HOST_MAX_INTERFACE_DRIVERS; i++)
        {
            if(!devConfig[i])
            {
                devConfig[i] = pdev;
                return i;
            }
        }
        //return UHS_HOST_ERROR_CANT_REGISTER_DEVICE_CLASS;
        return -1;
    }

    uint8_t TestInterface(ENUMERATION_INFO *ei);
    uint8_t enumerateInterface(ENUMERATION_INFO *ei);
    uint8_t getNextInterface(ENUMERATION_INFO *ei, UHS_EpInfo *pep, uint8_t data[], uint16_t *left, uint16_t *read, uint8_t *offset);
    uint8_t initDescrStream(ENUMERATION_INFO *ei, USB_CONFIGURATION_DESCRIPTOR *ucd, UHS_EpInfo *pep, uint8_t *data, uint16_t *left, uint16_t *read, uint8_t *offset);
    uint8_t outTransfer(uint8_t addr, uint8_t ep, uint16_t nbytes, uint8_t* data);
    uint8_t inTransfer(uint8_t addr, uint8_t ep, uint16_t *nbytesptr, uint8_t* data);
    uint8_t doSoftReset(uint8_t parent, uint8_t port, uint8_t address);
    uint8_t getone(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset);
    uint8_t eat(UHS_EpInfo *pep, uint16_t *left, uint16_t *read, uint8_t *dataptr, uint8_t *offset, uint16_t *yum);
};


// All device interface drivers use this subclass
class UHS_USBInterface
{
public:

    UHS_USB_HOST_BASE *pUsb; // Parent USB host
    volatile uint8_t bNumEP; // total number of EP in this interface
    volatile UHS_EpInfo epInfo[16]; // This is a stub, override in the driver.

    volatile uint8_t bAddress; // address of the device
    volatile uint8_t bConfNum; // configuration number
    volatile uint8_t bIface; // interface value
    volatile bool bPollEnable; // poll enable flag, operating status
    volatile uint32_t qNextPollTime; // next poll time

    /**
     * Resets interface driver to unused state. You should override this in
     * your driver if it requires extra class variable cleanup.
     */
    virtual void DriverDefaults()
    {
        DBG("Default driver defaults\n");
        pUsb->DeviceDefaults(bNumEP, this);
    }

    /**
     * Checks if this interface is supported.
     * Executed called when new devices are connected.
     *
     * @param ei
     * @return true if the interface is supported
     */
    virtual bool OKtoEnumerate(NOTUSED(ENUMERATION_INFO *ei))
    {
        return false;
    }

    /**
     * Configures any needed endpoint information for an interface.
     * You must provide this in your driver.
     * Executed when new devices are connected and OKtoEnumerate()
     * returned true.
     *
     * @param ei
     * @return zero on success
     */
    virtual uint8_t SetInterface(NOTUSED(ENUMERATION_INFO *ei))
    {
        return UHS_HOST_ERROR_NOT_IMPLEMENTED;
    }

    /**
     * Interface specific additional setup and enumeration that
     * can't occur when the descriptor stream is open.
     * Also used for collection of unclaimed interfaces, to link to the master.
     *
     * @return zero on success
     */
    virtual uint8_t Finalize()
    {
        return 0;
    };

    /**
     *  Executed after interface is finalized but, before polling has started.
     *
     * @return 0 on success
     */
    virtual uint8_t OnStart()
    {
        return 0;
    }

    /**
     * Start interface polling
     * @return
     */
    virtual uint8_t Start()
    {
        const uint8_t rcode = OnStart();

        if(!rcode) bPollEnable = true;

        return rcode;
    }

    /**
     * Executed before anything else in Release().
     *
     */
    virtual void OnRelease()
    {}

    /**
     * Release resources when device is disconnected.
     * Normally this does not need to be overridden.
     */
    virtual void Release()
    {
        OnRelease();
        DriverDefaults();
    }

    /**
     * Executed After driver polls.
     * Can be used when there is an important change detected during polling
     * and you want to handle it elsewhere.
     * Examples:
     * Media status change for bulk, e.g. ready, not-ready, media changed, door opened.
     * Button state/joystick position/etc changes on a HID device.
     * Flow control status change on a communication device, e.g. CTS on serial
     */
    virtual void OnPoll() {}

    /**
     * Poll interface driver. You should override this in your driver if you
     * require polling faster or slower than every 100 milliseconds, or your
     * driver requires special housekeeping.
     */
    virtual void Poll()
    {
        OnPoll();
        qNextPollTime = millis() + 100;
    }

    virtual bool UHS_NI Polling()
    {
        return bPollEnable;
    }

    /**
     * This is only for a hub.
     * @param port
     */
    virtual void ResetHubPort(NOTUSED(uint8_t port))
    {}
};


#endif //_USBHOST_H_
#endif

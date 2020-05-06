
#include "USBCDC.h"
#include "pins_arduino.h"
#include <stdio.h>

using namespace x86Duino;

USBCDC::USBCDC() 
{
    USBDEV = NULL;
    _is_initialized = false;
    _nonblocking_writes = true;
}

void USBCDC::begin(uint32_t b) 
{
    if (USBDEV != NULL) return;
    //CDC
    USBDEV = CreateUSBDevice();
    if (USBDEV == NULL)
    {
        ::printf("USB CDC init error\n");
        return;
    }

    usb_SetUSBPins(USBDEV, 7, 0, 7, 1);
    usb_SetTimeOut(USBDEV, 0L, 500L); // USB RX timerout is 0ms and TX timeout is 500ms
    if (usb_Init(USBDEV) == false)
    {
        ::printf("USB CDC init2 error\n");
        return;
    }

    _is_initialized = true;
}
void USBCDC::begin(uint32_t b, uint16_t rxS, uint16_t txS) 
{
    /*
    As 86Duino have enough memory, we always create 4096 bytes RxQueue and TxQueue
    TX_QUEUE_SIZE and RX_QUEUE_SIZE are defined in USBCore.cpp
    */
    begin(b);
}

void USBCDC::end() 
{
    if (USBDEV == NULL) return;
    usb_Close(USBDEV);
    USBDEV = NULL;
    _is_initialized = false;
}

void USBCDC::flush() 
{
    if (USBDEV == NULL) return;
    usb_FlushTxQueue(USBDEV);
}

bool USBCDC::is_initialized() { return _is_initialized; }

void USBCDC::set_blocking_writes(bool blocking) 
{
    _nonblocking_writes = !blocking;
}

bool USBCDC::tx_pending() 
{
    if (USBDEV == NULL) return false;
    return (!usb_TxReady(USBDEV));
}

/* Empty implementations of Stream virtual methods */
uint32_t USBCDC::available() 
{
    if (USBDEV == NULL) return 0;
    return usb_QueryRxQueue(USBDEV);
}

uint32_t USBCDC::txspace()
{
    if (USBDEV == NULL) return 0;
    return TX_QUEUE_SIZE - usb_QueryTxQueue(USBDEV);
}

int16_t USBCDC::read() 
{ 
    int16_t c;
    if (USBDEV == NULL) return -1;
    c = usb_Read(USBDEV);
    return (c == 0xFFFF) ? -1 : c;
}

/* Empty implementations of Print virtual methods */
size_t USBCDC::write(uint8_t c) 
{
    if (USBDEV != NULL && usb_Ready(USBDEV) != false)
    {
        if (_nonblocking_writes && usb_TxQueueFull(USBDEV)) {
            return 0;
        }
        return (usb_Write(USBDEV, c) == true) ? 1 : 0;
    }

    return 0;
}

size_t USBCDC::write(const uint8_t* buffer, size_t size)
{
    if (USBDEV == NULL && usb_Ready(USBDEV) == false)
        return 0;

    size_t ret = 0;
    if (!_nonblocking_writes) {
        while (size--) {
            ret += write(*buffer++);
        }
        return ret;
    }

    // remaining queue space
    /*uint32_t space = txspace();
    if (space <= 0) {
        return 0;
    }*/

    ret = usb_SendSize(USBDEV, (uint8_t*)buffer, size);
    return (ret > 0) ? ret : 0;
}
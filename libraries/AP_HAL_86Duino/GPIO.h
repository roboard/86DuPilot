#pragma once

#include "AP_HAL_86Duino.h"

class x86Duino::GPIO : public AP_HAL::GPIO {
public:
    GPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t val);
    void    toggle(uint8_t pin);
	int8_t  analogPinToDigitalPin(uint8_t pin)
	{
		return -1;
	}
    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);

    void Close_Pwm(uint8_t pin);
};

class x86Duino::DigitalSource : public AP_HAL::DigitalSource {
public:
    DigitalSource(uint8_t pin);
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();
private:
    uint8_t _pin;
};

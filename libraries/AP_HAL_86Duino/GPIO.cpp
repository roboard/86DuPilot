
#include "GPIO.h"
#include "io.h"
#include "pins_arduino.h"

#define TRI_STATE     (0x00)
#define PULL_UP       (0x01)
#define PULL_DOWN     (0x02)
static int mc_md_inuse[PINS];

extern const AP_HAL::HAL& hal;

using namespace x86Duino;

GPIO::GPIO()
{}

void GPIO::init()
{
    int gpioBase;
    //set SB GPIO Base Address
    gpioBase = sb_Read16(SB_GPIOBASE) & 0xfffe;
    if (gpioBase == 0 || gpioBase == 0xfffe)
    {
        sb_Write16(SB_GPIOBASE, GPIOCTRLBASE | 0x01);
        gpioBase = GPIOCTRLBASE;
    }

    // Enable GPIO 0 ~ 9 
    io_outpdw(gpioBase, 0x03ff);

    // set GPIO Port 0~9 dircetion & data Address
    for (int8_t i = 0; i < 10; i++)
        io_outpdw(gpioBase + (i + 1) * 4, ((GPIODIRBASE + i * 4) << 16) + GPIODATABASE + i * 4);
}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
    int crossbar_bit;
    if (pin >= PINS || PIN86[pin].gpN == NOUSED) return;

    crossbar_bit = PIN86[pin].gpN;
    
    io_DisableINT();
    if (output == HAL_GPIO_INPUT)
    {
        io_outpb(CROSSBARBASE + 0x30 + crossbar_bit, TRI_STATE);
        io_outpb(GPIODIRBASE + 4 * (crossbar_bit / 8), io_inpb(GPIODIRBASE + 4 * (crossbar_bit / 8)) & ~(1 << (crossbar_bit % 8)));
    }
    else // OUTPUT 
        io_outpb(GPIODIRBASE + 4 * (crossbar_bit / 8), io_inpb(GPIODIRBASE + 4 * (crossbar_bit / 8)) | (1 << (crossbar_bit % 8)));

    io_RestoreINT();
}

uint8_t GPIO::read(uint8_t pin) {
    int crossbar_bit, val;
    if (pin >= PINS || PIN86[pin].gpN == NOUSED) return LOW;

    crossbar_bit = PIN86[pin].gpN;

//#if defined (DMP_DOS_BC) || defined (DMP_DOS_DJGPP) || defined (DMP_DOS_WATCOM)
//    if (pin == 32) timer1_pin32_isUsed = true;
//#endif

    io_DisableINT();

    if (crossbar_bit > 31)
        io_outpb(CROSSBARBASE + 0x80 + (crossbar_bit / 8), 0x01);
    else if (crossbar_bit <= 31 && io_inpb(CROSSBARBASE + 0x90 + crossbar_bit) != 0x01)
    {
        io_outpb(CROSSBARBASE + 0x90 + crossbar_bit, 0x01);
        Close_Pwm(pin);
    }

    val = io_inpb(GPIODATABASE + 4 * (crossbar_bit / 8)) & (1 << (crossbar_bit % 8));

    io_RestoreINT();

    if (val != 0) return HIGH;
    return LOW;
}

void GPIO::write(uint8_t pin, uint8_t val)
{
    unsigned int port;
    unsigned int value;
    int crossbar_bit;
    if (pin >= PINS || PIN86[pin].gpN == NOUSED) return;

//#if defined (DMP_DOS_BC) || defined (DMP_DOS_DJGPP) || defined (DMP_DOS_WATCOM)
//    if (pin == 32) timer1_pin32_isUsed = true;
//#endif

    crossbar_bit = PIN86[pin].gpN;
    port = GPIODATABASE + 4 * (crossbar_bit / 8);
    value = 1 << (crossbar_bit % 8);

    io_DisableINT();

    if (crossbar_bit > 31)
        io_outpb(CROSSBARBASE + 0x80 + (crossbar_bit / 8), 0x01);
    else if (crossbar_bit <= 31 && io_inpb(CROSSBARBASE + 0x90 + crossbar_bit) != 0x01)
    {
        io_outpb(CROSSBARBASE + 0x90 + crossbar_bit, 0x01);
        Close_Pwm(pin);
    }

    if (val == LOW)
        io_outpb(port, io_inpb(port) & (~value));
    else
        io_outpb(port, io_inpb(port) | value);

    io_RestoreINT();
}

void GPIO::toggle(uint8_t pin)
{
    write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(n);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {
    return true;
}

bool GPIO::usb_connected(void)
{
    return false;
}

void GPIO::Close_Pwm(uint8_t pin) {
    int mc, md;
    if (pin >= PINS || PIN86[pin].gpN == NOUSED) return;

    mc = PIN86[pin].PWMMC;
    md = PIN86[pin].PWMMD;

    io_DisableINT();

    if (mc_md_inuse[pin] == 1)
    {
        mcpwm_Disable(mc, md);
        mc_md_inuse[pin] = 0;
    }

    io_RestoreINT();
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
    hal.gpio->pinMode(_pin, output);
}

uint8_t DigitalSource::read() {
    return hal.gpio->read(_pin);
}

void DigitalSource::write(uint8_t value) {
    hal.gpio->write(_pin, value);
}

void DigitalSource::toggle() {
    write(!read());
}

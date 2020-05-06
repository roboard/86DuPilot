/*
    On 86Duino boards, the maximum input voltage of the analog to digital converter is 3.3V (In 86Duino One is pin A0~A6)
    and the battery module we used is 4.0V per battery. Therefore, we didn't support analogIn now.
    If you want to implement this, you should also modify AP_BATT_VOLT_PIN in AP_BattMonitor.
*/
#include "AnalogIn.h"
#include "io.h"
#include "dmpcfg.h"
#include "mcm.h"
#include "pins_arduino.h"

#define BaseAddress (0xfe00)
#define TimeOut		(1000)
#define MCM_MC      (0)
#define MCM_MD      (1)

#define ADC_RESOLUTION    (11) // for 86Duino 
#define PWM_RESOLUTION    (13) // for 86Duino 

extern const AP_HAL::HAL& hal;

using namespace x86Duino;

AnalogSource::AnalogSource(float v) :
    _v(v)
{}

float AnalogSource::read_average() {
    return _v;
}

float AnalogSource::voltage_average() {
    return 3.3f * _v / 2048.0f;
}

float AnalogSource::voltage_latest() {
    return 3.3f * _v / 2048.0f;
}

float AnalogSource::read_latest() {
    return _v;
}

void AnalogSource::set_pin(uint8_t p)
{}

void AnalogSource::set_stop_pin(uint8_t p)
{}

void AnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

AnalogIn::AnalogIn()
{}

void AnalogIn::init()
{
    sb_Write(0xbc, sb_Read(0xbc) & (~(1L << 28)));
    sb1_Write16(0xde, sb1_Read16(0xde) | 0x02);
    sb1_Write(0xe0, 0x0050fe00L);
    io_outpb(0xfe01, 0x00);
    for (int i = 0; (io_inpb(0xfe02) & 0x01) != 0 && i < 16; i++) io_inpb(0xfe04);
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {
    return new x86Duino::AnalogSource(1.11);
}

// need to port from wiring_analog.cpp/analogRead()
void AnalogIn::_timer_tick(void)
{
    
}

float AnalogIn::board_voltage(void)
{
    return 3.3f;
}

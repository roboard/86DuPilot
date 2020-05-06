#pragma once

#include "AP_HAL_86Duino.h"

#define ANALOG_MAX_CHANNELS 7

class x86Duino::AnalogSource : public AP_HAL::AnalogSource {
public:
    friend class x86Duino::AnalogIn;
    AnalogSource(float v);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }
private:
    float _v;
};

class x86Duino::AnalogIn : public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init() override;
    AP_HAL::AnalogSource* channel(int16_t n) override;
    void _timer_tick(void);
    float board_voltage(void) override;

};

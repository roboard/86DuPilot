#pragma once

#include "AP_HAL_86Duino.h"
#include "pins_arduino.h"

#define X86_NUM_OUTPUT_CHANNELS 4

class x86Duino::RCOutput : public AP_HAL::RCOutput {
public:
    RCOutput();
    void     init();
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
    void     cork(void) override;
    void     push(void) override;

private:
    typedef struct
    {
        uint8_t     _pin;    // 86duino pin
        bool        _enable;
        double    _duty;  // PWM duty in 10 ns
        double    _period; // PWM period in 10 ns
    } CH_Info;

    CH_Info  CH_List[X86_NUM_OUTPUT_CHANNELS];
    void pwmInit(int mcn, int mdn);
    void safeClosePwmModule(int mcn, int mdn, double period);
    void _init_ch(uint8_t ch, uint8_t pin, long microseconds);

    bool _corking = false;
    uint16_t pending[X86_NUM_OUTPUT_CHANNELS];
    uint32_t pending_mask;
};

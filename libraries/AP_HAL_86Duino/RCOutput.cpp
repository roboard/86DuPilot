#include <AP_HAL/AP_HAL.h>

#include "RCOutput.h"
#include "io.h"
#include "mcm.h"
#include "irq.h"

#define YES    (1)
#define NO     (2)

extern const AP_HAL::HAL& hal;

using namespace x86Duino;

RCOutput::RCOutput()
{
    _corking = false;
    pending_mask = 0;
}

void RCOutput::init() 
{
    // pin 29~32 map to channel 1~4 default frequenc = 50Hz (20000 us)
    _init_ch(CH_1, 29, 20000);
    _init_ch(CH_2, 30, 20000);
    _init_ch(CH_3, 31, 20000);
    _init_ch(CH_4, 32, 20000);
}

void RCOutput::_init_ch(uint8_t ch, uint8_t pin, long microseconds)
{
    int mcn, mdn;
    unsigned short crossbar_ioaddr = sb_Read16(0x64) & 0xfffe;

    if (pin >= PINS) return;

    mcn = PIN86[pin].PWMMC;
    mdn = PIN86[pin].PWMMD;

    if (mcn == NOPWM || mdn == NOPWM) return;
    if (microseconds <= 0 || microseconds > 21474836) return;

    CH_List[ch]._pin = pin;
    CH_List[ch]._enable = false;
    CH_List[ch]._duty = 1.0;
    CH_List[ch]._period = (double)microseconds * SYSCLK;

    io_DisableINT();

    pwmInit(mcn, mdn);

    if (mcpwm_ReadReloadPWM(mcn, mdn) != 0) mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_CANCEL);
    mcpwm_SetWidth(mcn, mdn, CH_List[ch]._period - 1.0, CH_List[ch]._duty - 1.0);
    mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_PEREND);

    io_RestoreINT();

    io_outpb(crossbar_ioaddr + 0x90 + PIN86[pin].gpN, 0x08); // Enable PWM pin
    mcpwm_Enable(mcn, mdn);

}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) 
{
    int mcn, mdn, pin;

    for (uint8_t ch = 0; ch < X86_NUM_OUTPUT_CHANNELS; ch++) {
        if (chmask & (1UL << ch)) {
            CH_List[ch]._period = 1000000UL / freq_hz * SYSCLK;
            pin = CH_List[ch]._pin;
            mcn = PIN86[pin].PWMMC;
            mdn = PIN86[pin].PWMMD;
            if (mcpwm_ReadReloadPWM(mcn, mdn) != 0) mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_CANCEL);
            mcpwm_SetWidth(mcn, mdn, CH_List[ch]._period - 1.0, CH_List[ch]._duty - 1.0);
            mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_PEREND);
        }
    }
}

uint16_t RCOutput::get_freq(uint8_t ch) 
{
    if (ch >= X86_NUM_OUTPUT_CHANNELS)
        return 0;
    else
        return 1000000 / CH_List[ch]._period * SYSCLK;
}

void RCOutput::enable_ch(uint8_t ch)
{
    if (ch >= X86_NUM_OUTPUT_CHANNELS)
        return;
    CH_List[ch]._enable = true;
}

void RCOutput::disable_ch(uint8_t ch)
{
    if (ch >= X86_NUM_OUTPUT_CHANNELS)
        return;
    CH_List[ch]._enable = false;
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= X86_NUM_OUTPUT_CHANNELS)
        return;
    if (CH_List[ch]._enable == false)
        return;

    if (_corking) {
        pending_mask |= (1U << ch);
        pending[ch] = period_us;
        return;
    }

    int mcn, mdn;
    unsigned short crossbar_ioaddr = sb_Read16(0x64) & 0xfffe;
    uint8_t pin = CH_List[ch]._pin;

    mcn = PIN86[pin].PWMMC;
    mdn = PIN86[pin].PWMMD;

    if (period_us <= 0)
    {
        if (CH_List[ch]._duty > 0) {
            CH_List[ch]._duty = 0;
            safeClosePwmModule(mcn, mdn, CH_List[ch]._period);
            hal.gpio->pinMode(pin, OUTPUT);
            hal.gpio->write(pin, LOW);
        }
        return;
    }

    io_DisableINT();
    if (CH_List[ch]._duty == 0)
    {
        pwmInit(mcn, mdn);
        
        // if (pin <= 9)
            // io_outpb(crossbar_ioaddr + 2, 0x01); // GPIO port2: 0A, 0B, 0C, 3A
        // else if (pin > 28)
            // io_outpb(crossbar_ioaddr, 0x03); // GPIO port0: 2A, 2B, 2C, 3C
        // else
            // io_outpb(crossbar_ioaddr + 3, 0x02); // GPIO port3: 1A, 1B, 1C, 3B
    }

    double duty = (double)period_us * SYSCLK;

    if (mcpwm_ReadReloadPWM(mcn, mdn) != 0) mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_CANCEL);
    mcpwm_SetWidth(mcn, mdn, CH_List[ch]._period - 1.0, duty - 1.0);
    mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_PEREND);
    io_RestoreINT();

    if (CH_List[ch]._duty == 0) {
        io_outpb(crossbar_ioaddr + 0x90 + PIN86[pin].gpN, 0x08); // Enable PWM pin
        mcpwm_Enable(mcn, mdn);
    }

    CH_List[ch]._duty = duty;

}

uint16_t RCOutput::read(uint8_t ch) 
{
    if (ch >= X86_NUM_OUTPUT_CHANNELS)
        return 0;
    else
        return CH_List[ch]._duty / SYSCLK;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput::cork(void)
{
    _corking = true;
}

void RCOutput::push(void)
{
    if (!_corking) {
        return;
    }
    _corking = false;
    for (uint8_t i = 0; i < X86_NUM_OUTPUT_CHANNELS; i++) {
        if (pending_mask & (1U << i)) {
            write(i, pending[i]);
        }
    }
    pending_mask = 0;
}

void RCOutput::pwmInit(int mcn, int mdn) {
    mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_CANCEL);
    mcpwm_SetOutMask(mcn, mdn, MCPWM_HMASK_NONE + MCPWM_LMASK_NONE);
    mcpwm_SetOutPolarity(mcn, mdn, MCPWM_HPOL_NORMAL + MCPWM_LPOL_NORMAL);
    mcpwm_SetDeadband(mcn, mdn, 0L);
    mcpwm_ReloadOUT_Unsafe(mcn, mdn, MCPWM_RELOAD_NOW);

    mcpwm_SetWaveform(mcn, mdn, MCPWM_EDGE_A0I1);
    mcpwm_SetSamplCycle(mcn, mdn, 1999L); // sample cycle: 20ms
}

void RCOutput::safeClosePwmModule(int mcn, int mdn, double period) {
    if (mcpwm_ReadReloadPWM(mcn, mdn) != 0) mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_CANCEL);
    mcpwm_SetWidth(mcn, mdn, period - 1.0, 0L);
    mcpwm_ReloadPWM(mcn, mdn, MCPWM_RELOAD_PEREND);
    while (mcpwm_ReadReloadPWM(mcn, mdn) != 0L);
    while (mcpwm_ReadSTATREG2(mcn, mdn) > (period - 1L));
    mcpwm_Disable(mcn, mdn);
}
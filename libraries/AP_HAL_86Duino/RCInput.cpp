
#include "RCInput.h"
#include "mcm.h"
#include "irq.h"
#include "io.h"
#include "pins_arduino.h"
#include <assert.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO

extern const AP_HAL::HAL& hal;

using namespace x86Duino;

static uint8_t used_irq = 0xff;
static char* name = "rcinputInt";

static void clear_INTSTATUS(int mc) {
    mc_outp(mc, 0x04, 0xff000000L); //for EX SIFB
}

#define MCMINT_ENABLE_REG  (0x00)
#define MCMINT_STAT_REG    (0x04)

// define SIFB interrupt bits for encoder mode
#define SIFB_TRIGRESETBIT      (31)
#define SIFB_USEREVTBIT        (30)
#define SIFB_PCNT_OV           (25)
#define SIFB_PCNT_UV           (24)

// define SIFB interrupt bits for pulse & capture mode
#define SIFB_CAP1INTBIT        (29)
#define SIFB_CAP2INTBIT        (30)
#define SIFB_CAP3INTBIT        (31)

static void enable_MCINT(int mc, int bit) {
    mc_outp(mc, 0x00, mc_inp(mc, 0x00) | (0x01 << bit));
}

static void disable_MCINT(int mc, int bit) {
    mc_outp(mc, 0x00, mc_inp(mc, 0x00) & ~(0x01 << bit));
}

static void clear_interrupt_state(int mc, int bit) {
    mc_outp(mc, MCMINT_STAT_REG, (0x01) << bit);
}

static bool check_interrupt_state(int mc, int bit) {
    if ((mc_inp(mc, MCMINT_STAT_REG) & (0x01 << bit)) != 0L) return true;
    return false;
}

static bool check_interrupt_enable(int mc, int bit) {
    if ((mc_inp(mc, MCMINT_ENABLE_REG) & (0x01 << bit)) != 0L) return true;
    return false;
}

static void (*(sifIntMode[3]))(int, int, unsigned long) = { mcpfau_SetCapMode1, mcpfau_SetCapMode2, mcpfau_SetCapMode3 };
static void (*(sifSetInt[3]))(int, int, unsigned long) = { mcpfau_SetCap1INT, mcpfau_SetCap2INT, mcpfau_SetCap3INT };
static unsigned long (*(readCapStat[3]))(int, int) = { mcpfau_ReadCAPSTAT1, mcpfau_ReadCAPSTAT2, mcpfau_ReadCAPSTAT3 };
static unsigned long (*(readCapFIFO[3]))(int, int, unsigned long*) = { mcpfau_ReadCAPFIFO1, mcpfau_ReadCAPFIFO2, mcpfau_ReadCAPFIFO3 };
static volatile unsigned long _mcmode[4] = { MODE_NOSET, MODE_NOSET, MODE_NOSET, MODE_NOSET };
static volatile unsigned long long int pulseHighData[4][3] = { 0 };
static unsigned long long int ovdata[3] = { 0, 0, 0 };
static volatile int _ch_flag = 0x00;
static volatile unsigned long long _rc_values[RC_INPUT_MAX_CH + 1] = { 0 }; // +1 for reserving channel 9

static void _filterAndSampleWindowInit(int mc, int md) {
    mcsif_SetInputFilter(mc, md, 0L);
    mcsif_SetSWDeadband(mc, md, 0L);
    mcsif_SetSWPOL(mc, md, MCSIF_SWPOL_REMAIN);
    mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_DISABLE + MCSIF_SWEND_NOW);
    mcsif_SetSamplWin(mc, md, MCSIF_SWSTART_NOW + MCSIF_SWEND_DISABLE);
}

RCInput::RCInput()
{
    _init = false;
}

static int _user_int(int irq, void* data) {
    int i, mc, ch, irq_handled = 0;
    unsigned long capdata, stat;

    // detect all sensor interface
    for (mc = 1; mc < 4; mc++)  // only use MC_MODULE1, MC_MODULE2 and MC_MODULE3
    {
        if (_mcmode[mc] == MODE_CAPTURE)
        {
            for (i = 0; i < 3; i++)
            {
                if (check_interrupt_enable(mc, SIFB_CAP1INTBIT + i) == true && check_interrupt_state(mc, SIFB_CAP1INTBIT + i) == true) // USER EVT
                {
                    clear_interrupt_state(mc, SIFB_CAP1INTBIT + i);
                    irq_handled |= 0x04;
					
                    while (readCapStat[i](mc, MCSIF_MODULEB) != MCPFAU_CAPFIFO_EMPTY)
                    {
                        stat = readCapFIFO[i](mc, MCSIF_MODULEB, &capdata);
                        if (stat == MCPFAU_CAP_CAPCNT_OVERFLOW)
                            ovdata[i] += 0x10000000L;
                        else if (stat == MCPFAU_CAP_1TO0EDGE)
                        {
                            pulseHighData[mc][i] = capdata;
                            ovdata[i] = 0L;
							if (mc == 1)
								ch = 6 + i;	// MC_MODULE1 (pin 18, 19) for channel 7 and 8
							else 
								ch = (mc - 2) * 3 + i;
							

							// reserve channel 9
                            if (ch != 8 && _rc_values[ch] != pulseHighData[mc][i]) {
                                _rc_values[ch] = pulseHighData[mc][i];
                                _ch_flag |= (0x01 << ch);
                            }   
                        }
                    }
					
                }
            }
        }
    }
    if (irq_handled == 0x00) return ISR_NONE;
    return ISR_HANDLED;
}


static bool _interrupt_init(int mc) {
    if (used_irq == 0xff)
    {
        used_irq = GetMCIRQ();
        if (irq_InstallISR(used_irq, _user_int, (void*)name) == false)
        {
            hal.console->printf("irq_install fail\n"); return false;
        }
    }

    // enable mcm general interrupt function
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << mc));
    return true;
}

static void _open_encoder_pin(int mc, int pin) {
    unsigned short crossbar_ioaddr;
    crossbar_ioaddr = sb_Read16(0x64) & 0xfffe;
    io_outpb(crossbar_ioaddr + 0x90 + PIN86[INTPINSMAP[mc * 3 + pin]].gpN, 0x08);//RICH IO
}

void RCInput::init()
{
    // Capture mode init
	_pcapInit(MC_MODULE1, MCSIF_MODULEB);
    _pcapInit(MC_MODULE2, MCSIF_MODULEB);
    _pcapInit(MC_MODULE3, MCSIF_MODULEB);

    // attach interrupt
    // init ISR
	if (_interrupt_init(MC_MODULE1) == false) return;
    if (_interrupt_init(MC_MODULE2) == false) return;
    if (_interrupt_init(MC_MODULE3) == false) return;

	mcsif_Disable(MC_MODULE1, MCSIF_MODULEB);
    mcsif_Disable(MC_MODULE2, MCSIF_MODULEB);
    mcsif_Disable(MC_MODULE3, MCSIF_MODULEB);

	clear_INTSTATUS(MC_MODULE1);
    clear_INTSTATUS(MC_MODULE2);
    clear_INTSTATUS(MC_MODULE3);

	enable_MCINT(MC_MODULE1, SIFB_CAP1INTBIT);
	enable_MCINT(MC_MODULE1, SIFB_CAP2INTBIT);
	enable_MCINT(MC_MODULE1, SIFB_CAP3INTBIT);
    enable_MCINT(MC_MODULE2, SIFB_CAP1INTBIT);
    enable_MCINT(MC_MODULE2, SIFB_CAP2INTBIT);
    enable_MCINT(MC_MODULE2, SIFB_CAP3INTBIT);
    enable_MCINT(MC_MODULE3, SIFB_CAP1INTBIT);
    enable_MCINT(MC_MODULE3, SIFB_CAP2INTBIT);
    enable_MCINT(MC_MODULE3, SIFB_CAP3INTBIT);

    // Enable interrupt option
    for (int i = 0; i < 3; i++)
    {
		sifIntMode[i](MC_MODULE1, MCSIF_MODULEB, MCPFAU_CAP_BOTH_CLEAR);
		sifSetInt[i](MC_MODULE1, MCSIF_MODULEB, 1L);   // one pwm period is about 15ms depending on EX hardware
        sifIntMode[i](MC_MODULE2, MCSIF_MODULEB, MCPFAU_CAP_BOTH_CLEAR);
        sifSetInt[i](MC_MODULE2, MCSIF_MODULEB, 1L);   // one pwm period is about 15ms depending on EX hardware
        sifIntMode[i](MC_MODULE3, MCSIF_MODULEB, MCPFAU_CAP_BOTH_CLEAR);
        sifSetInt[i](MC_MODULE3, MCSIF_MODULEB, 1L);
    }

	_open_encoder_pin(MC_MODULE1, 0);
	_open_encoder_pin(MC_MODULE1, 1);
	_open_encoder_pin(MC_MODULE1, 2);
    _open_encoder_pin(MC_MODULE2, 0);
    _open_encoder_pin(MC_MODULE2, 1);
    _open_encoder_pin(MC_MODULE2, 2);
    _open_encoder_pin(MC_MODULE3, 0);
    _open_encoder_pin(MC_MODULE3, 1);
    _open_encoder_pin(MC_MODULE3, 2);

	mcsif_Enable(MC_MODULE1, MCSIF_MODULEB);
    mcsif_Enable(MC_MODULE2, MCSIF_MODULEB);
    mcsif_Enable(MC_MODULE3, MCSIF_MODULEB);
    
    _init = true;
    return;

}

bool RCInput::new_input() {
    if (!_init || _ch_flag == 0)
        return false;
    return true;
}

uint8_t RCInput::num_channels() {
    if (!_init) 
        return 0;
    return RC_INPUT_MAX_CH;
}

uint16_t RCInput::read(uint8_t ch) {
	if (!_init || ch > RC_INPUT_MAX_CH) {
		return 0;
	}
	io_DisableINT();
    _ch_flag &= ~(0x01 << ch);
	uint16_t rcinp = _rc_values[ch] / 100;
	io_RestoreINT();
    return rcinp;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len) {
    if (!_init)
        return 0;
    if (len > RC_INPUT_MAX_CH)
        len = RC_INPUT_MAX_CH;
    for (uint8_t i = 0; i < len; i++){
        periods[i] = read(i);
    }
    return len;
}

void RCInput::_pcapInit(int16_t mcn, int16_t mdn) {
    _filterAndSampleWindowInit(mcn, mdn);

    mcsif_SetMode(mcn, mdn, MCSIF_PFAU);

    mcpfau_SetCapMode1(mcn, mdn, MCPFAU_CAP_BOTH_CLEAR);
    mcpfau_SetCapInterval1(mcn, mdn, 1L);
    mcpfau_SetCap1INT(mcn, mdn, 0L);
    mcpfau_SetPolarity1(mcn, mdn, MCPFAU_POL_NORMAL);
    mcpfau_SetMask1(mcn, mdn, MCPFAU_MASK_NONE);
    mcpfau_SetRLDTRIG1(mcn, mdn, MCPFAU_RLDTRIG_DISABLE);
    mcpfau_SetFAU1TRIG(mcn, mdn, MCPFAU_FAUTRIG_INPUT1);
    mcpfau_SetFAU1RELS(mcn, mdn, MCPFAU_FAURELS_INPUT0);

    mcpfau_SetCapMode2(mcn, mdn, MCPFAU_CAP_BOTH_CLEAR);
    mcpfau_SetCapInterval2(mcn, mdn, 1L);
    mcpfau_SetCap2INT(mcn, mdn, 0L);
    mcpfau_SetPolarity2(mcn, mdn, MCPFAU_POL_NORMAL);
    mcpfau_SetMask2(mcn, mdn, MCPFAU_MASK_NONE);
    mcpfau_SetRLDTRIG2(mcn, mdn, MCPFAU_RLDTRIG_DISABLE);
    mcpfau_SetFAU2TRIG(mcn, mdn, MCPFAU_FAUTRIG_INPUT1);
    mcpfau_SetFAU2RELS(mcn, mdn, MCPFAU_FAURELS_INPUT0);

    mcpfau_SetCapMode3(mcn, mdn, MCPFAU_CAP_BOTH_CLEAR);
    mcpfau_SetCapInterval3(mcn, mdn, 1L);
    mcpfau_SetCap3INT(mcn, mdn, 0L);
    mcpfau_SetPolarity3(mcn, mdn, MCPFAU_POL_NORMAL);
    mcpfau_SetMask3(mcn, mdn, MCPFAU_MASK_NONE);
    mcpfau_SetRLDTRIG3(mcn, mdn, MCPFAU_RLDTRIG_DISABLE);
    mcpfau_SetFAU3TRIG(mcn, mdn, MCPFAU_FAUTRIG_INPUT1);
    mcpfau_SetFAU3RELS(mcn, mdn, MCPFAU_FAURELS_INPUT0);

    io_DisableINT();
    _mcmode[mcn] = MODE_CAPTURE;
    io_RestoreINT();
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_86DUINO


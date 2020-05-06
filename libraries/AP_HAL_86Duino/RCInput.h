#pragma once

#include "AP_HAL_86Duino.h"

#define RC_INPUT_MAX_CH 8 // CH_7 & CH_8 are software simulate

// Encoder mode
#define MODE_NOSET       (0xFF)
#define MODE_STEP_DIR    (0)
#define MODE_CWCCW       (1)
#define MODE_AB_PHASE    (2)
#define MODE_CAPTURE     (3)
#define MODE_SSI         (4) // continue mode, no interrupt event
#define MODE_STEP_DIR_x2 (5) // another mode
#define MODE_CWCCW_x2    (6) // another mode
#define MODE_AB_PHASE_x2 (7) // another mode


class x86Duino::RCInput : public AP_HAL::RCInput {
public:
    RCInput();
    void init();
    bool  new_input();
    uint8_t num_channels();
    uint16_t read(uint8_t ch);
    uint8_t read(uint16_t* periods, uint8_t len);

	bool set_overrides(int16_t* overrides, uint8_t len) {
		return true;
	}
	bool set_override(uint8_t channel, int16_t override) {
		return true;
	}
	void clear_overrides() {}
private:
    bool _init;

    void _pcapInit(int16_t mcn, int16_t mdn);
};

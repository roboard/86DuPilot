#pragma once

#include "AP_HAL_86Duino.h"
#include "io.h"
#include "com.h"

class x86Duino::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(int com_port, unsigned long com_baudrate, unsigned char com_format, unsigned long com_rxtimeout, unsigned long com_txtimeout);
    void begin(uint32_t baud);
    void begin(uint32_t baud, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();
    void set_flow_control(enum flow_control flow_control_setting) ;
    enum flow_control get_flow_control(void) { return _flow_control; }

    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

private:
    enum flow_control _flow_control;
	bool _nonblocking_writes;
    int port;
    unsigned long baudrate;
    unsigned char format;
    unsigned long rxtimeout;
    unsigned long txtimeout;
    COMPort *handle;
};

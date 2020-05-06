
#include "Semaphores.h"
#include "io.h"

extern const AP_HAL::HAL& hal;

namespace x86Duino {

bool Semaphore::give() {
    if (_taken) {
        _taken = false;
        return true;
    } else {
        return false;
    }
}

bool Semaphore::take(uint32_t timeout_ms) {
    if (hal.scheduler->in_timerprocess()) {
        AP_HAL::panic("PANIC: Semaphore::take used from "
            "inside timer process");
        return false; /* Never reached - panic does not return */
    }
    return _take_from_mainloop(timeout_ms);
}

bool Semaphore::take_nonblocking() {
    /* No syncronisation primitives to garuntee this is correct */
    if (hal.scheduler->in_timerprocess()) {
        return _take_nonblocking();
    }
    else {
        return _take_from_mainloop(0);
    }
}

bool Semaphore::_take_from_mainloop(uint32_t timeout_ms) {
    /* Try to take immediately */
    if (_take_nonblocking()) {
        return true;
    }
    else if (timeout_ms == 0) {
        /* Return immediately if timeout is 0 */
        return false;
    }

    uint16_t timeout_ticks = timeout_ms * 10;
    do {
        /* Delay 1ms until we can successfully take, or we timed out */
        hal.scheduler->delay_microseconds(100);
        timeout_ticks--;
        if (_take_nonblocking()) {
            return true;
        }
    } while (timeout_ticks > 0);

    return false;
}

bool Semaphore::_take_nonblocking() {
    bool result = false;
    io_DisableINT();
    if (!_taken) {
        _taken = true;
        result = true;
    }
    io_RestoreINT();
    return result;
}

}


#pragma once

#include "AP_HAL_86Duino.h"
#include <stdio.h>
#include <unistd.h>
#include <AP_Common/Bitmask.h>

#define X86_STORAGE_LINE_SHIFT 9
#define X86_STORAGE_LINE_SIZE (1<<X86_STORAGE_LINE_SHIFT)
#define X86_STORAGE_NUM_LINES (HAL_STORAGE_SIZE/X86_STORAGE_LINE_SIZE)

class x86Duino::Storage : public AP_HAL::Storage {
public:
    Storage();
    void init();
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);
    void _timer_tick(void);

private:
    volatile bool _initialised;
    FILE *_file;
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[HAL_STORAGE_SIZE] __attribute__((aligned(4)));
    Bitmask _dirty_mask{X86_STORAGE_NUM_LINES};
    void _mtd_load(void);
    void _mtd_write(uint16_t line);
    void _init_file();
    void _check_file();

};

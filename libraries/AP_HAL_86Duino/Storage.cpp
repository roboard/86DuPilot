#include <AP_HAL/AP_HAL.h>
#include <string.h>
#include <errno.h>
#include "io.h"
#include "Storage.h"

extern const AP_HAL::HAL& hal;

namespace x86Duino {

using namespace std;
#define MTD_PARAMS_FILE "PARM.bin"

Storage::Storage()
{
    _file = nullptr ;
}

void Storage::init()
{
    _check_file();
}

void Storage::_storage_open(void)
{
    if (_initialised) return;

    _dirty_mask.clearall();
    _mtd_load();
    _initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
    uint16_t end = loc + length;
    for (uint16_t line=loc>>X86_STORAGE_LINE_SHIFT;
         line <= end>>X86_STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised || _dirty_mask.empty()) {
        return;
    }

    if (_file == nullptr) {
        _file = fopen(MTD_PARAMS_FILE, "rb+");
        if (_file == nullptr) {
            return;
        }
    }
    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<X86_STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == X86_STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

    // save to storage backend
    _mtd_write(i);

}

/*
  write one storage line. This also updates _dirty_mask.
*/
void Storage::_mtd_write(uint16_t line)
{
    uint16_t ofs = line * X86_STORAGE_LINE_SIZE;
    fseek(_file, ofs, SEEK_SET);
    if (ftell(_file) != ofs) {
        return;
    }

    // mark the line clean
    _dirty_mask.clear(line);

    ssize_t ret = fwrite(&_buffer[ofs], 1, X86_STORAGE_LINE_SIZE, _file);

    if (ret != X86_STORAGE_LINE_SIZE) {
        // write error - likely EINTR
        _dirty_mask.set(line);
        fclose(_file);
        _file = nullptr;
    }

    // !!! not sure about the time consumption
    fflush(_file);  // flush buffer to OS buffer
    //fsync(fileno(_file)); // flush OS buffer to SD card

}

/*
  load all data from mtd
 */
void Storage::_mtd_load(void)
{
//    int fd = open(MTD_PARAMS_FILE, O_RDONLY);
    FILE *file ;
    file = fopen(MTD_PARAMS_FILE, "rb");
    if (file == nullptr) {
        AP_HAL::panic("Failed to open " MTD_PARAMS_FILE "\n");
    }

    // reset buffer frist
    memset( _buffer, 0 , sizeof(_buffer));
    ssize_t ret = fread(_buffer, 1, sizeof(_buffer), file);
    if (ret != HAL_STORAGE_SIZE) {
        printf("Failed to read " MTD_PARAMS_FILE "\n");
    }

    fclose(file);
}

void Storage::_check_file()
{
    FILE *file ;
    file = fopen(MTD_PARAMS_FILE, "rb");
    // file exsitance
    if (file == nullptr)
    {
        _init_file() ;
        return;
    }

    // file size check
    fseek(file, 0, SEEK_END);
    int size = ftell(file);
    if( size != HAL_STORAGE_SIZE)
    {
        fclose( file );
        _init_file();
    }
    fclose( file );
}

void Storage::_init_file( )
{
    FILE *file ;
    file = fopen(MTD_PARAMS_FILE, "wb");
    memset( _buffer, 0 , sizeof(_buffer));  // reset buffer
    ssize_t ret = fwrite(_buffer, 1, sizeof(_buffer), file);
    if( ret != HAL_STORAGE_SIZE )
        printf("init file error\n");
    fflush(_file);  // flush buffer to OS buffer
    fclose(file);
    
    // test size for new file
    file = fopen(MTD_PARAMS_FILE, "rb");
    const uint16_t chunk_size = 128;
    for (uint16_t ofs=0; ofs<sizeof(_buffer); ofs += chunk_size) {
        ret = fread(&_buffer[ofs], 1 , chunk_size, file);
        if (ret != chunk_size) {
            printf("storage read of %u bytes at %u to %p failed - got %d \n",
                     (unsigned)sizeof(_buffer), (unsigned)ofs, &_buffer[ofs], (int)ret);
        }
    }
    fclose(file);
    
    file = fopen(MTD_PARAMS_FILE, "rb");
    memset( _buffer, 0 , sizeof(_buffer));
    ret = fread(_buffer, 1, sizeof(_buffer), file);
    if (ret != HAL_STORAGE_SIZE) {
        printf("Failed to read at a time\n");
    }
    fclose(file);
    printf("size check done\n");
}

}

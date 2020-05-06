/*******************************************************************************

  io_mmio.h - Part of DM&P Vortex86 Base I/O Library
  Copyright (c) 2013 AAA <aaa@dmp.com.tw>. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  (If you need a commercial license, please contact soc@dmp.com.tw 
   to get more information.)

*******************************************************************************/


/***************************  Primitive MMIO Access  **************************/
__dmp_inline(void) mmio_outpdw(IO_BASE_t* base, unsigned long offset, unsigned long val) {
    #if   defined   DMP_DOS_DJGPP
        _farpokel((unsigned short)base->mmio_selector, offset, val);
    #elif defined   USE_PCIDEBUG
        _MemWriteLong(base->addr + offset, val);
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        *(volatile unsigned long*)(base->addr + offset) = val;
        ATOMIC_MEM_BARRIER();
    #endif
}

__dmp_inline(unsigned long) mmio_inpdw(IO_BASE_t* base, unsigned long offset) {
    #if   defined   DMP_DOS_DJGPP
        return _farpeekl((unsigned short)base->mmio_selector, offset);
    #elif defined   USE_PCIDEBUG
        return _MemReadLong(base->addr + offset);
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        unsigned long result = *(volatile unsigned long*)(base->addr + offset); 
        ATOMIC_MEM_BARRIER();
        return result;
    #else
        return 0L;
    #endif
}

__dmp_inline(void) mmio_outpw(IO_BASE_t* base, unsigned long offset, unsigned short val) {
    #if   defined   DMP_DOS_DJGPP
        _farpokew((unsigned short)base->mmio_selector, offset, val);
    #elif defined   USE_PCIDEBUG
        _MemWriteShort(base->addr + offset, val);
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        *(volatile unsigned short*)(base->addr + offset) = val;
        ATOMIC_MEM_BARRIER();        
    #endif
}

__dmp_inline(unsigned short) mmio_inpw(IO_BASE_t* base, unsigned long offset) {
    #if   defined   DMP_DOS_DJGPP
        return _farpeekw((unsigned short)base->mmio_selector, offset);
    #elif defined   USE_PCIDEBUG
        return _MemReadShort(base->addr + offset);
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        unsigned short result = *(volatile unsigned short*)(base->addr + offset);
        ATOMIC_MEM_BARRIER();
        return result;
    #else
        return 0;
    #endif
}

__dmp_inline(void) mmio_outpb(IO_BASE_t* base, unsigned long offset, unsigned char val) {
    #if   defined   DMP_DOS_DJGPP
        _farpokeb((unsigned short)base->mmio_selector, offset, val);
    #elif defined   USE_PCIDEBUG
        _MemWriteChar(base->addr + offset, val);
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        *(volatile unsigned char*)(base->addr + offset) = val;
        ATOMIC_MEM_BARRIER();        
    #endif
}

__dmp_inline(unsigned char) mmio_inpb(IO_BASE_t* base, unsigned long offset) {
    #if   defined   DMP_DOS_DJGPP
        return _farpeekb((unsigned short)base->mmio_selector, offset);
    #elif defined   USE_PCIDEBUG
        return _MemReadChar(base->addr + offset);
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        unsigned char result = *(volatile unsigned char*)(base->addr + offset);
        ATOMIC_MEM_BARRIER();
        return result;
    #else
        return 0;
    #endif
}


__dmp_inline(void) mmio_seq_outpdw(IO_BASE_t* base, unsigned long offset, unsigned long* data, int cnt) {
    #if   defined   DMP_DOS_DJGPP
        int n = (cnt + 7) >> 3;

        // loop unrolling with Duff's Device (refer to https://en.wikipedia.org/wiki/Duff's_device)
        _farsetsel(base->mmio_selector);
        switch (cnt & 0x07) 
        {
            case 0: do { _farnspokel(offset, *data++);
            case 7:      _farnspokel(offset, *data++);
            case 6:      _farnspokel(offset, *data++);
            case 5:      _farnspokel(offset, *data++);
            case 4:      _farnspokel(offset, *data++);
            case 3:      _farnspokel(offset, *data++);
            case 2:      _farnspokel(offset, *data++);
            case 1:      _farnspokel(offset, *data++);
                    } while (--n > 0);
        }
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        int n = (cnt + 7) >> 3;
        volatile unsigned long* reg = (volatile unsigned long*)(base->addr + offset);
        data = data - ((0 - cnt) & 0x07);
        switch (cnt & 0x07) 
        {
            case 0: do { *reg = data[0];
            case 7:      *reg = data[1];
            case 6:      *reg = data[2];
            case 5:      *reg = data[3];
            case 4:      *reg = data[4];
            case 3:      *reg = data[5];
            case 2:      *reg = data[6];
            case 1:      *reg = data[7];
                         data = data + 8;
                    } while (--n > 0);
        }
        ATOMIC_MEM_BARRIER();
    #else
        for ( ; cnt > 0; data++, cnt--) mmio_outpdw(base, offset, *data);
    #endif
}

__dmp_inline(void) mmio_seq_inpdw(IO_BASE_t* base, unsigned long offset, unsigned long* buf, int cnt) {
    #if   defined   DMP_DOS_DJGPP
        int n = (cnt + 7) >> 3;

        // loop unrolling with Duff's Device (refer to https://en.wikipedia.org/wiki/Duff's_device)
        _farsetsel(base->mmio_selector);
        switch (cnt & 0x07) 
        {
            case 0: do { *buf++ = _farnspeekl(offset);
            case 7:      *buf++ = _farnspeekl(offset);
            case 6:      *buf++ = _farnspeekl(offset);
            case 5:      *buf++ = _farnspeekl(offset);
            case 4:      *buf++ = _farnspeekl(offset);
            case 3:      *buf++ = _farnspeekl(offset);
            case 2:      *buf++ = _farnspeekl(offset);
            case 1:      *buf++ = _farnspeekl(offset);
                    } while (--n > 0);
        }
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        int n = (cnt + 7) >> 3;
        volatile unsigned long* reg = (volatile unsigned long*)(base->addr + offset);
        buf = buf - ((0 - cnt) & 0x07);
        switch (cnt & 0x07) 
        {
            case 0: do { buf[0] = *reg;
            case 7:      buf[1] = *reg;
            case 6:      buf[2] = *reg;
            case 5:      buf[3] = *reg;
            case 4:      buf[4] = *reg;
            case 3:      buf[5] = *reg;
            case 2:      buf[6] = *reg;
            case 1:      buf[7] = *reg;
                         buf = buf + 8;
                    } while (--n > 0);
        }
        ATOMIC_MEM_BARRIER();
    #else
        for ( ; cnt > 0; buf++, cnt--) *buf = mmio_inpdw(base, offset);
    #endif
}

__dmp_inline(void) mmio_seq_outpw(IO_BASE_t* base, unsigned long offset, unsigned short* data, int cnt) {
    #if   defined   DMP_DOS_DJGPP
        int n = (cnt + 7) >> 3;
        _farsetsel(base->mmio_selector);
        switch (cnt & 0x07) 
        {
            case 0: do { _farnspokew(offset, *data++);
            case 7:      _farnspokew(offset, *data++);
            case 6:      _farnspokew(offset, *data++);
            case 5:      _farnspokew(offset, *data++);
            case 4:      _farnspokew(offset, *data++);
            case 3:      _farnspokew(offset, *data++);
            case 2:      _farnspokew(offset, *data++);
            case 1:      _farnspokew(offset, *data++);
                    } while (--n > 0);
        }
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        int n = (cnt + 7) >> 3;
        volatile unsigned short* reg = (volatile unsigned short*)(base->addr + offset);
        data = data - ((0 - cnt) & 0x07);
        switch (cnt & 0x07) 
        {
            case 0: do { *reg = data[0];
            case 7:      *reg = data[1];
            case 6:      *reg = data[2];
            case 5:      *reg = data[3];
            case 4:      *reg = data[4];
            case 3:      *reg = data[5];
            case 2:      *reg = data[6];
            case 1:      *reg = data[7];
                         data = data + 8;
                    } while (--n > 0);
        }
        ATOMIC_MEM_BARRIER();
    #else
        for ( ; cnt > 0; data++, cnt--) mmio_outpw(base, offset, *data);
    #endif
}

__dmp_inline(void) mmio_seq_inpw(IO_BASE_t* base, unsigned long offset, unsigned short* buf, int cnt) {
    #if   defined   DMP_DOS_DJGPP
        int n = (cnt + 7) >> 3;
        _farsetsel(base->mmio_selector);
        switch (cnt & 0x07) 
        {
            case 0: do { *buf++ = _farnspeekw(offset);
            case 7:      *buf++ = _farnspeekw(offset);
            case 6:      *buf++ = _farnspeekw(offset);
            case 5:      *buf++ = _farnspeekw(offset);
            case 4:      *buf++ = _farnspeekw(offset);
            case 3:      *buf++ = _farnspeekw(offset);
            case 2:      *buf++ = _farnspeekw(offset);
            case 1:      *buf++ = _farnspeekw(offset);
                    } while (--n > 0);
        }
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        int n = (cnt + 7) >> 3;
        volatile unsigned short* reg = (volatile unsigned short*)(base->addr + offset);
        buf = buf - ((0 - cnt) & 0x07);
        switch (cnt & 0x07) 
        {
            case 0: do { buf[0] = *reg;
            case 7:      buf[1] = *reg;
            case 6:      buf[2] = *reg;
            case 5:      buf[3] = *reg;
            case 4:      buf[4] = *reg;
            case 3:      buf[5] = *reg;
            case 2:      buf[6] = *reg;
            case 1:      buf[7] = *reg;
                         buf = buf + 8;
                    } while (--n > 0);
        }
        ATOMIC_MEM_BARRIER();
    #else
        for ( ; cnt > 0; buf++, cnt--) *buf = mmio_inpw(base, offset);
    #endif
}

__dmp_inline(void) mmio_seq_outpb(IO_BASE_t* base, unsigned long offset, unsigned char* data, int cnt) {
    #if   defined   DMP_DOS_DJGPP
        int n = (cnt + 7) >> 3;
        _farsetsel(base->mmio_selector);
        switch (cnt & 0x07) 
        {
            case 0: do { _farnspokeb(offset, *data++);
            case 7:      _farnspokeb(offset, *data++);
            case 6:      _farnspokeb(offset, *data++);
            case 5:      _farnspokeb(offset, *data++);
            case 4:      _farnspokeb(offset, *data++);
            case 3:      _farnspokeb(offset, *data++);
            case 2:      _farnspokeb(offset, *data++);
            case 1:      _farnspokeb(offset, *data++);
                    } while (--n > 0);
        }
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        int n = (cnt + 7) >> 3;
        volatile unsigned char* reg = (volatile unsigned char*)(base->addr + offset);
        data = data - ((0 - cnt) & 0x07);
        switch (cnt & 0x07) 
        {
            case 0: do { *reg = data[0];
            case 7:      *reg = data[1];
            case 6:      *reg = data[2];
            case 5:      *reg = data[3];
            case 4:      *reg = data[4];
            case 3:      *reg = data[5];
            case 2:      *reg = data[6];
            case 1:      *reg = data[7];
                         data = data + 8;
                    } while (--n > 0);
        }
        ATOMIC_MEM_BARRIER();
    #else
        for ( ; cnt > 0; data++, cnt--) mmio_outpb(base, offset, *data);
    #endif
}

__dmp_inline(void) mmio_seq_inpb(IO_BASE_t* base, unsigned long offset, unsigned char* buf, int cnt) {
    #if   defined   DMP_DOS_DJGPP
        int n = (cnt + 7) >> 3;
        _farsetsel(base->mmio_selector);
        switch (cnt & 0x07) 
        {
            case 0: do { *buf++ = _farnspeekb(offset);
            case 7:      *buf++ = _farnspeekb(offset);
            case 6:      *buf++ = _farnspeekb(offset);
            case 5:      *buf++ = _farnspeekb(offset);
            case 4:      *buf++ = _farnspeekb(offset);
            case 3:      *buf++ = _farnspeekb(offset);
            case 2:      *buf++ = _farnspeekb(offset);
            case 1:      *buf++ = _farnspeekb(offset);
                    } while (--n > 0);
        }
    #elif defined(DMP_DOS_WATCOM) || defined(USE_WINIO3) || defined(USE_PHYMEM) || defined(DMP_LINUX)
        int n = (cnt + 7) >> 3;
        volatile unsigned char* reg = (volatile unsigned char*)(base->addr + offset);
        buf = buf - ((0 - cnt) & 0x07);
        switch (cnt & 0x07) 
        {
            case 0: do { buf[0] = *reg;
            case 7:      buf[1] = *reg;
            case 6:      buf[2] = *reg;
            case 5:      buf[3] = *reg;
            case 4:      buf[4] = *reg;
            case 3:      buf[5] = *reg;
            case 2:      buf[6] = *reg;
            case 1:      buf[7] = *reg;
                         buf = buf + 8;
                    } while (--n > 0);
        }
        ATOMIC_MEM_BARRIER();
    #else
        for ( ; cnt > 0; buf++, cnt--) *buf = mmio_inpb(base, offset);
    #endif
}
/*-----------------------  end. Primitive MMIO Access  -----------------------*/

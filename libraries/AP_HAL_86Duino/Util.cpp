#include "Util.h"
#include "io.h"
#include "mcm.h"
#include "irq.h"
#include <cmath>
#include <dos.h>
#include <stdlib.h>
#include <assert.h>

#define SYSTEM_RESET    (0)
#define INTERRUPT       (1)

#define WDTIRQ    (7)

extern const AP_HAL::HAL& hal ;

namespace x86Duino {

#ifndef PRIu64
#define PRIu64 "llu"
#endif

static struct wdt_status {
	unsigned char ctrl_reg;   // 0xA8
	unsigned char sigsel_reg; // 0xA9
	unsigned char count0_reg; // 0xAA
	unsigned char count1_reg; // 0xAB
	unsigned char count2_reg; // 0xAC
	unsigned char stat_reg;   // 0xAD
	unsigned char reload_reg; // 0xAE
} WDT_t;

static inline uint64_t now_nsec()
{
    uint64_t nowclocks;
    __asm__ __volatile__ ("rdtsc" : "=A"(nowclocks) );
    return (nowclocks*1000)/vx86_CpuCLK();
}

Util::Util():
	timerWDTEnable(false),
	timerWDTInit(false),
	timerWDTIntEnable(false),
	_wdt_proc({ nullptr }),
	_num_wdt_procs(0),
	_in_wdt_proc(false)
{
    /* TODO: this number should come from vehicle code - just estimate the
     * number of perf counters for now; if we grow more, it will just
     * reallocate the memory pool */
    _perf_counters.reserve(50);
	_init_perf = false;
}

void Util::init_perf()
{
	_init_perf = true;
}

enum Util::safety_state Util::safety_switch_state()
{
    return SAFETY_NONE;
}

//void Util::set_hw_rtc(uint64_t time_utc_usec)
//{
//    struct timeval tp;
//    tp.tv_sec = time_utc_usec/1000000 ;
//    tp.tv_usec = time_utc_usec%1000000 ;
//    settimeofday(&tp);
//}
//
//uint64_t Util::get_hw_rtc() const
//{
//    struct timeval ts;
//    gettimeofday(&ts, nullptr);
//    return ((uint64_t)(ts.tv_sec) * 1000) + (ts.tv_usec / 1000);
//}

void Util::set_system_clock(uint64_t time_utc_usec)
{
	struct timeval tp;
	tp.tv_sec = time_utc_usec / 1000000;
	tp.tv_usec = time_utc_usec % 1000000;
	settimeofday(&tp);
}

bool Util::get_system_id(char buf[]) { return false; }

uint32_t Util::available_memory()
{
    // As djgpp cross compiler didn't release mallinfo() structure in stdlib.h,
    // we assumed that memory is always big enough for malloc.
    return 40960;
}

Util::perf_counter_t Util::perf_alloc(perf_counter_type t, const char *name)
{
    if (t != Util::PC_COUNT && t != Util::PC_ELAPSED) {
        /*
         * Other perf counters not implemented for now since they are not
         * used anywhere.
         */
        return (Util::perf_counter_t)(uintptr_t) -1;
    }
	
    Util::perf_counter_t pc = (Util::perf_counter_t) _perf_counters.size();
    _perf_counters.emplace_back(t, name);
	
    return pc;
}

void Util::perf_begin(perf_counter_t h)
{
	if (!_init_perf) {
		return;
	}

    uintptr_t idx = (uintptr_t)h;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters.at(idx);
    if (perf.type != Util::PC_ELAPSED) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_ELAPSED type.\n",
                            perf.name);
        return;
    }

    if (perf.start != 0) {
        hal.console->printf("perf_begin() called twice on perf_counter_t(%s)\n",
                            perf.name);
        return;
    }

    _update_count++;

    perf.start = now_nsec();
}

void Util::perf_end(perf_counter_t h)
{
	if (!_init_perf) {
		return;
	}
    uintptr_t idx = (uintptr_t)h;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters.at(idx);
    if (perf.type != Util::PC_ELAPSED) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_ELAPSED type.\n",
                            perf.name);
        return;
    }

    if (perf.start == 0) {
        hal.console->printf("perf_end() called before begin() on perf_counter_t(%s)\n",
                            perf.name);
        return;
    }

    _update_count++;

    const uint64_t elapsed = now_nsec() - perf.start;
    perf.count++;
    perf.total += elapsed;

    if (perf.min > elapsed) {
        perf.min = elapsed;
    }

    if (perf.max < elapsed) {
        perf.max = elapsed;
    }

    /*
     * Maintain avg and variance of interval in nanoseconds
     * Knuth/Welford recursive avg and variance of update intervals (via Wikipedia)
     * Same implementation of PX4.
     */

    const double delta_intvl = elapsed - perf.avg;
    perf.avg += (delta_intvl / perf.count);
    perf.m2 += (delta_intvl * (elapsed - perf.avg));
    perf.start = 0;
}

void Util::perf_count(perf_counter_t h)
{
	if (!_init_perf) {
		return;
	}
    uintptr_t idx = (uintptr_t)h;

    if (idx >= _perf_counters.size()) {
        return;
    }

    Perf_Counter &perf = _perf_counters.at(idx);
    if (perf.type != Util::PC_COUNT) {
        hal.console->printf("perf_begin() called on perf_counter_t(%s) that"
                            " is not of PC_COUNT type.\n",
                            perf.name);
        return;
    }

    _update_count++;
    perf.count++;

}

void Util::_debug_counters()
{
	
    uint64_t now = AP_HAL::millis64();

    if (now - _last_debug_msec < 5000) {
        return;
    }
	io_DisableINT();
    unsigned int uc = _update_count;
    auto v = _perf_counters;
	io_RestoreINT();
    if (uc != _update_count) {
		hal.console->printf("WARNING!! potentially wrong counters!!!");
    }

    for (auto &c : v) {
        if (!c.count) {
			continue;
            /*hal.console->printf(
                    "%-30s\t" "(no events)\n", c.name);*/
        } else if (c.type == Util::PC_ELAPSED) {
            hal.console->printf(
                    "%-30s\t"
                    "count: %" PRIu64 "\t"
                    "min: %" PRIu64 "\t"
                    "max: %" PRIu64 "\t"
                    "avg: %.4f\t"
                    "stddev: %.4f\n",
                    c.name, c.count, c.min, c.max, c.avg, sqrt(c.m2));
        } else {
            hal.console->printf(
                    "%-30s\t" "count: %" PRIu64 "\n",
                    c.name, c.count);
        }
    }

    _last_debug_msec = now;
	
	reset_counter();
}

void Util::reset_counter()
{
	io_DisableINT();
	for (auto& c : _perf_counters) {
		if (!c.count) {
			continue;
		}
		else if (c.type == Util::PC_ELAPSED) {
			c.count = 0;
			c.min = ULONG_MAX;
			c.max = 0;
			c.avg = 0;
			c.m2 = 0;
			c.total = 0;
		}
		else {
			c.count = 0;
		}
	}
	io_RestoreINT();
}

AP_HAL::Semaphore *Util::new_semaphore() { return new Semaphore; }



void Util::init_wdt(uint32_t microseconds, bool type)
{
	if (timerWDTInit == true) return;

	// wdt_init
	unsigned char val;
	val = io_inpb(0xa8);
	io_outpb(0xa8, val & (~0x40));

	if ((io_inpb(0xad) & 0x80) == 0)
		rebootByWDT = false;
	else
		rebootByWDT = true;

	// restore current WDT1 register
	WDT_t.ctrl_reg = io_inpb(0xA8);
	WDT_t.sigsel_reg = io_inpb(0xA9);
	WDT_t.count0_reg = io_inpb(0xAA);
	WDT_t.count1_reg = io_inpb(0xAB);
	WDT_t.count2_reg = io_inpb(0xAC);
	WDT_t.stat_reg = io_inpb(0xAD);
	WDT_t.reload_reg = io_inpb(0xAE);

	io_outpb(0xad, 0x80); // clear timeout event bit

	// set time period
	wdt_settimer(microseconds);

	// set reset signal mode
	if (type == true)
	{
		// not use here
		io_outpb(0xa9, 0x0D << 4); // system reset
		wdt_mode = SYSTEM_RESET;
	}
	else
	{
		io_outpb(0xa9, 0x05 << 4); // use IRQ7
		wdt_mode = INTERRUPT;

		// set IRQ
		if (timerWDTIntEnable == false) {
			if (irq_Setting(WDTIRQ, IRQ_LEVEL_TRIGGER | IRQ_USE_FPU) == false)
			{
				printf("WDT IRQ Setting fail\n"); return;
			}
			if (irq_InstallISR(WDTIRQ, timerwdt_isr_handler, this) == false)
			{
				printf("irq_install fail\n"); return;
			}
			timerWDTIntEnable = true;
		}
	}

	_wdt_enable();

	timerWDTInit = true;
}

void Util::wdt_settimer(uint32_t microseconds)
{
	unsigned long ival;
	double dval;
	dval = ((double)microseconds) / 30.5;
	ival = (unsigned long)dval;

	if (ival > 0x00ffffffL) ival = 0x00ffffffL;

	io_outpb(0xac, (ival >> 16) & 0x00ff);
	io_outpb(0xab, (ival >> 8) & 0x00ff);
	io_outpb(0xaa, ival & 0x00ff);
}

void Util::_wdt_enable()
{
	unsigned char val;
	if (timerWDTEnable == true) return;
	val = io_inpb(0xa8);
	io_outpb(0xa8, val | 0x40);  // reset bit 6 to disable WDT1
	timerWDTEnable = true;
}

void Util::_wdt_disable()
{
	unsigned char val;
	if (timerWDTEnable == false) return;
	val = io_inpb(0xa8);
	io_outpb(0xa8, val & (~0x40));  // reset bit 6 to disable WDT1
	timerWDTEnable = false;
}

void Util::reset_wdt()
{
	if (timerWDTEnable == false) return;
	io_outpb(0xae, 0xff);  // WDT1 reload register
}

bool Util::isResetByWDT()
{
	return ((io_inpb(0xad) & 0x80) == 0) ? (false) : (true);
}

void Util::stop_wdt()
{
	if (timerWDTInit == false) return;
	detachInterrupt_wdt();
	_wdt_disable();

	io_outpb(0xA8, WDT_t.ctrl_reg);
	io_outpb(0xA9, WDT_t.sigsel_reg);
	io_outpb(0xAA, WDT_t.count0_reg);
	io_outpb(0xAB, WDT_t.count1_reg);
	io_outpb(0xAC, WDT_t.count2_reg);
	io_outpb(0xAD, WDT_t.stat_reg);
	io_outpb(0xAE, WDT_t.reload_reg);

	timerWDTInit = false;
}

void Util::detachInterrupt_wdt()
{
	if (timerWDTInit == false || timerWDTIntEnable == false) return;

	irq_UninstallISR(WDTIRQ, this);
	timerWDTIntEnable = false;
	// No close WDT
}

// static member
int Util::timerwdt_isr_handler(int irq, void* data)
{
	Util* watchdog = (Util*)data;
	assert(watchdog);

	if ((io_inpb(0xad) & 0x80) == 0) return ISR_NONE;

	watchdog->_wdt_disable();
	watchdog->_run_wdt_procs();
	watchdog->_wdt_enable();

	return ISR_HANDLED;
}

void Util::register_wdt_process(AP_HAL::MemberProc proc)
{
	// IO processes not implemented yet.
	for (uint8_t i = 0; i < _num_wdt_procs; i++) {
		if (_wdt_proc[i] == proc) {
			return;
		}
	}

	if (_num_wdt_procs < X86DUINO_MAX_WDT_PROCS) {
		/* this write to _wdt_proc can be outside the critical section
		 * because that memory won't be used until _num_timer_procs is
		 * incremented. */
		_wdt_proc[_num_wdt_procs] = proc;
		/* _num_wdt_procs is used from interrupt, and multiple bytes long. */
		io_DisableINT();
		_num_wdt_procs++;
		io_RestoreINT();
		//hal.console->printf("register wdt process: %u\n", _num_wdt_procs);
	}
	else {
		hal.console->printf("Out of wdt processes\n");
	}
}

void Util::_run_wdt_procs()
{
	if (_in_wdt_proc) {
		return;
	}
	_in_wdt_proc = true;


	for (uint8_t i = 0; i < _num_wdt_procs; i++) {
		if (_wdt_proc[i]) {
			_wdt_proc[i]();
		}
	}

	_in_wdt_proc = false;
}

}

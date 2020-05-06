
#include <DataFlash/DataFlash.h>

#include "Scheduler.h"
#include "AnalogIn.h"
#include "io.h"
#include "mcm.h"
#include "irq.h"

#include <assert.h>
#include <stdarg.h>
#include <time.h>
#include <fcntl.h>

#define MC_1k 3     // 1k hz timerOne
#define MD_1k 2

#define USE_TIMER 1
#define USE_ACCUMULATE 0

#define RTCIRQ    (8)
// 20000 microseconds, notes that RTC only would actually set an approximate period rather than the specified period.
#define RTC_TIMER_PROC_PERIOD 20000L 

extern const AP_HAL::HAL & hal;

using namespace x86Duino;

Scheduler::Scheduler() :
	_initialized(false),
	_failsafe(nullptr),
	_timer_proc({ nullptr }),
	_num_timer_procs(0),
	_in_timer_proc(false),
	_io_proc({ nullptr }),
	_num_io_procs(0),
	_in_io_proc(false),
	_timer_suspended(false),
	_timer_event_missed(false),
	_timer_1k_enable(false),
	timerRTCInit(false),
	timerRTCEnable(false),
	_perf_timers(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_timers")),
	_perf_io_timers(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_IO_timers")),
	_perf_storage_timer(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_storage_timers")),
	_perf_delay(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "APM_delay")),
	_perf_overrun_timers(hal.util->perf_alloc(AP_HAL::Util::PC_COUNT, "timer_overruns")),
	_perf_overrun_timerRTC(hal.util->perf_alloc(AP_HAL::Util::PC_COUNT, "timerRTC_overruns"))
{
	_min_delay_cb_ms = 65535;
	_delay_cb = nullptr;
	_spi_in_timer = USE_TIMER;
	_i2c_in_timer = USE_ACCUMULATE;
}

DMP_INLINE(unsigned char) inpb_cmos(unsigned char reg) {
	unsigned char tmp;
	io_DisableINT();
	io_outpb(0x70, 0x80 | reg); // disable NMI (by setting the 0x80 bit) and assign a RTC register address
	tmp = io_inpb(0x71);
	io_RestoreINT();
	return tmp;
}

DMP_INLINE(void) outpb_cmos(unsigned char reg, unsigned char data) {
	io_DisableINT();
	io_outpb(0x70, 0x80 | reg); // disable NMI (by setting the 0x80 bit) and assign a RTC register address
	io_outpb(0x71, data);
	io_RestoreINT();
}

static int mcint_offset[3] = { 0, 8, 16 };
int timer_1k_count = 0;

// timerOne 1khz ISR'
static char* isrname_one = (char*)"timerOne_1k";
static int timer1k_isr_handler(int irq, void* data)
{
	if ((mc_inp(MC_1k, 0x04) & (PULSE_END_INT << mcint_offset[MD_1k])) == 0) return ISR_NONE;
	mc_outp(MC_1k, 0x04, (PULSE_END_INT << mcint_offset[MD_1k]));   // clear flag
	timer_1k_count++;

	((Scheduler*)hal.scheduler)->_run_timer_procs(true);

	return ISR_HANDLED;
}

void Scheduler::init()
{
	// setup file name case
	setenv("FNCASE", "y", 1);
	// set default file open mode
	_fmode = O_BINARY;
	// setup Time Zone
	setenv("TZ", "GMT+8", 1);   // set TZ system variable (set to GMT+0)
	tzset();    // setup time zone

	// init timer One
	init_timerOne();

	//init timer RTC
	init_timerRTC();
}

void Scheduler::init_timerOne() 
{
	// initialize()
	mcpwm_Disable(MC_1k, MD_1k);

	// disable_MCINT
	mc_outp(MC_1k, 0x00, mc_inp(MC_1k, 0x00) & ~(0xffL << mcint_offset[MD_1k]));  // disable mc interrupt
	mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) | (1L << MC_1k));
	// clear_INTSTATUS
	mc_outp(MC_1k, 0x04, 0xffL << mcint_offset[MD_1k]); //for EX

	if (!irq_InstallISR(GetMCIRQ(), timer1k_isr_handler, isrname_one))
		printf("timer 1k hz IRQ_install fail\n");

	// enable_MCINT(MC_1k, MD_1k, PULSE_END_INT);
	mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << MC_1k));
	mc_outp(MC_1k, 0x00, (mc_inp(MC_1k, 0x00) & ~(0xffL << mcint_offset[MD_1k])) | (PULSE_END_INT << mcint_offset[MD_1k]));

	mcpwm_SetWidth(MC_1k, MD_1k, 1000 * SYSCLK, 0L);    // 1k hz timer loop
	mcpwm_Enable(MC_1k, MD_1k);
	_timer_1k_enable = true;
}

// timerRTC is used to run IO process
void Scheduler::init_timerRTC()
{
	unsigned char tmp;

	// initialize()
	timerRTCInit = true;
	setRTCPeriod(RTC_TIMER_PROC_PERIOD);

	// attachInterrupt()
	if (timerRTCEnable == false)
	{
		if (irq_Setting(RTCIRQ, IRQ_EDGE_TRIGGER | IRQ_USE_FPU ) == false)
		{
			::printf("MCM IRQ Setting fail\n"); return;
		}
		if (irq_InstallISR(RTCIRQ, timerrtc_isr_handler, this) == false)
		{
			::printf("irq_install fail\n"); return;
		}
	}

	tmp = inpb_cmos(0x0B);
	outpb_cmos(0x0B, tmp | 0x40);
	timerRTCEnable = true;
}

int Scheduler::timerrtc_isr_handler(int irq, void* data) {
	Scheduler* sch = (Scheduler*)data;
	assert(sch);
	unsigned char tmp;
	io_outpb(0x70, 0x0C); // enable NMI and read RTC register C
	tmp = io_inpb(0x71); // clear RTC interrupt state

	if ((tmp & 0x40) > 0) sch->_run_io();
	if ((tmp & 0x70) == 0) return ISR_NONE;

	return ISR_HANDLED;
}

void Scheduler::setRTCPeriod(long microseconds) {
	unsigned char tmp;
	if (timerRTCInit == false || microseconds <= 0) return;

	if (microseconds < 183L)    _freq = 3;
	else if (microseconds < 366L)    _freq = 4;
	else if (microseconds < 732L)    _freq = 5;
	else if (microseconds < 1464L)   _freq = 6;
	else if (microseconds < 2929L)   _freq = 7;
	else if (microseconds < 5859L)   _freq = 8;
	else if (microseconds < 11718L)  _freq = 9;
	else if (microseconds < 23437L)  _freq = 10;
	else if (microseconds < 46875L)  _freq = 11;
	else if (microseconds < 93750L)  _freq = 12;
	else if (microseconds < 187500L) _freq = 13;
	else if (microseconds < 375000L) _freq = 14;
	else if (microseconds < 500000L) _freq = 15;
	else                            _freq = 15;

	tmp = inpb_cmos(0x0A);
	outpb_cmos(0x0A, (tmp & 0xF0) | _freq);
}

void Scheduler::delay(uint16_t ms)
{
	if (in_timerprocess()) {
		printf("ERROR: delay() from timer process\n");
		return;
	}
	uint64_t start = AP_HAL::micros64();

	while ((AP_HAL::micros64() - start) / 1000 < ms) {
		delay_microseconds(1000);
		if (_min_delay_cb_ms <= ms) {
			call_delay_cb();
		}
	}
}

void Scheduler::delay_microseconds(uint16_t us)
{
	timer_DelayMicroseconds(us);
}

void Scheduler::register_delay_callback(AP_HAL::Proc proc,
	uint16_t min_time_ms)
{
	_delay_cb = proc;
	_min_delay_cb_ms = min_time_ms;
}

void Scheduler::call_delay_cb()
{
	if (_delay_cb == nullptr) {
		return;
	}
	if (_in_delay_callback) {
		// don't recurse!
		return;
	}
	_in_delay_callback = true;
	_delay_cb();
	_in_delay_callback = false;
}

void Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
	for (uint8_t i = 0; i < _num_timer_procs; i++) {
		if (_timer_proc[i] == proc) {
			return;
		}
	}

	if (_num_timer_procs < X86DUINO_SCHEDULER_MAX_TIMER_PROCS) {
		/* this write to _timer_proc can be outside the critical section
		 * because that memory won't be used until _num_timer_procs is
		 * incremented. */
		_timer_proc[_num_timer_procs] = proc;
		/* _num_timer_procs is used from interrupt, and multiple bytes long. */
		io_DisableINT();
		_num_timer_procs++;
		io_RestoreINT();
		//hal.console->printf("register timer process: %u\n", _num_timer_procs);
	}
	else {
		hal.console->printf("Out of timer processes\n");
	}
}
//unsigned char OLD8259M;
//unsigned char OLD8259S;
void Scheduler::suspend_timer_procs() {
	_timer_suspended = true;
	//OLD8259M = io_inpb(0x21);
	//OLD8259S = io_inpb(0xA1);

	//io_DisableINT();
	//io_outpb(0xA1, 0xff);
	//io_outpb(0x21, 0xff);
	//io_RestoreINT();

}

void Scheduler::resume_timer_procs() {
	_timer_suspended = false;

	//io_DisableINT();
	//io_outpb(0xA1, OLD8259S);
	//io_outpb(0x21, OLD8259M);
	//io_RestoreINT();

	if (_timer_event_missed == true) {
		//_run_timer_procs(false);
		_timer_event_missed = false;
	}
	//io_RestoreINT();
}

bool Scheduler::in_timerprocess() {
	return _in_timer_proc;
}

void Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
	// IO processes not implemented yet.
	for (uint8_t i = 0; i < _num_io_procs; i++) {
		if (_io_proc[i] == proc) {
			return;
		}
	}

	if (_num_io_procs < X86DUINO_SCHEDULER_MAX_IO_PROCS) {
		/* this write to _timer_proc can be outside the critical section
		 * because that memory won't be used until _num_timer_procs is
		 * incremented. */
		_io_proc[_num_io_procs] = proc;
		/* _num_timer_procs is used from interrupt, and multiple bytes long. */
		io_DisableINT();
		_num_io_procs++;
		io_RestoreINT();
		//hal.console->printf("register io process: %u\n", _num_io_procs);
	}
	else {
		hal.console->printf("Out of io processes\n");
	}
}

void Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
	_failsafe = failsafe;
}

void Scheduler::system_initialized()
{
	if (_initialized) {
		AP_HAL::panic("PANIC: scheduler::system_initialized called"
			"more than once");
	}
	_initialized = true;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
	// disarm motors to ensure they are off during a bootloader upload
	//hal.rcout->force_safety_on();
	//hal.rcout->force_safety_no_wait();

	//stop logging
	//DataFlash_Class::instance()->StopLogging();

	hal.console->printf("GOING DOWN FOR A REBOOT\r\n");
	delay(100);

	io_DisableINT();
	if (hold_in_bootloader) io_outpb(0xf21A, 0x5a); // write soft reset key
	io_outpb(0x64, 0xfe); // reboot
}

void Scheduler::_run_timer_procs(bool called_from_isr)
{
	if (_in_timer_proc) {
		//AP_HAL::panic("_in_timer_proc should not happened\n");
		return;
	}
	
	_in_timer_proc = true;
	hal.util->perf_begin(_perf_timers);
	uint32_t t_over = AP_HAL::micros();

	if (!_timer_suspended) {
		// now call the timer based drivers
		for (int i = 0; i < _num_timer_procs; i++) {
			if (_timer_proc[i]) {
				_timer_proc[i]();
			}
		}

		uint32_t t;
		// and the failsafe, if one is setup
		if (_failsafe != nullptr) {
			static uint32_t last_failsafe = 0;
			t = AP_HAL::millis();
			if (t - last_failsafe > 10) {
				last_failsafe = t + 50; // 50ms = 20Hz
				_failsafe();
			}
		}

	}
	else if (called_from_isr) {
		_timer_event_missed = true;
	}


	// process analog input
	//((AnalogIn*)hal.analogin)->_timer_tick();

	if (AP_HAL::micros() - t_over > 999UL) {
		hal.util->perf_count(_perf_overrun_timers);
	}
	hal.util->perf_end(_perf_timers);
	_in_timer_proc = false;
}

void Scheduler::_run_io(void)
{
	if (_in_io_proc) {
		return;
	}
	_in_io_proc = true;
	hal.util->perf_begin(_perf_io_timers);
	uint32_t t_over = AP_HAL::micros();

	static uint32_t last_dataflash = 0;
	uint32_t t = AP_HAL::millis();
	if (t - last_dataflash > 10) {
		last_dataflash = t + 200; // 200ms ~= 5Hz
		for (int i = 0; i < _num_io_procs; i++) {
			if (_io_proc[i]) {
				_io_proc[i]();
			}
		}
	}
	
	hal.storage->_timer_tick();

	if (AP_HAL::micros() - t_over > 999UL) {
		hal.util->perf_count(_perf_overrun_timerRTC);
	}
	hal.util->perf_end(_perf_io_timers);
	
	_in_io_proc = false;
}
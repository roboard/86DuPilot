#pragma once
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_86DUINO
#include "AP_HAL_86Duino.h"

#define X86DUINO_SCHEDULER_MAX_TIMER_PROCS 8
#define X86DUINO_SCHEDULER_MAX_IO_PROCS 8


class x86Duino::Scheduler : public AP_HAL::Scheduler {
public:
	Scheduler();
	void     init() override;
	void     delay(uint16_t ms) override;
	void     delay_microseconds(uint16_t us) override;
	void	 register_delay_callback(AP_HAL::Proc proc, uint16_t min_time_ms) override;
	void     register_timer_process(AP_HAL::MemberProc) override;

	void     suspend_timer_procs() override;
	void     resume_timer_procs() override;
	bool     in_timerprocess() override;

	void     register_io_process(AP_HAL::MemberProc) override;

	void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;

	void     system_initialized() override;

	void     reboot(bool hold_in_bootloader) override;

	bool     in_main_thread() const override { return true; };

	void     _run_timer_procs(bool called_from_isr);

	bool     spi_in_timer() { return _spi_in_timer; }
	bool	 i2c_in_timer() { return _i2c_in_timer; }
private:
	void init_timerOne();
	void init_timerRTC();
	bool _initialized;
	AP_HAL::Proc _failsafe;

	AP_HAL::MemberProc _timer_proc[X86DUINO_SCHEDULER_MAX_TIMER_PROCS];
	volatile uint8_t _num_timer_procs;
	volatile bool _in_timer_proc;
	bool _spi_in_timer;
	bool _i2c_in_timer;

	AP_HAL::MemberProc _io_proc[X86DUINO_SCHEDULER_MAX_TIMER_PROCS];
	volatile uint8_t _num_io_procs;
	volatile bool _in_io_proc;

	volatile bool _timer_suspended;
	volatile bool _timer_event_missed;

	void _run_io(void);

	bool _timer_1k_enable;

	unsigned char _freq;
	bool timerRTCInit;
	bool timerRTCEnable;
	static int timerrtc_isr_handler(int irq, void* data);
	void setRTCPeriod(long microseconds);

	AP_HAL::Util::perf_counter_t  _perf_timers;
	AP_HAL::Util::perf_counter_t  _perf_io_timers;
	AP_HAL::Util::perf_counter_t  _perf_storage_timer;
	AP_HAL::Util::perf_counter_t  _perf_delay;
	AP_HAL::Util::perf_counter_t  _perf_overrun_timers;
	AP_HAL::Util::perf_counter_t  _perf_overrun_timerRTC;

	void call_delay_cb();
	uint16_t _min_delay_cb_ms;
	AP_HAL::Proc _delay_cb;
	bool _in_delay_callback : 1;
};

#endif

/*  MAME's scheduler as the Seibu boards need it, for their CPUs' interleave to be MAME's: the devices run in order to a
    target of the base time plus the quantum or the first timer, a device's abort (a synchronize, a timer set first)
    moving the target back to its time; timers fire in order of time, then of setting.
    After MAME src/emu/schedule.cpp (device_scheduler::timeslice, execute_timers; emu_timer::adjust) and
    src/emu/diexec.cpp (the cycles from attoseconds), BSD-3-Clause, copyright-holders: Aaron Giles.
    This file is part of ReBOX. GPL-2.0-or-later.

    Times in attoseconds from the frame's start (int64); sched_rebase moves them all by a frame. One instance.
*/
#ifndef RBX_SEIBU_SCHED_H
#define RBX_SEIBU_SCHED_H

#include <stdint.h>

#define SCHED_AS_PER_SEC 1000000000000000000LL

typedef struct {
	int64_t local;                 // attoseconds
	int64_t period;                // attoseconds a cycle (HZ_TO_ATTOSECONDS)
	int shift;
	int64_t divisor;
	int (*run)(int cycles);        // runs about `cycles`; returns those ran (overshoot in, stopped early on an abort)
	int64_t (*done)(void);         // inside run: the cycles done so far (for sched_now)
	void (*abort)(void);           // inside run: stop after this instruction
	int suspended;                 // (MAME suspend with eatcycles: its slices pass without running)
} sched_exec;

typedef struct {
	int64_t expire;
	uint32_t seq;
	int16_t id;                    // the board's; < 0 = free
	int16_t on;
	int32_t param;
} sched_timer;

#define SCHED_EXECS 4
#define SCHED_TIMERS 24

typedef struct {
	sched_exec ex[SCHED_EXECS];
	int nex;
	sched_timer tm[SCHED_TIMERS];
	int64_t base, quantum;
	uint32_t seq;
	int executing;                 // the running device, -1 none
	int in_callback;
	int64_t cb_time;
	// optional (NULL: none): runs slices of the base loop itself (the devices' records replayed without their run
	// calls) while it can, the same as the loop would; returns the slices done, 0 when the loop must run the next one
	int (*replay)(void);
} sched_t;

extern sched_t g_sched;

void sched_init(int64_t quantum);
int sched_add_exec(uint32_t clock, int (*run)(int), int64_t (*done)(void), void (*abort)(void));   // index
void sched_set_clock(int exec, uint32_t clock);
// the context's time: the running device's local time, the firing timer's, else the base
int64_t sched_now(void);
// a board timer by id (one of each id: MAME emu_timer adjust), firing at `expire`; aborts the running device when first
void sched_timer_set(int id, int64_t expire, int32_t param);
void sched_timer_off(int id);
int sched_timer_on(int id, int64_t *expire);
// a one-shot at now (MAME synchronize): the id fires with param after the running device stops
void sched_sync(int id, int32_t param);
// runs until the timer `stop_id` has fired (the frame's vblank), timers to `fire`
void sched_run(int stop_id, void (*fire)(int id, int32_t param));
// all times minus `t`
void sched_rebase(int64_t t);
int sched_state(void *ctx, int load);
// MAME spin_until_time from the running device: stopped now, its cycles eaten until the timer `id` (the board's, firing
// sched_resume) at now + duration
void sched_spin(int exec, int id, int64_t duration);
void sched_resume(int exec);
// the first timer's time (INT64_MAX: none): how far a device may run ahead of the others (lazy synchronization)
int64_t sched_head_expire(void);
// floor(x / d) (x below 2^53): the double's quotient corrected (no 64-bit division: a library call on the Vita)
static inline int sched_div_cycles(uint64_t x, uint64_t d)
{
	if (x < 8 * d) {   // (a few cycles: the quantum of the boards interleaved finely, without the division)
		int q = 0;
		for (uint64_t m = d; m <= x; m += d)
			q++;
		return q;
	}
	uint64_t q = (uint64_t)((double)x / (double)d);
	while (q && q * d > x)
		q--;
	while ((q + 1) * d <= x)
		q++;
	return (int)q;
}
// attoseconds in `ticks` of `clock` Hz (attotime::from_ticks, below a second)
static inline int64_t sched_ticks(int64_t ticks, uint32_t clock) { return ticks * (SCHED_AS_PER_SEC / clock); }

#endif

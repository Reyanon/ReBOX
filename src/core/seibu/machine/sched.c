/*  MAME's scheduler for the Seibu boards: see sched.h.
    After MAME src/emu/schedule.cpp and src/emu/diexec.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "sched.h"
#include "../../rbx.h"

#include <stdint.h>
#include <string.h>

sched_t g_sched;

// the first timer's slot, cached (-2: to find again). Every change of a timer here drops it; the boards change
// g_sched.tm directly only outside sched_run (their reset), and sched_run finds it again on entry
static int s_head = -2;

void sched_init(int64_t quantum)
{
	s_head = -2;
	memset(&g_sched, 0, sizeof(g_sched));
	g_sched.quantum = quantum;
	g_sched.executing = -1;
	for (int i = 0; i < SCHED_TIMERS; i++)
		g_sched.tm[i].id = -1;
}

void sched_set_clock(int exec, uint32_t clock)
{
	sched_exec *e = &g_sched.ex[exec];
	e->period = SCHED_AS_PER_SEC / clock;   // HZ_TO_ATTOSECONDS
	int64_t a = e->period;
	e->shift = 0;
	while (a >= (1LL << 31)) {
		e->shift++;
		a >>= 1;
	}
	e->divisor = a;
}

int sched_add_exec(uint32_t clock, int (*run)(int), int64_t (*done)(void), void (*abort)(void))
{
	const int i = g_sched.nex++;
	sched_exec *e = &g_sched.ex[i];
	e->local = 0;
	e->run = run;
	e->done = done;
	e->abort = abort;
	sched_set_clock(i, clock);
	return i;
}

int64_t sched_now(void)
{
	if (g_sched.in_callback)
		return g_sched.cb_time;
	if (g_sched.executing >= 0) {
		const sched_exec *e = &g_sched.ex[g_sched.executing];
		return e->local + e->done() * e->period;
	}
	return g_sched.base;
}

// the first timer (expire, then seq), -1 none
static int find_first_timer(void)
{
	int best = -1;
	for (int i = 0; i < SCHED_TIMERS; i++) {
		const sched_timer *t = &g_sched.tm[i];
		if (t->id < 0 || !t->on)
			continue;
		if (best < 0 || t->expire < g_sched.tm[best].expire ||
		    (t->expire == g_sched.tm[best].expire && (int32_t)(t->seq - g_sched.tm[best].seq) < 0))
			best = i;
	}
	return best;
}
static inline int first_timer(void)
{
	if (s_head == -2)
		s_head = find_first_timer();
	return s_head;
}

int64_t sched_head_expire(void)
{
	const int h = first_timer();
	return h >= 0 ? g_sched.tm[h].expire : INT64_MAX;
}

static void insert(int slot, int64_t expire, int32_t param)
{
	sched_timer *t = &g_sched.tm[slot];
	t->expire = expire;
	t->param = param;
	t->on = 1;
	t->seq = g_sched.seq++;
	s_head = -2;
	// (MAME: inserted as the head, the running device's timeslice aborted)
	if (first_timer() == slot && g_sched.executing >= 0)
		g_sched.ex[g_sched.executing].abort();
}

static int find(int id)
{
	for (int i = 0; i < SCHED_TIMERS; i++)
		if (g_sched.tm[i].id == id && g_sched.tm[i].on != 3)
			return i;
	return -1;
}

static int alloc_slot(int id)
{
	for (int i = 0; i < SCHED_TIMERS; i++)
		if (g_sched.tm[i].id < 0) {
			g_sched.tm[i].id = (int16_t)id;
			g_sched.tm[i].on = 0;
			s_head = -2;
			return i;
		}
	rbx_log("sched: out of timers");
	return -1;
}

void sched_timer_set(int id, int64_t expire, int32_t param)
{
	int s = find(id);
	if (s < 0)
		s = alloc_slot(id);
	if (s >= 0)
		insert(s, expire, param);
}

void sched_timer_off(int id)
{
	const int s = find(id);
	if (s >= 0)
		g_sched.tm[s].on = 0;
	s_head = -2;
}

int sched_timer_on(int id, int64_t *expire)
{
	const int s = find(id);
	if (s < 0 || !g_sched.tm[s].on)
		return 0;
	if (expire)
		*expire = g_sched.tm[s].expire;
	return 1;
}

void sched_sync(int id, int32_t param)
{
	// (a temporary: freed when fired; marked apart from the board's timers of the same id)
	for (int i = 0; i < SCHED_TIMERS; i++)
		if (g_sched.tm[i].id < 0) {
			g_sched.tm[i].id = (int16_t)id;
			g_sched.tm[i].on = 0;
			insert(i, sched_now(), param);
			g_sched.tm[i].on = 3;   // (on, temporary)
			s_head = -2;
			return;
		}
	rbx_log("sched: out of timers");
}

void sched_run(int stop_id, void (*fire)(int id, int32_t param))
{
	sched_t *s = &g_sched;
	s_head = -2;   // (the board's reset may have changed the timers)
	for (;;) {
		int head = first_timer();
		while (head >= 0 && s->base < s->tm[head].expire) {
			if (s->replay && s->replay()) {
				head = first_timer();
				continue;
			}
			int64_t target = s->base + s->quantum;
			if (s->tm[head].expire < target)
				target = s->tm[head].expire;
			for (int i = 0; i < s->nex; i++) {
				sched_exec *e = &s->ex[i];
				const int64_t delta = target - e->local;
				if (delta < e->period)
					continue;
				const int n = sched_div_cycles((uint64_t)delta >> e->shift, (uint64_t)e->divisor);
				int ran = n;   // (suspended: eaten)
				if (!e->suspended) {
					s->executing = i;
					ran = e->run(n);
					s->executing = -1;
				}
				e->local += (int64_t)ran * e->period;
				if (e->local < target)
					target = e->local > s->base ? e->local : s->base;
			}
			s->base = target;
			head = first_timer();
		}
		// the timers due
		int stopped = 0;
		while ((head = first_timer()) >= 0 && s->tm[head].expire <= s->base) {
			sched_timer *t = &s->tm[head];
			const int id = t->id;
			const int32_t param = t->param;
			s->in_callback = 1;
			s->cb_time = t->expire;
			if (t->on == 3)
				t->id = -1;   // (temporary: gone)
			t->on = 0;
			s_head = -2;
			fire(id, param);
			s->in_callback = 0;
			if (id == stop_id)
				stopped = 1;
		}
		if (stopped)
			return;
	}
}

void sched_rebase(int64_t t)
{
	s_head = -2;
	g_sched.base -= t;
	for (int i = 0; i < g_sched.nex; i++)
		g_sched.ex[i].local -= t;
	for (int i = 0; i < SCHED_TIMERS; i++)
		if (g_sched.tm[i].id >= 0)
			g_sched.tm[i].expire -= t;
}

int sched_state(void *ctx, int load)
{
	struct {
		int64_t base;
		uint32_t seq, pad;
		int64_t local[SCHED_EXECS];
		int32_t suspended[SCHED_EXECS];
		sched_timer tm[SCHED_TIMERS];
	} st;
	memset(&st, 0, sizeof(st));
	st.base = g_sched.base;
	st.seq = g_sched.seq;
	for (int i = 0; i < SCHED_EXECS; i++)
		st.local[i] = g_sched.ex[i].local;
	for (int i = 0; i < SCHED_EXECS; i++)
		st.suspended[i] = g_sched.ex[i].suspended;
	memcpy(st.tm, g_sched.tm, sizeof(st.tm));
	const int rc = rbx_state_io(ctx, "sched", &st, sizeof(st));
	if (load && !rc) {
		g_sched.base = st.base;
		g_sched.seq = st.seq;
		for (int i = 0; i < SCHED_EXECS; i++)
			g_sched.ex[i].local = st.local[i];
		for (int i = 0; i < SCHED_EXECS; i++)
			g_sched.ex[i].suspended = st.suspended[i];
		memcpy(g_sched.tm, st.tm, sizeof(st.tm));
		s_head = -2;
	}
	return rc;
}

void sched_spin(int exec, int id, int64_t duration)
{
	const int64_t now = sched_now();
	g_sched.ex[exec].suspended = 1;
	if (g_sched.executing == exec)
		g_sched.ex[exec].abort();
	sched_timer_set(id, now + duration, 0);
}

void sched_resume(int exec) { g_sched.ex[exec].suspended = 0; }

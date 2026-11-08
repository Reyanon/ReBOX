/*  Several 68000s on the one Musashi core (m68k_multi.h). Each CPU owns an m68k_bus_t (page tables, handlers,
    interrupt lines); switching selects that bus (m68k_bus_select, a pointer) and swaps the Musashi context.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "m68k_multi.h"
#include "m68k_bus.h"
#include "m68k/m68k.h"
#include "../rbx.h"

#include <stdlib.h>
#include <string.h>

#define CTX_MAX 1024

typedef struct {
	m68k_bus_t bus;
	uint8_t ctx[CTX_MAX];
} cpu_slot;

static cpu_slot *s_cpu[M68K_MULTI_MAX];
static int s_n, s_cur;
static uint32_t s_ctx_size;

int m68k_multi_init(int ncpu)
{
	m68k_multi_exit();
	s_ctx_size = m68k_context_size();
	if (ncpu < 1 || ncpu > M68K_MULTI_MAX || s_ctx_size > CTX_MAX)
		return -1;
	for (int i = 0; i < ncpu; i++) {
		s_cpu[i] = (cpu_slot *)calloc(1, sizeof(cpu_slot));
		if (!s_cpu[i]) {
			m68k_multi_exit();
			return -1;
		}
	}
	s_n = ncpu;
	s_cur = 0;
	return 0;
}

void m68k_multi_exit(void)
{
	m68k_bus_select(NULL);
	for (int i = 0; i < M68K_MULTI_MAX; i++) {
		free(s_cpu[i]);
		s_cpu[i] = NULL;
	}
	s_n = 0;
}

void m68k_multi_capture(int cpu)
{
	s_cpu[cpu]->bus = *m68k_bus_cur;
	s_cpu[cpu]->bus.irq_pending = s_cpu[cpu]->bus.irq_asserted = 0;
}

void m68k_multi_finish(void)
{
	s_cur = 0;
	m68k_bus_select(&s_cpu[0]->bus);
}

void m68k_multi_switch(int cpu)
{
	if (cpu == s_cur)
		return;
	m68k_get_context(s_cpu[s_cur]->ctx);
	m68k_bus_select(&s_cpu[cpu]->bus);
	m68k_set_context(s_cpu[cpu]->ctx);
	m68k_bus_update_irq();
	s_cur = cpu;
}

int m68k_multi_current(void) { return s_cur; }

void m68k_multi_reset_cpu(int cpu)
{
	int prev = s_cur;
	m68k_multi_switch(cpu);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	m68k_multi_switch(prev);
}

void m68k_multi_reset(void)
{
	// fresh contexts: CPU 0 is current, the others get a copy of a reset core (then their own reset)
	s_cur = 0;
	m68k_bus_select(&s_cpu[0]->bus);
	m68k_bus_irq_reset();
	for (int i = 1; i < s_n; i++) {
		m68k_get_context(s_cpu[i]->ctx);
		s_cpu[i]->bus.irq_pending = s_cpu[i]->bus.irq_asserted = 0;
	}
	m68k_pulse_reset();
	for (int i = 1; i < s_n; i++)
		m68k_multi_reset_cpu(i);
}

void m68k_multi_hold_irq(int cpu, int level)
{
	if (cpu == s_cur)
		m68k_bus_hold_irq(level);
	else
		s_cpu[cpu]->bus.irq_pending |= 1u << level;
}

void m68k_multi_set_line(int cpu, int level, int state)
{
	if (cpu == s_cur)
		m68k_bus_set_line(level, state);
	else if (state)
		s_cpu[cpu]->bus.irq_asserted |= 1u << level;
	else
		s_cpu[cpu]->bus.irq_asserted &= ~(1u << level);
}

uint32_t m68k_multi_asserted(int cpu) { return s_cpu[cpu]->bus.irq_asserted; }

void m68k_multi_set_asserted(int cpu, uint32_t mask)
{
	s_cpu[cpu]->bus.irq_asserted = mask;
	if (cpu == s_cur)
		m68k_bus_update_irq();
}

int m68k_multi_state(void *ctx, int load)
{
	int prev = s_cur;
	m68k_multi_switch(0);
	if (!load)
		m68k_get_context(s_cpu[0]->ctx);
	int rc = 0;
	for (int i = 0; i < s_n; i++) {
		char tag[16] = "m68k_ctx0", tagi[16] = "m68k_irq0";
		tag[8] = tagi[8] = (char)('0' + i);
		rc |= rbx_state_io(ctx, tag, s_cpu[i]->ctx, s_ctx_size);
		rc |= rbx_state_io(ctx, tagi, &s_cpu[i]->bus.irq_pending, sizeof(uint32_t));
	}
	if (load && !rc) {
		m68k_set_context(s_cpu[0]->ctx);
		m68k_bus_update_irq();
	}
	if (!load)
		m68k_multi_switch(prev);
	return rc ? -1 : 0;
}

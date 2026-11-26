/*  Hyperstone E1 CPU core, ported to C from MAME 0.289 src/devices/cpu/e132xs: e132xs.cpp, e132xsop.hxx, e1defs.h
    (BSD-3-Clause, copyright-holders: Pierpaolo Prazzoli). See e132xs.h for what is covered.
    The opcode handlers keep MAME's order of operations (immediates read before check_delay_pc, the register
    wrapping, the flag formulas and the cycle counts, quirks included). MAME's templates (register banks, immediate
    sizes, conditions) are inline functions called with constants from the dispatch switch.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "e132xs.h"

#include <stdlib.h>
#include <string.h>

#define FI static inline __attribute__((always_inline))
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

enum {
	PC_REGISTER = 0, SR_REGISTER = 1, FER_REGISTER = 2, SP_REGISTER = 18, UB_REGISTER = 19, BCR_REGISTER = 20,
	TPR_REGISTER = 21, TCR_REGISTER = 22, TR_REGISTER = 23, WCR_REGISTER = 24, ISR_REGISTER = 25, FCR_REGISTER = 26,
	MCR_REGISTER = 27
};

enum {
	TRAPNO_IO2 = 48, TRAPNO_IO1 = 49, TRAPNO_INT4 = 50, TRAPNO_INT3 = 51, TRAPNO_INT2 = 52, TRAPNO_INT1 = 53,
	TRAPNO_IO3 = 54, TRAPNO_TIMER = 55, TRAPNO_TRACE_EXCEPTION = 57, TRAPNO_RANGE_ERROR = 60,
	TRAPNO_POINTER_ERROR = 60, TRAPNO_PRIVILEGE_ERROR = 60, TRAPNO_FRAME_ERROR = 60, TRAPNO_RESET = 62
};

// extended DSP instructions
#define EHMAC    0x02a
#define EHMACD   0x02e
#define EHCMULD  0x046
#define EHCMACD  0x04e
#define EHCSUMD  0x086
#define EHCFFTD  0x096
#define EMUL_N   0x100
#define EMUL     0x102
#define EMULU    0x104
#define EMULS    0x106
#define EMAC     0x10a
#define EMACD    0x10e
#define EMSUB    0x11a
#define EMSUBD   0x11e
#define EHCFFTSD 0x296

#define C_MASK   0x00000001u
#define Z_MASK   0x00000002u
#define N_MASK   0x00000004u
#define V_MASK   0x00000008u
#define M_MASK   0x00000010u
#define H_MASK   0x00000020u
#define I_MASK   0x00000080u
#define L_MASK   0x00008000u
#define T_MASK   0x00010000u
#define P_MASK   0x00020000u
#define S_MASK   0x00040000u
#define ILC_MASK 0x00180000u
#define S_SHIFT  18
#define ILC_SHIFT 19

#define OP  (c->op)
#define PC  (c->g[0])
#define SR  (c->g[1])
#define SP  (c->g[18])
#define UB  (c->g[19])
#define TPR (c->g[21])
#define TCR (c->g[22])
#define ISR (c->g[25])
#define FCR (c->g[26])

#define GET_C   (SR & C_MASK)
#define GET_V   ((SR & V_MASK) >> 3)
#define GET_L   ((SR & L_MASK) >> 15)
#define GET_T   ((SR & T_MASK) >> 16)
#define GET_P   ((SR & P_MASK) >> 17)
#define GET_S   ((SR & S_MASK) >> 18)
#define GET_FL  (((((SR >> 21) & 0xf) - 1) & 0xf) + 1)   // (FL 0 means 16: MAME fl_lut)
#define GET_FP  (SR >> 25)
#define SET_M(v)   (SR = (SR & ~M_MASK) | ((uint32_t)(v) << 4))
#define SET_L(v)   (SR = (SR & ~L_MASK) | ((uint32_t)(v) << 15))
#define SET_T(v)   (SR = (SR & ~T_MASK) | ((uint32_t)(v) << 16))
#define SET_P(v)   (SR = (SR & ~P_MASK) | ((uint32_t)(v) << 17))
#define SET_S(v)   (SR = (SR & ~S_MASK) | ((uint32_t)(v) << 18))
#define SET_ILC(v) (SR = (SR & 0xffe7ffffu) | (uint32_t)(v))
#define SET_FL(v)  (SR = (SR & ~0x01e00000u) | ((uint32_t)(v) << 21))
#define SET_FP(v)  (SR = (SR & ~0xfe000000u) | ((uint32_t)(v) << 25))
#define SET_LOW_SR(v) (SR = (SR & 0xffff0000u) | ((v) & 0x0000ffffu))

#define DST_CODE   ((OP & 0xf0) >> 4)
#define SRC_CODE   (OP & 0x0f)
#define N_VALUE    (((OP & 0x100) >> 4) | (OP & 0x0f))
#define HI_N_VALUE (0x10 | (OP & 0x0f))
#define LO_N_VALUE (OP & 0x0f)
#define N_OP_MASK  (OP & 0x10f)
#define SIGN_TO_N(v)   (((uint32_t)(v) & 0x80000000u) >> 29)
#define SIGN64_TO_N(v) ((uint32_t)(((uint64_t)(v) & 0x8000000000000000ull) >> 61))

#define REGS(gl) ((gl) ? c->g : c->l)
#define CC1 (c->cc1)
#define CC2 (c->cc2)
#define ICOUNT (c->icount)

static const uint32_t s_trap_entries[8] = { 0x00000000, 0x40000000, 0x80000000, 0xc0000000, 0, 0, 0, 0xffffff00 };
static const int32_t s_immediate_values[16] = {
	16, 0, 0, 0, 32, 64, 128, (int32_t)0x80000000, -8, -7, -6, -5, -4, -3, -2, -1
};
#define WRITE_ONLY_REGMASK ((1u << BCR_REGISTER) | (1u << TPR_REGISTER) | (1u << FCR_REGISTER) | (1u << MCR_REGISTER))

static inline int32_t sext7(uint32_t v) { return (int32_t)(v << 25) >> 25; }

// ---------------------------------------------------------------- memory

static inline uint8_t rd8(e132xs_t *c, uint32_t a)
{
	const e132xs_page *p = &c->rd[a >> E132XS_PAGE_SHIFT];
	if (p->base)
		return p->base[(a & p->mask) ^ 3];
	return c->read8 ? c->read8(c->user, a) : 0;
}
static inline uint16_t rd16(e132xs_t *c, uint32_t a)
{
	a &= ~1u;
	const e132xs_page *p = &c->rd[a >> E132XS_PAGE_SHIFT];
	if (p->base)
		return *(const uint16_t *)(p->base + ((a & p->mask) ^ 2));
	return c->read16 ? c->read16(c->user, a) : 0;
}
static inline uint32_t rd32(e132xs_t *c, uint32_t a)
{
	a &= ~3u;
	const e132xs_page *p = &c->rd[a >> E132XS_PAGE_SHIFT];
	if (p->base)
		return *(const uint32_t *)(p->base + (a & p->mask));
	return c->read32 ? c->read32(c->user, a) : 0;
}
static inline void wr8(e132xs_t *c, uint32_t a, uint8_t v)
{
	const e132xs_page *p = &c->wr[a >> E132XS_PAGE_SHIFT];
	if (p->base)
		p->base[(a & p->mask) ^ 3] = v;
	else if (c->write8)
		c->write8(c->user, a, v);
}
// a page of e132xs_map_skip15 (wq): the 16-bit halves with bit 15 clear stored, the others dropped
static inline void wq16(const e132xs_page *q, uint32_t a, uint16_t v)
{
	if (!(v & 0x8000))
		*(uint16_t *)(q->base + ((a & q->mask) ^ 2)) = v;
}
static inline void wr16(e132xs_t *c, uint32_t a, uint16_t v)
{
	a &= ~1u;
	const e132xs_page *p = &c->wr[a >> E132XS_PAGE_SHIFT];
	if (p->base)
		*(uint16_t *)(p->base + ((a & p->mask) ^ 2)) = v;
	else if (c->wq && c->wq[a >> E132XS_PAGE_SHIFT].base)
		wq16(&c->wq[a >> E132XS_PAGE_SHIFT], a, v);
	else if (c->write16)
		c->write16(c->user, a, v);
}
static inline void wr32(e132xs_t *c, uint32_t a, uint32_t v)
{
	a &= ~3u;
	const e132xs_page *p = &c->wr[a >> E132XS_PAGE_SHIFT];
	if (p->base)
		*(uint32_t *)(p->base + (a & p->mask)) = v;
	else if (c->wq && c->wq[a >> E132XS_PAGE_SHIFT].base) {
		const e132xs_page *q = &c->wq[a >> E132XS_PAGE_SHIFT];
		wq16(q, a, (uint16_t)(v >> 16));
		wq16(q, a + 2, (uint16_t)v);
	} else if (c->write32)
		c->write32(c->user, a, v);
}

uint8_t e132xs_read8(e132xs_t *c, uint32_t a) { return rd8(c, a); }
uint16_t e132xs_read16(e132xs_t *c, uint32_t a) { return rd16(c, a); }
uint32_t e132xs_read32(e132xs_t *c, uint32_t a) { return rd32(c, a); }
void e132xs_write8(e132xs_t *c, uint32_t a, uint8_t v) { wr8(c, a, v); }
void e132xs_write16(e132xs_t *c, uint32_t a, uint16_t v) { wr16(c, a, v); }
void e132xs_write32(e132xs_t *c, uint32_t a, uint32_t v) { wr32(c, a, v); }

static uint16_t fetch_slow(e132xs_t *c, uint32_t a)
{
	const uint32_t page = a >> E132XS_PAGE_SHIFT;
	const e132xs_page *p = &c->rd[page];
	if (p->base) {
		c->fetch_page = page;
		c->fetch_base = p->base;
		c->fetch_mask = p->mask & ~1u;
		return *(const uint16_t *)(c->fetch_base + ((a & c->fetch_mask) ^ 2));
	}
	return c->read16 ? c->read16(c->user, a & ~1u) : 0;
}
// MAME m_pr16 (the opcode cache)
static inline uint16_t pr16(e132xs_t *c, uint32_t a)
{
	if ((a >> E132XS_PAGE_SHIFT) != c->fetch_page)
		return fetch_slow(c, a);
	return *(const uint16_t *)(c->fetch_base + ((a & c->fetch_mask) ^ 2));
}

#define READ_B(a)      rd8(c, (a))
#define READ_HW(a)     rd16(c, (a))
#define READ_W(a)      rd32(c, (a))
#define WRITE_B(a, v)  wr8(c, (a), (v))
#define WRITE_HW(a, v) wr16(c, (a), (v))
#define WRITE_W(a, v)  wr32(c, (a), (v))

static uint32_t io_read(e132xs_t *c, uint32_t addr)
{
	if (c->type != E132XS_E1 && ((addr >> 27) & 1))
		return 0;   // (the E1-X internal I/O space has no readable registers)
	return c->io_read ? c->io_read(c->user, (addr >> 13) & (c->bus16 ? 0x1ffu : 0x1fffu)) : 0;
}
static void io_write(e132xs_t *c, uint32_t addr, uint32_t data)
{
	if (c->type != E132XS_E1 && ((addr >> 27) & 1)) {
		const uint32_t off = (addr >> 13) & 0x1fff;
		if (off >= 0x1c00 && off <= 0x1dff)
			c->powerdown = 1;   // hyperstone_x_device::power_down_w (sleep_w is not emulated)
		return;
	}
	if (c->io_write)
		c->io_write(c->user, (addr >> 13) & (c->bus16 ? 0x1ffu : 0x1fffu), data);
}
#define IO_READ_W(a)     io_read(c, (a))
#define IO_WRITE_W(a, v) io_write(c, (a), (v))

void e132xs_map(e132xs_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable)
{
	const uint32_t psize = 1u << E132XS_PAGE_SHIFT;
	for (uint64_t a = start & ~(uint64_t)(psize - 1); a <= end; a += psize) {
		const uint32_t page = (uint32_t)(a >> E132XS_PAGE_SHIFT);
		e132xs_page e = { NULL, 0 };
		if (mem) {
			if (size >= psize) {
				const uint32_t off = (uint32_t)(a - start) & (size - 1);
				e.base = (uint8_t *)mem + off;
				e.mask = psize - 1;
			} else {
				e.base = (uint8_t *)mem;
				e.mask = size - 1;
			}
		}
		if (readable || !mem)
			c->rd[page] = e;
		if (writable || !mem)
			c->wr[page] = e;
		if (c->wq && (writable || !mem))
			c->wq[page].base = NULL;
	}
	c->fetch_page = ~0u;
}

int e132xs_map_skip15(e132xs_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size)
{
	if (!c->wq && !(c->wq = (e132xs_page *)calloc(E132XS_PAGES, sizeof(e132xs_page))))
		return -1;
	const uint32_t psize = 1u << E132XS_PAGE_SHIFT;
	for (uint64_t a = start & ~(uint64_t)(psize - 1); a <= end; a += psize) {
		const uint32_t page = (uint32_t)(a >> E132XS_PAGE_SHIFT);
		e132xs_page e = { (uint8_t *)mem, size - 1 };
		if (size >= psize) {
			e.base = (uint8_t *)mem + ((uint32_t)(a - start) & (size - 1));
			e.mask = psize - 1;
		}
		c->wr[page].base = NULL;
		c->wr[page].mask = 0;
		c->wq[page] = e;
	}
	return 0;
}

void e132xs_swap32(void *p, uint32_t size)
{
	uint8_t *b = (uint8_t *)p;
	for (uint32_t i = 0; i + 3 < size; i += 4) {
		const uint8_t t0 = b[i], t1 = b[i + 1];
		b[i] = b[i + 3];
		b[i + 1] = b[i + 2];
		b[i + 2] = t1;
		b[i + 3] = t0;
	}
}

// ---------------------------------------------------------------- time and the on-chip timer

static inline int64_t now(const e132xs_t *c) { return c->running ? c->seg_end - c->icount : c->clock; }
int64_t e132xs_total_cycles(const e132xs_t *c) { return now(c); }

// floor(x / tr_clocks_per_tick) (2..257) without a division when x fits 32 bits (the ARM has none)
static uint32_t tr_div(const e132xs_t *c, uint64_t x)
{
	const uint32_t d = c->tr_clocks_per_tick;
	if (x >> 32)
		return (uint32_t)(x / d);
	const uint32_t xx = (uint32_t)x;
	uint32_t q = (uint32_t)(((uint64_t)xx * (0xffffffffu / d)) >> 32);
	uint32_t r = xx - q * d;
	while (r >= d) {
		q++;
		r -= d;
	}
	return q;
}

// (the base moved forward by whole ticks when far behind: the results are the same, the division stays 32-bit)
static void tr_rebase(e132xs_t *c, int64_t t)
{
	const uint64_t clocks = (uint64_t)(t - c->tr_base_cycles) >> c->clck_scale;
	if (clocks >= 0x80000000u) {
		const uint64_t q = clocks / c->tr_clocks_per_tick;
		c->tr_base_value += (uint32_t)q;
		c->tr_base_cycles += (int64_t)((q * c->tr_clocks_per_tick) << c->clck_scale);
	}
}

static void compute_tr(e132xs_t *c)
{
	const int64_t t = now(c);
	tr_rebase(c, t);
	const uint64_t clocks_since_base = (uint64_t)(t - c->tr_base_cycles) >> c->clck_scale;
	c->tr_result = c->tr_base_value + tr_div(c, clocks_since_base);
}

static void update_timer_prescale(e132xs_t *c)
{
	TPR &= ~0x80000000u;
	c->clck_scale = (TPR >> 26) & c->clock_scale_mask;
	c->cc1 = 1u << c->clck_scale;
	c->cc2 = 2u << c->clck_scale;
	c->cc3 = 3u << c->clck_scale;
	c->cc4 = 4u << c->clck_scale;
	c->cc6 = 6u << c->clck_scale;
	c->cc36 = 36u << c->clck_scale;
	c->tr_clocks_per_tick = ((TPR >> 16) & 0xff) + 2;
	c->tr_base_value = c->tr_result;
	c->tr_base_cycles = now(c);
}

// MAME m_timer->adjust: a cycle stamp; a running segment ends at it
static void timer_set(e132xs_t *c, int64_t at, uint32_t param)
{
	c->timer_at = at;
	c->timer_param = param;
	if (c->running && at < c->seg_end) {
		const int64_t t = now(c);
		if (at < t)
			at = t;
		c->icount -= (int32_t)(c->seg_end - at);
		c->seg_end = at;
	}
}

static void adjust_timer_interrupt(e132xs_t *c)
{
	const int64_t t = now(c);
	tr_rebase(c, t);
	const uint64_t cycles_since_base = (uint64_t)(t - c->tr_base_cycles);
	const uint64_t clocks_since_base = cycles_since_base >> c->clck_scale;
	uint64_t cycles_until_next_clock = cycles_since_base - (clocks_since_base << c->clck_scale);
	if (cycles_until_next_clock == 0)
		cycles_until_next_clock = (uint64_t)1 << c->clck_scale;

	if (TPR & 0x80000000u) {
		// special case: if we have a change pending, set a timer to fire then
		const uint64_t q = tr_div(c, clocks_since_base);
		const uint64_t clocks_until_int = c->tr_clocks_per_tick - (clocks_since_base - q * c->tr_clocks_per_tick);
		const uint64_t cycles_until_int = (clocks_until_int << c->clck_scale) + cycles_until_next_clock;
		timer_set(c, t + (int64_t)cycles_until_int + 1, 1);
	} else if (!(FCR & 0x00800000u)) {
		// else if the timer interrupt is enabled, configure it to fire at the appropriate time
		const uint32_t curtr = c->tr_base_value + tr_div(c, clocks_since_base);
		const uint32_t delta = TCR - curtr;
		if (delta > 0x80000000u) {
			if (!c->timer_int_pending)
				timer_set(c, t, 0);
		} else {
			const uint64_t clocks_until_int = (uint64_t)delta * c->tr_clocks_per_tick;
			const uint64_t cycles_until_int = (clocks_until_int << c->clck_scale) + cycles_until_next_clock;
			timer_set(c, t + (int64_t)cycles_until_int, 0);
		}
	} else {
		// otherwise, disable the timer
		timer_set(c, E132XS_NEVER, 0);
	}
}

// MAME timer_callback
static void timer_fire(e132xs_t *c)
{
	const uint32_t update = c->timer_param;
	c->timer_at = E132XS_NEVER;
	c->timer_param = 0;
	if (update)
		update_timer_prescale(c);
	compute_tr(c);
	if (!((c->tr_result - TCR) & 0x80000000u)) {
		c->timer_int_pending = 1;
		if (!((FCR >> 23) & 1))
			c->powerdown = 0;
	} else {
		adjust_timer_interrupt(c);
	}
}

// ---------------------------------------------------------------- control registers

static void update_memory_control(e132xs_t *c)
{
	const uint32_t val = c->g[MCR_REGISTER];
	// bits 14..12 EntryTableMap
	c->trap_entry = s_trap_entries[(val & 0x7000) >> 12];
	if (c->type == E132XS_E1XS)
		return;   // (hyperstone_xs_device: no power down request bit; the SDRAM handlers are not emulated)
	const uint32_t power_down_req = (val >> 22) & 1;
	if (!power_down_req && c->power_down_req)
		c->powerdown = 1;
	c->power_down_req = power_down_req;
}

static uint32_t get_trap_addr(e132xs_t *c, uint32_t trapno)
{
	uint32_t addr;
	if (c->trap_entry == 0xffffff00)   // @ MEM3
		addr = trapno * 4;
	else
		addr = (63 - trapno) * 4;
	return addr | c->trap_entry;
}

static uint32_t get_emu_code_addr(e132xs_t *c, uint32_t num)
{
	if (c->trap_entry == 0xffffff00)
		return (c->trap_entry - 0x100) | ((num & 0xf) << 4);
	return c->trap_entry | (0x10c | ((0xcf - num) << 4));
}

static void execute_exception(e132xs_t *c, uint32_t trapno);

static uint32_t get_global_register(e132xs_t *c, uint32_t code)
{
	if (code == TR_REGISTER) {
		// it is common to poll this in a loop
		if ((uint32_t)c->icount > c->tr_clocks_per_tick / 2)   // (MAME's signed / unsigned compare kept)
			c->icount -= (int32_t)(c->tr_clocks_per_tick / 2);
		compute_tr(c);
		return c->tr_result;
	}
	return c->g[code & 0x1f];
}

static void set_global_register(e132xs_t *c, uint32_t code, uint32_t val)
{
	code &= 0x1f;
	switch (code) {
	case PC_REGISTER:
		PC = val & 0xfffffffeu;
		return;
	case SR_REGISTER: {
		const int privilege_error = !GET_S && !GET_L && (val & L_MASK);
		SET_LOW_SR(val);   // only a RET instruction can change the full content of SR
		SR &= ~0x40u;      // reserved bit 6 always zero
		if (privilege_error)
			execute_exception(c, TRAPNO_PRIVILEGE_ERROR);
		return;
	}
	case SP_REGISTER:
	case UB_REGISTER:
		c->g[code] = val & ~3u;
		return;
	case BCR_REGISTER:
		c->g[code] = val;   // (update_bus_control: timing only logged)
		return;
	case TPR_REGISTER:
		c->g[code] = val;
		if (!(val & 0x80000000u)) {   // change immediately
			compute_tr(c);
			update_timer_prescale(c);
		}
		adjust_timer_interrupt(c);
		return;
	case TCR_REGISTER:
		if (c->g[code] != val) {
			c->g[code] = val;
			adjust_timer_interrupt(c);
		}
		return;
	case TR_REGISTER:
		c->g[code] = val;
		c->tr_base_value = val;
		c->tr_base_cycles = now(c);
		adjust_timer_interrupt(c);
		return;
	case ISR_REGISTER:
		return;
	case FCR_REGISTER:
		if ((c->g[code] ^ val) & 0x00800000u)
			adjust_timer_interrupt(c);
		c->g[code] = val;
		return;
	case MCR_REGISTER:
		c->g[code] = val;
		update_memory_control(c);
		return;
	default:   // 2-17, WCR, 28-31
		c->g[code] = val;
		return;
	}
}

// ---------------------------------------------------------------- decoding helpers

FI void check_delay_pc(e132xs_t *c)
{
	// if PC is used in a delay instruction, the delayed PC should be used
	if (!c->delay_slot) {
		c->delay_slot_taken = 0;
	} else {
		const uint32_t t = PC;
		PC = c->delay_pc;
		c->delay_pc = t;
		c->delay_slot = 0;
		c->delay_slot_taken = 1;
	}
}

static void ignore_immediate_s(e132xs_t *c)
{
	static const uint32_t lengths[16] = {
		1 << ILC_SHIFT, 3 << ILC_SHIFT, 2 << ILC_SHIFT, 2 << ILC_SHIFT, 1 << ILC_SHIFT, 1 << ILC_SHIFT, 1 << ILC_SHIFT,
		1 << ILC_SHIFT, 1 << ILC_SHIFT, 1 << ILC_SHIFT, 1 << ILC_SHIFT, 1 << ILC_SHIFT, 1 << ILC_SHIFT, 1 << ILC_SHIFT,
		1 << ILC_SHIFT, 1 << ILC_SHIFT };
	static const uint32_t offsets[16] = { 0, 4, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	const uint32_t nybble = OP & 0x0f;
	c->instruction_length = (int32_t)lengths[nybble];
	PC += offsets[nybble];
}

FI uint32_t decode_immediate_s(e132xs_t *c)
{
	const uint32_t nybble = OP & 0x0f;
	switch (nybble) {
	case 0:
		return 16;
	case 1: {
		c->instruction_length = 3 << ILC_SHIFT;
		const uint32_t extra_u = ((uint32_t)pr16(c, PC) << 16) | pr16(c, PC + 2);
		PC += 4;
		return extra_u;
	}
	case 2: {
		c->instruction_length = 2 << ILC_SHIFT;
		const uint32_t extra_u = pr16(c, PC);
		PC += 2;
		return extra_u;
	}
	case 3: {
		c->instruction_length = 2 << ILC_SHIFT;
		const uint32_t extra_u = 0xffff0000u | pr16(c, PC);
		PC += 2;
		return extra_u;
	}
	default:
		return (uint32_t)s_immediate_values[nybble];
	}
}

FI uint32_t decode_const(e132xs_t *c)
{
	const uint16_t imm_1 = pr16(c, PC);
	PC += 2;
	if (imm_1 & 0x8000) {
		const uint16_t imm_2 = pr16(c, PC);
		PC += 2;
		c->instruction_length = 3 << ILC_SHIFT;
		uint32_t imm = imm_2;
		imm |= ((uint32_t)(imm_1 & 0x3fff) << 16);
		if (imm_1 & 0x4000)
			imm |= 0xc0000000u;
		return imm;
	} else {
		c->instruction_length = 2 << ILC_SHIFT;
		uint32_t imm = imm_1 & 0x3fff;
		if (imm_1 & 0x4000)
			imm |= 0xffffc000u;
		return imm;
	}
}

FI int32_t decode_pcrel(e132xs_t *c)
{
	if (OP & 0x80) {
		const uint16_t next = pr16(c, PC);
		PC += 2;
		c->instruction_length = 2 << ILC_SHIFT;
		uint32_t offset = (OP & 0x7f) << 16;
		offset |= (next & 0xfffe);
		if (next & 1)
			offset |= 0xff800000u;
		return (int32_t)offset;
	} else {
		uint32_t offset = OP & 0x7e;
		if (OP & 1)
			offset |= 0xffffff80u;
		return (int32_t)offset;
	}
}

FI void ignore_pcrel(e132xs_t *c)
{
	if (OP & 0x80) {
		PC += 2;
		c->instruction_length = 2 << ILC_SHIFT;
	}
}

static uint32_t imm_length(uint32_t op)
{
	switch (op & 0x0f) {
	case 1: return 3;
	case 2: case 3: return 2;
	default: return 1;
	}
}

static int32_t get_instruction_length(e132xs_t *c, uint32_t op)
{
	switch (op >> 8) {
	case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
	case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
	case 0x90: case 0x91: case 0x92: case 0x93: case 0x94: case 0x95: case 0x96: case 0x97:
	case 0x98: case 0x99: case 0x9a: case 0x9b: case 0x9c: case 0x9d: case 0x9e: case 0x9f:
	case 0xee: case 0xef:
		c->instruction_length = ((pr16(c, PC + 2) & 0x8000) ? 3 : 2) << ILC_SHIFT;
		break;
	case 0x61: case 0x63: case 0x65: case 0x67: case 0x69: case 0x6b: case 0x6d: case 0x6f:
	case 0x71: case 0x73: case 0x75: case 0x77: case 0x79: case 0x7b: case 0x7d: case 0x7f:
		c->instruction_length = (int32_t)(imm_length(op) << ILC_SHIFT);
		break;
	case 0xb0: case 0xb1: case 0xb2: case 0xb3: case 0xb4: case 0xb5: case 0xb6: case 0xb7:
	case 0xb8: case 0xb9: case 0xba: case 0xbb: case 0xbc:
		c->instruction_length = ((op & 0x80) ? 2 : 1) << ILC_SHIFT;
		break;
	case 0xce:
		c->instruction_length = 2 << ILC_SHIFT;
		break;
	default:
		c->instruction_length = 1 << ILC_SHIFT;
		break;
	}
	c->instruction_length_valid = 1;
	return c->instruction_length;
}

// ---------------------------------------------------------------- traps, exceptions, interrupts

static void execute_trap(e132xs_t *c, uint32_t trapno)
{
	const uint32_t addr = get_trap_addr(c, trapno);
	const uint32_t reg = GET_FP + GET_FL;
	SET_ILC(c->instruction_length);
	const uint32_t oldSR = SR;
	SET_FL(6);
	SET_FP(reg);
	c->l[(0 + reg) & 0x3f] = (PC & ~1u) | GET_S;
	c->l[(1 + reg) & 0x3f] = oldSR;
	SR &= ~(M_MASK | T_MASK);
	SR |= (L_MASK | S_MASK);
	PC = addr;
	ICOUNT -= (int32_t)CC2;
}

static void execute_int(e132xs_t *c, uint32_t addr)
{
	const uint32_t reg = GET_FP + GET_FL;
	const uint32_t oldSR = SR;
	SET_FL(2);
	SET_FP(reg);
	c->l[(0 + reg) & 0x3f] = (PC & ~1u) | GET_S;
	c->l[(1 + reg) & 0x3f] = oldSR;
	SR &= ~(M_MASK | T_MASK);
	SR |= (L_MASK | S_MASK | I_MASK);
	PC = addr;
	ICOUNT -= (int32_t)CC2;
}

static void execute_exception(e132xs_t *c, uint32_t trapno)
{
	const uint32_t addr = get_trap_addr(c, trapno);
	const uint32_t reg = GET_FP + GET_FL;
	if (!c->delay_slot_taken)
		SET_ILC(c->instruction_length);
	else
		PC = c->delay_pc - ((uint32_t)c->instruction_length >> ILC_SHIFT);
	// RET does not automatically set P
	if (((OP & 0xfef0) != 0x0400) || !(OP & 0x010e))
		SET_P(1);
	const uint32_t oldSR = SR;
	SET_FL(2);
	SET_FP(reg);
	c->l[(0 + reg) & 0x3f] = (PC & ~1u) | GET_S;
	c->l[(1 + reg) & 0x3f] = oldSR;
	SR &= ~(M_MASK | T_MASK);
	SR |= (L_MASK | S_MASK);
	PC = addr;
	ICOUNT -= (int32_t)CC2;
}

static void execute_software(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SRC_CODE;
	const uint32_t sreg = c->l[(src_code + fp) & 0x3f];
	const uint32_t sregf = c->l[(src_code + 1 + fp) & 0x3f];
	SET_ILC(1 << ILC_SHIFT);
	const uint32_t addr = get_emu_code_addr(c, (OP & 0xff00) >> 8);
	const uint32_t reg = fp + GET_FL;
	// since it's sure the register is in the register part of the stack, set the stack address to a value above the
	// highest address that can be set by a following frame instruction
	const uint32_t stack_of_dst = (SP & ~0xffu) + 0x100 + (((fp + DST_CODE) & 0x3f) << 2);
	c->l[(reg + 0) & 0x3f] = stack_of_dst;
	c->l[(reg + 1) & 0x3f] = sreg;
	c->l[(reg + 2) & 0x3f] = sregf;
	c->l[(reg + 3) & 0x3f] = (PC & ~1u) | GET_S;
	c->l[(reg + 4) & 0x3f] = SR;
	SET_FL(6);
	SET_FP(reg);
	SR &= ~(M_MASK | T_MASK);
	SR |= L_MASK;
	PC = addr;
	ICOUNT -= (int32_t)c->cc6;
}

// MAME standard_irq_callback
static void irq_taken(e132xs_t *c, int line)
{
	if (c->hold & (1u << line)) {
		c->hold &= ~(1u << line);
		ISR &= ~(1u << line);
	}
	if (c->irq_ack)
		c->irq_ack(c->user, line);
}

static void check_interrupts(e132xs_t *c, int timer)
{
	// Interrupt-Lock flag isn't set
	if (GET_L)
		return;
	// quick exit if nothing
	if (!timer && (ISR & 0x7f) == 0)
		return;
	// IO3 is priority 5; state is in bit 6 of ISR; FCR bit 10 enables input and FCR bit 8 inhibits interrupt
	if ((ISR & 0x40) && (FCR & 0x00000500) == 0x00000400) {
		irq_taken(c, E132XS_IO3);
		execute_int(c, get_trap_addr(c, TRAPNO_IO3));
		return;
	}
	// timer int might be priority 6 if FCR bits 20-21 == 3; FCR bit 23 inhibits interrupt
	if (timer && (FCR & 0x00b00000) == 0x00300000) {
		c->timer_int_pending = 0;
		execute_int(c, get_trap_addr(c, TRAPNO_TIMER));
		return;
	}
	// INT1 is priority 7; state is in bit 0 of ISR; FCR bit 28 inhibits interrupt
	if ((ISR & 0x01) && (FCR & 0x10000000) == 0) {
		irq_taken(c, E132XS_INT1);
		execute_int(c, get_trap_addr(c, TRAPNO_INT1));
		return;
	}
	// timer int might be priority 8 if FCR bits 20-21 == 2
	if (timer && (FCR & 0x00b00000) == 0x00200000) {
		c->timer_int_pending = 0;
		execute_int(c, get_trap_addr(c, TRAPNO_TIMER));
		return;
	}
	// INT2 is priority 9; state is in bit 1 of ISR; FCR bit 29 inhibits interrupt
	if ((ISR & 0x02) && (FCR & 0x20000000) == 0) {
		irq_taken(c, E132XS_INT2);
		execute_int(c, get_trap_addr(c, TRAPNO_INT2));
		return;
	}
	// timer int might be priority 10 if FCR bits 20-21 == 1
	if (timer && (FCR & 0x00b00000) == 0x00100000) {
		c->timer_int_pending = 0;
		execute_int(c, get_trap_addr(c, TRAPNO_TIMER));
		return;
	}
	// INT3 is priority 11; state is in bit 2 of ISR; FCR bit 30 inhibits interrupt
	if ((ISR & 0x04) && (FCR & 0x40000000) == 0) {
		irq_taken(c, E132XS_INT3);
		execute_int(c, get_trap_addr(c, TRAPNO_INT3));
		return;
	}
	// timer int might be priority 12 if FCR bits 20-21 == 0
	if (timer && (FCR & 0x00b00000) == 0x00000000) {
		c->timer_int_pending = 0;
		execute_int(c, get_trap_addr(c, TRAPNO_TIMER));
		return;
	}
	// INT4 is priority 13; state is in bit 3 of ISR; FCR bit 31 inhibits interrupt
	if ((ISR & 0x08) && (FCR & 0x80000000u) == 0) {
		irq_taken(c, E132XS_INT4);
		execute_int(c, get_trap_addr(c, TRAPNO_INT4));
		return;
	}
	// IO1 is priority 14; state is in bit 4 of ISR; FCR bit 2 enables input and FCR bit 0 inhibits interrupt
	if ((ISR & 0x10) && (FCR & 0x00000005) == 0x00000004) {
		irq_taken(c, E132XS_IO1);
		execute_int(c, get_trap_addr(c, TRAPNO_IO1));
		return;
	}
	// IO2 is priority 15; state is in bit 5 of ISR; FCR bit 6 enables input and FCR bit 4 inhibits interrupt
	if ((ISR & 0x20) && (FCR & 0x00000050) == 0x00000040) {
		irq_taken(c, E132XS_IO2);
		execute_int(c, get_trap_addr(c, TRAPNO_IO2));
		return;
	}
}

void e132xs_set_input(e132xs_t *c, int line, int state)
{
	if (line < 0 || line >= 7)
		return;
	if (state) {
		if (state == E132XS_HOLD)
			c->hold |= 1u << line;
		else
			c->hold &= ~(1u << line);
		if (!((ISR >> line) & 1)) {
			ISR |= 1u << line;
			if (line < 4 && !((FCR >> (28 + line)) & 1))
				c->powerdown = 0;
			if (line == E132XS_IO3 && (FCR & 0x00000500) == 0x00000400)
				c->powerdown = 0;
		}
	} else {
		ISR &= ~(1u << line);
		c->hold &= ~(1u << line);
	}
}

// ---------------------------------------------------------------- opcodes (e132xsop.hxx)

static inline uint64_t get_double_word(e132xs_t *c, int gl, uint32_t dst_code, uint32_t dstf_code)
{
	const uint32_t *r = REGS(gl);
	return (uint64_t)r[dst_code] << 32 | r[dstf_code];
}
static inline void set_double_word(e132xs_t *c, int gl, uint32_t dst_code, uint32_t dstf_code, uint64_t val)
{
	uint32_t *r = REGS(gl);
	r[dst_code] = (uint32_t)(val >> 32);
	r[dstf_code] = (uint32_t)val;
}

FI void op_chk(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dreg = DG ? c->g[DST_CODE] : c->l[(DST_CODE + fp) & 0x3f];
	if (SG && src_code == SR_REGISTER) {
		if (dreg == 0)
			execute_exception(c, TRAPNO_RANGE_ERROR);
	} else {
		const uint32_t sreg = REGS(SG)[src_code];
		if ((SG && src_code == PC_REGISTER) ? (dreg >= sreg) : (dreg > sreg))
			execute_exception(c, TRAPNO_RANGE_ERROR);
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_movd(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t srcf_code = SG ? (src_code + 1) : ((src_code + 1) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t dstf_code = DG ? (dst_code + 1) : ((dst_code + 1) & 0x3f);
	const uint32_t sreg = REGS(SG)[src_code];
	const uint32_t sregf = REGS(SG)[srcf_code];

	if (DG && dst_code == PC_REGISTER) {
		// RET instruction
		if (SG && src_code < 2) {
			ICOUNT -= (int32_t)CC1;
			return;
		}
		const uint32_t old_s = SR & S_MASK;
		const uint32_t old_l = SR & L_MASK;
		PC = sreg & ~1u;
		SR = (sregf & ~(ILC_MASK | S_MASK)) | ((sreg & 0x01) << S_SHIFT);
		const uint32_t new_s = SR & S_MASK;
		const uint32_t new_l = SR & L_MASK;
		if ((!old_s && new_s) || (!new_s && !old_l && new_l))
			execute_exception(c, TRAPNO_PRIVILEGE_ERROR);
		for (int difference = sext7(GET_FP - ((SP & 0x1fc) >> 2)); difference < 0; difference++) {
			SP -= 4;
			c->l[(SP & 0xfc) >> 2] = READ_W(SP);
		}
		//TODO: not 2!
		ICOUNT -= (int32_t)CC2;
	} else if (SG && src_code == SR_REGISTER) {   // Rd doesn't denote PC and Rs denotes SR
		SR |= Z_MASK;
		SR &= ~N_MASK;
		if (DG) {
			set_global_register(c, dst_code, 0);
			set_global_register(c, dstf_code, 0);
		} else {
			c->l[dst_code] = 0;
			c->l[dstf_code] = 0;
		}
		ICOUNT -= (int32_t)CC2;
	} else {   // Rd doesn't denote PC and Rs doesn't denote SR
		SR &= ~(Z_MASK | N_MASK);
		if (sreg == 0 && sregf == 0)
			SR |= Z_MASK;
		SR |= SIGN_TO_N(sreg);
		if (DG) {
			set_global_register(c, dst_code, sreg);
			set_global_register(c, dstf_code, sregf);
		} else {
			c->l[dst_code] = sreg;
			c->l[dstf_code] = sregf;
		}
		ICOUNT -= (int32_t)CC2;
	}
}

static void op_divsu(e132xs_t *c, const int DG, const int SG, const int SIGNED)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t dstf_code = DG ? (dst_code + 1) : ((dst_code + 1) & 0x3f);
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);

	if ((SG == DG && (src_code == dst_code || src_code == dstf_code)) || (SG && src_code < 2)) {
		ICOUNT -= (int32_t)(36u << c->clck_scale);
		return;
	}
	const uint32_t sreg = REGS(SG)[src_code];
	const uint64_t dividend = get_double_word(c, DG, dst_code, dstf_code);
	if (sreg == 0 || (SIGNED && (dividend & 0x8000000000000000ull))) {
		//Rd//Rdf -> undefined, Z -> undefined, N -> undefined
		SR |= V_MASK;
		execute_exception(c, TRAPNO_RANGE_ERROR);
	} else {
		/* TODO: add quotient overflow */
		uint32_t quotient, rem;
		if (SIGNED) {
			quotient = (uint32_t)((int64_t)dividend / (int32_t)sreg);
			rem = (uint32_t)((int64_t)dividend % (int32_t)sreg);
		} else {
			quotient = (uint32_t)(dividend / sreg);
			rem = (uint32_t)(dividend % sreg);
		}
		SR &= ~(V_MASK | Z_MASK | N_MASK);
		if (quotient == 0)
			SR |= Z_MASK;
		SR |= SIGN_TO_N(quotient);
		REGS(DG)[dst_code] = rem;
		REGS(DG)[dstf_code] = quotient;
	}
	ICOUNT -= (int32_t)c->cc36;
}

FI void op_xm(e132xs_t *c, const int DG, const int SG)
{
	const uint32_t next = pr16(c, PC);
	PC += 2;
	const uint32_t sub_type = (next & 0x7000) >> 12;
	uint32_t extra_u = next & 0xfff;
	if (next & 0x8000) {
		extra_u = ((extra_u & 0xfff) << 16) | pr16(c, PC);
		PC += 2;
		c->instruction_length = (3 << 19);
	} else {
		c->instruction_length = (2 << 19);
	}
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	if ((SG && src_code == SR_REGISTER) || (DG && dst_code < 2)) {
		ICOUNT -= (int32_t)CC1;
		return;
	}
	const uint32_t sreg = REGS(SG)[src_code];
	REGS(DG)[dst_code] = sreg << (sub_type & 3);
	if (sub_type < 4) {
		if ((SG && src_code == PC_REGISTER) ? (sreg >= extra_u) : (sreg > extra_u))
			execute_exception(c, TRAPNO_RANGE_ERROR);
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_mask(e132xs_t *c, const int DG, const int SG)
{
	const uint32_t extra_u = decode_const(c);
	check_delay_pc(c);
	const uint32_t dreg = (SG ? c->g[SRC_CODE] : c->l[(SRC_CODE + GET_FP) & 0x3f]) & extra_u;
	if (dreg == 0)
		SR |= Z_MASK;
	else
		SR &= ~Z_MASK;
	if (DG)
		set_global_register(c, DST_CODE, dreg);
	else
		c->l[(DST_CODE + GET_FP) & 0x3f] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_sum(e132xs_t *c, const int DG, const int SG)
{
	const uint32_t extra_u = decode_const(c);
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t sreg = SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code];
	const uint64_t tmp = (uint64_t)sreg + (uint64_t)extra_u;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	SR |= (uint32_t)((tmp & 0x100000000ull) >> 32);
	SR |= (uint32_t)(((sreg ^ tmp) & (extra_u ^ tmp) & 0x80000000u) >> 28);
	const uint32_t dreg = (uint32_t)tmp;
	if (dreg == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	if (DG)
		set_global_register(c, DST_CODE, dreg);
	else
		c->l[(DST_CODE + fp) & 0x3f] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_sums(e132xs_t *c, const int DG, const int SG)
{
	const int32_t extra_s = (int32_t)decode_const(c);
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const int32_t sreg = (int32_t)(SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code]);
	const int64_t tmp = (int64_t)sreg + (int64_t)extra_s;
	SR &= ~(Z_MASK | N_MASK | V_MASK);
	SR |= (uint32_t)((((int64_t)sreg ^ tmp) & ((int64_t)extra_s ^ tmp) & 0x80000000ll) >> 28);
	const int32_t res = (int32_t)((uint32_t)sreg + (uint32_t)extra_s);
	if (res == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(res);
	if (DG)
		set_global_register(c, DST_CODE, (uint32_t)res);
	else
		c->l[(DST_CODE + fp) & 0x3f] = (uint32_t)res;
	ICOUNT -= (int32_t)CC1;
	if ((SR & V_MASK) && src_code != SR_REGISTER)
		execute_exception(c, TRAPNO_RANGE_ERROR);
}

FI void op_cmp(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t sreg = SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code];
	const uint32_t dreg = REGS(DG)[dst_code];
	const uint64_t tmp = (uint64_t)dreg - (uint64_t)sreg;
	SR &= ~(Z_MASK | N_MASK | V_MASK | C_MASK);
	SR |= (uint32_t)(((tmp ^ dreg) & (dreg ^ sreg) & 0x80000000u) >> 28);
	if (dreg < sreg)
		SR |= C_MASK;
	else if (dreg == sreg)
		SR |= Z_MASK;
	if ((int32_t)dreg < (int32_t)sreg)
		SR |= N_MASK;
	ICOUNT -= (int32_t)CC1;
}

FI void op_mov(e132xs_t *c, const int DG, const int SG)
{
	ICOUNT -= (int32_t)CC1;
	check_delay_pc(c);
	const int h = (SR & H_MASK) != 0;
	SR &= ~H_MASK;
	if (DG && h && !(SR & S_MASK)) {
		execute_exception(c, TRAPNO_PRIVILEGE_ERROR);
	} else {
		const uint32_t fp = GET_FP;
		const uint32_t src_code = SG ? (SRC_CODE + (h ? 16 : 0)) : ((SRC_CODE + fp) & 0x3f);
		const uint32_t sreg = SG ? ((WRITE_ONLY_REGMASK & (1u << src_code)) ? 0 : get_global_register(c, src_code))
		                         : c->l[src_code];
		SR &= ~(Z_MASK | N_MASK);
		if (sreg == 0)
			SR |= Z_MASK;
		SR |= SIGN_TO_N(sreg);
		if (DG) {
			const uint32_t dst_code = DST_CODE + (h ? 16 : 0);
			set_global_register(c, dst_code, sreg);
			if (dst_code == PC_REGISTER)
				SR &= ~M_MASK;
		} else {
			c->l[(DST_CODE + fp) & 0x3f] = sreg;
		}
	}
}

FI void op_add(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t sreg = SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code];
	uint32_t dreg = REGS(DG)[dst_code];
	const uint64_t tmp = (uint64_t)sreg + (uint64_t)dreg;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	SR |= (uint32_t)((tmp & 0x100000000ull) >> 32);
	SR |= (uint32_t)(((sreg ^ tmp) & (dreg ^ tmp) & 0x80000000u) >> 28);
	dreg = (uint32_t)tmp;
	if (dreg == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	if (DG) {
		set_global_register(c, dst_code, dreg);
		if (dst_code == 0)
			SR &= ~M_MASK;
	} else {
		c->l[dst_code] = dreg;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_adds(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const int32_t sreg = (int32_t)(SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code]);
	const int32_t dreg = (int32_t)REGS(DG)[dst_code];
	const int64_t tmp = (int64_t)sreg + (int64_t)dreg;
	SR &= ~(V_MASK | Z_MASK | N_MASK);
	SR |= (uint32_t)((((int64_t)sreg ^ tmp) & ((int64_t)dreg ^ tmp) & 0x80000000ll) >> 28);
	const uint32_t res = (uint32_t)sreg + (uint32_t)dreg;
	if (res == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(res);
	if (DG)
		set_global_register(c, dst_code, res);
	else
		c->l[dst_code] = res;
	ICOUNT -= (int32_t)CC1;
	if (SR & V_MASK)
		execute_exception(c, TRAPNO_RANGE_ERROR);
}

FI void op_cmpb(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t sreg = REGS(SG)[src_code];
	const uint32_t dreg = REGS(DG)[dst_code];
	if (dreg & sreg)
		SR &= ~Z_MASK;
	else
		SR |= Z_MASK;
	ICOUNT -= (int32_t)CC1;
}

// AND / ANDN / OR / XOR: kind 0 and, 1 andn, 2 or, 3 xor
FI void op_logic(e132xs_t *c, const int DG, const int SG, const int kind)
{
	check_delay_pc(c);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	const uint32_t sreg = SG ? c->g[SRC_CODE] : c->l[(SRC_CODE + GET_FP) & 0x3f];
	const uint32_t d = REGS(DG)[dst_code];
	const uint32_t dreg = kind == 0 ? (d & sreg) : kind == 1 ? (d & ~sreg) : kind == 2 ? (d | sreg) : (d ^ sreg);
	if (dreg == 0)
		SR |= Z_MASK;
	else
		SR &= ~Z_MASK;
	if (DG)
		set_global_register(c, dst_code, dreg);
	else
		c->l[dst_code] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_subc(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t sreg = SG ? ((src_code == SR_REGISTER) ? 0 : c->g[src_code]) : c->l[src_code];
	uint32_t dreg = REGS(DG)[dst_code];
	const uint32_t cc = GET_C;
	const uint64_t tmp = (uint64_t)dreg - ((uint64_t)sreg + (uint64_t)cc);
	const uint32_t old_z = SR & Z_MASK;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	const uint32_t sreg_c = sreg + cc;
	SR |= (uint32_t)(((tmp ^ dreg) & (dreg ^ sreg_c) & 0x80000000u) >> 28);
	SR |= (uint32_t)((tmp & 0x100000000ull) >> 32);
	dreg -= sreg + cc;
	if (old_z && dreg == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	if (DG)
		set_global_register(c, DST_CODE, dreg);
	else
		c->l[dst_code] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_not(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t dreg = ~(SG ? c->g[SRC_CODE] : c->l[(SRC_CODE + GET_FP) & 0x3f]);
	if (dreg == 0)
		SR |= Z_MASK;
	else
		SR &= ~Z_MASK;
	if (DG)
		set_global_register(c, DST_CODE, dreg);
	else
		c->l[(DST_CODE + GET_FP) & 0x3f] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_sub(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t sreg = SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code];
	uint32_t dreg = REGS(DG)[dst_code];
	const uint64_t tmp = (uint64_t)dreg - (uint64_t)sreg;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	SR |= (uint32_t)((tmp & 0x100000000ull) >> 32);
	SR |= (uint32_t)(((tmp ^ dreg) & (dreg ^ sreg) & 0x80000000u) >> 28);
	dreg -= sreg;
	if (dreg == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	if (DG) {
		set_global_register(c, dst_code, dreg);
		if (dst_code == PC_REGISTER)
			SR &= ~M_MASK;
	} else {
		c->l[dst_code] = dreg;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_subs(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const int32_t sreg = (int32_t)(SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code]);
	const int32_t dreg = (int32_t)REGS(DG)[dst_code];
	const int64_t tmp = (int64_t)dreg - (int64_t)sreg;
	SR &= ~(V_MASK | Z_MASK | N_MASK);
	SR |= (uint32_t)(((tmp ^ (int64_t)dreg) & ((int64_t)dreg ^ (int64_t)sreg) & 0x80000000ll) >> 28);
	const uint32_t res = (uint32_t)dreg - (uint32_t)sreg;
	if (res == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(res);
	if (DG)
		set_global_register(c, dst_code, res);
	else
		c->l[dst_code] = res;
	ICOUNT -= (int32_t)CC1;
	if (SR & V_MASK)
		execute_exception(c, TRAPNO_RANGE_ERROR);
}

FI void op_addc(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t sreg = REGS(SG)[src_code];
	uint32_t dreg = REGS(DG)[dst_code];
	const int old_z = (SR & Z_MASK) != 0;
	const uint32_t cc = GET_C;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	uint64_t tmp;
	if (SG && src_code == SR_REGISTER) {
		tmp = (uint64_t)dreg + (uint64_t)cc;
		SR |= (uint32_t)(((dreg ^ tmp) & tmp & 0x80000000u) >> 28);
		dreg += cc;
	} else {
		tmp = (uint64_t)sreg + (uint64_t)dreg + (uint64_t)cc;
		SR |= (uint32_t)(((sreg ^ tmp) & (dreg ^ tmp) & tmp & 0x80000000u) >> 28);
		dreg += sreg + cc;
	}
	SR |= (uint32_t)((tmp & 0x100000000ull) >> 32);
	if (dreg == 0 && old_z)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	if (DG)
		set_global_register(c, dst_code, dreg);
	else
		c->l[dst_code] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_neg(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t sreg = SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code];
	const uint64_t tmp = -(uint64_t)sreg;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	SR |= (uint32_t)((tmp & 0x100000000ull) >> 32);
	SR |= (uint32_t)((tmp & sreg & 0x80000000u) >> 28);
	const uint32_t dreg = -sreg;
	if (dreg == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	if (DG)
		set_global_register(c, DST_CODE, dreg);
	else
		c->l[(DST_CODE + fp) & 0x3f] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_negs(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const int32_t sreg = (int32_t)(SG ? ((src_code == SR_REGISTER) ? GET_C : c->g[src_code]) : c->l[src_code]);
	const int64_t tmp = -(int64_t)sreg;
	SR &= ~(V_MASK | Z_MASK | N_MASK);
	if (tmp & sreg & 0x80000000ll)
		SR |= V_MASK;
	const uint32_t res = -(uint32_t)sreg;
	if (res == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(res);
	if (DG)
		set_global_register(c, DST_CODE, res);
	else
		c->l[(DST_CODE + fp) & 0x3f] = res;
	ICOUNT -= (int32_t)CC1;
	if (GET_V)
		execute_exception(c, TRAPNO_RANGE_ERROR);
}

FI void op_cmpi(e132xs_t *c, const int DG, const int LIMM)
{
	uint32_t imm = 0;
	if (LIMM)
		imm = decode_immediate_s(c);
	check_delay_pc(c);
	if (!LIMM)
		imm = OP & 0x0f;
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	const uint32_t dreg = REGS(DG)[dst_code];
	SR &= ~(V_MASK | Z_MASK | N_MASK | C_MASK);
	const uint64_t tmp = (uint64_t)dreg - (uint64_t)imm;
	SR |= (uint32_t)(((tmp ^ dreg) & (dreg ^ imm) & 0x80000000u) >> 28);
	if (dreg < imm)
		SR |= C_MASK;
	else if (dreg == imm)
		SR |= Z_MASK;
	if ((int32_t)dreg < (int32_t)imm)
		SR |= N_MASK;
	ICOUNT -= (int32_t)CC1;
}

FI void op_movi(e132xs_t *c, const int DG, const int LIMM)
{
	ICOUNT -= (int32_t)CC1;
	uint32_t imm = 0;
	if (LIMM)
		imm = decode_immediate_s(c);
	check_delay_pc(c);
	if (!LIMM)
		imm = OP & 0x0f;
	const int h = (SR & H_MASK) != 0;
	SR &= ~H_MASK;
	if (DG && h && !(SR & S_MASK)) {
		execute_exception(c, TRAPNO_PRIVILEGE_ERROR);
	} else {
		// manual says V and C are undefined - assume V is cleared (Mission Craft seems to expect this behaviour)
		SR &= ~(Z_MASK | N_MASK | V_MASK);
		if (imm == 0)
			SR |= Z_MASK;
		SR |= SIGN_TO_N(imm);
		if (DG) {
			const uint32_t dst_code = DST_CODE + (h ? 16 : 0);
			set_global_register(c, dst_code, imm);
			if (dst_code == PC_REGISTER)
				SR &= ~M_MASK;
		} else {
			c->l[(DST_CODE + GET_FP) & 0x3f] = imm;
		}
	}
}

FI void op_addi(e132xs_t *c, const int DG, const int LIMM)
{
	uint32_t imm = 0;
	if (LIMM)
		imm = decode_immediate_s(c);
	check_delay_pc(c);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	uint32_t dreg = REGS(DG)[dst_code];
	if (!N_OP_MASK)
		imm = GET_C & (((SR & Z_MASK) ? 0 : 1) | (dreg & 0x01));
	else if (!LIMM)
		imm = OP & 0x0f;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	const uint64_t tmp = (uint64_t)imm + (uint64_t)dreg;
	SR |= (uint32_t)((tmp & 0x100000000ull) >> 32);
	SR |= (uint32_t)(((imm ^ tmp) & (dreg ^ tmp) & 0x80000000u) >> 28);
	dreg = (uint32_t)tmp;
	if (dreg == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	if (DG) {
		set_global_register(c, dst_code, dreg);
		if (dst_code == 0)
			SR &= ~M_MASK;
	} else {
		c->l[dst_code] = dreg;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_addsi(e132xs_t *c, const int DG, const int LIMM)
{
	if (!LIMM)
		check_delay_pc(c);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	const int32_t dreg = (int32_t)REGS(DG)[dst_code];
	int32_t imm;
	if (N_OP_MASK) {
		if (LIMM) {
			imm = (int32_t)decode_immediate_s(c);
			check_delay_pc(c);
		} else {
			imm = (int32_t)(OP & 0x0f);
		}
	} else {
		if (LIMM) {
			ignore_immediate_s(c);
			check_delay_pc(c);
		}
		imm = (int32_t)(SR & (((SR & Z_MASK) ? 0 : 1) | ((uint32_t)dreg & 0x01)));
	}
	SR &= ~(V_MASK | Z_MASK | N_MASK);
	const int64_t tmp = (int64_t)imm + (int64_t)dreg;
	SR |= (uint32_t)((((int64_t)imm ^ tmp) & ((int64_t)dreg ^ tmp) & 0x80000000ll) >> 28);
	const uint32_t res = (uint32_t)imm + (uint32_t)dreg;
	if (res == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(res);
	if (DG)
		set_global_register(c, dst_code, res);
	else
		c->l[dst_code] = res;
	ICOUNT -= (int32_t)CC1;
	if (SR & V_MASK)
		execute_exception(c, TRAPNO_RANGE_ERROR);
}

FI void op_cmpbi(e132xs_t *c, const int DG, const int LIMM)
{
	if (!LIMM)
		check_delay_pc(c);
	const uint32_t dreg = DG ? c->g[DST_CODE] : c->l[(DST_CODE + GET_FP) & 0x3f];
	const uint32_t n = N_VALUE;
	if (n) {
		uint32_t imm;
		if (n == 31) {
			if (LIMM) {
				ignore_immediate_s(c);
				check_delay_pc(c);
			}
			imm = 0x7fffffff;   // bit 31 = 0, others = 1
		} else {
			if (LIMM) {
				imm = decode_immediate_s(c);
				check_delay_pc(c);
			} else {
				imm = OP & 0x0f;
			}
		}
		if (dreg & imm)
			SR &= ~Z_MASK;
		else
			SR |= Z_MASK;
	} else {
		if (LIMM) {
			ignore_immediate_s(c);
			check_delay_pc(c);
		}
		if ((dreg & 0xff000000u) == 0 || (dreg & 0x00ff0000u) == 0 || (dreg & 0x0000ff00u) == 0 || (dreg & 0xffu) == 0)
			SR |= Z_MASK;
		else
			SR &= ~Z_MASK;
	}
	ICOUNT -= (int32_t)CC1;
}

// ANDNI / ORI / XORI: kind 1 andn, 2 or, 3 xor
FI void op_logici(e132xs_t *c, const int DG, const int LIMM, const int kind)
{
	uint32_t imm = 0;
	if (LIMM)
		imm = decode_immediate_s(c);
	check_delay_pc(c);
	if (kind == 1 && N_OP_MASK == 0x10f)
		imm = 0x7fffffff;   // bit 31 = 0, others = 1
	else if (!LIMM)
		imm = OP & 0x0f;
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	const uint32_t d = REGS(DG)[dst_code];
	const uint32_t dreg = kind == 1 ? (d & ~imm) : kind == 2 ? (d | imm) : (d ^ imm);
	if (DG) {
		if (dreg)
			SR &= ~Z_MASK;
		else
			SR |= Z_MASK;
		set_global_register(c, dst_code, dreg);
	} else {
		c->l[dst_code] = dreg;
		if (dreg)
			SR &= ~Z_MASK;
		else
			SR |= Z_MASK;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_shrdi(e132xs_t *c, const int HI)
{
	check_delay_pc(c);
	const uint32_t dst_code = (DST_CODE + GET_FP) & 0x3f;
	const uint32_t dstf_code = (dst_code + 1) & 0x3f;
	uint64_t val = get_double_word(c, 0, dst_code, dstf_code);
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	const uint32_t n = HI ? HI_N_VALUE : LO_N_VALUE;
	if (HI || n) {
		SR |= (uint32_t)(val >> (n - 1)) & 1;
		val >>= n;
	}
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN64_TO_N(val);
	set_double_word(c, 0, dst_code, dstf_code, val);
	ICOUNT -= (int32_t)CC2;
}

static void op_shrd(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = (SRC_CODE + fp) & 0x3f;
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t dstf_code = (dst_code + 1) & 0x3f;
	if (src_code == dst_code || src_code == dstf_code) {
		ICOUNT -= (int32_t)CC2;
		return;
	}
	uint64_t val = get_double_word(c, 0, dst_code, dstf_code);
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	const uint32_t n = c->l[src_code] & 0x1f;
	if (n) {
		SR |= (uint32_t)(val >> (n - 1)) & 1;
		val >>= n;
	}
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN64_TO_N(val);
	set_double_word(c, 0, dst_code, dstf_code, val);
	ICOUNT -= (int32_t)CC2;
}

FI void op_shr(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t n = c->l[(SRC_CODE + fp) & 0x3f] & 0x1f;
	uint32_t dreg = c->l[dst_code];
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	if (n && (dreg & (1u << (n - 1))))
		SR |= C_MASK;
	dreg >>= n;
	if (dreg == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(dreg);
	c->l[dst_code] = dreg;
	ICOUNT -= (int32_t)CC1;
}

FI void op_sardi(e132xs_t *c, const int HI)
{
	check_delay_pc(c);
	const uint32_t dst_code = (DST_CODE + GET_FP) & 0x3f;
	const uint32_t dstf_code = (dst_code + 1) & 0x3f;
	uint64_t val = get_double_word(c, 0, dst_code, dstf_code);
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	const uint32_t n = HI ? HI_N_VALUE : LO_N_VALUE;
	if (HI || n) {
		SR |= (uint32_t)(val >> (n - 1)) & 1;
		val = (uint64_t)((int64_t)val >> n);
	}
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN64_TO_N(val);
	set_double_word(c, 0, dst_code, dstf_code, val);
	ICOUNT -= (int32_t)CC2;
}

static void op_sard(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = (SRC_CODE + fp) & 0x3f;
	const uint32_t d_code = DST_CODE;
	const uint32_t dst_code = (d_code + fp) & 0x3f;
	const uint32_t dstf_code = (d_code + 1 + fp) & 0x3f;
	if (src_code == dst_code || src_code == dstf_code) {
		ICOUNT -= (int32_t)CC2;
		return;
	}
	uint64_t val = get_double_word(c, 0, dst_code, dstf_code);
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	const uint32_t n = c->l[src_code] & 0x1f;
	if (n) {
		SR |= (uint32_t)(val >> (n - 1)) & 1;
		val = (uint64_t)((int64_t)val >> n);
	}
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN64_TO_N(val);
	set_double_word(c, 0, dst_code, dstf_code, val);
	ICOUNT -= (int32_t)CC2;
}

FI void op_sar(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t n = c->l[(SRC_CODE + fp) & 0x3f] & 0x1f;
	uint32_t ret = c->l[dst_code];
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	if (n) {
		SR |= (ret >> (n - 1)) & 1;
		ret = (uint32_t)((int32_t)ret >> n);
	}
	if (ret == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(ret);
	c->l[dst_code] = ret;
	ICOUNT -= (int32_t)CC1;
}

static void shld_common(e132xs_t *c, uint32_t dst_code, uint32_t dstf_code, uint32_t n)
{
	const uint32_t high_order = c->l[dst_code];
	const uint32_t low_order = c->l[dstf_code];
	uint64_t val = (uint64_t)high_order << 32 | low_order;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	SR |= n ? (((val << (n - 1)) & 0x8000000000000000ull) ? 1 : 0) : 0;
	const uint32_t tmp = high_order << n;
	const uint32_t mask = (uint32_t)(0xffffffff00000000ull >> n);
	if (((high_order & mask) && !(tmp & 0x80000000u)) || (((high_order & mask) ^ mask) && (tmp & 0x80000000u)))
		SR |= V_MASK;
	val <<= n;
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN64_TO_N(val);
	set_double_word(c, 0, dst_code, dstf_code, val);
	ICOUNT -= (int32_t)CC2;
}

static void op_shldi(e132xs_t *c, const int HI)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t code = DST_CODE;
	const uint32_t n = HI ? HI_N_VALUE : LO_N_VALUE;
	shld_common(c, (code + fp) & 0x3f, (code + 1 + fp) & 0x3f, n);
}

static void op_shld(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t d_code = DST_CODE;
	const uint32_t src_code = (SRC_CODE + fp) & 0x3f;
	const uint32_t dst_code = (d_code + fp) & 0x3f;
	const uint32_t dstf_code = (d_code + fp + 1) & 0x3f;
	// result undefined if Ls denotes the same register as Ld or Ldf
	if (src_code == dst_code || src_code == dstf_code) {
		ICOUNT -= (int32_t)CC2;
		return;
	}
	shld_common(c, dst_code, dstf_code, c->l[src_code] & 0x1f);
}

FI void shl_common(e132xs_t *c, uint32_t *r, uint32_t n)
{
	const uint32_t base = *r;
	const uint32_t mask = n ? (0xffffffffu << (32 - n)) : 0;
	SR &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
	SR |= (n && ((base << (n - 1)) & 0x80000000u)) ? C_MASK : 0;
	const uint32_t ret = base << n;
	if (!(ret & 0x80000000u) ? (base & mask) : ((base & mask) ^ mask))
		SR |= V_MASK;
	if (ret == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(ret);
	*r = ret;
}

FI void op_shl(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t n = c->l[(SRC_CODE + fp) & 0x3f] & 0x1f;
	shl_common(c, &c->l[(DST_CODE + fp) & 0x3f], n);
	ICOUNT -= (int32_t)CC1;
}

static void op_testlz(e132xs_t *c)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t sreg = c->l[(SRC_CODE + fp) & 0x3f];
	c->l[(DST_CODE + fp) & 0x3f] = sreg ? (uint32_t)__builtin_clz(sreg) : 32;
	ICOUNT -= (int32_t)CC2;
}

static void op_rol(e132xs_t *c)
{
	// manual says V and C are undefined - assume they work like SHL
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t n = c->l[(SRC_CODE + fp) & 0x3f] & 0x1f;
	const uint32_t mask = n ? (~0u >> (32 - n)) : 0;
	uint32_t val = c->l[dst_code];
	val = n ? ((val << n) | (val >> (32 - n))) : val;
	SR &= ~(V_MASK | Z_MASK | C_MASK | N_MASK);
	if (!(val & 0x80000000u) ? (val & mask) : ((val & mask) ^ mask))
		SR |= V_MASK;
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(val);
	if (n && (val & 1))
		SR |= C_MASK;
	c->l[dst_code] = val;
	ICOUNT -= (int32_t)CC1;
}

// the displacement of the LD / ST .D / .N forms
FI uint32_t decode_dis(e132xs_t *c, uint32_t *sub_type)
{
	const uint16_t next_1 = pr16(c, PC);
	PC += 2;
	*sub_type = (next_1 & 0x3000) >> 12;
	uint32_t extra_s;
	if (next_1 & 0x8000) {
		const uint16_t next_2 = pr16(c, PC);
		PC += 2;
		c->instruction_length = (3 << 19);
		extra_s = next_2;
		extra_s |= ((uint32_t)(next_1 & 0xfff) << 16);
		if (next_1 & 0x4000)
			extra_s |= 0xf0000000u;
	} else {
		c->instruction_length = (2 << 19);
		extra_s = next_1 & 0xfff;
		if (next_1 & 0x4000)
			extra_s |= 0xfffff000u;
	}
	return extra_s;
}

// a load into Rs (global through set_global_register)
#define LOAD_SRC(code, v) do { if (SG) set_global_register(c, (code), (v)); else c->l[(code)] = (v); } while (0)

FI void op_ldxx1(e132xs_t *c, const int DG, const int SG)
{
	uint32_t sub_type;
	const uint32_t extra_s = decode_dis(c, &sub_type);
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t srcf_code = SG ? (src_code + 1) : ((src_code + 1) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t dreg = (DG && dst_code == SR_REGISTER) ? 0 : REGS(DG)[dst_code];
	if ((!DG || dst_code != SR_REGISTER) && !dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	switch (sub_type) {
	case 0: // LDBS.D
		LOAD_SRC(src_code, (uint32_t)(int32_t)(int8_t)READ_B(dreg + extra_s));
		break;
	case 1: // LDBU.D
		LOAD_SRC(src_code, READ_B(dreg + extra_s));
		break;
	case 2:
		if (extra_s & 1)
			LOAD_SRC(src_code, (uint32_t)(int32_t)(int16_t)READ_HW(dreg + (extra_s & ~1u)));
		else
			LOAD_SRC(src_code, READ_HW(dreg + (extra_s & ~1u)));
		break;
	case 3:
		switch (extra_s & 3) {
		case 0: // LDW.D
			LOAD_SRC(src_code, READ_W(dreg + extra_s));
			break;
		case 1: // LDD.D
			LOAD_SRC(src_code, READ_W(dreg + (extra_s & ~1u)));
			LOAD_SRC(srcf_code, READ_W(dreg + (extra_s & ~1u) + 4));
			ICOUNT -= (int32_t)CC1; // extra cycle
			break;
		case 2: // LDW.IOD
			LOAD_SRC(src_code, IO_READ_W(dreg + (extra_s & ~3u)));
			break;
		case 3: // LDD.IOD
			LOAD_SRC(src_code, IO_READ_W(dreg + (extra_s & ~3u)));
			LOAD_SRC(srcf_code, IO_READ_W(dreg + (extra_s & ~3u) + (1 << 13)));
			ICOUNT -= (int32_t)CC1; // extra cycle
			break;
		}
		break;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_ldxx2(e132xs_t *c, const int DG, const int SG)
{
	uint32_t sub_type;
	const uint32_t extra_s = decode_dis(c, &sub_type);
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t srcf_code = SG ? (src_code + 1) : ((src_code + 1) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	uint32_t *const dr = &REGS(DG)[dst_code];
	const uint32_t dreg = *dr;
	if (DG && dst_code <= SR_REGISTER) {
		ICOUNT -= (int32_t)CC1;
		return;
	}
	if (!dreg) {
		switch (sub_type) {
		case 0: case 1: *dr += extra_s; break;          // LDBS.N, LDBU.N
		case 2: *dr += extra_s & ~1u; break;            // LDHS.N, LDHU.N
		case 3: *dr += extra_s & ~3u; break;            // LDW.N, LDD.N, LDW.S
		}
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	const int same = DG == SG && src_code == dst_code;
	switch (sub_type) {
	case 0: // LDBS.N
		LOAD_SRC(src_code, (uint32_t)(int32_t)(int8_t)READ_B(dreg));
		if (!same)
			*dr += extra_s;
		break;
	case 1: // LDBU.N
		LOAD_SRC(src_code, READ_B(dreg));
		if (!same)
			*dr += extra_s;
		break;
	case 2:
		if (extra_s & 1) // LDHS.N
			LOAD_SRC(src_code, (uint32_t)(int32_t)(int16_t)READ_HW(dreg));
		else // LDHU.N
			LOAD_SRC(src_code, READ_HW(dreg));
		if (!same)
			*dr += (extra_s & ~1u);
		break;
	case 3:
		switch (extra_s & 3) {
		case 0: // LDW.N
			LOAD_SRC(src_code, READ_W(dreg));
			if (!same)
				*dr += extra_s;
			break;
		case 1: // LDD.N
			LOAD_SRC(src_code, READ_W(dreg));
			LOAD_SRC(srcf_code, READ_W(dreg + 4));
			if (DG != SG || (src_code != dst_code && srcf_code != dst_code))
				*dr += extra_s & ~1u;
			ICOUNT -= (int32_t)CC1; // extra cycle
			break;
		case 2: // Reserved
			break;
		case 3: // LDW.S
			if (dreg < SP)
				LOAD_SRC(src_code, READ_W(dreg));
			else
				LOAD_SRC(src_code, c->l[(dreg & 0xfc) >> 2]);
			if (!same)
				*dr += extra_s & ~3u;
			ICOUNT -= (int32_t)CC2; // extra cycles
			break;
		}
		break;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_stxx1(e132xs_t *c, const int DG, const int SG)
{
	uint32_t sub_type;
	uint32_t extra_s = decode_dis(c, &sub_type);
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t dreg = (DG && dst_code == SR_REGISTER) ? 0 : REGS(DG)[dst_code];
	const uint32_t sreg = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[src_code];
	if ((!DG || dst_code != SR_REGISTER) && !dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	switch (sub_type) {
	case 0: // STBS.D
		WRITE_B(dreg + extra_s, (uint8_t)sreg);
		if (((uint8_t)sreg != sreg) && ((int32_t)(int8_t)(uint8_t)sreg != (int32_t)sreg))
			execute_exception(c, TRAPNO_RANGE_ERROR);
		break;
	case 1: // STBU.D
		WRITE_B(dreg + extra_s, (uint8_t)sreg);
		break;
	case 2: // STHS.D, STHU.D
		WRITE_HW(dreg + (extra_s & ~1u), (uint16_t)sreg);
		if ((extra_s & 1) && ((uint16_t)sreg != sreg) && ((int32_t)(int16_t)(uint16_t)sreg != (int32_t)sreg))
			execute_exception(c, TRAPNO_RANGE_ERROR);
		break;
	case 3:
		switch (extra_s & 3) {
		case 0: // STW.D
			WRITE_W(dreg + (extra_s & ~1u), sreg);
			break;
		case 1: { // STD.D
			const uint32_t srcf_code = SG ? (src_code + 1) : ((src_code + 1) & 0x3f);
			const uint32_t sregf = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[srcf_code];
			extra_s &= ~1u;
			WRITE_W(dreg + extra_s, sreg);
			WRITE_W(dreg + extra_s + 4, sregf);
			ICOUNT -= (int32_t)CC1; // extra cycle
			break;
		}
		case 2: // STW.IOD
			IO_WRITE_W(dreg + (extra_s & ~3u), sreg);
			break;
		case 3: { // STD.IOD
			const uint32_t srcf_code = SG ? (src_code + 1) : ((src_code + 1) & 0x3f);
			const uint32_t sregf = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[srcf_code];
			extra_s &= ~3u;
			IO_WRITE_W(dreg + extra_s, sreg);
			IO_WRITE_W(dreg + extra_s + (1 << 13), sregf);
			ICOUNT -= (int32_t)CC1; // extra cycle
			break;
		}
		}
		break;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_stxx2(e132xs_t *c, const int DG, const int SG)
{
	uint32_t sub_type;
	const uint32_t extra_s = decode_dis(c, &sub_type);
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t sreg = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[src_code];
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	uint32_t *const dr = &REGS(DG)[dst_code];
	const uint32_t dreg = *dr;
	if (DG && dst_code <= SR_REGISTER) {
		ICOUNT -= (int32_t)CC1;
		return;
	}
	if (!dreg) {
		switch (sub_type) {
		case 0: case 1: *dr += extra_s; break;
		case 2: *dr += extra_s & ~1u; break;
		case 3: *dr += extra_s & ~3u; break;
		}
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	int range_error;
	switch (sub_type) {
	case 0: // STBS.N
		WRITE_B(dreg, (uint8_t)sreg);
		range_error = ((uint8_t)sreg != sreg) && ((int32_t)(int8_t)(uint8_t)sreg != (int32_t)sreg);
		*dr += extra_s;
		if (range_error)
			execute_exception(c, TRAPNO_RANGE_ERROR);
		break;
	case 1: // STBU.N
		WRITE_B(dreg, (uint8_t)sreg);
		*dr += extra_s;
		break;
	case 2: // STHS.N, STHU.N
		WRITE_HW(dreg, (uint16_t)sreg);
		range_error = (extra_s & 1) && ((uint16_t)sreg != sreg) && ((int32_t)(int16_t)(uint16_t)sreg != (int32_t)sreg);
		*dr += extra_s & ~1u;
		if (range_error)
			execute_exception(c, TRAPNO_RANGE_ERROR);
		break;
	case 3:
		switch (extra_s & 3) {
		case 0: // STW.N
			WRITE_W(dreg, sreg);
			*dr += extra_s;
			break;
		case 1: { // STD.N
			WRITE_W(dreg, sreg);
			*dr += extra_s & ~1u;
			const uint32_t srcf_code = SG ? (src_code + 1) : ((src_code + 1) & 0x3f);
			const uint32_t sregf = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[srcf_code];
			WRITE_W(dreg + 4, sregf);
			ICOUNT -= (int32_t)CC1; // extra cycle
			break;
		}
		case 2: // Reserved
			break;
		case 3: // STW.S
			if (dreg < SP)
				WRITE_W(dreg, sreg);
			else
				c->l[(dreg & 0xfc) >> 2] = sreg;
			*dr += (extra_s & ~3u);
			ICOUNT -= (int32_t)CC2; // extra cycles
			break;
		}
		break;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_shri(e132xs_t *c, const int HI, const int DG)
{
	check_delay_pc(c);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	uint32_t val = REGS(DG)[dst_code];
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	const uint32_t n = HI ? HI_N_VALUE : LO_N_VALUE;
	if (HI || n)
		SR |= (val >> (n - 1)) & 1;
	val >>= n;
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(val);
	if (DG)
		set_global_register(c, dst_code, val);
	else
		c->l[dst_code] = val;
	ICOUNT -= (int32_t)CC1;
}

FI void op_sari(e132xs_t *c, const int HI, const int DG)
{
	check_delay_pc(c);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	const uint32_t n = HI ? HI_N_VALUE : LO_N_VALUE;
	uint32_t val = REGS(DG)[dst_code];
	SR &= ~(C_MASK | Z_MASK | N_MASK);
	if (HI || n) {
		SR |= (val >> (n - 1)) & 1;
		val = (uint32_t)((int32_t)val >> n);
	}
	if (val == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(val);
	if (DG)
		set_global_register(c, dst_code, val);
	else
		c->l[dst_code] = val;
	ICOUNT -= (int32_t)CC1;
}

FI void op_shli(e132xs_t *c, const int HI, const int DG)
{
	check_delay_pc(c);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	const uint32_t n = HI ? HI_N_VALUE : LO_N_VALUE;
	if (DG) {
		uint32_t v = c->g[dst_code];
		shl_common(c, &v, n);
		set_global_register(c, dst_code, v);
	} else {
		shl_common(c, &c->l[dst_code], n);
	}
	ICOUNT -= (int32_t)CC1;
}

static void op_mulsu(e132xs_t *c, const int DG, const int SG, const int SIGNED)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	const uint32_t dstf_code = DG ? (dst_code + 1) : ((dst_code + 1) & 0x3f);
	if ((SG && src_code < 2) || (DG && dst_code < 2))
		return;
	const uint32_t dreg = REGS(DG)[dst_code];
	const uint32_t sreg = REGS(SG)[src_code];
	const uint64_t double_word = SIGNED ? (uint64_t)((int64_t)(int32_t)sreg * (int32_t)dreg)
	                                    : (uint64_t)sreg * dreg;
	SR &= ~(Z_MASK | N_MASK);
	if (double_word == 0)
		SR |= Z_MASK;
	SR |= SIGN64_TO_N(double_word);
	set_double_word(c, DG, dst_code, dstf_code, double_word);
	ICOUNT -= (int32_t)c->cc6;
	if (SIGNED && ((int32_t)sreg >= -0x8000 && (int32_t)sreg <= 0x7fff) && ((int32_t)dreg >= -0x8000 && (int32_t)dreg <= 0x7fff))
		ICOUNT += (int32_t)CC2;
	else if (!SIGNED && sreg <= 0xffff && dreg <= 0xffff)
		ICOUNT += (int32_t)CC2;
}

FI void op_set(e132xs_t *c, const int HI, const int DG)
{
	static const uint32_t mask[16] = { 0, 0, 0, 0, (N_MASK | Z_MASK), (N_MASK | Z_MASK), N_MASK, N_MASK,
		(C_MASK | Z_MASK), (C_MASK | Z_MASK), C_MASK, C_MASK, Z_MASK, Z_MASK, V_MASK, V_MASK };
	check_delay_pc(c);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + GET_FP) & 0x3f);
	const uint32_t n = LO_N_VALUE;
	if (DG && dst_code < 2) {
		ICOUNT -= (int32_t)CC1;
		return;
	}
	uint32_t *const dr = &REGS(DG)[dst_code];
	if (HI) {
		if (n >= 4 || n == 2) {
			// set: 0xffffffff for even n >= 4; unset: 0xffffffff for n == 2 and odd n
			if (SR & mask[n])
				*dr = (n >= 4 && !(n & 1)) ? 0xffffffffu : 0;
			else
				*dr = (n == 2 || (n >= 5 && (n & 1))) ? 0xffffffffu : 0;
		}
	} else {
		if (n == 0) {
			*dr = (SP & 0xfffffe00u) | (GET_FP << 2) | (((SP & 0x100) && ((SR >> 31) == 0)) ? 1 : 0);
		} else if (n >= 2) {
			if (SR & mask[n])
				*dr = (n >= 4 && !(n & 1)) ? 1 : 0;
			else
				*dr = (n == 2 || (n >= 5 && (n & 1))) ? 1 : 0;
		}
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_mul(e132xs_t *c, const int DG, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = DG ? DST_CODE : ((DST_CODE + fp) & 0x3f);
	if ((SG && src_code < 2) || (DG && dst_code < 2))
		return;
	const uint32_t dreg = REGS(DG)[dst_code];
	const uint32_t sreg = REGS(SG)[src_code];
	const uint32_t result = sreg * dreg;
	SR &= ~(Z_MASK | N_MASK);
	if (result == 0)
		SR |= Z_MASK;
	SR |= SIGN_TO_N(result);
	REGS(DG)[dst_code] = result;
	if (((int32_t)sreg < -0x8000) || ((int32_t)sreg > 0x7fff) || ((int32_t)dreg < -0x8000) || ((int32_t)dreg > 0x7fff))
		ICOUNT -= (int32_t)(5u << c->clck_scale);
	else
		ICOUNT -= (int32_t)(3u << c->clck_scale);
}

static inline int32_t get_lhs(uint32_t v) { return (int16_t)(uint16_t)(v & 0xffff); }
static inline int32_t get_rhs(uint32_t v) { return (int16_t)(uint16_t)(v >> 16); }

static void op_extend(e132xs_t *c)
{
	c->instruction_length = (2 << 19);
	const uint32_t func = pr16(c, PC);
	PC += 2;
	check_delay_pc(c);
	//TODO: add locks, overflow error and other things
	const uint32_t fp = GET_FP;
	const uint32_t vals = c->l[(SRC_CODE + fp) & 0x3f];
	const uint32_t vald = c->l[(DST_CODE + fp) & 0x3f];
	uint32_t *const g = c->g;
	switch (func) {
	case EMUL:
	case EMUL_N:   // used in "N" type cpu
		g[15] = vals * vald;
		break;
	case EMULU:
		set_double_word(c, 1, 14, 15, (uint64_t)vals * vald);
		break;
	case EMULS:
		set_double_word(c, 1, 14, 15, (uint64_t)((int64_t)(int32_t)vals * (int32_t)vald));
		break;
	case EMAC:
		g[15] += vals * vald;
		break;
	case EMACD:
		set_double_word(c, 1, 14, 15, get_double_word(c, 1, 14, 15) + (uint64_t)((int64_t)(int32_t)vals * (int32_t)vald));
		break;
	case EMSUB:
		g[15] -= vals * vald;
		break;
	case EMSUBD:
		set_double_word(c, 1, 14, 15, get_double_word(c, 1, 14, 15) - (uint64_t)((int64_t)(int32_t)vals * (int32_t)vald));
		break;
	case EHMAC:
		g[15] = g[15] + (uint32_t)(get_lhs(vald) * get_lhs(vals)) + (uint32_t)(get_rhs(vald) * get_rhs(vals));
		break;
	case EHMACD: {
		const int64_t result = (int64_t)get_double_word(c, 1, 14, 15) + (int64_t)(get_lhs(vald) * get_lhs(vals)) +
		                       (int64_t)(get_rhs(vald) * get_rhs(vals));
		set_double_word(c, 1, 14, 15, (uint64_t)result);
		break;
	}
	case EHCMULD:
		g[14] = (uint32_t)((int64_t)get_lhs(vald) * get_lhs(vals) - (int64_t)get_rhs(vald) * get_rhs(vals));
		g[15] = (uint32_t)((int64_t)get_lhs(vald) * get_rhs(vals) + (int64_t)get_rhs(vald) * get_lhs(vals));
		break;
	case EHCMACD:
		g[14] += (uint32_t)((int64_t)get_lhs(vald) * get_lhs(vals) - (int64_t)get_rhs(vald) * get_rhs(vals));
		g[15] += (uint32_t)((int64_t)get_lhs(vald) * get_rhs(vals) + (int64_t)get_rhs(vald) * get_lhs(vals));
		break;
	case EHCSUMD: {
		const uint32_t r14 = g[14], r15 = g[15];
		g[14] = (((vals >> 16) + r14) << 16) | (((vals & 0xffff) + r15) & 0xffff);
		g[15] = (((vals >> 16) - r14) << 16) | (((vals & 0xffff) - r15) & 0xffff);
		break;
	}
	case EHCFFTD: {
		const uint32_t r14 = g[14], r15 = g[15];
		g[14] = (((((vals & 0xffff0000u) >> 16) + (r14 >> 15)) << 16) & 0xffff0000u) | (((vals & 0xffff) + (r15 >> 15)) & 0xffff);
		g[15] = (((((vals & 0xffff0000u) >> 16) - (r14 >> 15)) << 16) & 0xffff0000u) | (((vals & 0xffff) - (r15 >> 15)) & 0xffff);
		break;
	}
	case EHCFFTSD: {
		const uint32_t r14 = g[14], r15 = g[15];
		g[14] = ((((((vals & 0xffff0000u) >> 16) + (r14 >> 15)) >> 1) << 16) & 0xffff0000u) |
		        (((((vals & 0xffff) + (r15 >> 15)) >> 1) & 0xffff));
		g[15] = ((((((vals & 0xffff0000u) >> 16) - (r14 >> 15)) >> 1) << 16) & 0xffff0000u) |
		        (((((vals & 0xffff) - (r15 >> 15)) >> 1) & 0xffff));
		break;
	}
	default:
		break;
	}
	ICOUNT -= (int32_t)CC1; // TODO: with the latency it can change
}

FI void op_ldwr(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t dreg = c->l[(DST_CODE + fp) & 0x3f];
	if (!dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	if (SG)
		set_global_register(c, SRC_CODE, READ_W(dreg));
	else
		c->l[(SRC_CODE + fp) & 0x3f] = READ_W(dreg);
	ICOUNT -= (int32_t)CC1;
}

FI void op_lddr(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dreg = c->l[(DST_CODE + fp) & 0x3f];
	if (!dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	if (SG) {
		set_global_register(c, src_code, READ_W(dreg));
		set_global_register(c, src_code + 1, READ_W(dreg + 4));
	} else {
		c->l[src_code] = READ_W(dreg);
		c->l[(src_code + 1) & 0x3f] = READ_W(dreg + 4);
	}
	ICOUNT -= (int32_t)CC2;
}

FI void op_ldwp(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t dreg = c->l[dst_code];
	if (!dreg) {
		c->l[dst_code] = dreg + 4;
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	if (SG) {
		set_global_register(c, SRC_CODE, READ_W(dreg));
		c->l[dst_code] = dreg + 4;
	} else {
		const uint32_t src_code = (SRC_CODE + fp) & 0x3f;
		c->l[src_code] = READ_W(dreg);
		// post increment the destination register if it's different from the source one (needed by Hidden Catch)
		if (src_code != dst_code)
			c->l[dst_code] = dreg + 4;
	}
	ICOUNT -= (int32_t)CC1;
}

FI void op_lddp(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t dreg = c->l[dst_code];
	if (!dreg) {
		c->l[dst_code] = dreg + 8;
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	if (SG) {
		set_global_register(c, src_code, READ_W(dreg));
		set_global_register(c, src_code + 1, READ_W(dreg + 4));
		c->l[dst_code] = dreg + 8;
	} else {
		const uint32_t srcf_code = (src_code + 1) & 0x3f;
		c->l[src_code] = READ_W(dreg);
		c->l[srcf_code] = READ_W(dreg + 4);
		// post increment the destination register if it's different from the source one and the "next source" one
		if (src_code != dst_code && srcf_code != dst_code)
			c->l[dst_code] = dreg + 8;
	}
	ICOUNT -= (int32_t)CC2;
}

FI void op_stwr(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t sreg = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[src_code];
	const uint32_t dreg = c->l[(DST_CODE + fp) & 0x3f];
	if (!dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	WRITE_W(dreg, sreg);
	ICOUNT -= (int32_t)CC1;
}

FI void op_stdr(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t srcf_code = SG ? (SRC_CODE + 1) : ((src_code + 1) & 0x3f);
	const uint32_t sreg = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[src_code];
	const uint32_t sregf = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[srcf_code];
	const uint32_t dreg = c->l[(DST_CODE + GET_FP) & 0x3f];
	if (!dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	WRITE_W(dreg, sreg);
	WRITE_W(dreg + 4, sregf);
	ICOUNT -= (int32_t)CC2;
}

FI void op_stwp(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t sreg = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[src_code];
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t dreg = c->l[dst_code];
	c->l[dst_code] = dreg + 4;
	if (!dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	WRITE_W(dreg, sreg);
	ICOUNT -= (int32_t)CC1;
}

FI void op_stdp(e132xs_t *c, const int SG)
{
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	const uint32_t srcf_code = SG ? (src_code + 1) : ((src_code + 1) & 0x3f);
	const uint32_t sreg = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[src_code];
	const uint32_t dst_code = (DST_CODE + fp) & 0x3f;
	const uint32_t dreg = c->l[dst_code];
	c->l[dst_code] = dreg + 8;
	if (!dreg) {
		execute_exception(c, TRAPNO_POINTER_ERROR);
		return;
	}
	WRITE_W(dreg, sreg);
	const uint32_t sregf = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[srcf_code];
	WRITE_W(dreg + 4, sregf);
	ICOUNT -= (int32_t)CC2;
}

static const uint32_t s_condition_masks[6] = { V_MASK, Z_MASK, C_MASK, C_MASK | Z_MASK, N_MASK, N_MASK | Z_MASK };

FI void op_db(e132xs_t *c, const int cond, const int set)
{
	const int taken = set ? (SR & s_condition_masks[cond]) != 0 : !(SR & s_condition_masks[cond]);
	if (!c->delay_slot) {
		c->delay_slot_taken = 0;
		if (taken) {
			const int32_t offset = decode_pcrel(c);
			c->delay_slot = 1;
			c->delay_pc = PC + (uint32_t)offset;
			c->intblock = 2;
			ICOUNT -= (int32_t)CC2;
		} else {
			ignore_pcrel(c);
			ICOUNT -= (int32_t)CC1;
		}
	} else {
		c->delay_slot = 0;
		c->delay_slot_taken = 1;
		if (taken) {
			const int32_t offset = decode_pcrel(c);
			PC = c->delay_pc + (uint32_t)offset;
			SR &= ~M_MASK;
			ICOUNT -= (int32_t)CC2;
		} else {
			ignore_pcrel(c);
			PC = c->delay_pc;
			ICOUNT -= (int32_t)CC1;
		}
	}
}

static void op_dbr(e132xs_t *c)
{
	const int32_t offset = decode_pcrel(c);
	if (!c->delay_slot) {
		c->delay_pc = PC + (uint32_t)offset;
		c->delay_slot = 1;
		c->delay_slot_taken = 0;
		c->intblock = 2;
	} else {
		c->delay_slot = 0;
		c->delay_slot_taken = 1;
		PC = c->delay_pc + (uint32_t)offset;
		SR &= ~M_MASK;
	}
	ICOUNT -= (int32_t)CC2;
}

static void op_frame(e132xs_t *c)
{
	check_delay_pc(c);
	const uint8_t realfp = (uint8_t)(GET_FP - SRC_CODE);
	const uint32_t dst_code = DST_CODE;
	SET_FP(realfp);
	SET_FL(dst_code);
	SR &= ~M_MASK;
	int difference = sext7(((SP & 0x1fc) >> 2) + (64 - 10) - (realfp + GET_FL));
	if (difference < 0) {   // else it's finished
		const int tmp_flag = SP >= UB;
		do {
			WRITE_W(SP, c->l[(SP & 0xfc) >> 2]);
			SP += 4;
			difference++;
		} while (difference < 0);
		if (tmp_flag)
			execute_exception(c, TRAPNO_FRAME_ERROR);
	}
	// TODO: More than 1 cycle!
	ICOUNT -= (int32_t)CC1;
}

FI void op_call(e132xs_t *c, const int SG)
{
	const uint16_t imm_1 = pr16(c, PC);
	PC += 2;
	uint32_t extra_s;
	if (imm_1 & 0x8000) {
		const uint16_t imm_2 = pr16(c, PC);
		PC += 2;
		SET_ILC(3 << 19);
		c->instruction_length = (3 << 19);
		extra_s = imm_2;
		extra_s |= ((uint32_t)(imm_1 & 0x3fff) << 16);
		if (imm_1 & 0x4000)
			extra_s |= 0xc0000000u;
	} else {
		extra_s = imm_1 & 0x3fff;
		SET_ILC(2 << 19);
		c->instruction_length = (2 << 19);
		if (imm_1 & 0x4000)
			extra_s |= 0xffffc000u;
	}
	check_delay_pc(c);
	const uint32_t fp = GET_FP;
	const uint32_t src_code = SG ? SRC_CODE : ((SRC_CODE + fp) & 0x3f);
	uint32_t dst_code = DST_CODE;
	const uint32_t sreg = (SG && src_code == SR_REGISTER) ? 0 : REGS(SG)[src_code];
	if (!DST_CODE)
		dst_code = 16;
	const uint32_t dreg_index = dst_code + fp;
	c->l[dreg_index & 0x3f] = (PC & ~1u) | GET_S;
	c->l[(dreg_index + 1) & 0x3f] = SR;
	SET_FP(fp + dst_code);
	SET_FL(6);   // default value for call
	SR &= ~M_MASK;
	PC = (extra_s & ~1u) + sreg;
	c->intblock = 2;
	// TODO: More than 1 cycle!
	ICOUNT -= (int32_t)CC1;
}

FI void op_br(e132xs_t *c)
{
	const int32_t offset = decode_pcrel(c);
	check_delay_pc(c);
	PC += (uint32_t)offset;
	SR &= ~M_MASK;
	ICOUNT -= (int32_t)CC2;
}

FI void op_b(e132xs_t *c, const int cond, const int set)
{
	if (((SR & s_condition_masks[cond]) != 0) == (set != 0)) {
		op_br(c);
	} else {
		ignore_pcrel(c);
		check_delay_pc(c);
		ICOUNT -= (int32_t)CC1;
	}
}

static void op_trap(e132xs_t *c)
{
	static const uint32_t conditions[16] = {
		0, 0, 0, 0, N_MASK | Z_MASK, N_MASK | Z_MASK, N_MASK, N_MASK, C_MASK | Z_MASK, C_MASK | Z_MASK, C_MASK, C_MASK,
		Z_MASK, Z_MASK, V_MASK, 0 };
	static const uint8_t trap_if_set[16] = { 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };
	ICOUNT -= (int32_t)CC1;
	check_delay_pc(c);
	const uint32_t trapno = (OP & 0xfc) >> 2;
	const uint32_t code = ((OP & 0x300) >> 6) | (OP & 0x03);
	if (trap_if_set[code]) {
		if (SR & conditions[code])
			execute_trap(c, trapno);
	} else {
		if (!(SR & conditions[code]))
			execute_trap(c, trapno);
	}
}

// ---------------------------------------------------------------- copy loops

// 1 when check_interrupts would take an interrupt now (its tests without the action)
static inline int irq_would_fire(const e132xs_t *c)
{
	const uint32_t isr = c->g[ISR_REGISTER], fcr = c->g[FCR_REGISTER];
	if (GET_L || !(c->timer_int_pending | (isr & 0x7f)))
		return 0;
	if (c->timer_int_pending && !(fcr & 0x00800000u))
		return 1;
	return ((isr & 0x40) && (fcr & 0x00000500) == 0x00000400) || ((isr & 0x01) && !(fcr & 0x10000000u)) ||
	       ((isr & 0x02) && !(fcr & 0x20000000u)) || ((isr & 0x04) && !(fcr & 0x40000000u)) ||
	       ((isr & 0x08) && !(fcr & 0x80000000u)) || ((isr & 0x10) && (fcr & 0x00000005) == 0x00000004) ||
	       ((isr & 0x20) && (fcr & 0x00000050) == 0x00000040);
}

// how many of n rounds keep a pointer at a, stepping inc a round, inside a's 64 KB page and not 0
static inline uint32_t fl_page_rounds(uint32_t a, uint32_t inc, uint32_t n)
{
	if (!a)
		return 0;
	const int32_t s = (int32_t)inc;
	if (!s)
		return n;
	uint32_t room;   // (the rounds after the first)
	if (s > 0)
		room = ((a | 0xffffu) - a) / (uint32_t)s;
	else {
		uint32_t lo = a & ~0xffffu;
		if (!lo)
			lo = 1;
		room = (a - lo) / (uint32_t)-s;
	}
	return room < n ? room + 1 : n;
}

enum { FL_LDWP, FL_LDBS, FL_LDBU, FL_LDHU, FL_LDHS, FL_LDW, FL_LDD };   // the loads (LDD.N: loop_fast2 only)
enum { FL_STWP, FL_STBU, FL_STHU, FL_STW, FL_STD };                    // the stores (STD.N: loop_fast2 only)

// The copy loops the Eolith games spend most of their time in (their library's blits), a DBNE back over a load and
// an ADDI of the count, the store in the delay slot:
//     T: LDW.P Ls, Ld         (or LDBS/LDBU/LDHU/LDHS/LDW.N Ls, Ld, dis: local registers, a 12-bit displacement)
//        ADDI Lc, imm         (a one-word immediate)
//     A: DBNE T
//        STW.P Ls', Ld'       (or STBU/STHU/STW.N Ls', Ld', dis)
// run an iteration at a time with each instruction's effects, cycles and interrupt checks as the interpreter makes
// them, without its per-instruction bookkeeping. Called right after the DBNE at A was taken (its delay slot next).
// It leaves at an instruction boundary (the state as after the last instruction it ran, instruction_length that
// one's) whenever the interpreter has to go on: the cycles run out, an interrupt would be taken, a pointer is 0 (the
// exception), a handler aborted the slice, or the count reached 0 (the DBNE falls through).
static void loop_fast(e132xs_t *c)
{
	const uint32_t A = PC - 2, T = c->delay_pc;
	const uint32_t body = A - T;
	if (body != 4 && body != 6)
		return;
	const uint32_t fp = GET_FP;
	// the load at T
	const uint32_t w0 = pr16(c, T);
	int ld;
	uint32_t ld_inc = 4, ld_len;
	if (body == 4 && (w0 >> 8) == 0xd5) {
		ld = FL_LDWP;
		ld_len = 1;
	} else if (body == 6 && (w0 >> 8) == 0x97) {
		const uint32_t w1 = pr16(c, T + 2);
		if (w1 & 0x8000)
			return;
		uint32_t extra = w1 & 0xfff;
		if (w1 & 0x4000)
			extra |= 0xfffff000u;
		switch ((w1 >> 12) & 3) {
		case 0: ld = FL_LDBS; ld_inc = extra; break;
		case 1: ld = FL_LDBU; ld_inc = extra; break;
		case 2: ld = (extra & 1) ? FL_LDHS : FL_LDHU; ld_inc = extra & ~1u; break;
		default:
			if (extra & 3)
				return;
			ld = FL_LDW;
			ld_inc = extra;
			break;
		}
		ld_len = 2;
	} else {
		return;
	}
	const uint32_t ldd = (((w0 & 0xf0) >> 4) + fp) & 0x3f, lds = ((w0 & 0x0f) + fp) & 0x3f;
	// the ADDI at A - 2
	const uint32_t wa = pr16(c, A - 2);
	uint32_t imm;
	if ((wa >> 8) == 0x6a && (wa & 0x0f))
		imm = wa & 0x0f;
	else if ((wa >> 8) == 0x6b && ((wa & 0x0f) == 0 || (wa & 0x0f) >= 4))
		imm = (uint32_t)s_immediate_values[wa & 0x0f];
	else
		return;
	const uint32_t cnt = (((wa & 0xf0) >> 4) + fp) & 0x3f;
	// the store in the delay slot
	const uint32_t ws = pr16(c, A + 2);
	int st;
	uint32_t st_inc = 4, st_len;
	if ((ws >> 8) == 0xdd) {
		st = FL_STWP;
		st_len = 1;
	} else if ((ws >> 8) == 0x9f) {
		const uint32_t w1 = pr16(c, A + 4);
		if (w1 & 0x8000)
			return;
		uint32_t extra = w1 & 0xfff;
		if (w1 & 0x4000)
			extra |= 0xfffff000u;
		switch ((w1 >> 12) & 3) {
		case 1: st = FL_STBU; st_inc = extra; break;
		case 2:
			if (extra & 1)
				return;   // (STHS: a range check)
			st = FL_STHU;
			st_inc = extra;
			break;
		case 3:
			if (extra & 3)
				return;
			st = FL_STW;
			st_inc = extra;
			break;
		default:
			return;   // (STBS: a range check)
		}
		st_len = 2;
	} else {
		return;
	}
	const uint32_t std_ = (((ws & 0xf0) >> 4) + fp) & 0x3f, sts = ((ws & 0x0f) + fp) & 0x3f;
	const uint32_t c1 = c->cc1, c2 = c->cc2;
	uint32_t *const l = c->l;
	const uint32_t st_delay_pc = A + 2 + 2 * st_len;
	// the loop's state in locals, written back where it leaves; a handler sees the PC and the cycles it would see
	int32_t icount = c->icount;
	uint32_t sr = SR;
	int irq = irq_would_fire(c);   // (changes only through a handler: e132xs_set_input)
	int phase;   // the last instruction run: 0 the DBNE taken, 1 the store, 2 the load, 3 the ADDI, 4 the DBNE not taken
#define FL_HANDLER(pc, call) do { \
		c->icount = icount; \
		PC = (pc); \
		call; \
		icount = c->icount; \
		irq = irq_would_fire(c); \
		aborted = c->abort; \
	} while (0)

	// whole rounds at once when the registers are apart
	const int batch = std_ != ldd && std_ != cnt && std_ != lds && ldd != cnt && ldd != lds &&
	                  cnt != lds && sts != std_ && sts != ldd && sts != cnt;
	const uint32_t per = 3 * c1 + c2, st_step = st == FL_STWP ? 4 : st_inc;

	for (;;) {
		int aborted = 0;
		if (batch && !irq && icount > (int32_t)(2 * per)) {
			// rounds that all run in full: cycles left after each, the DBNE taken (the count not reaching 0), both
			// pointers inside their page (host memory: no handler, nothing can change the interrupt state) and not 0
			uint32_t sp = l[std_], lp = l[ldd], cv = l[cnt];
			uint32_t n = (uint32_t)(icount - 1) / per;
			{   // (the round whose ADDI makes the count 0 falls through: the ones before it)
				const uint32_t step = (int32_t)imm < 0 ? 0u - imm : imm;
				const uint32_t m = (int32_t)imm < 0 ? cv : 0u - cv;   // (0 after k rounds: k * step == m mod 2^32)
				const uint32_t k = m && m % step == 0 ? m / step : 0xffffffffu / step + 1;   // (first k, or past 2^32)
				if (k - 1 < n)
					n = k - 1;
			}
			n = fl_page_rounds(sp, st_step, n);
			n = fl_page_rounds(lp, ld_inc, n);
			const e132xs_page *wp = &c->wr[sp >> E132XS_PAGE_SHIFT], *rp = &c->rd[lp >> E132XS_PAGE_SHIFT];
			if (!wp->base && c->wq && st != FL_STBU)
				wp = &c->wq[sp >> E132XS_PAGE_SHIFT];   // (bit 15 set: not stored)
			else if (!wp->base)
				n = 0;
			if (n >= 2 && rp->base && wp->base) {
				const int q15 = !c->wr[sp >> E132XS_PAGE_SHIFT].base;
				uint32_t v = l[lds];
				const uint32_t sconst = l[sts];
				const int s_is_v = sts == lds;
				uint8_t *const wb = wp->base, *const rb = rp->base;
				const uint32_t wm = wp->mask, rm = rp->mask;
				if (ld == FL_LDWP && st == FL_STWP && s_is_v) {   // (the usual word copy, without the switches)
					if (!q15)
						for (uint32_t i = 0; i < n; i++, sp += 4, lp += 4) {
							*(uint32_t *)(wb + (sp & ~3u & wm)) = v;
							v = *(const uint32_t *)(rb + (lp & ~3u & rm));
						}
					else
						for (uint32_t i = 0; i < n; i++, sp += 4, lp += 4) {
							if (!(v & 0x80000000u))
								*(uint16_t *)(wb + (((sp & ~3u) & wm) ^ 2)) = (uint16_t)(v >> 16);
							if (!(v & 0x8000))
								*(uint16_t *)(wb + ((((sp & ~3u) + 2) & wm) ^ 2)) = (uint16_t)v;
							v = *(const uint32_t *)(rb + (lp & ~3u & rm));
						}
				} else
				for (uint32_t i = 0; i < n; i++) {
					const uint32_t sreg = s_is_v ? v : sconst;
					switch (st) {
					case FL_STBU:
						wb[(sp & wm) ^ 3] = (uint8_t)sreg;
						break;
					case FL_STHU:
						if (!q15 || !(sreg & 0x8000))
							*(uint16_t *)(wb + ((sp & ~1u & wm) ^ 2)) = (uint16_t)sreg;
						break;
					default:
						if (!q15)
							*(uint32_t *)(wb + (sp & ~3u & wm)) = sreg;
						else {
							if (!(sreg & 0x80000000u))
								*(uint16_t *)(wb + (((sp & ~3u) & wm) ^ 2)) = (uint16_t)(sreg >> 16);
							if (!(sreg & 0x8000))
								*(uint16_t *)(wb + ((((sp & ~3u) + 2) & wm) ^ 2)) = (uint16_t)sreg;
						}
						break;
					}
					sp += st_step;
					switch (ld) {
					case FL_LDWP:
					case FL_LDW: v = *(const uint32_t *)(rb + (lp & ~3u & rm)); break;
					case FL_LDBS: v = (uint32_t)(int32_t)(int8_t)rb[(lp & rm) ^ 3]; break;
					case FL_LDBU: v = rb[(lp & rm) ^ 3]; break;
					case FL_LDHS: v = (uint32_t)(int32_t)(int16_t)*(const uint16_t *)(rb + ((lp & ~1u & rm) ^ 2)); break;
					default: v = *(const uint16_t *)(rb + ((lp & ~1u & rm) ^ 2)); break;
					}
					lp += ld_inc;
				}
				cv += n * imm;
				l[std_] = sp;
				l[lds] = v;
				l[ldd] = lp;
				l[cnt] = cv;
				{   // (the last ADDI's flags)
					const uint32_t dreg = cv - imm;
					const uint64_t tmp = (uint64_t)imm + dreg;
					sr &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
					sr |= (uint32_t)(tmp >> 32) & 1;
					sr |= (uint32_t)(((imm ^ tmp) & (dreg ^ tmp) & 0x80000000u) >> 28);
					sr |= SIGN_TO_N(cv);
				}
				icount -= (int32_t)(n * per);
			}
		}
		// ---- the store (the delay slot: no interrupt check before it)
		if (icount <= 0) {
			phase = 0;
			break;
		}
		const uint32_t sp = l[std_];
		if (!sp) {   // (the pointer error: the interpreter raises it)
			phase = 0;
			break;
		}
		{
			const uint32_t sreg = l[sts];
			const e132xs_page *pg = &c->wr[sp >> E132XS_PAGE_SHIFT];
			switch (st) {
			case FL_STWP:
				l[std_] = sp + 4;
				if (pg->base)
					*(uint32_t *)(pg->base + (sp & ~3u & pg->mask)) = sreg;
				else if (c->wq && c->wq[sp >> E132XS_PAGE_SHIFT].base) {
					wq16(&c->wq[sp >> E132XS_PAGE_SHIFT], sp & ~3u, (uint16_t)(sreg >> 16));
					wq16(&c->wq[sp >> E132XS_PAGE_SHIFT], (sp & ~3u) + 2, (uint16_t)sreg);
				} else if (c->write32)
					FL_HANDLER(T, c->write32(c->user, sp & ~3u, sreg));
				break;
			case FL_STBU:
				if (pg->base)
					pg->base[(sp & pg->mask) ^ 3] = (uint8_t)sreg;
				else if (c->write8)
					FL_HANDLER(T, c->write8(c->user, sp, (uint8_t)sreg));
				l[std_] += st_inc;
				break;
			case FL_STHU:
				if (pg->base)
					*(uint16_t *)(pg->base + ((sp & ~1u & pg->mask) ^ 2)) = (uint16_t)sreg;
				else if (c->wq && c->wq[sp >> E132XS_PAGE_SHIFT].base)
					wq16(&c->wq[sp >> E132XS_PAGE_SHIFT], sp & ~1u, (uint16_t)sreg);
				else if (c->write16)
					FL_HANDLER(T, c->write16(c->user, sp & ~1u, (uint16_t)sreg));
				l[std_] += st_inc;
				break;
			default:
				if (pg->base)
					*(uint32_t *)(pg->base + (sp & ~3u & pg->mask)) = sreg;
				else if (c->wq && c->wq[sp >> E132XS_PAGE_SHIFT].base) {
					wq16(&c->wq[sp >> E132XS_PAGE_SHIFT], sp & ~3u, (uint16_t)(sreg >> 16));
					wq16(&c->wq[sp >> E132XS_PAGE_SHIFT], (sp & ~3u) + 2, (uint16_t)sreg);
				} else if (c->write32)
					FL_HANDLER(T, c->write32(c->user, sp & ~3u, sreg));
				l[std_] += st_inc;
				break;
			}
		}
		icount -= (int32_t)c1;
		if (aborted) {
			phase = 1;
			break;
		}
		// ---- the load
		if (icount <= 0 || irq) {
			phase = 1;
			break;
		}
		const uint32_t lp = l[ldd];
		if (!lp) {
			phase = 1;
			break;
		}
		{
			const e132xs_page *pg = &c->rd[lp >> E132XS_PAGE_SHIFT];
			const uint32_t ldpc = T + 2 * ld_len;
			uint32_t v = 0;
			switch (ld) {
			case FL_LDWP:
			case FL_LDW:
				if (pg->base)
					v = *(const uint32_t *)(pg->base + (lp & ~3u & pg->mask));
				else if (c->read32)
					FL_HANDLER(ldpc, v = c->read32(c->user, lp & ~3u));
				break;
			case FL_LDBS:
			case FL_LDBU:
				if (pg->base)
					v = pg->base[(lp & pg->mask) ^ 3];
				else if (c->read8)
					FL_HANDLER(ldpc, v = c->read8(c->user, lp));
				if (ld == FL_LDBS)
					v = (uint32_t)(int32_t)(int8_t)v;
				break;
			default:
				if (pg->base)
					v = *(const uint16_t *)(pg->base + ((lp & ~1u & pg->mask) ^ 2));
				else if (c->read16)
					FL_HANDLER(ldpc, v = c->read16(c->user, lp & ~1u));
				if (ld == FL_LDHS)
					v = (uint32_t)(int32_t)(int16_t)v;
				break;
			}
			l[lds] = v;
			if (lds != ldd)
				l[ldd] = lp + ld_inc;
		}
		icount -= (int32_t)c1;
		if (aborted) {
			phase = 2;
			break;
		}
		// ---- the ADDI
		if (icount <= 0 || irq) {
			phase = 2;
			break;
		}
		{
			const uint32_t dreg = l[cnt];
			const uint64_t tmp = (uint64_t)imm + dreg;
			const uint32_t res = (uint32_t)tmp;
			sr &= ~(C_MASK | V_MASK | Z_MASK | N_MASK);
			sr |= (uint32_t)(tmp >> 32) & 1;
			sr |= (uint32_t)(((imm ^ tmp) & (dreg ^ tmp) & 0x80000000u) >> 28);
			if (res == 0)
				sr |= Z_MASK;
			sr |= SIGN_TO_N(res);
			l[cnt] = res;
		}
		icount -= (int32_t)c1;
		// ---- the DBNE (its interrupt check: nothing changed since the ADDI's)
		if (icount <= 0) {
			phase = 3;
			break;
		}
		if (sr & Z_MASK) {   // not taken: the store runs as an instruction of its own
			icount -= (int32_t)c1;
			phase = 4;
			break;
		}
		icount -= (int32_t)c2;
	}
#undef FL_HANDLER
	c->icount = icount;
	SR = sr;
	switch (phase) {
	case 0:   // after the DBNE taken (as op_db left it)
		PC = A + 2;
		c->delay_slot = 1;
		c->delay_pc = T;
		c->delay_slot_taken = 0;
		c->intblock = 2;
		c->instruction_length = 1 << ILC_SHIFT;
		break;
	case 1:   // after the store (check_delay_pc in the delay slot)
		PC = T;
		c->delay_pc = st_delay_pc;
		c->delay_slot = 0;
		c->delay_slot_taken = 1;
		c->intblock = 1;
		c->instruction_length = (int32_t)(st_len << ILC_SHIFT);
		break;
	case 2:   // after the load
	case 3:   // after the ADDI
		PC = phase == 2 ? T + 2 * ld_len : A;
		c->delay_pc = st_delay_pc;
		c->delay_slot = 0;
		c->delay_slot_taken = 0;
		c->intblock = 0;
		c->instruction_length = (int32_t)((phase == 2 ? ld_len : 1) << ILC_SHIFT);
		break;
	default:  // after the DBNE not taken
		PC = A + 2;
		c->delay_pc = st_delay_pc;
		c->delay_slot = 0;
		c->delay_slot_taken = 0;
		c->intblock = 0;
		c->instruction_length = 1 << ILC_SHIFT;
		break;
	}
}

// a load at a as loop_fast takes them (LDW.P, or LDBS / LDBU / LDHU / LDHS / LDW.N with a 12-bit step: locals):
// its words (0 = not one), kind, step and registers
static uint32_t fl_load_at(e132xs_t *c, uint32_t a, uint32_t fp, int *kind, uint32_t *inc, uint32_t *rd, uint32_t *rs)
{
	const uint32_t w0 = pr16(c, a);
	uint32_t len;
	*inc = 4;
	if ((w0 >> 8) == 0xd5) {
		*kind = FL_LDWP;
		len = 1;
	} else if ((w0 >> 8) == 0x97) {
		const uint32_t w1 = pr16(c, a + 2);
		if (w1 & 0x8000)
			return 0;
		uint32_t extra = w1 & 0xfff;
		if (w1 & 0x4000)
			extra |= 0xfffff000u;
		switch ((w1 >> 12) & 3) {
		case 0: *kind = FL_LDBS; *inc = extra; break;
		case 1: *kind = FL_LDBU; *inc = extra; break;
		case 2: *kind = (extra & 1) ? FL_LDHS : FL_LDHU; *inc = extra & ~1u; break;
		default:
			if ((extra & 3) == 1) {   // (LDD.N: a cycle more)
				*kind = FL_LDD;
				*inc = extra & ~1u;
				break;
			}
			if (extra & 3)
				return 0;
			*kind = FL_LDW;
			*inc = extra;
			break;
		}
		len = 2;
	} else
		return 0;
	*rd = (((w0 & 0xf0) >> 4) + fp) & 0x3f;
	*rs = ((w0 & 0x0f) + fp) & 0x3f;
	return len;
}

// a store at a as loop_fast takes them (STW.P, or STBU / STHU / STW.N with a 12-bit step: locals)
static uint32_t fl_store_at(e132xs_t *c, uint32_t a, uint32_t fp, int *kind, uint32_t *inc, uint32_t *rd, uint32_t *rs)
{
	const uint32_t w0 = pr16(c, a);
	uint32_t len;
	*inc = 4;
	if ((w0 >> 8) == 0xdd) {
		*kind = FL_STWP;
		len = 1;
	} else if ((w0 >> 8) == 0x9f) {
		const uint32_t w1 = pr16(c, a + 2);
		if (w1 & 0x8000)
			return 0;
		uint32_t extra = w1 & 0xfff;
		if (w1 & 0x4000)
			extra |= 0xfffff000u;
		switch ((w1 >> 12) & 3) {
		case 1: *kind = FL_STBU; *inc = extra; break;
		case 2:
			if (extra & 1)
				return 0;   // (STHS: a range check)
			*kind = FL_STHU;
			*inc = extra;
			break;
		case 3:
			if ((extra & 3) == 1) {   // (STD.N: a cycle more)
				*kind = FL_STD;
				*inc = extra & ~1u;
				break;
			}
			if (extra & 3)
				return 0;
			*kind = FL_STW;
			*inc = extra;
			break;
		default:
			return 0;   // (STBS: a range check)
		}
		len = 2;
	} else
		return 0;
	*rd = (((w0 & 0xf0) >> 4) + fp) & 0x3f;
	*rs = ((w0 & 0x0f) + fp) & 0x3f;
	return len;
}

// The other copy loop shape (Fortress 2 Blue Arcade's blits), a BNE back over a load, a store and an ADDI:
//     T: LDxx Ls, Ld      (as loop_fast's loads)
//        STxx Ls', Ld'    (as loop_fast's stores)
//        ADDI Lc, imm     (a one-word immediate)
//     A: BNE T
// Called right after the BNE at A was taken (PC = T), with nothing pending (no interrupt block, line, timer or power
// down: no interrupt check can act) and no trace: whole rounds that keep cycles left and the BNE taken, both pointers
// inside a host page and not 0, run at once with the interpreter's memory, register and flag effects and cycles (it
// leaves right after a BNE taken, as it came). Anything else is left to the interpreter.
static void loop_fill(e132xs_t *c, uint32_t A);
static void loop_fast2(e132xs_t *c, uint32_t A)
{
	const uint32_t T = PC, fp = GET_FP;
	if ((pr16(c, T) >> 8) == 0xdf || (pr16(c, T) >> 8) == 0xdd || (pr16(c, T) >> 8) == 0x9f) {
		loop_fill(c, A);
		return;
	}
	int ld, st;
	uint32_t ld_inc, st_inc, ldd, lds, std_, sts;
	const uint32_t ll = fl_load_at(c, T, fp, &ld, &ld_inc, &ldd, &lds);
	if (!ll)
		return;
	const uint32_t sl = fl_store_at(c, T + 2 * ll, fp, &st, &st_inc, &std_, &sts);
	if (!sl || T + 2 * (ll + sl) + 2 != A)
		return;
	const uint32_t wa = pr16(c, A - 2);
	uint32_t imm;
	if ((wa >> 8) == 0x6a && (wa & 0x0f))
		imm = wa & 0x0f;
	else if ((wa >> 8) == 0x6b && ((wa & 0x0f) == 0 || (wa & 0x0f) >= 4))
		imm = (uint32_t)s_immediate_values[wa & 0x0f];
	else
		return;
	const uint32_t cnt = (((wa & 0xf0) >> 4) + fp) & 0x3f;
	if (std_ == ldd || std_ == cnt || std_ == lds || ldd == cnt || ldd == lds || cnt == lds || sts == std_ ||
	    sts == ldd || sts == cnt)
		return;
	const uint32_t lds2 = (lds + 1) & 0x3f, sts2 = (sts + 1) & 0x3f;   // (the second registers of LDD / STD)
	if (ld == FL_LDD && (lds2 == ldd || lds2 == std_ || lds2 == cnt || (sts != lds && sts == lds2)))
		return;
	if (st == FL_STD && (sts2 == std_ || sts2 == ldd || sts2 == cnt || (ld == FL_LDD ? sts != lds && (sts2 == lds || sts2 == lds2) : sts2 == lds)))
		return;
	uint32_t *const l = c->l;
	const uint32_t per = (3 + (ld == FL_LDD) + (st == FL_STD)) * c->cc1 + c->cc2, st_step = st == FL_STWP ? 4 : st_inc;
	uint32_t lp = l[ldd], sp = l[std_], cv = l[cnt];
	uint32_t n = c->icount > 0 ? (uint32_t)(c->icount - 1) / per : 0;
	{
		const uint32_t step = (int32_t)imm < 0 ? 0u - imm : imm;
		const uint32_t m = (int32_t)imm < 0 ? cv : 0u - cv;
		const uint32_t k = m && m % step == 0 ? m / step : 0xffffffffu / step + 1;
		if (k - 1 < n)
			n = k - 1;
	}
	n = fl_page_rounds(lp, ld_inc, n);
	n = fl_page_rounds(sp, st_step, n);
	if (ld == FL_LDD) {   // (the second word in the same page every round)
		if (((lp + 4) ^ lp) >> E132XS_PAGE_SHIFT)
			return;
		n = fl_page_rounds(lp + 4, ld_inc, n);
	}
	if (st == FL_STD) {
		if (((sp + 4) ^ sp) >> E132XS_PAGE_SHIFT)
			return;
		n = fl_page_rounds(sp + 4, st_step, n);
	}
	if (!n)
		return;
	const e132xs_page *wp = &c->wr[sp >> E132XS_PAGE_SHIFT], *rp = &c->rd[lp >> E132XS_PAGE_SHIFT];
	const int q15 = !wp->base;
	if (q15) {
		if (!c->wq || st == FL_STBU)
			return;
		wp = &c->wq[sp >> E132XS_PAGE_SHIFT];
	}
	if (!rp->base || !wp->base)
		return;
	uint8_t *const wb = wp->base, *const rb = rp->base;
	const uint32_t wm = wp->mask, rm = rp->mask;
	const uint32_t sconst = l[sts], sconst2 = l[sts2];
	const int s_is_v = sts == lds;
	uint32_t v = l[lds], v2 = l[lds2];
	for (uint32_t i = 0; i < n; i++) {
		switch (ld) {
		case FL_LDD:
			v = *(const uint32_t *)(rb + (lp & ~3u & rm));
			v2 = *(const uint32_t *)(rb + ((lp + 4) & ~3u & rm));
			break;
		case FL_LDWP:
		case FL_LDW: v = *(const uint32_t *)(rb + (lp & ~3u & rm)); break;
		case FL_LDBS: v = (uint32_t)(int32_t)(int8_t)rb[(lp & rm) ^ 3]; break;
		case FL_LDBU: v = rb[(lp & rm) ^ 3]; break;
		case FL_LDHS: v = (uint32_t)(int32_t)(int16_t)*(const uint16_t *)(rb + ((lp & ~1u & rm) ^ 2)); break;
		default: v = *(const uint16_t *)(rb + ((lp & ~1u & rm) ^ 2)); break;
		}
		lp += ld_inc;
		const uint32_t sreg = s_is_v ? v : sconst;
		switch (st) {
		case FL_STD: {
			const uint32_t sreg2 = s_is_v && ld == FL_LDD ? v2 : sts2 == lds ? v : sconst2;
			const uint32_t a0 = sp & ~3u, a1 = (sp + 4) & ~3u;
			if (!q15) {
				*(uint32_t *)(wb + (a0 & wm)) = sreg;
				*(uint32_t *)(wb + (a1 & wm)) = sreg2;
			} else {
				if (!(sreg & 0x80000000u))
					*(uint16_t *)(wb + ((a0 & wm) ^ 2)) = (uint16_t)(sreg >> 16);
				if (!(sreg & 0x8000))
					*(uint16_t *)(wb + (((a0 + 2) & wm) ^ 2)) = (uint16_t)sreg;
				if (!(sreg2 & 0x80000000u))
					*(uint16_t *)(wb + ((a1 & wm) ^ 2)) = (uint16_t)(sreg2 >> 16);
				if (!(sreg2 & 0x8000))
					*(uint16_t *)(wb + (((a1 + 2) & wm) ^ 2)) = (uint16_t)sreg2;
			}
			break;
		}
		case FL_STBU:
			wb[(sp & wm) ^ 3] = (uint8_t)sreg;
			break;
		case FL_STHU:
			if (!q15 || !(sreg & 0x8000))
				*(uint16_t *)(wb + ((sp & ~1u & wm) ^ 2)) = (uint16_t)sreg;
			break;
		default:
			if (!q15)
				*(uint32_t *)(wb + (sp & ~3u & wm)) = sreg;
			else {
				if (!(sreg & 0x80000000u))
					*(uint16_t *)(wb + (((sp & ~3u) & wm) ^ 2)) = (uint16_t)(sreg >> 16);
				if (!(sreg & 0x8000))
					*(uint16_t *)(wb + ((((sp & ~3u) + 2) & wm) ^ 2)) = (uint16_t)sreg;
			}
			break;
		}
		sp += st_step;
	}
	cv += n * imm;
	l[lds] = v;
	if (ld == FL_LDD)
		l[lds2] = v2;
	l[ldd] = lp;
	l[std_] = sp;
	l[cnt] = cv;
	const uint32_t dreg = cv - imm;   // (the last ADDI's flags; its BNE cleared M)
	const uint64_t tmp = (uint64_t)imm + dreg;
	uint32_t sr = SR & ~(C_MASK | V_MASK | Z_MASK | N_MASK | M_MASK);
	sr |= (uint32_t)(tmp >> 32) & 1;
	sr |= (uint32_t)(((imm ^ tmp) & (dreg ^ tmp) & 0x80000000u) >> 28);
	sr |= SIGN_TO_N(cv);
	SR = sr;
	c->icount -= (int32_t)(n * per);
}

// a word store as wr32 makes it into a host page (q15: an e132xs_map_skip15 page)
static inline void fl_st32(uint8_t *wb, uint32_t wm, int q15, uint32_t a, uint32_t v)
{
	a &= ~3u;
	if (!q15)
		*(uint32_t *)(wb + (a & wm)) = v;
	else {
		if (!(v & 0x80000000u))
			*(uint16_t *)(wb + ((a & wm) ^ 2)) = (uint16_t)(v >> 16);
		if (!(v & 0x8000))
			*(uint16_t *)(wb + (((a + 2) & wm) ^ 2)) = (uint16_t)v;
	}
}

// the store page of a pointer for n rounds of `step` (`words` 32-bit words a round from it): its host page, or NULL
// (and the rounds cut to those that stay in it)
static uint8_t *fl_wpage(e132xs_t *c, uint32_t sp, uint32_t step, int words, uint32_t *n, uint32_t *wm, int *q15)
{
	*n = fl_page_rounds(sp, step, *n);
	if (words == 2) {
		if (((sp + 4) ^ sp) >> E132XS_PAGE_SHIFT)
			return NULL;
		*n = fl_page_rounds(sp + 4, step, *n);
	}
	const e132xs_page *wp = &c->wr[sp >> E132XS_PAGE_SHIFT];
	*q15 = !wp->base;
	if (*q15) {
		if (!c->wq)
			return NULL;
		wp = &c->wq[sp >> E132XS_PAGE_SHIFT];
	}
	*wm = wp->mask;
	return wp->base;
}

// the ADDI of a copy loop at a: its immediate (a one-word form) and register; 0 = not one
static int fl_addi_at(e132xs_t *c, uint32_t a, uint32_t fp, uint32_t *imm, uint32_t *reg)
{
	const uint32_t wa = pr16(c, a);
	if ((wa >> 8) == 0x6a && (wa & 0x0f))
		*imm = wa & 0x0f;
	else if ((wa >> 8) == 0x6b && ((wa & 0x0f) == 0 || (wa & 0x0f) >= 4))
		*imm = (uint32_t)s_immediate_values[wa & 0x0f];
	else
		return 0;
	*reg = (((wa & 0xf0) >> 4) + fp) & 0x3f;
	return 1;
}

// the rounds before the one whose ADDI (imm) brings the count cv to 0, at most n
static inline uint32_t fl_count_rounds(uint32_t cv, uint32_t imm, uint32_t n)
{
	const uint32_t step = (int32_t)imm < 0 ? 0u - imm : imm;
	const uint32_t m = (int32_t)imm < 0 ? cv : 0u - cv;
	const uint32_t k = m && m % step == 0 ? m / step : 0xffffffffu / step + 1;
	return k - 1 < n ? k - 1 : n;
}

// the ADDI's flags (the result cv, the immediate imm) into SR (C V Z N), M cleared when clear_m
static inline void fl_addi_flags(e132xs_t *c, uint32_t cv, uint32_t imm, int clear_m)
{
	const uint32_t dreg = cv - imm;
	const uint64_t tmp = (uint64_t)imm + dreg;
	uint32_t sr = SR & ~(C_MASK | V_MASK | Z_MASK | N_MASK | (clear_m ? M_MASK : 0));
	sr |= (uint32_t)(tmp >> 32) & 1;
	sr |= (uint32_t)(((imm ^ tmp) & (dreg ^ tmp) & 0x80000000u) >> 28);
	if (!cv)
		sr |= Z_MASK;
	sr |= SIGN_TO_N(cv);
	SR = sr;
}

// loop_fast2's fill form (landbrk, puzzlekg: memset): a store of constant registers, ADDI, BNE back
//     T: STD.P / STW.P Ls, Ld   or STBU / STHU / STW / STD.N Ls, Ld, dis
//        ADDI Lc, imm
//     A: BNE T
static void loop_fill(e132xs_t *c, uint32_t A)
{
	const uint32_t T = PC, fp = GET_FP;
	const uint32_t w0 = pr16(c, T);
	int st;
	uint32_t st_inc, std_, sts, sl, cost = c->cc1;
	if ((w0 >> 8) == 0xdf) {   // STD.P: 2 cycles
		st = FL_STD;
		st_inc = 8;
		std_ = (((w0 & 0xf0) >> 4) + fp) & 0x3f;
		sts = ((w0 & 0x0f) + fp) & 0x3f;
		sl = 1;
		cost = c->cc2;
	} else {
		sl = fl_store_at(c, T, fp, &st, &st_inc, &std_, &sts);
		if (!sl)
			return;
		if (st == FL_STWP)
			st_inc = 4;
		if (st == FL_STD)
			cost = 2 * c->cc1;
	}
	uint32_t imm, cnt;
	if (T + 2 * sl + 2 != A || !fl_addi_at(c, T + 2 * sl, fp, &imm, &cnt))
		return;
	const uint32_t sts2 = (sts + 1) & 0x3f;
	if (std_ == cnt || sts == std_ || sts == cnt || (st == FL_STD && (sts2 == std_ || sts2 == cnt)))
		return;
	uint32_t *const l = c->l;
	const uint32_t per = cost + c->cc1 + c->cc2;
	uint32_t sp = l[std_], cv = l[cnt];
	uint32_t n = c->icount > 0 ? (uint32_t)(c->icount - 1) / per : 0, wm;
	n = fl_count_rounds(cv, imm, n);
	int q15;
	uint8_t *const wb = fl_wpage(c, sp, st_inc, st == FL_STD ? 2 : 1, &n, &wm, &q15);
	if (!wb || !n || (q15 && st == FL_STBU))
		return;
	const uint32_t v = l[sts], v2 = l[sts2];
	for (uint32_t i = 0; i < n; i++, sp += st_inc) {
		switch (st) {
		case FL_STD:
			fl_st32(wb, wm, q15, sp, v);
			fl_st32(wb, wm, q15, sp + 4, v2);
			break;
		case FL_STBU:
			wb[(sp & wm) ^ 3] = (uint8_t)v;
			break;
		case FL_STHU:
			if (!q15 || !(v & 0x8000))
				*(uint16_t *)(wb + ((sp & ~1u & wm) ^ 2)) = (uint16_t)v;
			break;
		default:
			fl_st32(wb, wm, q15, sp, v);
			break;
		}
	}
	cv += n * imm;
	l[std_] = sp;
	l[cnt] = cv;
	fl_addi_flags(c, cv, imm, 1);   // (the BNE cleared M)
	c->icount -= (int32_t)(n * per);
}

// The library's double-word copy (Candy Candy, Land Breaker: memcpy by 8 bytes while the count is at least 8):
//     H: CMPI Lc, k        (the short form)
//        BN out
//        LDD.P Ls, Ld
//        ADDI Lc, imm
//     A: DBR H
//        STD.P Ls', Ld'     (the delay slot)
// Called right after the DBR at A (not in a delay slot: its slot next), with no line, timer or power down pending
// (the check at CMPI after the DBR's interrupt block finds nothing) and no trace: whole rounds (the store, CMPI, BN not
// taken, the load, ADDI, DBR) that keep cycles left, both pointers inside a host page and not 0, run at once (it
// leaves as it came, right after the DBR).
static void loop_fast3(e132xs_t *c, uint32_t A)
{
	const uint32_t H = c->delay_pc, fp = GET_FP;
	if (A != H + 8)
		return;
	const uint32_t wc = pr16(c, H), wb_ = pr16(c, H + 2), wl = pr16(c, H + 4), ws = pr16(c, A + 2);
	if ((wc >> 8) != 0x62 || (wb_ >> 8) != 0xf8 || (wb_ & 0x80) || (wl >> 8) != 0xd7 || (ws >> 8) != 0xdf)
		return;
	uint32_t imm, cnt;
	if (!fl_addi_at(c, H + 6, fp, &imm, &cnt) || cnt != ((((wc & 0xf0) >> 4) + fp) & 0x3f))
		return;
	const int32_t k = (int32_t)(wc & 0x0f), d = -(int32_t)imm;
	if (d <= 0)
		return;
	const uint32_t ldd = (((wl & 0xf0) >> 4) + fp) & 0x3f, lds = ((wl & 0x0f) + fp) & 0x3f, lds2 = (lds + 1) & 0x3f;
	const uint32_t std_ = (((ws & 0xf0) >> 4) + fp) & 0x3f, sts = ((ws & 0x0f) + fp) & 0x3f, sts2 = (sts + 1) & 0x3f;
	const uint32_t r[5] = { std_, ldd, cnt, lds, lds2 };
	for (int i = 0; i < 5; i++)
		for (int j = i + 1; j < 5; j++)
			if (r[i] == r[j])
				return;
	if (sts != lds)
		for (int i = 0; i < 5; i++)
			if (sts == r[i] || sts2 == r[i])
				return;
	uint32_t *const l = c->l;
	const uint32_t per = 3 * c->cc1 + 3 * c->cc2;
	uint32_t sp = l[std_], lp = l[ldd], cv = l[cnt];
	uint32_t n = c->icount > 0 ? (uint32_t)(c->icount - 1) / per : 0, wm;
	{   // (the rounds whose CMPI finds the count at least k, signed)
		const int64_t c0 = (int32_t)cv;
		const uint32_t m = c0 >= k ? (uint32_t)((c0 - k) / d + 1) : 0;
		if (m < n)
			n = m;
	}
	// (the store comes first: its pointer for n rounds, the load's for the n loads)
	int q15;
	uint8_t *const wb = fl_wpage(c, sp, 8, 2, &n, &wm, &q15);
	if (!wb || !n)
		return;
	if (((lp + 4) ^ lp) >> E132XS_PAGE_SHIFT)
		return;
	n = fl_page_rounds(lp, 8, n);
	n = fl_page_rounds(lp + 4, 8, n);
	const e132xs_page *rp = &c->rd[lp >> E132XS_PAGE_SHIFT];
	if (!rp->base || !n)
		return;
	const uint8_t *const rb = rp->base;
	const uint32_t rm = rp->mask;
	uint32_t v = l[lds], v2 = l[lds2];
	const uint32_t s1 = l[sts], s2 = l[sts2];
	const int s_is_v = sts == lds;
	for (uint32_t i = 0; i < n; i++) {
		fl_st32(wb, wm, q15, sp, s_is_v ? v : s1);
		fl_st32(wb, wm, q15, sp + 4, s_is_v ? v2 : s2);
		sp += 8;
		v = *(const uint32_t *)(rb + (lp & ~3u & rm));
		v2 = *(const uint32_t *)(rb + ((lp + 4) & ~3u & rm));
		lp += 8;
	}
	cv += n * imm;
	l[std_] = sp;
	l[ldd] = lp;
	l[lds] = v;
	l[lds2] = v2;
	l[cnt] = cv;
	fl_addi_flags(c, cv, imm, 0);
	c->icount -= (int32_t)(n * per);
}

// The rectangle copy around loop_fast's loop (Raccoon World, Puzzle King, Land Breaker: sprites a few pixels wide,
// whose rows the inner loop alone does little for):
//     P: MOV Lw, Lwidth
//     T: LDxx Ls, Ld         (loop_fast's loads)
//        ADDI Lw, imm
//        DBNE T
//        STxx Ls', Ld'       (loop_fast's stores; the delay slot, or after the DBNE on the row's last pixel)
//        ADD Ld, Lsstride
//        ADDI Lrows, imm2
//     A: DBNE P
//        ADD Ld', Ldstride   (the delay slot)
// Called right after the DBNE at A was taken (its slot next), nothing pending, no trace: whole rows (the slot's ADD to
// the DBNE at A taken again) that keep cycles left, the row's pointers inside a host page each and not 0, run at once
// with the interpreter's effects, flags and cycles; it leaves as it came.
static inline uint32_t decode_pcrel_short(uint32_t op)   // (decode_pcrel's one-word form)
{
	return (op & 0x7e) | ((op & 1) ? 0xffffff80u : 0);
}

static void loop_rows(e132xs_t *c)
{
	const uint32_t A = PC - 2, P = c->delay_pc, fp = GET_FP;
	const uint32_t wm_ = pr16(c, P), wsa = pr16(c, A + 2);
	if ((wm_ >> 8) != 0x27 || (wsa >> 8) != 0x2b)
		return;
	int ld, st;
	uint32_t ld_inc, st_inc, ldd, lds, std_, sts, imm_w, cw, imm_r, cr;
	const uint32_t T = P + 2;
	const uint32_t ll = fl_load_at(c, T, fp, &ld, &ld_inc, &ldd, &lds);
	if (!ll || ld == FL_LDD || !fl_addi_at(c, T + 2 * ll, fp, &imm_w, &cw))
		return;
	const uint32_t A1 = T + 2 * ll + 2, wd = pr16(c, A1);
	if (wd >> 8 != 0xe3 || (wd & 0x80) || A1 + 2 + decode_pcrel_short(wd) != T)
		return;
	const uint32_t sl = fl_store_at(c, A1 + 2, fp, &st, &st_inc, &std_, &sts);
	if (!sl || st == FL_STD)
		return;
	const uint32_t aa = A1 + 2 + 2 * sl, wadd = pr16(c, aa);
	if ((wadd >> 8) != 0x2b || !fl_addi_at(c, aa + 2, fp, &imm_r, &cr) || aa + 4 != A)
		return;
	const uint32_t width = ((wm_ & 0x0f) + fp) & 0x3f, wdst = (((wm_ & 0xf0) >> 4) + fp) & 0x3f;
	const uint32_t sadd_d = (((wadd & 0xf0) >> 4) + fp) & 0x3f, sstride = ((wadd & 0x0f) + fp) & 0x3f;
	const uint32_t dadd_d = (((wsa & 0xf0) >> 4) + fp) & 0x3f, dstride = ((wsa & 0x0f) + fp) & 0x3f;
	if (wdst != cw || sadd_d != ldd || dadd_d != std_)
		return;
	{   // (the registers apart: Lw, Ld, Ld', Lrows, Ls; the others only read, Ls' Ls or read only)
		const uint32_t r[5] = { cw, ldd, std_, cr, lds };
		for (int i = 0; i < 5; i++) {
			for (int j = i + 1; j < 5; j++)
				if (r[i] == r[j])
					return;
			if (width == r[i] || sstride == r[i] || dstride == r[i] || (sts != lds && sts == r[i]))
				return;
		}
	}
	uint32_t *const l = c->l;
	const uint32_t c1 = c->cc1, c2 = c->cc2, st_step = st == FL_STWP ? 4 : st_inc;
	int32_t icount = c->icount;
	uint32_t rows = l[cr];
	int done = 0;
	for (;;) {
		if (rows + imm_r == 0)
			break;   // (the last row: its DBNE falls through)
		const uint32_t w = l[width];
		uint32_t k;   // (the row's pixels: the inner ADDI brings w to 0 after k)
		{
			const uint32_t step = (int32_t)imm_w < 0 ? 0u - imm_w : imm_w;
			const uint32_t m = (int32_t)imm_w < 0 ? w : 0u - w;
			if (!m || m % step)
				break;
			k = m / step;
		}
		const int64_t R = (int64_t)(5 + 3 * k) * c1 + (int64_t)k * c2;
		if (k > 0x10000 || icount - R < 1)
			break;
		uint32_t sp = l[std_] + l[dstride], lp = l[ldd];
		if (fl_page_rounds(lp, ld_inc, k) != k || fl_page_rounds(sp, st_step, k) != k)
			break;
		const e132xs_page *rp = &c->rd[lp >> E132XS_PAGE_SHIFT], *wp = &c->wr[sp >> E132XS_PAGE_SHIFT];
		const int q15 = !wp->base;
		if (q15 && (!c->wq || st == FL_STBU))
			break;
		if (q15)
			wp = &c->wq[sp >> E132XS_PAGE_SHIFT];
		if (!rp->base || !wp->base)
			break;
		uint8_t *const wb = wp->base;
		const uint8_t *const rb = rp->base;
		const uint32_t wm = wp->mask, rm = rp->mask;
		uint32_t v = l[lds];
		const uint32_t sconst = l[sts];
		for (uint32_t i = 0; i < k; i++) {
			switch (ld) {
			case FL_LDWP:
			case FL_LDW: v = *(const uint32_t *)(rb + (lp & ~3u & rm)); break;
			case FL_LDBS: v = (uint32_t)(int32_t)(int8_t)rb[(lp & rm) ^ 3]; break;
			case FL_LDBU: v = rb[(lp & rm) ^ 3]; break;
			case FL_LDHS: v = (uint32_t)(int32_t)(int16_t)*(const uint16_t *)(rb + ((lp & ~1u & rm) ^ 2)); break;
			default: v = *(const uint16_t *)(rb + ((lp & ~1u & rm) ^ 2)); break;
			}
			lp += ld_inc;
			const uint32_t sreg = sts == lds ? v : sconst;
			switch (st) {
			case FL_STBU: wb[(sp & wm) ^ 3] = (uint8_t)sreg; break;
			case FL_STHU:
				if (!q15 || !(sreg & 0x8000))
					*(uint16_t *)(wb + ((sp & ~1u & wm) ^ 2)) = (uint16_t)sreg;
				break;
			default: fl_st32(wb, wm, q15, sp, sreg); break;
			}
			sp += st_step;
		}
		l[lds] = v;
		l[cw] = w + k * imm_w;
		l[std_] = sp;
		l[ldd] = lp + l[sstride];
		rows += imm_r;
		l[cr] = rows;
		icount -= (int32_t)R;
		done = 1;
	}
	if (!done)
		return;
	c->icount = icount;
	SR &= ~H_MASK;   // (MOV)
	fl_addi_flags(c, rows, imm_r, 0);
}

// ---------------------------------------------------------------- the execution loop


#define G 1
#define L 0
static void run(e132xs_t *c)
{
	if (!c->instruction_length_valid)
		SET_ILC(get_instruction_length(c, pr16(c, PC)));

	while (c->icount > 0) {
		// (one test for the usual case: no power down, no interrupt block, no line or timer interrupt pending; MAME's
		// --intblock / check_interrupts then do nothing)
		if (UNLIKELY(c->powerdown | (uint32_t)c->intblock | c->timer_int_pending | (ISR & 0x7f))) {
			if (c->powerdown) {
				c->icount = 0;
				break;
			}
			if (--c->intblock <= 0) {
				c->intblock = 0;
				check_interrupts(c, c->timer_int_pending != 0);
			}
		}

		const uint32_t op = pr16(c, PC);
		OP = op;
		PC += 2;
		c->instruction_length = 1 << ILC_SHIFT;

		switch (op >> 8) {
		case 0x00: op_chk(c, G, G); break;
		case 0x01: op_chk(c, G, L); break;
		case 0x02: op_chk(c, L, G); break;
		case 0x03: op_chk(c, L, L); break;
		case 0x04:
			op_movd(c, G, G);
			if ((OP & 0xf0) == 0 && (OP & 0x010e))
				goto ilc_done;   // RET: ILC and P not updated
			break;
		case 0x05:
			op_movd(c, G, L);
			if ((OP & 0xf0) == 0 && (OP & 0x010e))
				goto ilc_done;
			break;
		case 0x06: op_movd(c, L, G); break;
		case 0x07: op_movd(c, L, L); break;
		case 0x08: op_divsu(c, G, G, 0); break;
		case 0x09: op_divsu(c, G, L, 0); break;
		case 0x0a: op_divsu(c, L, G, 0); break;
		case 0x0b: op_divsu(c, L, L, 0); break;
		case 0x0c: op_divsu(c, G, G, 1); break;
		case 0x0d: op_divsu(c, G, L, 1); break;
		case 0x0e: op_divsu(c, L, G, 1); break;
		case 0x0f: op_divsu(c, L, L, 1); break;
		case 0x10: op_xm(c, G, G); break;
		case 0x11: op_xm(c, G, L); break;
		case 0x12: op_xm(c, L, G); break;
		case 0x13: op_xm(c, L, L); break;
		case 0x14: op_mask(c, G, G); break;
		case 0x15: op_mask(c, G, L); break;
		case 0x16: op_mask(c, L, G); break;
		case 0x17: op_mask(c, L, L); break;
		case 0x18: op_sum(c, G, G); break;
		case 0x19: op_sum(c, G, L); break;
		case 0x1a: op_sum(c, L, G); break;
		case 0x1b: op_sum(c, L, L); break;
		case 0x1c: op_sums(c, G, G); break;
		case 0x1d: op_sums(c, G, L); break;
		case 0x1e: op_sums(c, L, G); break;
		case 0x1f: op_sums(c, L, L); break;
		case 0x20: op_cmp(c, G, G); break;
		case 0x21: op_cmp(c, G, L); break;
		case 0x22: op_cmp(c, L, G); break;
		case 0x23: op_cmp(c, L, L); break;
		case 0x24: op_mov(c, G, G); break;
		case 0x25: op_mov(c, G, L); break;
		case 0x26: op_mov(c, L, G); break;
		case 0x27: op_mov(c, L, L); break;
		case 0x28: op_add(c, G, G); break;
		case 0x29: op_add(c, G, L); break;
		case 0x2a: op_add(c, L, G); break;
		case 0x2b: op_add(c, L, L); break;
		case 0x2c: op_adds(c, G, G); break;
		case 0x2d: op_adds(c, G, L); break;
		case 0x2e: op_adds(c, L, G); break;
		case 0x2f: op_adds(c, L, L); break;
		case 0x30: op_cmpb(c, G, G); break;
		case 0x31: op_cmpb(c, G, L); break;
		case 0x32: op_cmpb(c, L, G); break;
		case 0x33: op_cmpb(c, L, L); break;
		case 0x34: op_logic(c, G, G, 1); break;
		case 0x35: op_logic(c, G, L, 1); break;
		case 0x36: op_logic(c, L, G, 1); break;
		case 0x37: op_logic(c, L, L, 1); break;
		case 0x38: op_logic(c, G, G, 2); break;
		case 0x39: op_logic(c, G, L, 2); break;
		case 0x3a: op_logic(c, L, G, 2); break;
		case 0x3b: op_logic(c, L, L, 2); break;
		case 0x3c: op_logic(c, G, G, 3); break;
		case 0x3d: op_logic(c, G, L, 3); break;
		case 0x3e: op_logic(c, L, G, 3); break;
		case 0x3f: op_logic(c, L, L, 3); break;
		case 0x40: op_subc(c, G, G); break;
		case 0x41: op_subc(c, G, L); break;
		case 0x42: op_subc(c, L, G); break;
		case 0x43: op_subc(c, L, L); break;
		case 0x44: op_not(c, G, G); break;
		case 0x45: op_not(c, G, L); break;
		case 0x46: op_not(c, L, G); break;
		case 0x47: op_not(c, L, L); break;
		case 0x48: op_sub(c, G, G); break;
		case 0x49: op_sub(c, G, L); break;
		case 0x4a: op_sub(c, L, G); break;
		case 0x4b: op_sub(c, L, L); break;
		case 0x4c: op_subs(c, G, G); break;
		case 0x4d: op_subs(c, G, L); break;
		case 0x4e: op_subs(c, L, G); break;
		case 0x4f: op_subs(c, L, L); break;
		case 0x50: op_addc(c, G, G); break;
		case 0x51: op_addc(c, G, L); break;
		case 0x52: op_addc(c, L, G); break;
		case 0x53: op_addc(c, L, L); break;
		case 0x54: op_logic(c, G, G, 0); break;
		case 0x55: op_logic(c, G, L, 0); break;
		case 0x56: op_logic(c, L, G, 0); break;
		case 0x57: op_logic(c, L, L, 0); break;
		case 0x58: op_neg(c, G, G); break;
		case 0x59: op_neg(c, G, L); break;
		case 0x5a: op_neg(c, L, G); break;
		case 0x5b: op_neg(c, L, L); break;
		case 0x5c: op_negs(c, G, G); break;
		case 0x5d: op_negs(c, G, L); break;
		case 0x5e: op_negs(c, L, G); break;
		case 0x5f: op_negs(c, L, L); break;
		case 0x60: op_cmpi(c, G, 0); break;
		case 0x61: op_cmpi(c, G, 1); break;
		case 0x62: op_cmpi(c, L, 0); break;
		case 0x63: op_cmpi(c, L, 1); break;
		case 0x64: op_movi(c, G, 0); break;
		case 0x65: op_movi(c, G, 1); break;
		case 0x66: op_movi(c, L, 0); break;
		case 0x67: op_movi(c, L, 1); break;
		case 0x68: op_addi(c, G, 0); break;
		case 0x69: op_addi(c, G, 1); break;
		case 0x6a: op_addi(c, L, 0); break;
		case 0x6b: op_addi(c, L, 1); break;
		case 0x6c: op_addsi(c, G, 0); break;
		case 0x6d: op_addsi(c, G, 1); break;
		case 0x6e: op_addsi(c, L, 0); break;
		case 0x6f: op_addsi(c, L, 1); break;
		case 0x70: op_cmpbi(c, G, 0); break;
		case 0x71: op_cmpbi(c, G, 1); break;
		case 0x72: op_cmpbi(c, L, 0); break;
		case 0x73: op_cmpbi(c, L, 1); break;
		case 0x74: op_logici(c, G, 0, 1); break;
		case 0x75: op_logici(c, G, 1, 1); break;
		case 0x76: op_logici(c, L, 0, 1); break;
		case 0x77: op_logici(c, L, 1, 1); break;
		case 0x78: op_logici(c, G, 0, 2); break;
		case 0x79: op_logici(c, G, 1, 2); break;
		case 0x7a: op_logici(c, L, 0, 2); break;
		case 0x7b: op_logici(c, L, 1, 2); break;
		case 0x7c: op_logici(c, G, 0, 3); break;
		case 0x7d: op_logici(c, G, 1, 3); break;
		case 0x7e: op_logici(c, L, 0, 3); break;
		case 0x7f: op_logici(c, L, 1, 3); break;
		case 0x80: op_shrdi(c, 0); break;
		case 0x81: op_shrdi(c, 1); break;
		case 0x82: op_shrd(c); break;
		case 0x83: op_shr(c); break;
		case 0x84: op_sardi(c, 0); break;
		case 0x85: op_sardi(c, 1); break;
		case 0x86: op_sard(c); break;
		case 0x87: op_sar(c); break;
		case 0x88: op_shldi(c, 0); break;
		case 0x89: op_shldi(c, 1); break;
		case 0x8a: op_shld(c); break;
		case 0x8b: op_shl(c); break;
		case 0x8c: case 0x8d: break;   // reserved
		case 0x8e: op_testlz(c); break;
		case 0x8f: op_rol(c); break;
		case 0x90: op_ldxx1(c, G, G); break;
		case 0x91: op_ldxx1(c, G, L); break;
		case 0x92: op_ldxx1(c, L, G); break;
		case 0x93: op_ldxx1(c, L, L); break;
		case 0x94: op_ldxx2(c, G, G); break;
		case 0x95: op_ldxx2(c, G, L); break;
		case 0x96: op_ldxx2(c, L, G); break;
		case 0x97: op_ldxx2(c, L, L); break;
		case 0x98: op_stxx1(c, G, G); break;
		case 0x99: op_stxx1(c, G, L); break;
		case 0x9a: op_stxx1(c, L, G); break;
		case 0x9b: op_stxx1(c, L, L); break;
		case 0x9c: op_stxx2(c, G, G); break;
		case 0x9d: op_stxx2(c, G, L); break;
		case 0x9e: op_stxx2(c, L, G); break;
		case 0x9f: op_stxx2(c, L, L); break;
		case 0xa0: op_shri(c, 0, G); break;
		case 0xa1: op_shri(c, 1, G); break;
		case 0xa2: op_shri(c, 0, L); break;
		case 0xa3: op_shri(c, 1, L); break;
		case 0xa4: op_sari(c, 0, G); break;
		case 0xa5: op_sari(c, 1, G); break;
		case 0xa6: op_sari(c, 0, L); break;
		case 0xa7: op_sari(c, 1, L); break;
		case 0xa8: op_shli(c, 0, G); break;
		case 0xa9: op_shli(c, 1, G); break;
		case 0xaa: op_shli(c, 0, L); break;
		case 0xab: op_shli(c, 1, L); break;
		case 0xac: case 0xad: case 0xae: case 0xaf: break;   // reserved
		case 0xb0: op_mulsu(c, G, G, 0); break;
		case 0xb1: op_mulsu(c, G, L, 0); break;
		case 0xb2: op_mulsu(c, L, G, 0); break;
		case 0xb3: op_mulsu(c, L, L, 0); break;
		case 0xb4: op_mulsu(c, G, G, 1); break;
		case 0xb5: op_mulsu(c, G, L, 1); break;
		case 0xb6: op_mulsu(c, L, G, 1); break;
		case 0xb7: op_mulsu(c, L, L, 1); break;
		case 0xb8: op_set(c, 0, G); break;
		case 0xb9: op_set(c, 1, G); break;
		case 0xba: op_set(c, 0, L); break;
		case 0xbb: op_set(c, 1, L); break;
		case 0xbc: op_mul(c, G, G); break;
		case 0xbd: op_mul(c, G, L); break;
		case 0xbe: op_mul(c, L, G); break;
		case 0xbf: op_mul(c, L, L); break;
		case 0xc0: case 0xc1: case 0xc2: case 0xc3: case 0xc4: case 0xc5: case 0xc6: case 0xc7:
		case 0xc8: case 0xc9: case 0xca: case 0xcb: case 0xcc: case 0xcd:
			execute_software(c);   // fadd ... fcvtd
			break;
		case 0xce: op_extend(c); break;
		case 0xcf: break;   // DO (MAME: fatalerror)
		case 0xd0: op_ldwr(c, G); break;
		case 0xd1: op_ldwr(c, L); break;
		case 0xd2: op_lddr(c, G); break;
		case 0xd3: op_lddr(c, L); break;
		case 0xd4: op_ldwp(c, G); break;
		case 0xd5: op_ldwp(c, L); break;
		case 0xd6: op_lddp(c, G); break;
		case 0xd7: op_lddp(c, L); break;
		case 0xd8: op_stwr(c, G); break;
		case 0xd9: op_stwr(c, L); break;
		case 0xda: op_stdr(c, G); break;
		case 0xdb: op_stdr(c, L); break;
		case 0xdc: op_stwp(c, G); break;
		case 0xdd: op_stwp(c, L); break;
		case 0xde: op_stdp(c, G); break;
		case 0xdf: op_stdp(c, L); break;
		case 0xe0: op_db(c, 0, 1); break;
		case 0xe1: op_db(c, 0, 0); break;
		case 0xe2: op_db(c, 1, 1); break;
		case 0xe3:
			op_db(c, 1, 0);
			if (c->delay_slot && !(OP & 0x80) && !(SR & T_MASK)) {
				if (PC - 2 - c->delay_pc <= 6)
					loop_fast(c);   // (a copy loop: LD / ADDI / DBNE / ST)
				else if (!(c->powerdown | c->timer_int_pending | (ISR & 0x7f)))
					loop_rows(c);   // (a rectangle: MOV / that loop / ADD / ADDI / DBNE / ADD)
			}
			break;
		case 0xe4: op_db(c, 2, 1); break;
		case 0xe5: op_db(c, 2, 0); break;
		case 0xe6: op_db(c, 3, 1); break;
		case 0xe7: op_db(c, 3, 0); break;
		case 0xe8: op_db(c, 4, 1); break;
		case 0xe9: op_db(c, 4, 0); break;
		case 0xea: op_db(c, 5, 1); break;
		case 0xeb: op_db(c, 5, 0); break;
		case 0xec: {
			const uint32_t a = PC - 2;
			const int in_slot = c->delay_slot;
			op_dbr(c);
			if (!in_slot && !(OP & 0x80) && !(SR & T_MASK) && !(c->powerdown | c->timer_int_pending | (ISR & 0x7f)))
				loop_fast3(c, a);   // (a double-word copy: CMPI / BN / LDD.P / ADDI / DBR / STD.P)
			break;
		}
		case 0xed: op_frame(c); break;
		case 0xee: op_call(c, G); break;
		case 0xef: op_call(c, L); break;
		case 0xf0: op_b(c, 0, 1); break;
		case 0xf1: op_b(c, 0, 0); break;
		case 0xf2: op_b(c, 1, 1); break;
		case 0xf3: {
			const uint32_t a = PC - 2;
			op_b(c, 1, 0);
			if (PC < a && !(OP & 0x80) && !(SR & T_MASK) &&
			    !(c->powerdown | (uint32_t)c->intblock | c->timer_int_pending | (ISR & 0x7f)))
				loop_fast2(c, a);   // (a copy loop: LD / ST / ADDI / BNE back)
			break;
		}
		case 0xf4: op_b(c, 2, 1); break;
		case 0xf5: op_b(c, 2, 0); break;
		case 0xf6: op_b(c, 3, 1); break;
		case 0xf7: op_b(c, 3, 0); break;
		case 0xf8: op_b(c, 4, 1); break;
		case 0xf9: op_b(c, 4, 0); break;
		case 0xfa: op_b(c, 5, 1); break;
		case 0xfb: op_b(c, 5, 0); break;
		case 0xfc: op_br(c); break;
		case 0xfd: case 0xfe: case 0xff: op_trap(c); break;
		}

		// anything other than RET updates ILC and sets P
		SR = (SR & 0xffe7ffffu) | (uint32_t)c->instruction_length | P_MASK;
	ilc_done:
		if (UNLIKELY(SR & T_MASK) && GET_P && !c->delay_slot) {   // Not in a Delayed Branch instructions
			c->delay_slot_taken = 0;
			execute_exception(c, TRAPNO_TRACE_EXCEPTION);
		}
	}
}
#undef G
#undef L

int e132xs_execute(e132xs_t *c, int cycles)
{
	const int64_t start = c->clock;
	c->target += cycles;
	c->abort = 0;
	while (c->clock < c->target && !c->abort) {
		if (c->timer_at <= c->clock) {
			timer_fire(c);
			continue;
		}
		int64_t end = c->target;
		if (c->timer_at < end)
			end = c->timer_at;
		c->seg_end = end;
		c->icount = (int32_t)(end - c->clock);
		c->running = 1;
		run(c);
		c->running = 0;
		c->clock = c->seg_end - c->icount;
		if (c->timer_at <= c->clock)
			timer_fire(c);
	}
	return (int)(c->clock - start);
}

void e132xs_eat(e132xs_t *c, int cycles)
{
	c->target += cycles;
	while (c->clock < c->target) {
		if (c->timer_at <= c->target) {
			if (c->timer_at > c->clock)
				c->clock = c->timer_at;
			timer_fire(c);
		} else {
			c->clock = c->target;
		}
	}
}

void e132xs_abort(e132xs_t *c)
{
	c->abort = 1;
	if (c->running) {
		c->seg_end -= c->icount;   // (the segment ends now: the clock stays where it is)
		c->icount = 0;
	}
}

void e132xs_burn(e132xs_t *c, int cycles)
{
	if (!c->running)
		return;
	if (cycles > c->icount)
		cycles = c->icount;
	if (cycles > 0)
		c->icount -= cycles;
}

int e132xs_skip_rounds(e132xs_t *c, int round)
{
	if (!c->running || round <= 0)
		return 0;
	// (this round's `round` cycles are still to be counted; the rounds after it that end with cycles left in the
	// segment are taken, the interpreter runs the rest as it would)
	const int32_t n = (c->icount - round - 1) / round;
	if (n <= 0)
		return 0;
	c->icount -= n * round;
	return n * round;
}

// ---------------------------------------------------------------- init / reset / state

int e132xs_init(e132xs_t *c, int type, int bus16)
{
	memset(c, 0, sizeof(*c));
	c->rd = (e132xs_page *)calloc(E132XS_PAGES, sizeof(e132xs_page));
	c->wr = (e132xs_page *)calloc(E132XS_PAGES, sizeof(e132xs_page));
	if (!c->rd || !c->wr) {
		e132xs_exit(c);
		return -1;
	}
	c->type = type;
	c->bus16 = bus16;
	c->clock_scale_mask = type == E132XS_E1XS ? 7 : type == E132XS_E1X ? 3 : 0;
	c->power_down_req = 1;
	c->timer_at = E132XS_NEVER;
	c->fetch_page = ~0u;
	c->tr_clocks_per_tick = 2;
	return 0;
}

void e132xs_exit(e132xs_t *c)
{
	free(c->rd);
	free(c->wr);
	free(c->wq);
	c->rd = c->wr = c->wq = NULL;
}

void e132xs_reset(e132xs_t *c)
{
	// TODO: Add different reset initializations for BCR, MCR, FCR, TPR
	c->tr_clocks_per_tick = 2;
	c->trap_entry = s_trap_entries[7];   // default entry point @ MEM3
	c->g[BCR_REGISTER] = ~0u;
	c->g[MCR_REGISTER] = ~0u;
	update_memory_control(c);
	set_global_register(c, FCR_REGISTER, ~0u);
	set_global_register(c, TPR_REGISTER, 0xc000000);
	PC = get_trap_addr(c, TRAPNO_RESET);
	SET_FP(0);
	SET_FL(2);
	SET_M(0);
	SET_T(0);
	SET_L(1);
	SET_S(1);
	SET_ILC(1 << ILC_SHIFT);
	c->l[(0 + GET_FP) & 0x3f] = (PC & 0xfffffffeu) | GET_S;
	c->l[(1 + GET_FP) & 0x3f] = SR;
	c->clock += c->cc2;   // (MAME: m_core->icount -= clock_cycles_2)
	c->fetch_page = ~0u;
}

void e132xs_post_load(e132xs_t *c)
{
	c->fetch_page = ~0u;
	c->running = 0;
	c->abort = 0;
	if (c->tr_clocks_per_tick < 2)
		c->tr_clocks_per_tick = 2;
}

/*  Mitsubishi M37702 (MELPS 7700: M37702M2 / M37702S1) CPU, ported to C from MAME 0.289
    src/devices/cpu/m37710: m37710.cpp, m37710.h, m37710cm.h, m37710il.h, m37710op.h
    (BSD-3-Clause, copyright-holders: R. Belmont, Karl Stenerud).
    The 7700 core (65C816-like: A / B accumulators, M / X flags, 24-bit address space, 3-bit interrupt
    priority level) with MAME's opcode tables (four M/X modes, the 0x42 B-accumulator and 0x89 mul/div
    prefix tables) and MAME's cycle counts. The on-chip peripherals are what MAME's m37702m2_device /
    m37702s1_device map at 0x00-0x7f: ports P0-P8 with direction registers, the A/D converter, the
    UART registers (latched only, as in MAME), timers A0-A4 / B0-B2 (timer mode raises the interrupt
    each period; the other modes are latched only, as in MAME), the interrupt control registers, the
    processor mode / watchdog registers; internal RAM 0x80-0x27f, internal mask ROM 0xc000-0xffff (M2).
    Clock: MAME divides the input clock by 2 (execute_clocks_to_cycles); m37710_execute counts CPU cycles,
    the timer periods are converted from input clocks to CPU cycles (all MAME periods are even).
    Differences from MAME: the emu_timers are cycle counters checked at instruction boundaries (MAME ends the
    timeslice at the expiry: the same boundary; m37710_set_clock reproduces its attosecond rounding);
    external interrupt lines are evaluated at the start of m37710_execute (MAME: the next execute_run), so
    the driver's slicing stands in for MAME's scheduler. 16-bit bus accesses reach the callbacks as two byte
    accesses. Only the M37702 map (P0-P8, RAM 0x80-0x27f) is modeled, not the M37710S4 / M37720S1 / M3773x
    variants (other port maps, D/A, DMA).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M37710_H
#define RBX_M37710_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Input lines (MAME m37710.h; the same order as the vector table)
enum {
	// maskable interrupts
	M37710_LINE_DMA3 = 0, M37710_LINE_DMA2, M37710_LINE_DMA1, M37710_LINE_DMA0,
	M37710_LINE_ADC,
	M37710_LINE_UART1XMIT, M37710_LINE_UART1RECV, M37710_LINE_UART0XMIT, M37710_LINE_UART0RECV,
	M37710_LINE_TIMERB2, M37710_LINE_TIMERB1, M37710_LINE_TIMERB0,
	M37710_LINE_TIMERA4, M37710_LINE_TIMERA3, M37710_LINE_TIMERA2, M37710_LINE_TIMERA1, M37710_LINE_TIMERA0,
	M37710_LINE_IRQ2, M37710_LINE_IRQ1, M37710_LINE_IRQ0,
	// non-maskable
	M37710_LINE_WATCHDOG, M37710_LINE_DEBUG, M37710_LINE_BRK, M37710_LINE_ZERODIV, M37710_LINE_RESET,
	// signals external hardware can send (timer event inputs, timer out pins for the up/down select)
	M37710_LINE_TIMERA0IN, M37710_LINE_TIMERA1IN, M37710_LINE_TIMERA2IN, M37710_LINE_TIMERA3IN,
	M37710_LINE_TIMERA4IN, M37710_LINE_TIMERB0IN, M37710_LINE_TIMERB1IN, M37710_LINE_TIMERB2IN,
	M37710_LINE_TIMERA0OUT, M37710_LINE_TIMERA1OUT, M37710_LINE_TIMERA2OUT, M37710_LINE_TIMERA3OUT,
	M37710_LINE_TIMERA4OUT, M37710_LINE_TIMERB0OUT, M37710_LINE_TIMERB1OUT, M37710_LINE_TIMERB2OUT,
	M37710_LINE_MAX
};
#define M37710_INTERRUPT_MAX (M37710_LINE_RESET + 1)
#define M37710_MASKABLE_INTERRUPTS (M37710_INTERRUPT_MAX - 5)

typedef struct m37710_s m37710_t;
typedef void (*m37710_opfn)(m37710_t *cpu);

struct m37710_s {
	// ---- state (saved: from a up to opcodes) ----
	uint32_t a, b, ba, bb;      // A (b: its high byte while M=1), B (bb: its high byte while M=1)
	uint32_t x, y, xh, yh;      // X, Y (xh / yh: their high bytes while X=1)
	uint32_t s, pc, ppc, pg, dt, dpr;   // pg / dt shifted left 16
	uint32_t flag_e, flag_m, flag_x, flag_n, flag_v, flag_d, flag_i, flag_z, flag_c;
	uint32_t line_irq, ipl, ir, im, im2, im3, im4, irq_delay;
	uint32_t source, destination, stopped;
	int32_t icount;             // cycles left in the current m37710_execute (<= 0 after: the overshoot, carried)
	int32_t insn_icount;        // icount at the last accounting point (start of the instruction in progress)
	int64_t cycles;             // CPU cycles accounted since init (the time base of the timers)
	// peripherals
	uint8_t port_regs[11], port_dir[11];
	uint8_t ad_control, ad_sweep;
	uint16_t ad_result[8];
	uint8_t uart_mode[2], uart_baud[2], uart_ctrl_reg0[2], uart_ctrl_reg1[2];
	uint8_t count_start, one_shot_start, up_down_reg;
	uint8_t timer_mode[8];
	uint16_t timer_reg[8];
	int32_t timer_out[8];
	uint8_t proc_mode, watchdog_freq, rto_control, dram_control;
	uint8_t int_control[M37710_MASKABLE_INTERRUPTS];
	uint8_t timer_on[8], ad_on;  // MAME emu_timer armed
	int64_t timer_due[8];        // cycle of the next expiry (MAME m_timers[i])
	int64_t timer_reload[8];     // period in CPU cycles (MAME m_reload)
	int64_t ad_due;              // (MAME m_ad_timer)
	int64_t next_due;            // earliest armed timer (INT64_MAX: none)
	uint8_t ram[0x200];          // internal RAM 0x80-0x27f
	// ---- configuration (not saved) ----
	const m37710_opfn *opcodes, *opcodes42, *opcodes89;
	// internal mask ROM at 0xc000-0xffff (M37702M2, 16 KB, little-endian byte image); NULL: M37702S1 (the
	// external bus there)
	const uint8_t *rom;
	uint8_t early;               // m37710_set_clock: internal events one cycle early (MAME rounding); default 0
	// the wait loop LDA d ; BEQ *-2 (8-bit A, d in the internal RAM) skipped by whole turns up to the slice's end or
	// the next internal timer event: exact (the turns skipped would read the same 0; the interrupts are taken only
	// at the slice's start and at those events). Set by the board; default 0
	uint8_t idle_skip;
	uint8_t idle_cnt;            // (instructions since the loop's top; its address, the cycles there)
	uint32_t idle_pc;
	int64_t idle_c0;
	// external bus, 4 KB pages: read / written directly when the pointer is set (byte at page + (a & 0xfff)),
	// else through the callbacks. The internal RAM / SFRs / ROM come first. A page with rpage set and wpage
	// NULL sends its writes to the write callback (drop them for ROM). 16-bit accesses are two byte accesses,
	// low byte first (MAME read_word_unaligned on the 16-bit bus; the handlers here see bytes).
	const uint8_t *rpage[4096];
	uint8_t *wpage[4096];
	uint8_t (*read)(void *ctx, uint32_t a);             // NULL: 0 (MAME unmapped)
	void (*write)(void *ctx, uint32_t a, uint8_t v);    // NULL: dropped
	// ports P0..P10 (MAME p<N>_in_cb / p<N>_out_cb). Read: the input bits (direction 0); NULL: 0xff (MAME
	// devcb default). Write: called when a direction bit is set, data already masked by it (MAME mem_mask)
	uint8_t (*port_read)(void *ctx, int port);
	void (*port_write)(void *ctx, int port, uint8_t data, uint8_t mask);
	uint16_t (*analog_read)(void *ctx, int ch);         // AN0..AN7 (MAME an<N>_cb); NULL: 0
	// optional: a 16-bit access at an even address of the external bus not on a direct page as one access (MAME's
	// 16-bit bus: a device that sees the access width, the C352); NULL: two byte accesses
	uint16_t (*read16)(void *ctx, uint32_t a);
	void (*write16)(void *ctx, uint32_t a, uint16_t v);
	void *ctx;
};

// clear everything, configuration included (MAME device_start); then set the bus / ROM / clock and call
// m37710_reset
void m37710_init(m37710_t *c);
// MAME device_reset: peripherals to their reset values, I set, IPL 0, M/X clear, PC from 0xfffe (read through
// the bus: set rom / pages first)
void m37710_reset(m37710_t *c);
// the input clock (Hz, before MAME's / 2): only to reproduce MAME's attosecond rounding of the internal timer
// periods (they can fire one cycle early); the CPU counts cycles regardless
void m37710_set_clock(m37710_t *c, uint32_t hz);
// MAME execute_set_input: ADC / IRQ0..2 (any state but 0 = ASSERT / HOLD: the line is cleared when taken),
// TIMERxIN (event count on a rising call), TIMERxOUT (the pin level). Taken at the next update point
// (m37710_execute start, CLI / RTI / PLP / REP / PUL, an interrupt control write, an internal timer).
void m37710_set_input(m37710_t *c, int line, int state);
// run for `cycles` CPU cycles (the overshoot of the last instruction is taken from the next call); returns the
// cycles executed in this call. A WAI / STP CPU idles (the internal timers still run and can wake WAI).
int m37710_execute(m37710_t *c, int cycles);
// end the current m37710_execute after the instruction in progress (from a bus callback)
static inline void m37710_abort_timeslice(m37710_t *c) { if (c->icount > 0) c->icount = 0; }
// program counter with bank (MAME GENPC: PG | PC)
static inline uint32_t m37710_pc(const m37710_t *c) { return (c->pg | c->pc) & 0xffffff; }
// processor status (low byte) as MAME m37710i_get_reg_ps
uint32_t m37710_get_ps(const m37710_t *c);

// save state: the state block (from a up to opcodes) as one blob; call m37710_post_load after restoring it
#define M37710_STATE_SIZE ((uint32_t)(offsetof(m37710_t, opcodes)))
void m37710_post_load(m37710_t *c);

#ifdef __cplusplus
}
#endif
#endif

/*  Hyperstone E1-32 / E1-16 CPU family (E1-32N/T, E1-32X(N/T), E1-16(X), GMS30C2116/32 ...), ported to C from MAME 0.289
    src/devices/cpu/e132xs: e132xs.cpp, e132xs.h, e132xsop.hxx, e1defs.h (BSD-3-Clause, copyright-holders: Pierpaolo
    Prazzoli). The interpreter (MAME's non-DRC path) with MAME's cycle counts, delay slots (check_delay_pc), traps,
    exceptions, the software (floating point) emulation entries, the EXTEND DSP instructions, the register stack (FRAME /
    RET spilling to and filling from memory), the interrupt priorities (INT1-4, IO1-3, the timer) and the on-chip timer
    (TR / TCR / TPR with the prescaler; MAME's emu_timer becomes a cycle stamp on the CPU's own clock).
    Memory: a page table of 64 KB pages to host memory (host-order 32-bit words: the guest byte at a is at
    base[(a & mask) ^ 3]), each page with a mirror mask; pages without memory go to the driver's callbacks.
    Speed: one test per instruction for the rare cases (power down, interrupt block, a pending interrupt, trace), and
    the copy loops (LD / ADDI / DBNE back / ST in the delay slot: the Eolith games' blits) run an iteration at a time
    with the same effects, cycles and interrupt checks (e132xs.c loop_fast; the harness hashes are the same without it),
    whole rounds at once while both pointers stay in a host page; so do the other blit shapes of the Eolith library
    (loop_fast2: LD / ST / ADDI / BNE, loop_fill: ST / ADDI / BNE, loop_fast3: the double-word copy with CMPI / BN /
    DBR, loop_rows: rectangles of loop_fast's loop), and a driver can take a wait loop's rounds (e132xs_skip_rounds).
    Not ported (as in MAME's interpreter, or not needed): the DRC, the disassembler, the bus / memory control registers'
    timing (latched only), the E1-XS SDRAM mode / control handlers, sleep mode, the watchdog.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_E132XS_H
#define RBX_E132XS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// input lines (MAME hyperstone_device::INPUT_*)
enum { E132XS_INT1 = 0, E132XS_INT2, E132XS_INT3, E132XS_INT4, E132XS_IO1, E132XS_IO2, E132XS_IO3 };
// line states: MAME CLEAR_LINE / ASSERT_LINE / HOLD_LINE (a held line is cleared when the CPU takes it)
enum { E132XS_CLEAR = 0, E132XS_ASSERT = 1, E132XS_HOLD = 2 };

// the core families: E1 (E1-16/32, GMS30C2116/32: no PLL), E1-X (E1-16X/32X, GMS30C2216/32: x4 PLL, internal I/O),
// E1-XS (x8 PLL). The clock given to the driver's frame is the core clock (MAME: 4 * XTAL for an X part)
enum { E132XS_E1 = 0, E132XS_E1X = 1, E132XS_E1XS = 2 };

#define E132XS_PAGE_SHIFT 16
#define E132XS_PAGES (1u << (32 - E132XS_PAGE_SHIFT))
#define E132XS_NEVER INT64_MAX

typedef struct { uint8_t *base; uint32_t mask; } e132xs_page;

typedef struct e132xs_s e132xs_t;
struct e132xs_s {
	/* ---- saved state (POD, from g up to `running`) ---- */
	uint32_t g[32];                 // global registers (PC, SR, FER, G3-G15, SP, UB, BCR, TPR, TCR, TR, WCR, ISR, FCR, MCR)
	uint32_t l[64];                 // the local register stack
	uint32_t trap_entry;
	uint32_t clck_scale, clock_scale_mask;
	uint32_t cc1, cc2, cc3, cc4, cc6, cc36;   // MAME clock_cycles_n
	uint32_t tr_base_value, tr_result, tr_clocks_per_tick, timer_int_pending;
	uint32_t delay_pc, delay_slot, delay_slot_taken;
	int32_t intblock;
	uint32_t powerdown, power_down_req;
	int32_t instruction_length;
	uint32_t instruction_length_valid;
	uint32_t op;
	uint32_t hold;                  // lines asserted with E132XS_HOLD
	uint32_t timer_param;           // MAME's timer callback parameter (1: a TPR change pending)
	int32_t icount;                 // MAME m_core->icount (valid inside e132xs_execute)
	int64_t tr_base_cycles;
	int64_t timer_at;               // MAME m_timer expiry (E132XS_NEVER = off)
	int64_t clock;                  // the CPU's clock outside a run segment
	int64_t target;                 // the clock e132xs_execute runs up to (an overshoot is taken off the next call)

	/* ---- not saved (run state, configuration, bus) ---- */
	int running;
	int abort;
	int64_t seg_end;                // the clock at which the running segment ends (now = seg_end - icount)
	int type;                       // E132XS_E1 / E1X / E1XS
	int bus16;                      // 16-bit data bus (E1-16 parts): only the I/O width differs here
	// the instruction fetch page (pc >> E132XS_PAGE_SHIFT whose host page is fetch_base, ~0 = none)
	uint32_t fetch_page, fetch_mask;
	const uint8_t *fetch_base;
	// accesses that miss the page table (address aligned: word accesses & ~1, long & ~3)
	uint8_t (*read8)(void *user, uint32_t addr);
	uint16_t (*read16)(void *user, uint32_t addr);
	uint32_t (*read32)(void *user, uint32_t addr);
	void (*write8)(void *user, uint32_t addr, uint8_t data);
	void (*write16)(void *user, uint32_t addr, uint16_t data);
	void (*write32)(void *user, uint32_t addr, uint32_t data);
	// the I/O space (LDW.IOD / STW.IOD ...): MAME's I/O offset = addr >> 13 (the E1-X internal I/O at bit 27 is handled
	// here); NULL = reads 0, writes dropped
	uint32_t (*io_read)(void *user, uint32_t offset);
	void (*io_write)(void *user, uint32_t offset, uint32_t data);
	// MAME standard_irq_callback (an external line taken, after a held line has been cleared); NULL = none
	void (*irq_ack)(void *user, int line);
	void *user;
	// page tables (calloc'd by e132xs_init): host memory of each page, NULL = callbacks
	e132xs_page *rd, *wr;
	e132xs_page *wq;               // e132xs_map_skip15 pages (calloc'd on its first call; NULL = none)
};

#define E132XS_STATE_SIZE ((uint32_t)offsetof(e132xs_t, running))

// clear everything and allocate the page tables (0 = ok). Then set the callbacks, map memory and e132xs_reset.
int e132xs_init(e132xs_t *c, int type, int bus16);
void e132xs_exit(e132xs_t *c);   // free the page tables
// MAME device_reset (registers kept as in MAME: only the control registers, PC, SR and the frame are set)
void e132xs_reset(e132xs_t *c);
// map [start, end] (64 KB aligned) to host memory (layout above). size = the size of the block at mem (a power of
// two), repeated over the range (mirrors); a block smaller than a page is mirrored inside each page. readable /
// writable choose the directions served from it; mem == NULL unmaps (callbacks)
void e132xs_map(e132xs_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size, int readable, int writable);
// map [start, end] for writes as a frame buffer whose 16-bit writes with bit 15 set are ignored (MAME eolith_vram_w):
// halfword and word stores go to mem (layout as above, a word as its two halves), each half only with bit 15 clear;
// byte stores go to the write8 callback. e132xs_map with writable or mem == NULL undoes it. 0 = ok
int e132xs_map_skip15(e132xs_t *c, uint32_t start, uint32_t end, void *mem, uint32_t size);
// run until the CPU clock reaches target + cycles; returns the cycles run in this call
int e132xs_execute(e132xs_t *c, int cycles);
// cycles pass without instructions (MAME: a suspended CPU, spin_until_trigger); the on-chip timer runs
void e132xs_eat(e132xs_t *c, int cycles);
// stop e132xs_execute after the current instruction (MAME abort_timeslice); the cycles not run stay owed
void e132xs_abort(e132xs_t *c);
// MAME eat_cycles from a handler: the running segment's cycles taken (at most what is left of it)
void e132xs_burn(e132xs_t *c, int cycles);
// a driver's wait-loop skip, called from the handler of the loop's one read while none of the round's `round` cycles
// has been counted yet (a load counts its cycles after its read): the following rounds that end before the running
// segment does are taken at once (cycles only: the driver makes sure they would change nothing else). Returns the
// cycles taken
int e132xs_skip_rounds(e132xs_t *c, int round);
// MAME execute_set_input. state E132XS_CLEAR / ASSERT / HOLD
void e132xs_set_input(e132xs_t *c, int line, int state);
// MAME total_cycles (valid inside handlers too)
int64_t e132xs_total_cycles(const e132xs_t *c);
// MAME pc(): the PC register (inside a handler: past the current instruction's words read so far)
static inline uint32_t e132xs_pc(const e132xs_t *c) { return c->g[0]; }
// after loading E132XS_STATE_SIZE bytes of state
void e132xs_post_load(e132xs_t *c);
// swap a big-endian byte image into host-order 32-bit words in place (size multiple of 4)
void e132xs_swap32(void *p, uint32_t size);

// bus accesses as the CPU makes them (page table, then callbacks)
uint8_t e132xs_read8(e132xs_t *c, uint32_t a);
uint16_t e132xs_read16(e132xs_t *c, uint32_t a);
uint32_t e132xs_read32(e132xs_t *c, uint32_t a);
void e132xs_write8(e132xs_t *c, uint32_t a, uint8_t v);
void e132xs_write16(e132xs_t *c, uint32_t a, uint16_t v);
void e132xs_write32(e132xs_t *c, uint32_t a, uint32_t v);

#ifdef __cplusplus
}
#endif

#endif

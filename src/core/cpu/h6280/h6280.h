/*  Hudson HuC6280 CPU (65C02 core + MMU, block transfers, timer, interrupt controller), ported to C from
    MAME 0.289 src/devices/cpu/h6280 (h6280.cpp, h6280.h; BSD-3-Clause, copyright-holders: Bryan McPhail;
    based on the 6502 emulator by Juergen Buchmueller).
    Instruction semantics, the T flag, the delayed interrupt taking (m_irq_pending), the timer, the CSL / CSH
    clock multiplier, the VDC / VCE access penalty and the cycle counts follow h6280_device one to one.
    The internal PSG (C6280) is not part of this file: writes to $1FE800-$1FEBFF go to psg_write, reads there
    return the I/O buffer, as in MAME's internal map. The disassembler and the debugger state are not ported.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_H6280_H
#define RBX_H6280_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { H6280_IRQ1 = 0, H6280_IRQ2 = 1, H6280_IRQT = 2 };

#define H6280_PAGE_SHIFT 13
#define H6280_PAGES      256      // 8 KB pages of the 21-bit physical space

typedef struct h6280_s h6280_t;
struct h6280_s {
	// ---- state (saved) ----
	uint16_t pc, ppc;          // ppc = address of the last instruction started
	uint16_t ea;               // effective address (MAME m_ea; low 16 bits are all that matter)
	uint8_t a, x, y, p, s;     // s: stack pointer low byte (the stack is $2100-$21FF through MPR1)
	uint8_t mmr[8];            // MPR0-7
	uint8_t irq_mask;          // $1402: bit 0 IRQ2, bit 1 IRQ1, bit 2 timer (1 = disabled)
	uint8_t timer_status;      // timer running
	uint8_t timer_ack;
	uint8_t clocks_per_cycle;  // 4 = low speed (CSL, reset), 1 = high speed (CSH)
	uint8_t nmi_state;
	uint8_t irq_state[3];      // IRQ1, IRQ2, timer
	uint8_t irq_pending;       // MAME m_irq_pending (2 = take after the next instruction, 1 = after this one)
	uint8_t io_buffer;         // last value written to the PSG / timer / port / interrupt pages
	uint8_t pad_[2];
	int32_t timer_value;       // in input clocks
	int32_t timer_load;
	int64_t cycles;            // input clocks executed since init; updated inside each instruction as MAME's
	                           // m_icount is, so a memory handler sees the same time MAME's would
	// ---- configuration / bus (not saved) ----
	int timer_scale;           // MAME set_timer_scale (default 1; kept by reset)
	// Physical 8 KB pages read / written directly when set, else through read / write.
	// Page 255 ($1FE000-$1FFFFF) is the I/O page: keep rpage[255] / wpage[255] NULL (h6280_reset clears them);
	// $1FE800-$1FEBFF PSG, $1FEC00-$1FEFFF timer, $1FF000-$1FF3FF port, $1FF400-$1FF7FF interrupts;
	// the rest of it ($1FE000-$1FE7FF VDC / VCE, $1FF800-$1FFFFF) goes to read / write.
	const uint8_t *rpage[H6280_PAGES];
	uint8_t *wpage[H6280_PAGES];
	uint8_t (*read)(void *ctx, uint32_t pa);              // NULL = reads 0
	void (*write)(void *ctx, uint32_t pa, uint8_t v);      // NULL = ignored
	void (*psg_write)(void *ctx, int offset, uint8_t v);   // offset 0-15 (C6280 c6280_w)
	uint8_t (*port_in)(void *ctx);                         // K port; NULL = reads the I/O buffer
	void (*port_out)(void *ctx, uint8_t v);                // O port
	void (*st_write)(void *ctx, int port, uint8_t v);      // ST0 / ST1 / ST2: I/O space 0, 2, 3
	// called when IRQ1 / IRQ2 is taken (MAME standard_irq_callback), line = H6280_IRQ1 / H6280_IRQ2
	void (*irq_ack)(void *ctx, int line);
	void *ctx;
};

// clear everything (lines clear, timer_scale 1, no bus) - call once, set the bus, then h6280_reset
void h6280_init(h6280_t *c);
// MAME device_reset: MPRs 0, P = I|B, S = $FF, PC from $FFFE (through MPR7 = 0), low speed, timer off.
// The interrupt input lines are kept, as in MAME
void h6280_reset(h6280_t *c);
// one instruction, then the interrupt and timer checks of MAME's execute_run loop; returns the clocks used
int h6280_step(h6280_t *c);
// MAME execute_run: run until c->cycles >= target, or until h6280_break is set (checked after each instruction)
void h6280_run_until(h6280_t *c, int64_t target);
// line H6280_IRQ1 / H6280_IRQ2 / H6280_IRQT, state 0 / 1
void h6280_set_irq(h6280_t *c, int line, int state);
// NMI is edge triggered: only state 1 has an effect
void h6280_set_nmi(h6280_t *c, int state);
// logical -> physical (MAME translated)
static inline uint32_t h6280_translate(const h6280_t *c, uint16_t a)
{
	return ((uint32_t)c->mmr[a >> 13] << 13) | (a & 0x1fffu);
}

// ReBOX: set (by a memory handler) to end h6280_run_until after the current instruction (the caller clears it)
extern int h6280_break;

#define H6280_STATE_SIZE ((uint32_t)(offsetof(h6280_t, timer_scale)))

#ifdef __cplusplus
}
#endif

#endif

/*  Hitachi HD6309 / HD6309E CPU, ported to C from MAME src/devices/cpu/m6809 (hd6309.cpp, hd6309.h, hd6309.lst,
    base6x09.lst, m6809.cpp, m6809.h, m6809inl.h; BSD-3-Clause, copyright-holders: Nathan Woods, Tim Lindner;
    "Portable 6809 emulator", John Butler).
    Same structure as cpu/m6809: instruction semantics, flag rules and cycle counts follow MAME (every bus access and
    every dummy cycle costs one clock), including the native-mode (MD bit 0) timings, FIRQ-saves-all (MD bit 1),
    the illegal-instruction / division-by-zero traps (vector FFF0, MD bits 6/7) and the abortable TFM block move
    (one byte per step here, so interrupt lines raised between steps abort and restart it, as in MAME).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_HD6309_H
#define RBX_HD6309_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct hd6309_s hd6309_t;
struct hd6309_s {
	// registers (D = A:B, W = E:F, Q = D:W)
	uint16_t pc, x, y, u, s, v;
	uint8_t a, b, e, f, dp, cc, md;
	// interrupt lines / state
	uint8_t irq_line, firq_line, nmi_line, nmi_asserted, lds_encountered;
	uint8_t wait;              // 0 = running, 1 = CWAI (state pushed), 2 = SYNC
	uint8_t opcode;            // MAME m_opcode: last opcode byte, or the indexed postbyte
	uint8_t tfm;               // 1 = inside a TFM transfer (tfm_op / tfm_pb hold the instruction)
	uint8_t tfm_op, tfm_pb;
	uint8_t irq_hold;          // (hd6309_hold_irq: the IRQ line dropped when taken, MAME's HOLD_LINE)
	uint8_t firq_hold;         // (hd6309_hold_firq: the same for FIRQ; in the alignment padding as irq_hold)
	int64_t cycles;            // clocks executed since reset
	// bus: 256-byte pages read / written directly when the pointer is set, else through the callbacks
	const uint8_t *rpage[256];
	uint8_t *wpage[256];
	uint8_t (*read)(void *ctx, uint16_t a);
	void (*write)(void *ctx, uint16_t a, uint8_t v);
	void *ctx;
};

// clear registers and the bus tables (call once, then set the bus and hd6309_reset)
void hd6309_init(hd6309_t *c);
void hd6309_reset(hd6309_t *c);
// one instruction (or interrupt entry, or one TFM byte); returns the clocks used
int hd6309_step(hd6309_t *c);
// run until c->cycles >= target (a waiting CPU jumps to the target)
void hd6309_run_until(hd6309_t *c, int64_t target);
void hd6309_set_irq(hd6309_t *c, int state);
void hd6309_hold_irq(hd6309_t *c);   // (set_input_line(IRQ, HOLD_LINE))
void hd6309_set_firq(hd6309_t *c, int state);
void hd6309_hold_firq(hd6309_t *c);   // (set_input_line(FIRQ, HOLD_LINE))
void hd6309_set_nmi(hd6309_t *c, int state);

// save state: the register block (from pc up to cycles) as one blob
#define HD6309_STATE_SIZE ((uint32_t)(offsetof(hd6309_t, rpage)))

#ifdef __cplusplus
}
#endif

#endif

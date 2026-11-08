// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*  Motorola MC6801 / MC6801U4 (6800 family) microcontroller, ported to C99 from MAME
    src/devices/cpu/m6800/m6800.cpp, 6800ops.hxx and m6801.cpp (BSD-3-Clause, copyright-holders: Aaron Giles).
    On-chip: ports 1-4, programmable timer (FRC/OCR/ICR, plus OCR2/OCR3 on the U4), RAM, port 3 handshake (IS3/OS3).
    The SCI keeps its registers but does not shift data. Also runs a plain MC6800 (M6801_TYPE_6800).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M6801_H
#define RBX_M6801_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// host is little-endian (PC, Vita, PSP)
typedef union {
	struct { uint8_t l, h, h2, h3; } b;
	struct { uint16_t l, h; } w;
	uint32_t d;
} m6801_pair;

enum { M6801_IRQ1_LINE = 0, M6801_NMI_LINE = 1, M6801_IS3_LINE = 2, M6801_TIN_LINE = 3 };
// M6801_TYPE_6800: plain MC6800 / MC6802 core (6800 opcode and cycle tables, no on-chip I/O, RAM or timer;
// the whole address space is the external bus)
enum { M6801_TYPE_6801 = 0, M6801_TYPE_6801U4 = 1, M6801_TYPE_6800 = 2 };

typedef struct m6801 m6801;
struct m6801 {
	// ---- configuration (kept across reset and state load) ----
	uint8_t (*read)(void *user, uint16_t addr);               // external bus (everything outside the on-chip I/O and RAM)
	void (*write)(void *user, uint16_t addr, uint8_t data);
	uint8_t (*port_r)(void *user, int port);                  // port 1..4 input pins (may be NULL = 0xff)
	void (*port_w)(void *user, int port, uint8_t data, uint8_t ddr);  // port 1..4 outputs (inputs read as 1, as in MAME)
	void *user;
	const uint8_t *rom;          // optional on-chip ROM fast path: rom_base .. rom_base + rom_size - 1 (read only)
	uint32_t rom_base, rom_size;
	int type;                    // M6801_TYPE_*

	// ---- state (from 'pc' to the end; see m6801_state_data) ----
	m6801_pair ppc, pc, s, x, d, ea;
	uint8_t cc, wai_state, nmi_state, nmi_pending;
	uint8_t irq_state[4];
	uint8_t reset_held;
	int32_t icount;

	uint8_t port_ddr[4], port_data[4];
	uint8_t p3csr, tcsr, pending_tcsr, ram_ctrl;
	uint8_t pending_isf_clear, port3_latched, port2_written, latch09, is3_state;
	uint8_t trcsr, rmcr, rdr, tdr, trcsr_read_tdre, trcsr_read_orfe, trcsr_read_rdrf, tx;
	uint8_t tcr[2], tsr, pending_tsr;     // 6801U4
	m6801_pair counter, output_compare[3], timer_over;
	uint16_t input_capture;
	uint32_t timer_next;
	uint8_t ram[256];            // on-chip RAM (0x80-0xff, 0x40-0xff on the U4), indexed by address
};

// set the callbacks / rom / type fields first, then init (clears the state) and reset
void m6801_init(m6801 *m);
void m6801_reset(m6801 *m);
// run for about 'cycles' E-clock cycles (input clock / 4); returns the cycles used by this call. Overshoot is
// carried into the next call
int m6801_execute(m6801 *m, int cycles);
// IRQ1 is level sensitive; NMI, IS3 (port 3 strobe) and TIN (input capture) act on edges
void m6801_set_irq_line(m6801 *m, int line, int state);
// RESET pin: asserting resets the chip, the CPU stays stopped while held
void m6801_set_reset(m6801 *m, int state);

// save state: a plain copy of the state part of the struct
static inline void *m6801_state_data(m6801 *m) { return &m->ppc; }
static inline uint32_t m6801_state_size(void) { return (uint32_t)(sizeof(m6801) - offsetof(m6801, ppc)); }

#ifdef __cplusplus
}
#endif

#endif

/*  Motorola MC68705 (HMOS M6805 family) microcontroller, ported from MAME
    src/devices/cpu/m6805/m6805.cpp, 6805ops.hxx, m6805defs.h (BSD-3-Clause, copyright-holders: Aaron Giles, Vas Crabb)
    and src/devices/cpu/m6805/m68705.cpp/.h (BSD-3-Clause, copyright-holders: Vas Crabb).
    Variants: MC68705P3, MC68705P5 (11-bit space, ports A/B/C, C is 4 bits), MC68705R3 (12-bit space, ports A-D, A/D).
    Struct based: any number of instances. Time is counted in internal cycles (input clock / 4).
    Save state: the whole `st` member is plain data (no pointers) and can be saved / loaded with memcpy.
    Not emulated (as in MAME): the bootstrap ROM (its range reads the user ROM image), INT2 / miscellaneous register,
    STOP / WAIT (not HMOS opcodes anyway).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M68705_H
#define RBX_M68705_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { M68705_P3, M68705_P5, M68705_R3 };

// input lines (m68705_set_input)
enum { M68705_IRQ_LINE = 0, M68705_INT_TIMER = 1, M68705_VPP_LINE = 2, M68705_VIHTP_LINE = 3 };

// timer sources (MAME m6805_timer::timer_source)
enum { M68705_TSRC_CLOCK = 0, M68705_TSRC_CLOCK_TIMER = 1, M68705_TSRC_DISABLED = 2, M68705_TSRC_TIMER = 3 };

// port callbacks: n = port 0..3 (A..D). read: mask = bits currently inputs (MAME mem_mask), return the pin levels.
// write: called when the driven value changes, data / mask as MAME devcb_write8 (open drain port A: data = latch | ~ddr,
// mask = ~latch & ddr). Unset callbacks: reads keep the last value given by m68705_port_input, writes are dropped.
typedef uint8_t (*m68705_read_fn)(void *user, int n, uint8_t mask);
typedef void (*m68705_write_fn)(void *user, int n, uint8_t data, uint8_t mask);
typedef uint8_t (*m68705_analog_fn)(void *user, int n);   // R3 A/D inputs AN0..AN3

typedef struct {
	// registers
	uint16_t pc, s, ea;
	uint8_t a, x, cc;
	uint8_t pending;            // pending interrupts (bit 0 /INT, bit 1 timer)
	uint8_t irq_state[4];       // line levels of the inputs
	int32_t icount;             // cycles left in the current execute() (carries overshoot)
	uint8_t in_reset;           // RESET line held
	// digital I/O
	uint8_t port_input[4], port_latch[4], port_ddr[4];
	uint8_t acr_mux, mr;
	// timer / counter
	uint8_t t_options, t_divisor, t_source, t_timer;
	uint32_t t_edges;
	uint8_t t_prescale, tdr, tcr;
	// EPROM control
	uint8_t vihtp, pcr, pl_data;
	uint16_t pl_addr;
	uint8_t ram[0x70];          // 0x10-0x7f
} m68705_state;

typedef struct {
	m68705_state st;
	// configuration (not saved)
	int variant;
	uint8_t *rom;               // user EPROM image, 0x800 (P3/P5) or 0x1000 (R3) bytes, addressed from 0
	uint16_t amask;             // address mask (0x7ff / 0xfff)
	uint8_t port_mask[4];       // bits not present (read as 1)
	uint8_t open_drain[4];
	m68705_read_fn port_r[4];
	m68705_write_fn port_w[4];
	m68705_analog_fn analog_r;
	void *user;
} m68705_t;

// rom must stay valid while the MCU is in use; size is checked against the variant (0 = ok)
int m68705_init(m68705_t *c, int variant, uint8_t *rom, uint32_t size);
void m68705_reset(m68705_t *c);                       // MAME device_reset
// run for at least `cycles` internal cycles (input clock / 4); returns the cycles actually spent in this call
int m68705_execute(m68705_t *c, int cycles);
void m68705_set_input(m68705_t *c, int line, int state);   // M68705_IRQ_LINE (/INT asserted = 1) ...
void m68705_set_irq_line(m68705_t *c, int state);
// RESET line: asserting resets and halts the MCU until it is cleared (MAME INPUT_LINE_RESET)
void m68705_set_reset_line(m68705_t *c, int state);
// external pin levels of a port without a read callback (MAME pa_w / pb_w / pc_w / pd_w)
void m68705_port_input(m68705_t *c, int n, uint8_t data);
void m68705_timer_w(m68705_t *c, int state);            // TIMER pin (external timer input)

#ifdef __cplusplus
}
#endif

#endif

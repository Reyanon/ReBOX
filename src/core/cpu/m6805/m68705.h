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

enum { M68705_P3, M68705_P5, M68705_R3, M68705_HD63705Z0 };

/*  Hitachi HD63705Z0 (MAME hd6305.cpp hd63705z0_device, BSD-3-Clause, copyright-holders: Aaron Giles, Vas Crabb,
    Olivier Galibert). As in MAME: 16-bit space, internal RAM 0x0040-0x01bf, internal ROM 0x01c0-0x1fff (the ROM image is
    addressed from 0, size >= 0x2000), every other address goes to ext_r / ext_w (no internal ports, timer or SCI, MAME
    has them as TODO). Stack 0x0100-0x017f, HD63705 cycle table, reset vector 0x1ffe (MAME: 0xfffe & the vector mask 0x1fff, the internal ROM), SWI 0x1ffa, NMI 0x1ffc,
    IRQ1 0x1ff8, IRQ2 0x1fec, ADCONV 0x1fea, TIMER1 0x1ff6, TIMER2 0x1ff4, TIMER3 0x1ff2, PCI 0x1ff0, SCI 0x1fee.
    Lines: m68705_hd63705_set_input(c, HD63705_INT_*, state), state 0 = clear, 1 = assert, 2 = hold (MAME HOLD_LINE: the
    line clears itself when an interrupt is taken). Interrupts are latched on the assert edge, so a pulse (assert then
    clear, MAME pulse_input_line) is enough. BIL / BIH test the NMI line (MAME hd6305_device::test_il).
    State: save both c->st and c->zst. */
enum {
	HD63705_INT_IRQ1 = 0, HD63705_INT_IRQ2 = 1, HD63705_INT_TIMER1 = 2, HD63705_INT_TIMER2 = 3, HD63705_INT_TIMER3 = 4,
	HD63705_INT_PCI = 5, HD63705_INT_SCI = 6, HD63705_INT_ADCONV = 7, HD63705_INT_NMI = 8
};
enum { M68705_CLEAR_LINE = 0, M68705_ASSERT_LINE = 1, M68705_HOLD_LINE = 2 };
typedef uint8_t (*m68705_ext_read_fn)(void *user, uint16_t addr);
typedef void (*m68705_ext_write_fn)(void *user, uint16_t addr, uint8_t data);

typedef struct {
	uint16_t pending;           // bit n = HD63705_INT_n
	uint8_t irq_state[8];       // M68705_CLEAR_LINE / ASSERT / HOLD
	uint8_t nmi_state;
	uint8_t ram[0x180];         // 0x0040-0x01bf
} m68705_zstate;

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
	// HD63705Z0 only
	m68705_zstate zst;
	m68705_ext_read_fn ext_r;   // unset: reads 0xff
	m68705_ext_write_fn ext_w;
	uint16_t sp_mask, sp_low;
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
// HD63705Z0 interrupt lines (HD63705_INT_IRQ1 .. HD63705_INT_NMI), state M68705_CLEAR_LINE / ASSERT / HOLD
void m68705_hd63705_set_input(m68705_t *c, int line, int state);

#ifdef __cplusplus
}
#endif

#endif

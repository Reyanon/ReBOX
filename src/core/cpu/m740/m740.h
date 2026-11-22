/*  Mitsubishi M740 (MELPS 740) CPU and the M37450 (M3745x) microcontroller, ported to C from MAME 0.289
    src/devices/cpu/m6502: m740.cpp, m740.h, om740.lst, dm740.lst, m6502.cpp, m6502.h, om6502.lst
    (BSD-3-Clause, copyright-holders: Olivier Galibert) and m3745x.cpp, m3745x.h (BSD-3-Clause, copyright-holders:
    R. Belmont, Olivier Galibert). The instructions are MAME's microcode as tools/gen_m740.py translates it: one CPU
    cycle per line with a bus access (MAME m6502make.py), the next opcode fetched at the end of each instruction
    (where interrupts are taken), the T flag selecting the second decode table. An instruction runs as one step.
    The M37450 part is what MAME models: internal RAM 0x0000-0x00bf and 0x0100-0x01ff, ports P3-P6 with their
    direction registers at 0x00d6-0x00dd, the A/D converter at 0x00e2-0x00e3, the interrupt request / control
    registers at 0x00fc-0x00ff and the INT1-INT3 inputs. The timers are not modeled (MAME allocates them but never
    starts them). Everything else goes to the external bus.
    Clock: MAME divides the input clock by 4 (execute_clocks_to_cycles); m740_execute counts CPU cycles.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_M740_H
#define RBX_M740_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// m740_set_input lines (MAME m3745x_device::M3745X_INT1_LINE.. = INPUT_LINE_IRQ0..; namco68 ext_interrupt uses line 0)
enum { M3745X_INT1_LINE = 0, M3745X_INT2_LINE = 1, M3745X_INT3_LINE = 2 };

typedef struct m740_s m740_t;
struct m740_s {
	// ---- state (saved: from pc up to rpage) ----
	uint16_t pc, sp, tmp;
	uint8_t a, x, y, p, tmp2, ir;
	uint16_t inst_state_base;  // (MAME m_inst_state_base: 0x100 while the T flag is set)
	int32_t inst_state;        // (MAME m_inst_state: -1 normal; STATE_RESET the reset sequence pending)
	uint8_t irq_state, irq_taken, halted, waiting, in_reset;
	uint8_t ports[4], ddrs[4]; // P3..P6 latches and direction registers (1 = output)
	uint8_t intreq1, intreq2, intctrl1, intctrl2, adctrl;
	uint8_t adc_busy;          // an A/D conversion runs until adc_due
	uint16_t irq_vector, last_all_ints;
	uint32_t irq_multiplex;
	int32_t icount;            // cycles left in the current m740_execute (<= 0 after: the overshoot, carried)
	int64_t cycles;            // CPU cycles executed since init
	int64_t adc_due;
	uint8_t ram[0x200];        // internal RAM (0x00c0-0x00ff unused here: the SFR area)
	// ---- bus (configuration, not saved) ----
	// pages 0x02-0xff: 256-byte pages read / written directly when the pointer is set, else through the callbacks.
	// Pages 0x00 / 0x01 are internal (the SFR gaps in page 0 go to the callbacks). A page with rpage set and wpage
	// NULL (ROM) sends its writes to the write callback, which should drop them (MAME .rom()).
	const uint8_t *rpage[256];
	uint8_t *wpage[256];
	uint8_t (*read)(void *ctx, uint16_t a);            // NULL: 0 (MAME unmapped)
	void (*write)(void *ctx, uint16_t a, uint8_t v);   // NULL: dropped
	uint8_t (*port_read)(void *ctx, int port);         // port 3..6 (MAME read_p<N>); NULL: 0
	void (*port_write)(void *ctx, int port, uint8_t v);// port 3..6 (MAME write_p<N>): the latch & DDR
	uint8_t (*ad_read)(void *ctx, int ch);             // channel 0..7 (MAME read_ad<N>); NULL: 0
	void *ctx;
};

// clear everything (power on: MAME init values, the reset sequence pending); then set the bus
void m740_init(m740_t *c);
// MAME device_reset (m6502 / m740 / m3745x): SFRs cleared, the reset sequence (vector 0xfffe) at the next step
void m740_reset(m740_t *c);
// MAME INPUT_LINE_RESET: asserting resets the device and holds it; clearing lets it run (namco68 ext_reset)
void m740_set_reset(m740_t *c, int state);
// MAME execute_set_input for the M3745x: INT1..INT3 set / clear their request bits (level, not latched as a line)
void m740_set_input(m740_t *c, int line, int state);
// run for `cycles` CPU cycles (the overshoot of the last instruction is taken from the next call); returns the
// cycles executed in this call
int m740_execute(m740_t *c, int cycles);
// one instruction (or the reset / interrupt sequence); returns the cycles used (a waiting / held CPU: 1)
int m740_step(m740_t *c);
// MAME m3745x are_port_bits_output: port 3..6
static inline int m740_port_bits_output(const m740_t *c, int port, uint8_t mask)
{
	return (c->ddrs[port - 3] & mask) == mask;
}

// save state: the state block (from pc up to rpage) as one blob
#define M740_STATE_SIZE ((uint32_t)(offsetof(m740_t, rpage)))

#ifdef __cplusplus
}
#endif
#endif

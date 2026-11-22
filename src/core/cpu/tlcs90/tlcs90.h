/*  Toshiba TLCS-90 series MCUs (TMP90840 / TMP90841 / TMP90845 / TMP91640 / TMP91641 / TMP90PH44), ported to C99
    from MAME src/devices/cpu/tlcs90/tlcs90.cpp and tlcs90.h (license: BSD-3-Clause, copyright-holders: Luca Elia;
    "emulation by Luca Elia, based on the Z80 core by Juergen Buchmueller").
    This file is part of ReBOX. GPL-2.0-or-later.

    Time unit: tlcs90_execute() takes and returns input clock cycles, as MAME's icount (every instruction state
    is 2 clocks, e.g. NOP = 4 clocks). The internal timers count in the same unit: one timer tick every
    8 clocks x prescaler (MAME: m_timer_period = clock * 8), stepped inside tlcs90_execute instead of device timers.

    Memory: a 20-bit program space (ports P4 / BX / BY give the upper 4 bits for (IX) / (IY) accesses, as MAME).
    The internal ROM (rom pointer, 0x0000 - rom_size-1), the internal RAM and the SFRs at 0xffc0 - are handled
    here, like MAME's internal maps (internal entries win over the external map); every other address goes to
    read8 / write8. Writes to the internal ROM are ignored.

    Save state: the struct holds no run-time pointers outside the configuration part; save the bytes
    from TLCS90_STATE_OFFSET for TLCS90_STATE_SIZE (or memcpy the whole struct and restore the configuration).
*/
#ifndef RBX_TLCS90_H
#define RBX_TLCS90_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// chip types (MAME device types)
enum { TLCS90_TMP90840, TLCS90_TMP90841, TLCS90_TMP90845, TLCS90_TMP91640, TLCS90_TMP91641, TLCS90_TMP90PH44 };

// input lines for tlcs90_set_irq (MAME INPUT_LINE_IRQ0 / IRQ1 / IRQ2 / INPUT_LINE_NMI)
enum { TLCS90_LINE_INT0 = 0, TLCS90_LINE_INT1 = 1, TLCS90_LINE_INT2 = 2, TLCS90_LINE_NMI = 3 };

// interrupt sources (MAME tlcs90_e_irq), bit numbers of irq_state / irq_mask
enum {
	TLCS90_INTSWI = 0, TLCS90_INTNMI, TLCS90_INTWD, TLCS90_INT0, TLCS90_INTT0, TLCS90_INTT1, TLCS90_INTT2,
	TLCS90_INTT3, TLCS90_INTT4, TLCS90_INT1, TLCS90_INTT5, TLCS90_INT2, TLCS90_INTRX, TLCS90_INTTX, TLCS90_INTMAX
};

// MAME's PAIR (little-endian hosts: x86, ARM Vita, MIPS PSP)
typedef union {
	uint32_t d;
	struct { uint16_t l, h; } w;
	struct { uint8_t l, h, h2, h3; } b;
} tlcs90_pair;

typedef struct tlcs90 tlcs90;
struct tlcs90 {
	// ---- configuration (set after tlcs90_init, not part of the saved state) ----
	const uint8_t *rom;          // internal ROM (8 KB TMP90840, 16 KB TMP91640 / TMP90PH44); NULL = goes external
	void *userdata;
	uint8_t (*read8)(void *user, uint32_t addr);            // external memory, 20-bit address; NULL = 0
	void (*write8)(void *user, uint32_t addr, uint8_t v);
	uint8_t (*port_r)(void *user, int port);                 // P0 - P8 input (MAME m_port_read_cb); NULL = 0xff
	void (*port_w)(void *user, int port, uint8_t v);         // P0 - P8 output (MAME m_port_write_cb)
	int type;
	uint32_t rom_size;           // from the type (0 = rom-less)
	uint32_t ram_start;          // internal RAM: ram_start - 0xffbf
	uint32_t sfr_end;            // SFRs: 0xffc0 - sfr_end
	int regs844;                 // 1 = TMP90844 SFR layout (TMP90PH44)

	// ---- state (from here to the end: plain data, memcpy-able) ----
	uint8_t state_begin;
	tlcs90_pair prvpc, pc, sp, af, bc, de, hl, ix, iy;
	tlcs90_pair af2, bc2, de2, hl2;
	uint8_t halt, after_EI;
	uint16_t irq_state, irq_line_state, irq_mask;
	int icount;
	int extra_cycles;            // extra cycles for interrupts

	uint8_t port_latch[9];
	uint8_t p01cr, p2cr, p4cr, p67cr, p8cr, smmod;
	uint32_t ixbase, iybase;

	// timers: 4 x 8-bit + 1 x 16-bit
	uint8_t timer_value[4];
	uint16_t timer4_value;
	uint8_t tmod, tclk, trun;
	uint8_t treg_8bit[4];
	uint8_t t4mod;
	uint16_t treg_16bit[2];
	uint8_t timer_run;           // bit i = timer i scheduled (MAME emu_timer armed)
	int32_t timer_left[5];       // clocks until the next tick
	int32_t timer_period[5];     // clocks per tick

	// work registers
	uint8_t op;
	uint8_t mode1, mode2;
	uint16_t r1, r1b, r2, r2b;
	int cyc_t, cyc_f;
	uint32_t addr;

	uint8_t iram[512];
};

#define TLCS90_STATE_OFFSET offsetof(tlcs90, state_begin)
#define TLCS90_STATE_SIZE (sizeof(tlcs90) - offsetof(tlcs90, state_begin))

// clears the struct (configuration included) and sets the chip type; then set rom / callbacks and call tlcs90_reset
void tlcs90_init(tlcs90 *t, int type);
void tlcs90_reset(tlcs90 *t);
// run for at least 'cycles' clocks (always at least one instruction); returns the clocks actually run
int tlcs90_execute(tlcs90 *t, int cycles);
// external line level (1 = asserted): TLCS90_LINE_INT0 / INT1 / INT2 / NMI (MAME execute_set_input)
void tlcs90_set_irq(tlcs90 *t, int line, int state);

#ifdef __cplusplus
}
#endif

#endif

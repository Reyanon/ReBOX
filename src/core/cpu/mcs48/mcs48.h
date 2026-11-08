/*  Intel MCS-48 / UPI-41 CPU core, ported to C99 from MAME src/devices/cpu/mcs48/mcs48.cpp
    (license: BSD-3-Clause, copyright-holders: Dan Boris, Mirko Buffoni, Aaron Giles, Couriersud).
    This file is part of ReBOX. GPL-2.0-or-later.

    Time unit: mcs48_execute() takes and returns machine cycles. As in MAME, one machine cycle
    is 15 input clocks for both the MCS-48 parts (8035/8048/8049/8050) and the UPI-41/42 parts
    (8041/8042/8741/8742): an I8742 on a 6 MHz clock runs 400000 cycles per second.

    Save state: the struct holds no pointers that change at run time except the configuration
    (rom, callbacks, userdata); save with mcs48_state_size()/memcpy of the whole struct between
    the 'state_begin' and 'state_end' markers (see MCS48_STATE_* below), or simply memcpy the
    struct and restore the configuration fields afterwards.
*/
#ifndef RBX_MCS48_H
#define RBX_MCS48_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
	MCS48_I8035, MCS48_I8048, MCS48_I8039, MCS48_I8049, MCS48_I8040, MCS48_I8050,   // MCS-48
	MCS48_I8041A, MCS48_I8041AH, MCS48_I8042, MCS48_I8742, MCS48_I8042AH, MCS48_I8742AH   // UPI-41/42
};

typedef struct mcs48 mcs48;
struct mcs48 {
	// ---- configuration (set after mcs48_init, not part of the saved state) ----
	const uint8_t *rom;          // program memory (internal ROM, or external program for ROM-less parts)
	uint16_t rom_mask;           // program address mask (rom_size - 1, 0xfff for external)
	void *userdata;
	uint8_t (*port_r)(void *user, int port);           // P1 / P2 input (port = 1 or 2); NULL = 0xff
	void (*port_w)(void *user, int port, uint8_t v);   // P1 / P2 output
	int (*test_r)(void *user, int t);                  // T0 / T1 input (t = 0 or 1); NULL = 0
	uint8_t (*bus_r)(void *user);                      // BUS (MCS-48 only); NULL = 0xff
	void (*bus_w)(void *user, uint8_t v);
	uint8_t (*ext_r)(void *user, uint8_t a);           // MOVX (MCS-48 only)
	void (*ext_w)(void *user, uint8_t a, uint8_t v);
	void (*prog_w)(void *user, int state);             // PROG pin (8243 expander strobe)
	int type, upi41;
	uint16_t ram_mask;

	// ---- state (from here to the end: plain data, memcpy-able) ----
	uint8_t state_begin;
	uint16_t prevpc, pc;
	uint8_t a, psw;
	uint8_t f1;
	uint16_t a11;
	uint8_t p1, p2;
	uint8_t ea;
	uint8_t timer, prescaler, t1_history;
	uint8_t sts, dbbi, dbbo;
	uint8_t irq_state, irq_polled, irq_in_progress;
	uint8_t timer_overflow, timer_flag, tirq_enabled, xirq_enabled;
	uint8_t timecount_enabled, flags_enabled, dma_enabled;
	int icount;
	uint8_t ram[256];
};

#define MCS48_STATE_OFFSET offsetof(mcs48, state_begin)
#define MCS48_STATE_SIZE (sizeof(mcs48) - offsetof(mcs48, state_begin))

// clears the struct (configuration included) and sets the chip type; then set rom / callbacks and call mcs48_reset
void mcs48_init(mcs48 *c, int type);
void mcs48_reset(mcs48 *c);
// run for at least 'cycles' machine cycles (always at least one instruction); returns the cycles actually run
int mcs48_execute(mcs48 *c, int cycles);
// external /INT line level (1 = asserted)
void mcs48_set_irq(mcs48 *c, int state);
// UPI-41 host interface: offset 0 = DBBO (read) / data (write, A0 = 0), offset 1 = STS (read) / command (write, A0 = 1)
uint8_t upi41_master_r(mcs48 *c, int offset);
void upi41_master_w(mcs48 *c, int offset, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif

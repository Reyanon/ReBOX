/*  Intel MCS-51 (8031 / 8051 / 8751) microcontroller, ported from MAME src/devices/cpu/mcs51/i8051.cpp, i8051.h and
    mcs51ops.cpp (BSD-3-Clause, copyright-holders: Steve Ellenoff, Manuel Abadia, Couriersud).
    Struct based: any number of instances. Time is counted in machine cycles (input clock / 12).
    Covered as in MAME: the whole 8051 instruction set and cycle table, ports P0-P3 with read-modify-write
    behaviour, timers 0 / 1 (modes 0-3, gate, counter inputs), external interrupts INT0 / INT1 (edge / level),
    the two-level priority system and the JB INT0/INT1 polling hack. The 8052 / 80C52 parts (is_8052, after
    i8052.cpp / i80c52.cpp): the upper 128 bytes of RAM (indirect), timer 2 (auto reload, capture, baud rate
    counting; no T2 / T2EX pins), its interrupt, IPH (four priority levels). Not emulated: the serial port (SBUF
    writes are latched, TI / RI are only set by software), power-down / idle (no HMOS part has them).
    Save state: the `st` member (and `st2` of an 8052) is plain data and can be saved / loaded with memcpy.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_MCS51_H
#define RBX_MCS51_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// input lines (mcs51_set_input)
enum { MCS51_INT0_LINE = 0, MCS51_INT1_LINE, MCS51_T0_LINE, MCS51_T1_LINE };

typedef struct {
	uint16_t pc, ppc, dptr;
	uint8_t acc, psw, b, sp, pcon, tcon, tmod, scon, sbuf, ie, ip;
	uint8_t p0, p1, p2, p3;
	uint8_t tl0, tl1, th0, th1;
	uint8_t rwm, last_op, last_bit, irq_active;
	int8_t cur_irq_prio;
	uint8_t irq_prio[8];
	uint32_t last_line_state;
	int32_t t0_cnt, t1_cnt;
	int32_t icount;           // machine cycles left in the current execute() (carries overshoot)
	int32_t inst_cycles;
	uint8_t iram[128];
} mcs51_state;

typedef struct {               // the 8052 / 80C52 additions
	uint8_t iram_hi[128];      // 0x80-0xff (indirect addressing)
	uint8_t t2con, iph, saddr, saden;
	uint32_t rcap2, t2;
} mcs51_state52;

typedef struct mcs51_t mcs51_t;
struct mcs51_t {
	mcs51_state st;
	mcs51_state52 st2;
	// configuration (not saved)
	int is_8052;               // set after mcs51_init, before mcs51_reset
	const uint8_t *rom;        // program memory (internal ROM, or the external program ROM of an 8031)
	uint32_t rom_size;         // bytes; addresses at or past the end read 0 (MAME unmapped)
	uint8_t forced_inputs[4];
	uint8_t (*port_in)(void *user, int port);              // NULL: 0xff
	void (*port_out)(void *user, int port, uint8_t data);  // NULL: dropped
	uint8_t (*xdata_r)(void *user, uint16_t addr);         // MOVX space, NULL: 0
	void (*xdata_w)(void *user, uint16_t addr, uint8_t data);
	void *user;
};

void mcs51_init(mcs51_t *c, const uint8_t *rom, uint32_t rom_size);   // clears the callbacks
void mcs51_reset(mcs51_t *c);                  // MAME device_reset (writes 0xff to the ports)
int mcs51_execute(mcs51_t *c, int cycles);      // run at least `cycles` machine cycles; returns cycles spent
void mcs51_set_input(mcs51_t *c, int line, int state);

#ifdef __cplusplus
}
#endif

#endif

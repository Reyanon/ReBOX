/*  Toshiba TMP68301: the peripherals around its 68HC000 core (the core is Musashi, driven by the board): the address
    decoder registers, the interrupt controller (3 external lines, the serial, parallel and timer sources, vectored),
    the parallel port (GPIO modes), the 3 serial channels (transmit timing; nothing ever received) and the 3 16-bit
    timers. Ported from MAME src/devices/cpu/m68000/tmp68301.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert).
    Times are the CPU's clock cycles since power on (tmp68301_t.now).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TMP68301_H
#define RBX_TMP68301_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	// ---- state (saved: tmp68301_state) ----
	uint64_t serial_tx_next[3], serial_rx_next[3], serial_gclk[3];
	uint64_t timer_next[3], timer_last_sync[3];
	uint32_t interrupt_state;
	uint16_t arelr, imr, ipr, iisr, pdir, pdr, tcr[3], tmcr1[3], tmcr2[3], tctr[3];
	uint8_t ext_state, hold;   // the external lines (bit n: line n), those held (HOLD_LINE: cleared when taken)
	uint8_t amar[2], aamr[2], aacr[3], ator, icr[10], ivnr, ieir;
	uint8_t pcr, parallel_mode, psr, pcmr, pmr, ppr1, ppr2;
	uint8_t scr, spr, smr[3], scmr[3], sbrr[3], ssr[3], sdrr[3], sdrt[3], sr_intr[3];
	uint8_t serial_tx_state[3], serial_rx_state[3], serial_tx[3], serial_rx[3], serial_rx_line[3];
	uint8_t level;             // the interrupt level given to the core
	uint8_t pad[7];
	// ---- configuration (not saved) ----
	uint32_t clock;
	uint64_t (*now)(void);                // the CPU cycles so far
	uint16_t (*par_r)(void);              // the parallel port's inputs (NULL: 0xffff)
	void (*par_w)(uint16_t data);         // its outputs (NULL: none)
	void (*set_level)(int level);         // the interrupt level changed
	void (*events_changed)(void);         // the next event moved (the board ends the CPU's slice to look at it)
} tmp68301_t;

#define TMP68301_STATE_SIZE ((uint32_t)offsetof(tmp68301_t, clock))

void tmp68301_init(tmp68301_t *t, uint32_t clock);
void tmp68301_reset(tmp68301_t *t);
// the internal registers at 0xfffc00-0xffffff (mask: the byte lanes written)
uint16_t tmp68301_read(tmp68301_t *t, uint32_t addr);
void tmp68301_write(tmp68301_t *t, uint32_t addr, uint16_t data, uint16_t mask);
// an external interrupt line (0-2; 3-9: the expanded ones). hold: HOLD_LINE (cleared when the CPU takes it)
void tmp68301_set_input(tmp68301_t *t, int line, int state, int hold);
// the CPU takes the interrupt: the vector it reads (ivnr | source)
uint8_t tmp68301_ack(tmp68301_t *t);
// the next timer / serial event (0: none), and the events due at `now`
uint64_t tmp68301_next_event(const tmp68301_t *t);
void tmp68301_update(tmp68301_t *t, uint64_t now);

#endif

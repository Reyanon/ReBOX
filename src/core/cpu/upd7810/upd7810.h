/*  NEC uPD7810 / 78C10 / 78C11 CPU, ported from MAME src/devices/cpu/upd7810 (BSD-3-Clause,
    copyright-holders: Juergen Buchmueller). One instance (the C-Chip MCU of the Taito boards, the Metro sound CPU).
    Cycle counts are machine states (MAME: one state = 3 input clocks).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_UPD7810_H
#define RBX_UPD7810_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// input lines (MAME UPD7810_INTF1 / INTF2 / INPUT_LINE_NMI)
#define UPD7810_INTF1 0
#define UPD7810_INTF2 1
#define UPD7810_NMI   32

// port numbers
enum { UPD7810_PORTA, UPD7810_PORTB, UPD7810_PORTC, UPD7810_PORTD, UPD7810_PORTF };
// output lines
enum { UPD7810_LINE_TO, UPD7810_LINE_CO0, UPD7810_LINE_CO1, UPD7810_LINE_TXD };

typedef struct {
	const uint8_t *irom;              // internal mask ROM at 0x0000 (uPD7811 / 78C11: 4 KB), NULL = none
	uint32_t irom_size;
	uint8_t (*read)(uint16_t a);      // external memory
	void (*write)(uint16_t a, uint8_t v);
	uint8_t (*port_in)(int port);
	uint8_t in_ports;                 // ports with an input callback (1 << UPD7810_PORTx); others keep the latch
	void (*port_out)(int port, uint8_t v);
	uint8_t (*an_in)(int n);          // A/D inputs AN0..AN7
	void (*line_out)(int line, int state);
	int (*rxd)(void);                 // serial input line RXD (MAME rxd_func), NULL = 1
} upd7810_config;

void upd7810_init(const upd7810_config *cfg);   // also selects the uPD7810 / 78C1x opcode tables
void upd7810_reset(void);
// run for cycles machine states (overshoot carried to the next call); returns the states run
int upd7810_execute(int cycles);
void upd7810_set_input(int line, int state);    // state 1 = asserted
uint16_t upd7810_pc(void);
int upd7810_state(void *ctx, int load);         // through rbx_state_io

#ifdef __cplusplus
}
#endif

#endif

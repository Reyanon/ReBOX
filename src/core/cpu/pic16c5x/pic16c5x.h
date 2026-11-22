/*  Microchip PIC16C5x (and GI PIC1650 / 1654S / 1655), ported to C from MAME src/devices/cpu/pic16c5x/pic16c5x.cpp,
    pic16c5x.h (BSD-3-Clause, copyright-holders: Tony La Porta).
    A cycle is 4 input clocks (8 on the 1654S). Program space: 9 to 11-bit word addresses (the ROM as little-endian
    16-bit words); data space: the model's register file (ports, RAM, the 16C57/58 banks). The instruction set, flags,
    timer, watchdog and cycle counts as MAME's.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_PIC16C5X_H
#define RBX_PIC16C5X_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { PIC16C54 = 0x16C54, PIC16C55 = 0x16C55, PIC16C56 = 0x16C56, PIC16C57 = 0x16C57, PIC16C58 = 0x16C58,
       PIC1650 = 0x1650, PIC1654S = 0x1654, PIC1655 = 0x1655 };

typedef struct pic16c5x_s pic16c5x_t;
struct pic16c5x_s {
	// ---- state (saved: up to ram, included) ----
	uint16_t PC, PREVPC;
	uint8_t W, OPTION;
	uint16_t CONFIG;
	uint8_t ALU;
	uint16_t WDT;
	uint8_t TMR0, STATUS, FSR;
	uint8_t port_data[4], port_tris[3];
	uint16_t STACK[2];
	uint16_t prescaler;
	uint16_t opcode;
	int32_t icount, delay_timer, rtcc;
	uint16_t temp_config;
	uint8_t count_cycles, inst_cycles;
	uint64_t total_cycles;
	uint8_t ram[128];
	// ---- configuration (set by pic16c5x_init) ----
	int picmodel, data_width, program_width;
	uint8_t data_mask, status_mask;
	uint16_t program_mask;
	const uint8_t *rom;       // little-endian words
	uint32_t rom_words;
	void *ctx;
	// port callbacks (port 0..3 = A..D); read gets mem_mask 0xff, write gets the data with the driven bits only.
	// A read must not change within one pic16c5x_execute (the polling loop skip relies on it; RBX_PIC_NOSPIN=1 = off)
	uint8_t (*read_port)(void *ctx, int port);
	void (*write_port)(void *ctx, int port, uint8_t data, uint8_t mem_mask);
};

void pic16c5x_init(pic16c5x_t *p, int model, const uint8_t *rom, uint32_t rom_bytes, void *ctx);   // device_start
void pic16c5x_set_config(pic16c5x_t *p, uint16_t data);
void pic16c5x_reset(pic16c5x_t *p);                       // device_reset
int pic16c5x_execute(pic16c5x_t *p, int cycles);         // execute_run: returns the cycles run
void pic16c5x_set_input(pic16c5x_t *p, int state);       // the RTCC / T0CKI pin
size_t pic16c5x_state_size(void);                        // the saved part (from the start to ram, included)

#ifdef __cplusplus
}
#endif

#endif

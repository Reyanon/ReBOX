/*  IGS PGM board internals shared by the base driver (pgm.c) and the protection devices (pgm_prot_*.c), after MAME
    src/mame/igs/pgm.cpp / pgm.h (BSD-3-Clause, copyright-holders: David Haywood, ElSemi).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_IGS_PGM_H
#define RBX_IGS_PGM_H

#include <stdint.h>

#include "../../rbx.h"

// the board's 68000 side, set up by pgm.c
extern uint8_t *pgm_rom;          // "maincpu", host-order words (bios at 0, game at 0x100000)
extern uint32_t pgm_rom_size;
extern uint16_t pgm_mainram[0x10000];   // 0x800000 (0x20000 bytes, battery backed)
extern const rbx_game *pgm_game;
extern int pgm_irq4_disabled;     // MAME m_irq4_disabled (set by a protection's init: no vblank-end IRQ 4)

uint32_t pgm_region_port(void);   // the "Region" input port (0 without one)
uint32_t pgm_region_hack(void);   // the "RegionHack" configuration (0xff without one: untouched)
// clocks of a CPU at `clock` Hz for the next scanline (fractions in *frac)
int pgm_line_cycles(uint64_t *frac, uint32_t clock);
int pgm_port(const char *tag);    // rbx_port_find
uint8_t *pgm_region(const char *name, uint32_t *size);
int pgm_m68k_cycles(void);        // 68000 cycles into the current frame (for the protection CPUs' sync)

// a protection device: hooks around the base board (any may be NULL)
typedef struct {
	int (*init)(const char *init);   // the MAME init name ("kovsh"...); after the base map: 0 = ok
	void (*reset)(void);
	void (*exit)(void);
	// 68000 accesses the base map does not handle (mask: the byte lanes of a write; reads are whole words)
	int (*read16)(uint32_t a, uint16_t *v);   // 1 = handled
	int (*write16)(uint32_t a, uint16_t v, uint16_t mask);
	void (*line)(int line);          // after the 68000's slice of each scanline (a protection CPU runs here)
	void (*vblank)(void);
	int (*state)(void *ctx, int load);
	uint32_t rom_end;                // the game ROM's last address on the 68000 bus (0: 0x3fffff)
} pgm_prot;

// the 68000 map: [start, end] direct to memory (after the protection's init, before the game starts)
void pgm_map(uint32_t start, uint32_t end, void *mem, uint32_t size, int writable);
// end the 68000's slice now (a write that must reach a protection CPU before the 68000 goes on)
void pgm_m68k_yield(void);
// MAME spin_until_interrupt on the 68000: end its slice; it sleeps until the next interrupt is raised (IRQ4 / IRQ6)
void pgm_m68k_spin(void);

extern const pgm_prot pgm_prot_asic3;    // Oriental Legend
extern const pgm_prot pgm_prot_025_012;  // Dragon World 2
extern const pgm_prot pgm_prot_025_022;  // Killing Blade, Dragon World 3
extern const pgm_prot pgm_prot_025_028;  // Oriental Legend Special
extern const pgm_prot pgm_prot_arm1;     // IGS027A type 1 (ARM and simulations)
extern const pgm_prot pgm_prot_arm2;     // type 2
extern const pgm_prot pgm_prot_arm3;     // type 3

#endif

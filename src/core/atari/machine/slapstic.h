/*  Atari Slapstic (the ROM bank protection chip, 137412-101 to -118). Ported from MAME src/mame/atari/slapstic.cpp,
    slapstic.h (BSD-3-Clause, copyright-holders: Aaron Giles).
    MAME taps every access of the CPU's space; here the board calls slapstic_access for the accesses inside the
    slapstic's range only (as MAME did before its 0.230 rewrite: the bank sequences of the games run inside it).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SLAPSTIC_H
#define RBX_SLAPSTIC_H

#include <stdint.h>

typedef struct { uint32_t m, v; } slap_test;

typedef struct {
	// ---- saved ----
	uint8_t state;          // S_* of slapstic.cpp
	uint8_t current_bank, loaded_bank;
	// ---- configuration ----
	int chipnum;
	uint8_t bankstart;
	slap_test reset, inside, bank[4], alt1, alt2, alt3, alt4, bit1, bit2, add1, add2, addp1, addp2, add3;
	slap_test bset[2][4];   // bit_set (even / odd): clear0, set0, clear1, set1
	slap_test bit4;
	int altshift;
	uint32_t pf_inst, pf_skip;   // (slapstic_tap: the instruction seen last, the fetch to pass over)
	int pf_done;                 // (its fetch at the PC taken)
} slapstic_t;

// set_range(start, end, mirror) of a 16-bit (data_width 16) or 8-bit space; 0 = ok
int slapstic_init(slapstic_t *s, int chipnum, uint32_t start, uint32_t end, uint32_t mirror, int data_width);
void slapstic_reset(slapstic_t *s);   // (device_reset: the start bank)
// one access at (byte) address a; returns the current bank afterwards
int slapstic_access(slapstic_t *s, uint32_t a);
// An access in the range by a 68000 (the board's handler) in MAME's order: MAME's 68000 fetches the next instruction
// word (at the PC) before the operand accesses of an instruction, and its tap sees that fetch, where Musashi fetches
// it after them. So the word at the PC goes before the instruction's first operand access, once per instruction
// (inst: a value that changes from one instruction to the next), and the CPU's own fetch of it later is passed over. Out of the range the fetch counts
// only as an "anywhere" alt start (Peter Pack Rat's move.w (a0),(a1) at 0x101d4 with the fetch of 0x101d6; in the
// range Indiana Jones' alt sequence at 0x80078)
void slapstic_tap(slapstic_t *s, uint32_t a, uint32_t pc, int pc_inside, uint32_t inst);

#endif

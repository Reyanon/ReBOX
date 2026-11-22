/*  The Seibu COP bootleg (an FPGA clone of the COPDX on the Seibu Cup Soccer bootlegs): its address registers, the
    command triggers MAME simulates (moves, angles, distances, sine / cosine), the PRNG, the latches.
    Ported from MAME 0.289 src/mame/seibu/seicopbl.cpp / seicopbl.h (LGPL-2.1-or-later, used here under its GPL
    option; copyright-holders: Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina).
    This file is part of ReBOX. GPL-2.0-or-later.

    The host is the board's 68000 (its bus through m68k_read_memory_* / m68k_write_memory_*). One instance.
*/
#ifndef RBX_SEICOPBL_H
#define RBX_SEICOPBL_H

#include <stdint.h>

// host_cycles: the 68000's total cycles (prng_r)
void seicopbl_init(uint64_t (*host_cycles)(void));
void seicopbl_reset(void);
// the 68000's accesses to 0x100400-0x1005ff (byte offset from 0x100400, word aligned)
uint16_t seicopbl_read(uint32_t off);
void seicopbl_write(uint32_t off, uint16_t data, uint16_t mem_mask);
int seicopbl_state(void *ctx, int load);

#endif

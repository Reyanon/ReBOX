/*  Seibu COP (the "Raiden 2 COP"): its macro table uploads, the macro dispatch (the legacy one of the 68000 boards:
    Legionnaire, Heated Barrel, Godzilla, SD Gundam Sangokushi, Denjin Makai; cop_cmd_w of the V30 boards: Raiden II,
    Raiden DX, Zero Team, X Se Dae Quiz), its DMAs (tilemap and palette buffers, fills, palette fades, z-sort), the
    sprite DMA helpers, collisions, number conversion, the PRNG.
    Ported from MAME 0.289 src/mame/seibu/seibucop.cpp / .h / seibucop_cmd.ipp / seibucop_dma.ipp (LGPL-2.1+,
    copyright-holders: Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina).
    This file is part of ReBOX. GPL-2.0-or-later.

    The host is the board's 68000 (its bus through m68k_read_memory_* / m68k_write_memory_*) or, with the bus
    callbacks set, a little-endian CPU (the V30: MAME's m_host_endian false). One instance.
*/
#ifndef RBX_SEIBUCOP_H
#define RBX_SEIBUCOP_H

#include <stdint.h>

typedef struct {
	void (*videoram_w)(int offset, uint16_t data);   // videoramout_cb
	void (*palette_w)(int offset, uint16_t data);    // paletteramout_cb
	uint64_t (*host_cycles)(void);                   // the host's total cycles (cop_prng_r)
	// the host's bus (NULL: the 68000's, big-endian): a little-endian host's byte and word accesses (any address)
	uint8_t (*read8)(uint32_t a);
	uint16_t (*read16)(uint32_t a);
	void (*write8)(uint32_t a, uint8_t v);
	void (*write16)(uint32_t a, uint16_t v);
	int raiden2;                                     // 1: cop_cmd_w (the V30 boards' map), else LEGACY_cop_cmd_w
} seibucop_config;

void seibucop_init(const seibucop_config *cfg);
void seibucop_reset(void);
// the host's accesses to the COP's registers (offset from the block's base: 0x100400 on the 68000 boards, 0x400 on
// the V30 ones; 1 = taken)
int seibucop_read(uint32_t off, uint16_t *v);
int seibucop_write(uint32_t off, uint16_t data, uint16_t mem_mask);
int seibucop_state(void *ctx, int load);

#endif

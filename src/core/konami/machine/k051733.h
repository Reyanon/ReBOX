/*  Konami 051733 (protection / math: the division, the square root, the random numbers, the direction of two points,
    and an NMI timer). Ported from MAME src/mame/konami/k051733.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K051733_H
#define RBX_K051733_H

#include <stdint.h>

typedef struct {
	uint8_t ram[0x20];
	uint16_t lfsr, nmi_timer;
	uint8_t nmi_clock;
} k051733_t;

void k051733_reset(k051733_t *k);
uint8_t k051733_read(k051733_t *k, int offset);
void k051733_write(k051733_t *k, int offset, uint8_t data);
// the vblank's clock (nmiclock_w): 1 when the NMI fires now (MAME clears it 4 chip clocks later)
int k051733_nmiclock(k051733_t *k, int state);

#endif

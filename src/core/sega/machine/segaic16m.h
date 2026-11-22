/*  Sega 315-5248 multiplier and 315-5249 divider (MAME src/mame/sega/segaic16_m.cpp, BSD-3-Clause, copyright-holders:
    Aaron Giles). The registers are the board's (saved with its state).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEGAIC16M_H
#define RBX_SEGAIC16M_H

#include <stdint.h>

uint16_t sega_mult_r(const uint16_t r[2], uint32_t offset);   // offset in words
void sega_mult_w(uint16_t r[2], uint32_t offset, uint16_t data, uint16_t mask);
uint16_t sega_div_r(const uint16_t r[8], uint32_t offset);
void sega_div_w(uint16_t r[8], uint32_t offset, uint16_t data, uint16_t mask);

#endif

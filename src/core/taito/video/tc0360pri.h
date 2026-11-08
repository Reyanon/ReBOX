/*  Taito TC0360PRI priority manager, from MAME src/mame/taito/tc0360pri.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria). Sixteen byte registers: 0 = blend mode (top two bits), 1 = roz palette bank / priority select,
    4-5 = tilemap input priorities, 6-7 = sprite priorities by colour bits, 8-9 = third input (roz / second
    TC0100SCN / text). The mixing itself is done by the board's screen update (MAME taito_f2_v.cpp etc.).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_TC0360PRI_H
#define RBX_TC0360PRI_H

#include <stdint.h>
#include <string.h>

typedef struct { uint8_t regs[16]; } tc0360pri_t;

static inline void tc0360pri_reset(tc0360pri_t *p) { memset(p->regs, 0, sizeof(p->regs)); }
static inline void tc0360pri_w(tc0360pri_t *p, int offset, uint8_t data) { p->regs[offset & 15] = data; }
static inline uint8_t tc0360pri_r(const tc0360pri_t *p, int offset) { return p->regs[offset & 15]; }

#endif

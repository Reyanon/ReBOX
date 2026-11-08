/*  Data East graphics ROM encryption (custom chips 56 / 74 / 141), after MAME src/mame/dataeast/decocrpt.cpp
    (BSD-3-Clause, copyright-holders: Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECOCRPT_H
#define RBX_DECOCRPT_H
#include <stdint.h>
// decrypt a graphics region in place (its bytes as loaded); 0 = ok, -1 = no memory
int deco56_decrypt_gfx(uint8_t *rom, uint32_t size);
int deco74_decrypt_gfx(uint8_t *rom, uint32_t size);
int deco56_remap_gfx(uint8_t *rom, uint32_t size);   // the address remap only (no XOR / bit swap)
#endif

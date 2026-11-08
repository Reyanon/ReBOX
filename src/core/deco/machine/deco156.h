/*  Data East 156: the encrypted ARM's program decryption, after MAME src/mame/dataeast/deco156.cpp
    (BSD-3-Clause, copyright-holders: Bryan McPhail).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECO156_H
#define RBX_DECO156_H
#include <stdint.h>
// decrypt the ARM program region in place (little-endian 32-bit words, as MAME's region); 0 = ok, -1 = no memory
int deco156_decrypt(uint8_t *rom, uint32_t size);
#endif

/*  NEC MC-8123 decryption (MAME src/devices/cpu/z80/mc8123.cpp, BSD-3-Clause). This file is part of ReBOX.
    GPL-2.0-or-later.
*/
#ifndef RBX_MC8123_H
#define RBX_MC8123_H

#include <stdint.h>

// the Z80's ROM decrypted: the data in place, the opcodes to opcodes[] (key: 8 KB)
void mc8123_decode(uint8_t *rom, uint8_t *opcodes, unsigned length, const uint8_t *key);

#endif

/*  Hitachi FD1089A / FD1089B decryption (MAME src/mame/sega/fd1089.cpp, BSD-3-Clause). This file is part of ReBOX.
    GPL-2.0-or-later.
*/
#ifndef RBX_FD1089_H
#define RBX_FD1089_H

#include <stdint.h>

// the program (host-order words, mapped at 0) decrypted: the data in place, the opcodes to opcodes[]
void fd1089_decrypt(int type_b, uint16_t *rom, uint32_t size, const uint8_t *key, uint16_t *opcodes);

#endif

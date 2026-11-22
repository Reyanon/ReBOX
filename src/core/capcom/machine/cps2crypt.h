/*  CPS2 opcode decryption (MAME src/mame/capcom/cps2crypt.cpp, BSD-3-Clause). This file is part of ReBOX. GPL-2.0-or-later. */
#ifndef RBX_CPS2CRYPT_H
#define RBX_CPS2CRYPT_H

#include <stdint.h>

// rom: the program as host-order words; dec: the opcodes; length in bytes; the limits in words (inclusive)
void cps2_decrypt(const uint16_t *rom, uint16_t *dec, int length, const uint32_t *master_key, uint32_t lower_limit,
                  uint32_t upper_limit);

#endif

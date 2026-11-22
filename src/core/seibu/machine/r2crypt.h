/*  The Raiden II / Zero Team sprite ROM decryption (the SEI252's).
    Ported from MAME 0.289 src/mame/seibu/r2crypt.cpp / r2crypt.h (BSD-3-Clause, copyright-holders: Andreas Naive,
    Olivier Galibert) and src/mame/seibu/seibu_helper.cpp (BSD-3-Clause, copyright-holders: Ville Linde, Nicola
    Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_R2CRYPT_H
#define RBX_R2CRYPT_H

#include <stdint.h>

// the region's little-endian 32-bit words (ROM_REGION32_LE), `size` of them, decrypted in place
void raiden2_decrypt_sprites(uint8_t *data, uint32_t size);
void zeroteam_decrypt_sprites(uint8_t *data, uint32_t size);

#endif

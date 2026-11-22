/*  Capcom Kabuki Z80 decryption (MAME src/mame/capcom/kabuki.cpp, BSD-3-Clause, copyright-holders: Nicola Salmoria):
    the Mitchell boards and Capcom Baseball. The opcodes go to dst, the data are decoded in place.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_CAPCOM_KABUKI_H
#define RBX_CAPCOM_KABUKI_H
#include <stdint.h>
// mitchell_decode: 0x0000-0x7fff, then the 0x4000 banks from 0x10000 (decoded as at 0x8000)
void kabuki_mitchell_decode(uint8_t *src, uint8_t *dst, uint32_t size, uint32_t k1, uint32_t k2, int ak, int xk);
#endif

/*  Sega's encrypted Z80s: the opcode / data decryption of 0x0000-0x7fff (MAME src/devices/machine/segacrpt_device.cpp
    and segacrp2_device.cpp, BSD-3-Clause, copyright-holders: Nicola Salmoria, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEGACRPT_H
#define RBX_SEGACRPT_H

#include <stdint.h>

// part "315-5051" ... "317-0007": rom[0..0x7fff] decrypted in place (the data), the opcodes to opcodes[0..0x7fff];
// -1: an unknown part (nothing done)
int segacrpt_decode(const char *part, uint8_t *rom, uint8_t *opcodes);

#endif

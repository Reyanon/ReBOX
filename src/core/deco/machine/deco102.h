/*  Data East custom chip 102: the encrypted 68000 (deco102.c).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECO102_H
#define RBX_DECO102_H
#include <stdint.h>

// rom: the program as host-order 16-bit words (m68k_bus_swap16 done), decrypted in place to the data the CPU reads;
// opcodes: size bytes, the words it runs. 0 = ok
int deco102_decrypt_cpu(uint16_t *rom, uint16_t *opcodes, int size, int address_xor, int data_select_xor, int opcode_select_xor);
#endif

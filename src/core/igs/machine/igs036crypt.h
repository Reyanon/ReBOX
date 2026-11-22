/*  IGS036 program ROM decryption, ported from MAME src/mame/igs/igs036crypt.cpp (BSD-3-Clause, copyright-holders:
    Andreas Naive, David Haywood). Only the decryptor: the PGM2 games get their 256-byte key from the internal ROM
    at run time. This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_IGS036CRYPT_H
#define RBX_IGS036CRYPT_H

#include <stdint.h>

// MAME igs036_decryptor(key).decrypter_rom(rom, size, offset): the words of rom (size bytes) decrypted in place, the
// first one as word address `offset`
void igs036_decrypt_rom(const uint8_t key[0x100], uint16_t *rom, int size, int offset);
// ReBOX: its inverse (the reset puts the encrypted ROM back)
void igs036_encrypt_rom(const uint8_t key[0x100], uint16_t *rom, int size, int offset);

#endif

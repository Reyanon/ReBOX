/*  Seibu SPI graphics decryption: the SEI252 / RISE10 / RISE11 sprite ROMs (MAME 0.289 src/mame/seibu/seibuspi_m.cpp,
    BSD-3-Clause, copyright-holders: Ville Linde, Nicola Salmoria) and the tile ROMs (seibuspi_v.cpp decrypt_text /
    decrypt_bg, BSD-3-Clause, copyright-holders: Ville Linde, hap, Nicola Salmoria), with the partial carry sum of
    seibu_helper.cpp (BSD-3-Clause, copyright-holders: Ville Linde, Nicola Salmoria).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_SEIBUSPI_M_H
#define RBX_SEIBUSPI_M_H

#include <stdint.h>

void seibuspi_sprite_decrypt(uint8_t *src, int rom_size);
void seibuspi_rise10_sprite_decrypt(uint8_t *rom, int size);
void seibuspi_rise11_sprite_decrypt_rfjet(uint8_t *rom, int size);
// the tile keys (key1, key2, key3) the games write to the CRTC: text = the 0x30000 "chars", bg = "tiles"
void seibuspi_text_decrypt(uint8_t *rom, uint32_t key1, uint32_t key2, uint32_t key3);
void seibuspi_bg_decrypt(uint8_t *rom, int size, uint32_t key1, uint32_t key2, uint32_t key3);

#endif

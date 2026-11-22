/*  NMK214 graphics descrambler (MAME src/mame/nmk/nmk214.cpp, BSD-3-Clause). This file is part of ReBOX.
    GPL-2.0-or-later.
*/
#ifndef RBX_NMK214_H
#define RBX_NMK214_H

#include <stdint.h>

typedef struct {
	int mode;                    // 0 words (sprites), 1 bytes (background)
	const uint8_t *addr_bitswap; // 13 input address lines
	uint8_t init_config, initialized;
} nmk214_t;

uint16_t nmk214_decode_word(const nmk214_t *n, uint32_t addr, uint16_t data);
uint8_t nmk214_decode_byte(const nmk214_t *n, uint32_t addr, uint8_t data);
void nmk214_set_init_config(nmk214_t *n, uint8_t init_config);   // (taken when its bit 3 matches the mode)

#endif

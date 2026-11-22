/*  Hitachi FD1094 decryption (MAME src/mame/sega/fd1094.cpp, BSD-3-Clause). This file is part of ReBOX.
    GPL-2.0-or-later.
*/
#ifndef RBX_FD1094_H
#define RBX_FD1094_H

#include <stdint.h>

enum { FD1094_STATE_RESET = 0x100, FD1094_STATE_IRQ = 0x200, FD1094_STATE_RTE = 0x300 };

typedef struct {
	// saved
	uint8_t st, irqmode;
	// configuration
	const uint16_t *rom;   // the program (host-order words)
	uint32_t size;         // bytes
	const uint8_t *key;    // 8 KB
	uint16_t *cache[256];  // the opcodes decrypted for a state (on first use)
} fd1094_t;

void fd1094_init(fd1094_t *f, const uint16_t *rom, uint32_t size, const uint8_t *key);
void fd1094_exit(fd1094_t *f);
uint8_t fd1094_state(const fd1094_t *f);            // the state in force (the irq state in irq mode)
void fd1094_change_state(fd1094_t *f, int newstate);
const uint16_t *fd1094_opcodes(fd1094_t *f, uint8_t state);   // NULL: out of memory

#endif

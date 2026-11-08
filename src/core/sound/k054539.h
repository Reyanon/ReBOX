/*  Konami 054539 PCM sound chip, ported to C from MAME src/devices/sound/k054539.cpp (BSD-3-Clause,
    copyright-holders: Olivier Galibert). 8 channels of 8-bit / 16-bit PCM or 4-bit DPCM from a ROM, a 0x8000 byte
    reverb RAM, a programmable timer. Output: stereo at clock / 384 (18.432 MHz: 48 kHz).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_K054539_H
#define RBX_K054539_H

#include <stdint.h>

#define K054539_REVERSE_STEREO 1
#define K054539_DISABLE_REVERB 2
#define K054539_UPDATE_AT_KEYON 4

typedef struct k054539_s k054539_t;
struct k054539_s {
	// ---- state (k054539_state) ----
	uint8_t regs[0x230];
	uint8_t posreg_latch[8][3];
	int32_t flags, reverb_pos, cur_ptr;
	uint32_t rom_addr, timer_state;
	struct { uint32_t pos, pfrac; int32_t val, pval; } ch[8];
	uint32_t timer_div, timer_acc;   // the timer: (38 + reg 0x227) a sample against 7200 a half period (0: off)
	uint8_t ram[0x8000];
	// ---- configuration ----
	const uint8_t *rom;
	uint32_t rom_size;
	double voltab[256], pantab[0xf], gain[8];
	void (*timer_cb)(void *user, int state);   // the timer output (MAME timer_handler)
	void *user;
};

void k054539_init(k054539_t *k, const uint8_t *rom, uint32_t rom_size);
void k054539_set_gain(k054539_t *k, int channel, double gain);
void k054539_reset(k054539_t *k);
void k054539_write(k054539_t *k, uint32_t offset, uint8_t data);
uint8_t k054539_read(k054539_t *k, uint32_t offset);
// n samples (48 kHz at 18.432 MHz) added to l[] and r[] (16-bit full scale), the timer stepped with them
void k054539_render(k054539_t *k, int32_t *l, int32_t *r, int n);
// the samples to the timer's next toggle (it fires after that many render samples; -1: off)
int k054539_timer_due(const k054539_t *k);
int k054539_state(k054539_t *k, void *ctx, int load, const char *prefix);   // (prefix: the chip's tags)

#endif

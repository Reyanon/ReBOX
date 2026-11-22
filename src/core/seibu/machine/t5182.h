/*  Seibu / Toshiba T5182 sound MCU: a Z80 with its 8 KB internal ROM and 2 KB RAM, 256 bytes of RAM shared with the
    main CPU (with semaphores), the main CPU's and the YM2151's interrupts, the coin inputs; the board's YM2151 (rbx_ym
    chip 0) on its I/O ports.
    Ported from MAME 0.289 src/mame/seibu/t5182.cpp / t5182.h (BSD-3-Clause, copyright-holders: Jonathan Gevaryahu).
    This file is part of ReBOX. GPL-2.0-or-later.

    One instance. The main CPU's synchronizes are immediate (the board interleaves a scanline at a time).
*/
#ifndef RBX_T5182_H
#define RBX_T5182_H

#include <stdint.h>

#define T5182_CLOCK 3579545   // 14.318181 MHz / 4 (the Z80 and the YM2151)

// cpu: the internal ROM ("t5182:cpu", 0x2000), ext: "t5182:external" (0x8000 from 0x8000); coin_port: "T5182_COIN";
// ym_gain: each of the YM2151's two outputs to the mono mix (1/256). 0 = ok
int t5182_init(const uint8_t *cpu, const uint8_t *ext, uint32_t ext_size, int coin_port, int ym_gain);
void t5182_exit(void);
void t5182_reset(void);
// the main CPU's side
uint8_t t5182_sharedram_r(int offset);
void t5182_sharedram_w(int offset, uint8_t data);
void t5182_sound_irq_w(void);
uint8_t t5182_semaphore_snd_r(void);
void t5182_semaphore_main_acquire_w(void);
void t5182_semaphore_main_release_w(void);
// runs the Z80 `cycles` clocks
void t5182_run(int cycles);
// the YM2151 advanced `clocks` input clocks, its samples added to out (the count)
int t5182_sound(uint32_t clocks, int32_t *out, int max);
int t5182_state(void *ctx, int load);

#endif

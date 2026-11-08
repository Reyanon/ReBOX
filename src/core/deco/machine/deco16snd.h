/*  The Data East 16-bit boards' sound board (MAME: the h6280 sound_map of supbtime.cpp, dietgo.cpp, cninja.cpp ...):
    a HuC6280 with its ROM at 0 and RAM at 0x1f0000, a YM2151 (0x110000; its IRQ on the CPU's IRQ2, its CT port
    banking the OKIs on some boards), a YM2203 on some (0x100000), one or two OKI M6295 (0x120000, 0x130000) and the main CPU's sound latch
    (0x140000; a generic latch on the CPU's IRQ1, or a protection chip's). One instance; the board runs it a line at a
    time and takes the frame's samples at its end.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_DECO16SND_H
#define RBX_DECO16SND_H
#include <stdint.h>
#include "../../sound/okim6295.h"

typedef struct {
	uint32_t cpu_clock;              // the HuC6280
	uint32_t ym_clock;               // the YM2151
	int ym_gain[2];                  // its routes, 1/256 (MAME add_route: 0 left, 1 right)
	const uint8_t *oki_rom[2];       // [1] NULL: one OKI
	uint32_t oki_size[2], oki_clock[2];
	int oki_gain[2];                 // 1/256, to both sides
	int stereo;                      // 0: one speaker (the YM's two routes added)
	void (*ym_port)(uint8_t data);   // the YM's CT port (NULL: none)
	uint8_t (*latch_r)(void);        // the latch's read (NULL: the generic latch here, deco16snd_latch_w)
	uint32_t ym2203_clock;           // a YM2203 at 0x100000 (0: none), to both sides
	int ym2203_gain[4];              // its routes (rbx_ym_config.gain: SSG 0..2, FM)
	int timer_scale;                 // MAME set_timer_scale of the HuC6280 (0: 1)
} deco16snd_cfg;

int deco16snd_init(const uint8_t *rom, uint32_t rom_size, const deco16snd_cfg *cfg, uint32_t frame_mhz, int lines);
void deco16snd_exit(void);
void deco16snd_reset(void);
// the main CPU's write of the generic latch (its IRQ1 until the sound CPU reads it)
void deco16snd_latch_w(uint8_t v);
// a protection chip's latch: its IRQ1 line
void deco16snd_latch_irq(int state);
// one line: the sound CPU, the YM's clocks, the OKIs mixed in
void deco16snd_line(void);
// the frame's samples to rbx_audio
void deco16snd_frame_end(void);
okim6295_t *deco16snd_oki(int i);
int deco16snd_state(void *ctx, int load);
#endif

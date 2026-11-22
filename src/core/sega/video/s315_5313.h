/*  Sega 315-5313 (the Mega Drive VDP) as System 18 uses it: the ports, DMA, the status and HV counter, a line rendered
    into raw pixels (MAME's m_render_line_raw). Ported from MAME src/mame/sega/315_5313.cpp (BSD-3-Clause,
    copyright-holders: David Haywood), alternate timing (a callback each scanline), no interlace mode 3, no PSG.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#ifndef RBX_S315_5313_H
#define RBX_S315_5313_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	// state (saved: from here to state_end)
	uint16_t vram[0x8000];
	uint16_t cram[0x40], vsram[0x40], regs[0x20];
	uint16_t sat[0x200];           // (the internal copy of the sprite attribute table)
	uint16_t palette_lookup[0x40];
	uint16_t command_part1, command_part2, vdp_address, vram_fill_length;
	uint8_t command_pending, vdp_code, vram_fill_pending, imode_odd_frame;
	uint8_t sprite_collision, irq6_pending, irq4_pending, vblank_flag, imode, pad[3];
	int32_t irq4counter, visible_scanlines, irq6_scanline, z80irq_scanline, total_scanlines;
	uint8_t state_end;
	// configuration
	void *user;
	uint16_t (*read68k)(void *user, uint32_t addr);   // the DMA's reads of the 68000's space
	void (*spin)(void *user, int ns);                 // the 68000 held for a DMA (spin_until_time)
	void (*cram_w)(void *user, int index, uint16_t rgb9);   // a colour written (the board's palette)
} s315_5313_t;

#define S315_5313_STATE_SIZE offsetof(s315_5313_t, state_end)

void s315_5313_init(s315_5313_t *v);    // (state cleared; set the callbacks after)
void s315_5313_reset(s315_5313_t *v);   // device_reset + device_reset_old
uint16_t s315_5313_r(s315_5313_t *v, int offset, uint16_t mem_mask, int vpos, int hpos);
void s315_5313_w(s315_5313_t *v, int offset, uint16_t data, uint16_t mem_mask);
// megadriv_scanline_timer_callback_alt_timing: at the start of each of the screen's lines
void s315_5313_scanline(s315_5313_t *v, int vpos);
// render_scanline: the line's 320 raw pixels (0x100 not the backdrop, 0x80 sprite / 0x40 tile, the colour in 0x3f);
// returns 0 when the line is outside the visible ones (raw untouched)
int s315_5313_render_line(s315_5313_t *v, int vpos, uint16_t *raw);
// the MD palette (sega315_5313_palette): 512 x normal, shadow, hilight as 0xRRGGBB
uint32_t s315_5313_rgb(int index);

#endif

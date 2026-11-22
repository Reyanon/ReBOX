/*  IGS PGM2, ported from MAME src/mame/igs/pgm2.cpp, pgm2_v.cpp, pgm2.h (BSD-3-Clause, copyright-holders: David
    Haywood, Xing Xing, Andreas Naive) with the AIC of src/devices/machine/atmel_arm_aic.cpp (BSD-3-Clause,
    copyright-holders: MetalliC).
    IGS036 (ARM946E-S, cpu/arm7 with arch5) at 100 MHz: its internal ROM at 0, the cartridge's program ROM at
    0x10000000 (ddpdojt: 2 MB of RAM there, the ROM after it), decrypted in place when the program writes 0xfffffa08
    (machine/igs036crypt.c, the key from its writes to 0xfffffc00). IGS037 video: a 32x32 7bpp background with line
    scroll, an 8x8 4bpp text layer and zoomed 1bpp-masked sprites drawn into a bitmap at vblank (shown the next
    frame, as MAME's screen updates before its vblank callback). YMZ774 sound (sound/ymz774.c). The card reader MCU
    is MAME's HLE (commands only; no card inserted).
    Timing: MAME's screen (512 x 256 lines at 59.08 Hz, vblank from the end of the visible area); the ARM runs a line
    at a time, the MCU timer cuts a line where it expires.
    Not done (yet): KOV3's program module (kov3 sets refused), the memory cards, the speedup handlers (the core's idle
    loop skip instead), the TCMs (arm7.h tcm_use).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"

#include "../../cpu/arm7/arm7.h"
#include "../machine/igs036crypt.h"
#include "../../sound/ymz774.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 100000000u
#define LINES 256
#define FRAME_MHZ 59080u        // MAME: HZ_TO_ATTOSECONDS(59.08)
#define FRAME_MHZ_LORES 59186u  // 15625 / 264 (kof98umh)
#define SCR_W 512
#define SCR_H 256

#define PAL_SP 0x0000   // the three palettes in rbx_video.palette
#define PAL_BG 0x1000
#define PAL_TX 0x1800
#define PAL_N 0x1a00

// ---- the board ----
typedef struct {
	// (saved: the block from here to `end_saved`)
	uint32_t mainram[0x80000 / 4];
	uint32_t sram[0x10000 / 4];
	uint32_t sp_videoram[0x2000 / 4];
	uint32_t bg_videoram[0x2000 / 4];
	uint32_t fg_videoram[0x6000 / 4];
	uint32_t sp_palette[0x4000 / 4];
	uint32_t bg_palette[0x2000 / 4];
	uint32_t tx_palette[0x800 / 4];
	uint32_t sp_zoom[0x200 / 4];
	uint32_t lineram[0x400 / 4];
	uint32_t bgscroll, fgscroll, vidmode;
	uint8_t encryption_table[0x100];
	uint8_t has_decrypted, pad0[3];
	uint32_t spritekey, realspritekey;
	uint8_t shareram[0x100];
	uint16_t share_bank, pad1;
	uint32_t mcu_regs[8];
	uint32_t mcu_result0, mcu_result1;
	uint8_t mcu_last_cmd, pad2[3];
	int64_t mcu_timer;   // the ARM's total_cycles when the MCU timer fires (-1: off)
	// AIC
	uint32_t irqs_enabled, irqs_pending, current_irq_vector, current_firq_vector, status, core_status;
	uint32_t spurious_vector, debug, fast_irqs;
	int32_t lvlidx, level_stack[9];
	uint32_t aic_smr[32], aic_svr[32];
	uint32_t frames;    // (rtc_r: MAME's machine time in seconds)
	uint32_t tcm[2];    // (the ARM's TCM registers)
	uint64_t frac;
	int32_t vis_w, vis_h;
	uint32_t end_saved;
	uint16_t sprite_bitmap[SCR_W * SCR_H];   // (saved too: the sprites of the last vblank)
} pgm2_t;

static pgm2_t *s;
static arm7_t *s_arm;
static ymz774_t *s_ymz;
static uint8_t *s_irom, *s_mainrom, *s_romboard_ram;
static uint32_t s_irom_size, s_mainrom_size, s_romboard_size;
static const uint8_t *s_mask, *s_colour, *s_tiles, *s_bgtile;
static uint32_t s_mask_len, s_colour_len, s_tiles_size, s_bgtile_size;
static int s_port[2];
static int s_lores, s_hires, s_ramrom;
static uint32_t s_frame_mhz;
static char s_nv_path[256];
static int s_line;
static uint32_t s_line_end;   // (the ARM's cycles of the frame where the current line ends)
static int64_t s_frame_start;
static int16_t *s_snd;
static uint32_t s_snd_frac;
static int s_snd_n, s_snd_done;   // (the frame's samples, those made so far)
static uint32_t s_frame_cycles;   // (the ARM's cycles in this frame)

// the sound up to the ARM's time (MAME's stream update before a chip access)
static void snd_sync(int all)
{
	if (!s_ymz)
		return;
	int want = s_snd_n;
	if (!all && s_frame_cycles) {
		const int64_t now = s_arm->total_cycles + (s_arm->running ? (s_arm->slice - s_arm->icount) : 0) - s_frame_start;
		want = now <= 0 ? 0 : (int)((uint64_t)now * (uint64_t)s_snd_n / s_frame_cycles);
		if (want > s_snd_n)
			want = s_snd_n;
	}
	while (s_snd_done < want) {
		const int k = want - s_snd_done > 4096 ? 4096 : want - s_snd_done;
		ymz774_render(s_ymz, s_snd, k);
		for (int i = 0; i < k; i++)
			rbx_audio_put_stereo(s_snd[i * 2], s_snd[i * 2 + 1]);
		s_snd_done += k;
	}
}

// ---- AIC (atmel_arm_aic) ----
static int aic_bit_width(uint32_t m) { int n = 0; while (m) { n++; m >>= 1; } return n; }
static int aic_level(void) { return s->level_stack[s->lvlidx]; }
static void aic_set_lines(void)
{
	arm7_set_irq_line(s_arm, ARM7_IRQ_LINE, (s->core_status & ~s->debug & 2) ? 1 : 0);   // (TODO FIQ, as MAME)
}
static void aic_check(void)
{
	s->core_status = 0;
	uint32_t mask = s->irqs_enabled & s->irqs_pending & ~s->fast_irqs;
	if (mask) {
		const int pri = aic_level();
		do {
			const int idx = aic_bit_width(mask) - 1;
			if ((int)(s->aic_smr[idx] & 7) > pri) {
				s->core_status |= 2;
				break;
			}
			mask ^= 1u << idx;
		} while (mask);
	}
	if (s->irqs_enabled & s->irqs_pending & s->fast_irqs)
		s->core_status |= 1;
	aic_set_lines();
}
static void aic_set_irq(int line, int state)
{
	if (state)
		s->irqs_pending |= 1u << line;
	else if (s->aic_smr[line] & 0x40)
		s->irqs_pending &= ~(1u << line);
	aic_check();
}
static uint32_t aic_irq_vector_r(void)
{
	uint32_t mask = s->irqs_enabled & s->irqs_pending & ~s->fast_irqs;
	s->current_irq_vector = s->spurious_vector;
	if (mask) {
		int pri = aic_level();
		int midx = -1;
		do {
			const int idx = aic_bit_width(mask) - 1;
			if ((int)(s->aic_smr[idx] & 7) >= pri) {
				midx = idx;
				pri = (int)(s->aic_smr[idx] & 7);
			}
			mask ^= 1u << idx;
		} while (mask);
		if (midx > 0) {
			s->status = (uint32_t)midx;
			s->current_irq_vector = s->aic_svr[midx];
			if (s->lvlidx < 8)
				s->level_stack[++s->lvlidx] = (int)(s->aic_smr[midx] & 7);
			if (s->aic_smr[midx] & 0x20)   // auto clear pending if edge trigger mode
				s->irqs_pending ^= 1u << midx;
		}
	}
	s->core_status &= ~2u;
	aic_set_lines();
	return s->current_irq_vector;
}
static void aic_reset(void)
{
	s->irqs_enabled = s->irqs_pending = s->current_irq_vector = s->current_firq_vector = 0;
	s->status = s->core_status = s->spurious_vector = s->debug = 0;
	s->fast_irqs = 1;
	s->lvlidx = 0;
	memset(s->aic_smr, 0, sizeof(s->aic_smr));
	memset(s->aic_svr, 0, sizeof(s->aic_svr));
	memset(s->level_stack, 0, sizeof(s->level_stack));
	s->level_stack[0] = -1;
}
static uint32_t aic_r(uint32_t o)
{
	if (o < 0x80) return s->aic_smr[o >> 2];
	if (o < 0x100) return s->aic_svr[(o - 0x80) >> 2];
	switch (o) {
	case 0x100: return aic_irq_vector_r();
	case 0x104:
		s->current_firq_vector = (s->irqs_enabled & s->irqs_pending & s->fast_irqs) ? s->aic_svr[0] : s->spurious_vector;
		return s->current_firq_vector;
	case 0x108: return s->status;
	case 0x10c: return s->irqs_pending;
	case 0x110: return s->irqs_enabled;
	case 0x114: return s->core_status;
	case 0x148: return s->fast_irqs;
	}
	return 0;
}
static void aic_w(uint32_t o, uint32_t d, uint32_t mask)
{
	if (o < 0x80) { s->aic_smr[o >> 2] = (s->aic_smr[o >> 2] & ~mask) | (d & mask); return; }
	if (o < 0x100) { s->aic_svr[(o - 0x80) >> 2] = (s->aic_svr[(o - 0x80) >> 2] & ~mask) | (d & mask); return; }
	d &= mask;
	switch (o) {
	case 0x120: s->irqs_enabled |= d; aic_check(); break;
	case 0x124: s->irqs_enabled &= ~d; aic_check(); break;
	case 0x128: s->irqs_pending &= ~d; aic_check(); break;
	case 0x12c: s->irqs_pending |= d; aic_check(); break;
	case 0x130: s->status = 0; if (s->lvlidx) s->lvlidx--; aic_check(); break;
	case 0x134: s->spurious_vector = (s->spurious_vector & ~mask) | d; break;
	case 0x138: s->debug = (s->debug & ~mask) | d; aic_check(); break;
	case 0x140: s->fast_irqs |= d; aic_check(); break;
	case 0x144: s->fast_irqs &= ~d | 1; aic_check(); break;
	}
}

// ---- MCU (MAME's HLE: no card present) ----
static void mcu_timer_after_us(uint32_t us)
{
	s->mcu_timer = s_arm->total_cycles + (s_arm->running ? (s_arm->slice - s_arm->icount) : 0) +
	               (int64_t)us * (CPU_CLOCK / 1000000u);
	if (s_arm->running)
		arm7_end_timeslice(s_arm);   // (the line is cut where the timer fires)
}
static void mcu_command(int is_command)
{
	const uint8_t cmd = (uint8_t)(s->mcu_regs[0] & 0xff);
	if (is_command) {
		s->mcu_last_cmd = cmd;
		uint8_t status = 0xf7;   // "command accepted" status
		switch (cmd) {
		case 0xf6:   // get result
			s->mcu_regs[3] = s->mcu_result0;
			s->mcu_regs[4] = s->mcu_result1;
			s->mcu_last_cmd = 0;
			break;
		case 0xe0:   // command port test
			s->mcu_result0 = s->mcu_regs[0];
			s->mcu_result1 = s->mcu_regs[1];
			break;
		case 0xe1: {   // shared RAM access (unimplemented)
			const uint8_t mode = (uint8_t)(s->mcu_regs[0] >> 16);
			const uint8_t data = (uint8_t)(s->mcu_regs[0] >> 24);
			if (mode == 2)
				memset(&s->shareram[(~s->share_bank & 1) * 128], data, 128);
			s->mcu_result0 = cmd;
			s->mcu_result1 = 0;
			break;
		}
		case 0xc0: case 0xc1: case 0xc2: case 0xc3: case 0xc4: case 0xc5: case 0xc6: case 0xc7: case 0xc8: case 0xc9:
			// IC card commands: no card in the reader (MAME memcard present() == -1)
			if (cmd == 0xc2 || cmd == 0xc3) {
				if ((uint8_t)(s->mcu_regs[0] >> 24))
					status = 0xf4;
			} else
				status = 0xf4;
			s->mcu_result0 = cmd;
			break;
		default:
			status = 0xf4;   // error
			break;
		}
		s->mcu_regs[3] = (s->mcu_regs[3] & 0xff00ffffu) | ((uint32_t)status << 16);
		mcu_timer_after_us(1000);
	} else if (s->mcu_last_cmd) {
		s->mcu_regs[3] = (s->mcu_regs[3] & 0xff00ffffu) | 0x00f20000u;   // "command done and return data" status
		mcu_timer_after_us(100);
		s->mcu_last_cmd = 0;
	}
}
static void mcu_w(uint32_t a, uint32_t d, uint32_t mask)
{
	const int reg = (int)(((a - 0x03600000u) >> 17) & 7);
	s->mcu_regs[reg] = (s->mcu_regs[reg] & ~mask) | (d & mask);
	if (reg == 2 && s->mcu_regs[2])   // irq to mcu
		mcu_command(1);
	if (reg == 5 && s->mcu_regs[5]) {   // ack to mcu (written at the end of irq handler routine)
		mcu_command(0);
		aic_set_irq(3, 0);
	}
}

// ---- encryption ----
static void decrypt_all(void)
{
	if (s_romboard_ram) {
		igs036_decrypt_rom(s->encryption_table, (uint16_t *)s_romboard_ram, (int)s_romboard_size, 0);
		igs036_decrypt_rom(s->encryption_table, (uint16_t *)s_mainrom, (int)s_mainrom_size, (int)(s_romboard_size / 2));
	} else
		igs036_decrypt_rom(s->encryption_table, (uint16_t *)s_mainrom, (int)s_mainrom_size, 0);
	arm7_jit_reset();
}

// ddpdojt runs its program from the cartridge RAM it copied it into: once it is decrypted the RAM is mapped read-only,
// so that the ARM's code there is translated (arm7_jit.inc translates read-only pages only); a write still lands, through
// the callbacks, and throws the translated code away
static uint32_t s_romram_writes;
static void romram_protect(int on)
{
	if (s_romboard_ram)
		arm7_map(s_arm, 0x10000000, 0x101fffff, s_romboard_ram, !on);
}

// ---- the bus (what arm7_map leaves to the callbacks) ----
static uint32_t rd32(void *u, uint32_t a);
static void wr32(void *u, uint32_t a, uint32_t d) ;

static uint32_t io_r32(uint32_t a)
{
	if (a >= 0x03600000u && a <= 0x036bffffu)
		return s->mcu_regs[((a - 0x03600000u) >> 17) & 7];
	if (a == 0x03900000u)
		return s_port[0] >= 0 ? rbx_port_read(s_port[0]) : 0xffffffffu;
	if (a == 0x03a00000u)
		return s_port[1] >= 0 ? rbx_port_read(s_port[1]) : 0xffffffffu;
	if (a >= 0x300c0000u && a <= 0x300c01ffu)
		return s->sp_zoom[(a >> 2) & 0x7f];
	if (a >= 0x30100000u && a <= 0x301000ffu) {   // shared RAM, bytes in lanes 0 and 2 (umask32 0x00ff00ff)
		const uint32_t o = ((a - 0x30100000u) >> 2) * 2 + (s->share_bank & 1) * 128;
		return s->shareram[o] | (uint32_t)s->shareram[o + 1] << 16;
	}
	if (a == 0x30120000u) return s->bgscroll;
	if (a == 0x30120008u) return s->fgscroll;
	if (a == 0x3012000cu) return s->vidmode;
	if (a == 0x40000000u && s_ymz)
		snd_sync(0);
	if (a == 0x40000000u)
		return s_ymz ? (uint32_t)ymz774_read(s_ymz, 0) | (uint32_t)ymz774_read(s_ymz, 1) << 8 |
		               (uint32_t)ymz774_read(s_ymz, 2) << 16 | (uint32_t)ymz774_read(s_ymz, 3) << 24 : 0;
	if (a >= 0xfffff000u && a <= 0xfffff14bu)
		return aic_r(a - 0xfffff000u);
	if (a == 0xfffffd28u)
		return s->frames / 59;   // rtc_r: MAME machine().time().seconds()
	if (a == 0xfffffa0cu)
		return 0x00000180u;      // unk_startup_r
	if (a >= 0xfffffc00u && a <= 0xfffffcffu)
		return s->encryption_table[a & 0xfc] | (uint32_t)s->encryption_table[(a & 0xfc) + 1] << 8 |
		       (uint32_t)s->encryption_table[(a & 0xfc) + 2] << 16 | (uint32_t)s->encryption_table[(a & 0xfc) + 3] << 24;
	return 0;
}

static void io_w32(uint32_t a, uint32_t d, uint32_t mask)
{
	if (a >= 0x10000000u && a <= 0x101fffffu && s_romboard_ram) {   // (the program RAM while read-only)
		uint32_t *p = (uint32_t *)(s_romboard_ram + (a & 0x1ffffc));
		const uint32_t n = (*p & ~mask) | (d & mask);
		if (n != *p) {
			*p = n;
			s_romram_writes++;
			arm7_jit_reset();
		}
		return;
	}
	if (a >= 0x03600000u && a <= 0x036bffffu) { mcu_w(a, d, mask); return; }
	if (a >= 0x300c0000u && a <= 0x300c01ffu) {
		uint32_t *p = &s->sp_zoom[(a >> 2) & 0x7f];
		*p = (*p & ~mask) | (d & mask);
		return;
	}
	if (a >= 0x30100000u && a <= 0x301000ffu) {
		const uint32_t o = ((a - 0x30100000u) >> 2) * 2 + (s->share_bank & 1) * 128;
		if (mask & 0x000000ffu) s->shareram[o] = (uint8_t)d;
		if (mask & 0x00ff0000u) s->shareram[o + 1] = (uint8_t)(d >> 16);
		return;
	}
	switch (a) {
	case 0x30120000u: s->bgscroll = (s->bgscroll & ~mask) | (d & mask); return;
	case 0x30120008u: s->fgscroll = (s->fgscroll & ~mask) | (d & mask); return;
	case 0x3012000cu: s->vidmode = (s->vidmode & ~mask) | (d & mask); return;
	case 0x30120018u:   // vbl_ack_w (0x30120018-19)
		if (mask & 0xffffu) aic_set_irq(12, 0);
		return;
	case 0x30120030u:   // share_bank_w (0x30120032-33)
		if (mask & 0xffff0000u)
			s->share_bank = (uint16_t)((s->share_bank & ~(mask >> 16)) | ((d & mask) >> 16));
		return;
	case 0x30120038u: {   // sprite_encryption_w
		s->spritekey = (s->spritekey & ~mask) | (d & mask);
		const uint32_t k = s->spritekey ^ 0x90055555u;   // bitswap<32>(.., 0, 1, .. 31): the bits reversed
		uint32_t r = 0;
		for (int i = 0; i < 32; i++)
			r |= ((k >> i) & 1u) << (31 - i);
		s->realspritekey = r;
		return;
	}
	case 0x40000000u:
		snd_sync(0);
		if (s_ymz)
			for (int i = 0; i < 4; i++)
				if (mask & (0xffu << (i * 8)))
					ymz774_write(s_ymz, (uint32_t)i, (uint8_t)(d >> (i * 8)));
		return;
	case 0xfffffa08u:   // encryption_do_w
		decrypt_all();
		s->has_decrypted = 1;
		romram_protect(1);
		return;
	}
	if (a >= 0xfffff000u && a <= 0xfffff14bu) { aic_w(a - 0xfffff000u, d, mask); return; }
	if (a >= 0xfffffc00u && a <= 0xfffffcffu) {
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (i * 8)))
				s->encryption_table[(a & 0xfc) + i] = (uint8_t)(d >> (i * 8));
		return;
	}
}

static uint32_t rd32(void *u, uint32_t a) { (void)u; return io_r32(a); }
static uint16_t rd16(void *u, uint32_t a) { (void)u; return (uint16_t)(io_r32(a & ~3u) >> ((a & 2) * 8)); }
static uint8_t rd8(void *u, uint32_t a) { (void)u; return (uint8_t)(io_r32(a & ~3u) >> ((a & 3) * 8)); }
static void wr32(void *u, uint32_t a, uint32_t d) { (void)u; io_w32(a, d, 0xffffffffu); }
static void wr16(void *u, uint32_t a, uint16_t d)
{
	(void)u;
	const unsigned sh = (a & 2) * 8;
	io_w32(a & ~3u, (uint32_t)d << sh, 0xffffu << sh);
}
static void wr8(void *u, uint32_t a, uint8_t d)
{
	(void)u;
	const unsigned sh = (a & 3) * 8;
	io_w32(a & ~3u, (uint32_t)d << sh, 0xffu << sh);
}

// ---- video (pgm2_v.cpp) ----
// At vblank the emulation thread copies what drawing needs (s_snap) and the render (any thread) draws the screen
// from it with the sprites of the last vblank, then the new sprites into the sprite bitmap: MAME's order (the screen
// update, then screen_vblank's draw_sprites).
typedef struct {
	uint32_t bg[0x2000 / 4], fg[0x6000 / 4], line[0x400 / 4], sp[0x2000 / 4], zoom[0x200 / 4];
	uint32_t pal_sp[0x1000], pal_bg[0x800], pal_tx[0x200];
	uint32_t bgscroll, fgscroll, key;
	int vis_w, vis_h;
} pgm2_snap;
static pgm2_snap *s_snap;
// the sprite bitmaps: the front one (the sprites of the last vblank, shown by this render; s->sprite_bitmap or the
// second) and the back one the render draws the new sprites into, at the same time as the screen; swapped when the
// render is done. (MAME has one bitmap: only a change of the visible area could show the difference, outside the old
// area)
static uint16_t *s_spr[2];
static int s_spr_front;

static inline void sprite_pixel(const pgm2_snap *v, uint32_t palette_offset, int realx, int realy, uint16_t pal)
{
	realx = (int16_t)realx;   // (MAME: an s16 parameter)
	if (realx >= 0 && realx < v->vis_w && realy >= 0 && realy < v->vis_h) {
		const uint16_t pix = s_colour[palette_offset] & 0x3f;   // there are some stray 0xff bytes in some roms
		s_spr[s_spr_front ^ 1][realy * SCR_W + realx] = (uint16_t)(pix + pal * 0x40);
	}
}

static inline void sprite_chunk(const pgm2_snap *v, uint32_t *palette_offset, int x, int realy, uint16_t pal,
                                uint32_t maskdata, uint32_t zoomx_bits, uint8_t repeats, int *realxdraw,
                                int realdraw_inc, int palette_inc)
{
	const uint32_t cmask = s_colour_len - 1;
	if (repeats == 0) {   // shrink
		if (!maskdata) {   // (no pixel: the zoom bits move the position only)
			*realxdraw += realdraw_inc * __builtin_popcount(zoomx_bits);
			return;
		}
		uint32_t po = *palette_offset;
		int rx = *realxdraw;
		for (int xchunk = 0; xchunk < 32; xchunk++) {
			const int b = palette_inc == -1 ? xchunk : 31 - xchunk;
			const uint32_t pix = (maskdata >> b) & 1, xzoombit = (zoomx_bits >> b) & 1;
			if (pix) {
				if (xzoombit)
					sprite_pixel(v, po, x + rx, realy, pal);
				po = (po + (uint32_t)palette_inc) & cmask;
			}
			if (xzoombit)
				rx += realdraw_inc;
		}
		*palette_offset = po;
		*realxdraw = rx;
		return;
	}
	for (int xchunk = 0; xchunk < 32; xchunk++) {   // grow
		const int b = palette_inc == -1 ? xchunk : 31 - xchunk;
		const uint32_t pix = (maskdata >> b) & 1, xzoombit = (zoomx_bits >> b) & 1;
		if (pix) {
			for (int i = 0; i < repeats; i++) {
				sprite_pixel(v, *palette_offset, x + *realxdraw, realy, pal);
				*realxdraw += realdraw_inc;
			}
			if (xzoombit) {
				sprite_pixel(v, *palette_offset, x + *realxdraw, realy, pal);
				*realxdraw += realdraw_inc;
			}
			*palette_offset = (*palette_offset + (uint32_t)palette_inc) & cmask;
		} else
			*realxdraw += realdraw_inc * (repeats + (int)xzoombit);
	}
}

static inline uint32_t mask_word(const pgm2_snap *v, uint32_t mo)
{
	const uint32_t m = s_mask_len - 1;
	if (mo + 3 <= m)
		return ((uint32_t)s_mask[mo] << 24 | (uint32_t)s_mask[mo + 1] << 16 | (uint32_t)s_mask[mo + 2] << 8 |
		        s_mask[mo + 3]) ^ v->key;
	return ((uint32_t)s_mask[mo] << 24 | (uint32_t)s_mask[(mo + 1) & m] << 16 | (uint32_t)s_mask[(mo + 2) & m] << 8 |
	        s_mask[(mo + 3) & m]) ^ v->key;
}

static void sprite_line(const pgm2_snap *v, uint32_t *mask_offset, uint32_t *palette_offset, int x, int realy,
                        int flipx, int reverse, uint16_t sizex, uint16_t pal, uint32_t zoomx_bits, uint8_t xrepeats,
                        int band_y0, int band_y1)
{
	if (realy < band_y0 || realy >= band_y1) {   // (nothing drawn: the offsets move as the drawing moves them)
		uint32_t mo = *mask_offset, po = *palette_offset;
		for (int xdraw = 0; xdraw < sizex; xdraw++) {
			const uint32_t n = (uint32_t)__builtin_popcount(mask_word(v, mo));
			po = (reverse ? po - n : po + n) & (s_colour_len - 1);
			mo = (reverse ? mo - 4 : mo + 4) & (s_mask_len - 1);
		}
		*mask_offset = mo;
		*palette_offset = po;
		return;
	}
	int realxdraw = 0;
	if (flipx ^ reverse)
		realxdraw = (int16_t)(__builtin_popcount(zoomx_bits) * sizex - 1);
	for (int xdraw = 0; xdraw < sizex; xdraw++) {
		const uint32_t mo = *mask_offset;
		const uint32_t maskdata = mask_word(v, mo);
		*mask_offset = (reverse ? mo - 4 : mo + 4) & (s_mask_len - 1);
		if (!flipx) {
			if (!reverse) sprite_chunk(v, palette_offset, x, realy, pal, maskdata, zoomx_bits, xrepeats, &realxdraw, 1, 1);
			else sprite_chunk(v, palette_offset, x, realy, pal, maskdata, zoomx_bits, xrepeats, &realxdraw, -1, -1);
		} else {
			if (!reverse) sprite_chunk(v, palette_offset, x, realy, pal, maskdata, zoomx_bits, xrepeats, &realxdraw, -1, 1);
			else sprite_chunk(v, palette_offset, x, realy, pal, maskdata, zoomx_bits, xrepeats, &realxdraw, 1, -1);
		}
		realxdraw = (int16_t)realxdraw;
	}
}

// the sprites' lines [y0, y1) of the visible area (a band: the pixels of each line are drawn as the whole list draws
// them, so bands drawn apart give the same bitmap)
static void draw_sprites(const pgm2_snap *v, int y0, int y1)
{
	for (int y = y0; y < y1; y++)
		for (int x = 0; x < v->vis_w; x++)
			s_spr[s_spr_front ^ 1][y * SCR_W + x] = 0x8000;
	const uint32_t *sv = v->sp;
	int endoflist = -1;
	for (int i = 0; i < 0x2000 / 4; i += 4)
		if (sv[i + 2] & 0x80000000u) {
			endoflist = i;
			break;
		}
	if (endoflist <= 0)
		return;
	for (int i = 0; i < endoflist - 2; i += 4) {
		int x = (int)(sv[i + 0] & 0x000007ffu);
		int y = (int)((sv[i + 0] & 0x003ff800u) >> 11);
		uint16_t pal = (uint16_t)((sv[i + 0] & 0x0fc00000u) >> 22);
		const uint8_t pri = (uint8_t)(sv[i + 0] >> 31);
		if (sv[i + 0] & 0x40000000u)   // disable
			continue;
		const uint16_t sizex = (uint16_t)(sv[i + 1] & 0x3f);
		const uint16_t sizey = (uint16_t)((sv[i + 1] & 0x7fc0) >> 6);
		const int flipx = (sv[i + 1] >> 23) & 1;
		const int reverse = (sv[i + 1] >> 31) & 1;
		const uint8_t zoomx = (uint8_t)((sv[i + 1] >> 16) & 0x7f);
		const uint8_t zoomy = (uint8_t)((sv[i + 1] >> 24) & 0x7f);
		uint32_t mask_offset = sv[i + 2] << 1;
		uint32_t palette_offset = sv[i + 3];
		const uint32_t zoomy_bits = v->zoom[zoomy];
		const uint32_t zoomx_bits = v->zoom[zoomx];
		const uint8_t xrepeats = (uint8_t)((zoomx & 0x60) >> 5);
		const uint8_t yrepeats = (uint8_t)((zoomy & 0x60) >> 5);
		if (x & 0x400) x -= 0x800;
		if (y & 0x400) y -= 0x800;
		if (reverse)
			mask_offset -= 2;
		mask_offset &= s_mask_len - 1;
		palette_offset &= s_colour_len - 1;
		pal |= (uint16_t)(pri << 6);   // encode priority with the palette for manual mixing later
		int realy = y;
		int sourceline = 0;
		if (realy >= y1)   // (it starts below the band: its lines only go down)
			continue;
		for (int ydraw = 0; ydraw < sizey && realy < y1; sourceline++) {
			const uint32_t zoomy_bit = (zoomy_bits >> (sourceline & 0x1f)) & 1;
			const uint32_t pre_palette_offset = palette_offset, pre_mask_offset = mask_offset;
			if (yrepeats != 0) {   // grow
				for (int k = 0; k < yrepeats; k++) {
					palette_offset = pre_palette_offset;
					mask_offset = pre_mask_offset;
					sprite_line(v, &mask_offset, &palette_offset, x, (int16_t)realy, flipx, reverse, sizex, pal, zoomx_bits, xrepeats, y0, y1);
					realy++;
				}
				if (zoomy_bit) {
					palette_offset = pre_palette_offset;
					mask_offset = pre_mask_offset;
					sprite_line(v, &mask_offset, &palette_offset, x, (int16_t)realy, flipx, reverse, sizex, pal, zoomx_bits, xrepeats, y0, y1);
					realy++;
				}
			} else {   // shrink
				sprite_line(v, &mask_offset, &palette_offset, x, (int16_t)realy, flipx, reverse, sizex, pal, zoomx_bits, xrepeats, y0, y1);
				if (zoomy_bit)
					realy++;
			}
			ydraw++;
		}
	}
}

// one line of the background (0 = transparent): the tiles a run at a time
static void bg_line(const pgm2_snap *v, int y, uint16_t *out)
{
	const uint32_t ntiles = s_bgtile_size / 1024;
	const uint32_t ls = (y & 1) ? (v->line[y >> 1] >> 16) : (v->line[y >> 1] & 0xffff);
	const uint32_t sy = ((uint32_t)y + (v->bgscroll >> 16)) & 1023;
	const uint32_t sx0 = (v->bgscroll & 0xffff) + ls;
	const uint32_t *row = v->bg + (sy >> 5) * 64;
	for (int x = 0; x < v->vis_w;) {
		const uint32_t sx = (sx0 + (uint32_t)x) & 2047;
		const uint32_t t = row[sx >> 5];
		const uint32_t px = sx & 31;
		int n = 32 - (int)px;
		if (n > v->vis_w - x)
			n = v->vis_w - x;
		if (!ntiles) {
			memset(out + x, 0, (size_t)n * 2);
			x += n;
			continue;
		}
		const uint32_t code = (t & 0x3ffff) % ntiles;
		const uint16_t base = (uint16_t)(PAL_BG + ((t >> 18) & 0xf) * 0x80);
		const uint32_t flip = (t >> 23) & 3;
		const uint8_t *src = s_bgtile + code * 1024 + ((flip & 2) ? (sy & 31) ^ 31 : (sy & 31)) * 32;
		if (flip & 1)
			for (int k = 0; k < n; k++) {
				const uint8_t p = src[(px + (uint32_t)k) ^ 31] & 0x7f;
				out[x + k] = p ? (uint16_t)(base + p) : 0;
			}
		else
			for (int k = 0; k < n; k++) {
				const uint8_t p = src[px + (uint32_t)k] & 0x7f;
				out[x + k] = p ? (uint16_t)(base + p) : 0;
			}
		x += n;
	}
}

static void fg_line(const pgm2_snap *v, int y, uint16_t *out)
{
	const uint32_t ntiles = s_tiles_size / 32;
	const uint32_t sy = ((uint32_t)y + (v->fgscroll >> 16)) % 512;
	const uint32_t *row = v->fg + (sy >> 3) * 96;
	const uint32_t fx = v->fgscroll & 0xffff;
	for (int x = 0; x < v->vis_w;) {
		const uint32_t sx = ((uint32_t)x + fx) % 768;
		const uint32_t t = row[sx >> 3];
		const uint32_t px = sx & 7;
		int n = 8 - (int)px;
		if (n > v->vis_w - x)
			n = v->vis_w - x;
		if (!ntiles) {
			memset(out + x, 0, (size_t)n * 2);
			x += n;
			continue;
		}
		const uint32_t code = (t & 0x3ffff) % ntiles;
		const uint16_t base = (uint16_t)(PAL_TX + ((t >> 18) & 0x1f) * 16);
		const uint32_t flip = (t >> 23) & 3;
		const uint8_t *src = s_tiles + code * 32 + ((flip & 2) ? (sy & 7) ^ 7 : (sy & 7)) * 4;
		for (int k = 0; k < n; k++) {
			uint32_t q = px + (uint32_t)k;
			if (flip & 1)
				q ^= 7;
			const uint8_t b = src[q >> 1];
			const uint8_t p = (q & 1) ? (b >> 4) : (b & 0xf);
			out[x + k] = p ? (uint16_t)(base + p) : 0;
		}
		x += n;
	}
}

// the screen: the bg pen, the sprites of priority 1, the background, the sprites of priority 0, the text layer
static void compose_lines(const pgm2_snap *v, int y0, int y1)
{
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	uint16_t bgl[SCR_W], fgl[SCR_W];
	const uint16_t *front = s_spr[s_spr_front];
	for (int y = y0; y < y1; y++) {
		bg_line(v, y, bgl);
		fg_line(v, y, fgl);
		const uint16_t *sp = front + y * SCR_W;
		uint16_t *dst = bm + y * pitch;
		for (int x = 0; x < v->vis_w; x++) {
			uint16_t o;
			const uint16_t spx = sp[x];
			if (fgl[x])
				o = fgl[x];
			else if (spx != 0x8000 && !(spx & 0x1000))
				o = (uint16_t)(PAL_SP + (spx & 0xfff));
			else if (bgl[x])
				o = bgl[x];
			else if (spx != 0x8000)
				o = (uint16_t)(PAL_SP + (spx & 0xfff));
			else
				o = PAL_BG;
			dst[x] = o;
		}
	}
}

// the render's pieces: 0 = the new sprites (the longest, first), 1..BANDS = bands of the screen. The render thread takes
// them, and the threads waiting for it help (pgm2_render_help: the emulation thread, the frontend's helper core)
#define BANDS 6
#define SBANDS 4   // (the sprites' bands, first: the longest pieces)
static volatile int s_piece_next, s_piece_left, s_piece_active, s_piece_helpers;
static void run_piece(int k)
{
	const pgm2_snap *v = s_snap;
	const int h = v->vis_h;
	if (k < SBANDS) {   // (screen_vblank: the new sprites, shown the next frame)
		draw_sprites(v, h * k / SBANDS, h * (k + 1) / SBANDS);
		return;
	}
	k -= SBANDS;
	const int y0 = h * k / BANDS, y1 = h * (k + 1) / BANDS;
	compose_lines(v, y0, y1);
}

static void pgm2_render(void)
{
	const pgm2_snap *v = s_snap;
	if (!s || !v)
		return;
	for (int i = 0; i < 0x1000; i++) rbx_video.palette[PAL_SP + i] = v->pal_sp[i] & 0xffffff;
	for (int i = 0; i < 0x800; i++) rbx_video.palette[PAL_BG + i] = v->pal_bg[i] & 0xffffff;
	for (int i = 0; i < 0x200; i++) rbx_video.palette[PAL_TX + i] = v->pal_tx[i] & 0xffffff;
	rbx_video.palette_used = PAL_N;
	__atomic_store_n(&s_piece_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_piece_left, BANDS + SBANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_piece_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int k = __atomic_fetch_add(&s_piece_next, 1, __ATOMIC_ACQ_REL);
		if (k >= BANDS + SBANDS)
			break;
		run_piece(k);
		__atomic_sub_fetch(&s_piece_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_piece_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_piece_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_piece_helpers, __ATOMIC_ACQUIRE) > 0)
		;
	s_spr_front ^= 1;   // (the new sprites: the next frame's)
}

static int pgm2_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_piece_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_piece_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_piece_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int k = __atomic_fetch_add(&s_piece_next, 1, __ATOMIC_ACQ_REL);
			if (k >= BANDS + SBANDS)
				break;
			run_piece(k);
			__atomic_sub_fetch(&s_piece_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_piece_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// vblank: what the render needs, copied (the render of the last frame done first)
static void screen_vblank(void)
{
	rbx_video_wait_idle();
	switch ((s->vidmode >> 16) & 3) {
	default:
	case 0: s->vis_w = 320; s->vis_h = 240; break;
	case 1: s->vis_w = 448; s->vis_h = 224; break;
	case 2: s->vis_w = 512; s->vis_h = 240; break;
	}
	pgm2_snap *v = s_snap;
	memcpy(v->bg, s->bg_videoram, sizeof(v->bg));
	memcpy(v->fg, s->fg_videoram, sizeof(v->fg));
	memcpy(v->line, s->lineram, sizeof(v->line));
	memcpy(v->sp, s->sp_videoram, sizeof(v->sp));
	memcpy(v->zoom, s->sp_zoom, sizeof(v->zoom));
	memcpy(v->pal_sp, s->sp_palette, sizeof(v->pal_sp));
	memcpy(v->pal_bg, s->bg_palette, sizeof(v->pal_bg));
	memcpy(v->pal_tx, s->tx_palette, sizeof(v->pal_tx));
	v->bgscroll = s->bgscroll;
	v->fgscroll = s->fgscroll;
	v->key = s->realspritekey;
	v->vis_w = s->vis_w;
	v->vis_h = s->vis_h;
	rbx_video.width = s->vis_w;
	rbx_video.height = s->vis_h;
	rbx_video_frame_done();
}

// ---- machine ----
static void pgm2_exit(void);

// the sound ROM kept in a file (rbx_driver.file_regions): its pages read when the chip needs them
static int ymz_fetch(void *user, uint32_t offset, void *dst, uint32_t len)
{
	(void)user;
	return rbx_region_read("ymz774", offset, dst, len);
}
static const char *const s_file_regions[] = { "ymz774", NULL };

static uint8_t s_rev8[256];   // (each byte's bits reversed: the start's decoding by table)
static uint16_t rev8x2(uint16_t v)
{
	if (!s_rev8[1])
		for (int i = 0; i < 256; i++) {
			uint8_t r = 0;
			for (int b = 0; b < 8; b++)
				r |= (uint8_t)(((i >> b) & 1) << (7 - b));
			s_rev8[i] = r;
		}
	return (uint16_t)(s_rev8[v >> 8] << 8 | s_rev8[v & 0xff]);
}

static void map_all(void)
{
	arm7_map(s_arm, 0x00000000, 0x00003fff, s_irom, 0);
	arm7_map(s_arm, 0x02000000, 0x0200ffff, s->sram, 1);
	arm7_map(s_arm, 0x20000000, 0x2007ffff, s->mainram, 1);
	if (s_ramrom) {
		arm7_map(s_arm, 0x10000000, 0x101fffff, s_romboard_ram, 1);
		arm7_map(s_arm, 0x10200000, 0x10200000 + s_mainrom_size - 1, s_mainrom, 0);
	} else
		arm7_map(s_arm, 0x10000000, 0x10000000 + s_mainrom_size - 1, s_mainrom, 0);
	arm7_map(s_arm, 0x30000000, 0x30001fff, s->sp_videoram, 1);
	arm7_map(s_arm, 0x30020000, 0x30021fff, s->bg_videoram, 1);
	arm7_map(s_arm, 0x30040000, 0x30045fff, s->fg_videoram, 1);
	arm7_map(s_arm, 0x30060000, 0x30063fff, s->sp_palette, 1);
	arm7_map(s_arm, 0x30080000, 0x30081fff, s->bg_palette, 1);
	arm7_map(s_arm, 0x300a0000, 0x300a07ff, s->tx_palette, 1);
	for (uint32_t a = 0x300e0000; a < 0x300f0000; a += 0x400)   // (lineram, mirror 0xfc00)
		arm7_map(s_arm, a, a + 0x3ff, s->lineram, 1);
}

// MAME's encryption can be undone word by word: the reset puts the program ROM back as it was (MAME keeps a copy)
static void encrypt_back(void);

static void pgm2_reset(void)
{
	s->spritekey = s->realspritekey = 0;
	s->mcu_last_cmd = 0;
	s->share_bank = 0;
	if (s->has_decrypted)
		encrypt_back();
	s->has_decrypted = 0;
	romram_protect(0);
	s->mcu_timer = -1;
	aic_reset();
	arm7_reset(s_arm);
	arm7_jit_reset();
	if (s_ymz)
		ymz774_reset(s_ymz);
	s->frac = 0;
	s_snd_frac = 0;
}

static int pgm2_init(const rbx_game *g)
{
	const char *init = strchr(g->machine, ':');
	init = init ? init + 1 : "";
	if (!strncmp(init, "kov3", 4) || !strcmp(init, "bubucar")) {
		rbx_fail("%s: not emulated yet", g->set);
		return -1;
	}
	s_lores = !strncmp(g->machine, "pgm2_lores", 10);
	s_hires = !strncmp(g->machine, "pgm2_hires", 10);
	s_ramrom = !strncmp(g->machine, "pgm2_ramrom", 11);
	s_frame_mhz = s_lores ? FRAME_MHZ_LORES : FRAME_MHZ;
	s_irom = rbx_region("maincpu", &s_irom_size);
	s_mainrom = rbx_region("mainrom", &s_mainrom_size);
	uint8_t *mask = rbx_region("sprites_mask", &s_mask_len);
	uint8_t *colour = rbx_region("sprites_colour", &s_colour_len);
	s_tiles = rbx_region("tiles", &s_tiles_size);
	s_bgtile = rbx_region("bgtile", &s_bgtile_size);
	uint32_t ymz_size = rbx_region_file("ymz774"), sram_size = 0;
	uint8_t *ymz_rom = ymz_size ? NULL : rbx_region("ymz774", &ymz_size);
	const uint8_t *sram = rbx_region_opt("sram", &sram_size);
	if (!s_irom || !s_mainrom || !mask || !colour || !s_tiles || !s_bgtile || !ymz_size) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if ((s_mask_len & (s_mask_len - 1)) || (s_colour_len & (s_colour_len - 1))) {
		rbx_fail("%s: sprite regions not a power of two", g->set);
		return -1;
	}
	s = (pgm2_t *)calloc(1, sizeof(pgm2_t));
	s_arm = (arm7_t *)calloc(1, sizeof(arm7_t));
	s_ymz = (ymz774_t *)calloc(1, sizeof(ymz774_t));
	s_snd = (int16_t *)calloc(4096 * 2, sizeof(int16_t));
	s_snap = (pgm2_snap *)calloc(1, sizeof(pgm2_snap));
	s_spr[1] = (uint16_t *)calloc(SCR_W * SCR_H, sizeof(uint16_t));
	if (s_ramrom) {
		s_romboard_size = 0x200000;
		s_romboard_ram = (uint8_t *)calloc(1, s_romboard_size);
	}
	if (!s || !s_arm || !s_ymz || !s_snd || !s_snap || !s_spr[1] || (s_ramrom && !s_romboard_ram)) {
		rbx_fail("%s: out of memory", g->set);
		pgm2_exit();
		return -1;
	}
	if (ymz_rom ? ymz774_init(s_ymz, ymz_rom, ymz_size)
	            : ymz77x_init_fetch(s_ymz, YMZ_TYPE_774, 22579200, ymz_size, ymz_fetch, NULL, rbx_region_file_swap16("ymz774"))) {
		free(s_ymz);
		s_ymz = NULL;
		rbx_fail("%s: YMZ774", g->set);
		pgm2_exit();
		return -1;
	}
	// common_encryption_init: the sprite mask and colour data decoded once
	{
		uint16_t *rom = (uint16_t *)mask;
		const int len = (int)s_mask_len;
		for (int i = 0; i < len / 2; i += 2) {   // iga_u12_decode
			uint16_t x = 0;
			if ((i >> 1) & 0x000001) x ^= 0x9004;
			if ((i >> 1) & 0x000002) x ^= 0x0028;
			if ((i >> 1) & 0x000004) x ^= 0x0182;
			if ((i >> 1) & 0x000008) x ^= 0x0010;
			if ((i >> 1) & 0x000010) x ^= 0x2040;
			if ((i >> 1) & 0x000020) x ^= 0x0801;
			if ((i >> 1) & 0x000100) x ^= 0x4000;
			if ((i >> 1) & 0x000200) x ^= 0x0600;
			rom[i] ^= x;
			rom[i] = rev8x2(rom[i]);   // bitswap<16>(8..15, 0..7): each byte's bits reversed
		}
		for (int i = 1; i < len / 2; i += 2) {   // iga_u16_decode
			uint16_t x = 0;
			if ((i >> 1) & 0x000001) x ^= 0x0010;
			if ((i >> 1) & 0x000002) x ^= 0x2004;
			if ((i >> 1) & 0x000004) x ^= 0x0801;
			if ((i >> 1) & 0x000008) x ^= 0x0300;
			if ((i >> 1) & 0x000010) x ^= 0x0080;
			if ((i >> 1) & 0x000020) x ^= 0x0020;
			if ((i >> 1) & 0x000040) x ^= 0x4008;
			if ((i >> 1) & 0x000080) x ^= 0x1002;
			if ((i >> 1) & 0x000100) x ^= 0x0400;
			if ((i >> 1) & 0x000200) x ^= 0x0040;
			if ((i >> 1) & 0x000400) x ^= 0x8000;
			rom[i] ^= x;
			rom[i] = rev8x2(rom[i]);
		}
		uint16_t *c = (uint16_t *)colour;
		for (uint32_t i = 0; i < s_colour_len / 2; i++) {   // sprite_colour_decode
			const uint16_t v = c[i];
			// bitswap<16>(15, 14, 13, 12, 11, 5, 4, 3, 7, 6, 10, 9, 8, 2, 1, 0)
			c[i] = (uint16_t)((v & 0xf800) | ((v >> 5) & 1) << 10 | ((v >> 4) & 1) << 9 | ((v >> 3) & 1) << 8 |
			                  ((v >> 7) & 1) << 7 | ((v >> 6) & 1) << 6 | ((v >> 10) & 1) << 5 | ((v >> 9) & 1) << 4 |
			                  ((v >> 8) & 1) << 3 | (v & 7));
		}
	}
	s_mask = mask;
	s_colour = colour;
	s_spr[0] = s->sprite_bitmap;
	s_spr_front = 0;
	s_port[0] = rbx_port_find("INPUTS0");
	s_port[1] = rbx_port_find("INPUTS1");
	// SRAM: the NVRAM file, else the set's default ("sram" region), else zeros
	if (sram)
		memcpy(s->sram, sram, sram_size < sizeof(s->sram) ? sram_size : sizeof(s->sram));
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				uint8_t *tmp = (uint8_t *)malloc(sizeof(s->sram));
				if (tmp && fread(tmp, 1, sizeof(s->sram), f) == sizeof(s->sram))
					memcpy(s->sram, tmp, sizeof(s->sram));
				free(tmp);
				fclose(f);
			}
		}
	}
	if (rbx_video_setup(SCR_W, 240, PAL_N) || rbx_audio_setup_stereo(ymz774_rate(s_ymz))) {
		pgm2_exit();
		return -1;
	}
	s->vis_w = s_lores ? 320 : s_hires ? 512 : 448;
	s->vis_h = s_lores || s_hires ? 240 : 224;
	rbx_audio.frame_mhz = s_frame_mhz;
	arm7_init(s_arm);
	s_arm->arch5 = 1;
	s_arm->idle_skip = 1;
	s_arm->jit = 1;
	s_arm->read32 = rd32;
	s_arm->read16 = rd16;
	s_arm->read8 = rd8;
	s_arm->write32 = wr32;
	s_arm->write16 = wr16;
	s_arm->write8 = wr8;
	map_all();
	pgm2_reset();
	return 0;
}

static void pgm2_exit(void)
{
	if (s_romboard_ram)
		rbx_log("pgm2: program RAM writes after its decryption: %u", (unsigned)s_romram_writes);
	if (s && s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s->sram, 1, sizeof(s->sram), f);
			fclose(f);
		}
	}
	s_nv_path[0] = 0;
	if (s_ymz)
		ymz774_exit(s_ymz);
	free(s_ymz);
	free(s_arm);
	free(s);
	free(s_romboard_ram);
	free(s_snd);
	free(s_snap);
	s_snap = NULL;
	free(s_spr[1]);
	s_spr[0] = s_spr[1] = NULL;
	s_ymz = NULL;
	s_arm = NULL;
	s = NULL;
	s_romboard_ram = NULL;
	s_snd = NULL;
}

// the ARM up to `until` cycles of the frame, cut at the MCU timer
static void run_until(uint32_t until)
{
	for (;;) {
		const int64_t now = s_arm->total_cycles - s_frame_start;
		if (now >= (int64_t)until)
			return;
		int64_t end = until;
		if (s->mcu_timer >= 0 && s->mcu_timer - s_frame_start < end)
			end = s->mcu_timer - s_frame_start;
		if (end > now)
			arm7_run(s_arm, (int)(end - now));
		if (s->mcu_timer >= 0 && s_arm->total_cycles >= s->mcu_timer) {
			s->mcu_timer = -1;
			aic_set_irq(3, 1);   // mcu_interrupt
		}
	}
}

static void pgm2_frame(void)
{
	rbx_audio.count = 0;
	s_frame_start = s_arm->total_cycles;
	{
		// this frame's cycles and samples (the same fractions carried as below)
		uint64_t f = s->frac;
		uint32_t n = 0;
		for (int line = 0; line < LINES; line++)
			n += (uint32_t)rbx_line_cycles(&f, CPU_CLOCK, s_frame_mhz, LINES);
		s_frame_cycles = n;
		const uint64_t want = (uint64_t)(s_ymz ? ymz774_rate(s_ymz) : 0) * 1000u + s_snd_frac;
		s_snd_n = (int)(want / s_frame_mhz);
		s_snd_frac = (uint32_t)(want % s_frame_mhz);
		s_snd_done = 0;
	}
	uint32_t t = 0;
	const int vbl = s->vis_h;   // (vblank from the end of the visible area)
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == vbl) {
			screen_vblank();   // (the render: MAME's screen update, then its vblank callback's sprites)
			aic_set_irq(12, 1);
		}
		t += (uint32_t)rbx_line_cycles(&s->frac, CPU_CLOCK, s_frame_mhz, LINES);
		s_line_end = t;
		run_until(t);
	}
	s->frames++;
	snd_sync(1);
}

// ---- the inverse of igs036_decrypt_rom (the reset's encrypted program ROM) ----
static void encrypt_back(void)
{
	if (s_romboard_ram)
		igs036_encrypt_rom(s->encryption_table, (uint16_t *)s_mainrom, (int)s_mainrom_size, (int)(s_romboard_size / 2));
	else
		igs036_encrypt_rom(s->encryption_table, (uint16_t *)s_mainrom, (int)s_mainrom_size, 0);
	arm7_jit_reset();
}

static int pgm2_state(void *ctx, int load)
{
	pgm2_t *tmp = NULL;
	rbx_video_wait_idle();   // (the render owns the sprite bitmap while it runs)
	uint8_t was = s->has_decrypted;
	uint8_t key[0x100];
	memcpy(key, s->encryption_table, sizeof(key));
	int rc = 0;
	uint32_t tcm[2] = { s_arm->cp15_itcm_reg, s_arm->cp15_dtcm_reg };
	s->tcm[0] = tcm[0];
	s->tcm[1] = tcm[1];
	rc |= rbx_state_io(ctx, "arm", s_arm, ARM7_STATE_SIZE);
	rc |= rbx_state_io(ctx, "board", s, (uint32_t)offsetof(pgm2_t, end_saved));
	rc |= rbx_state_io(ctx, "spr", s_spr[s_spr_front], sizeof(s->sprite_bitmap));
	if (s_romboard_ram)
		rc |= rbx_state_io(ctx, "romram", s_romboard_ram, s_romboard_size);
	if (s_ymz)
		rc |= ymz77x_state(s_ymz, ctx, load, "ymz");
	(void)tmp;
	if (rc)
		return -1;
	if (load) {
		s_arm->cp15_itcm_reg = s->tcm[0];
		s_arm->cp15_dtcm_reg = s->tcm[1];
		arm7_set_cpsr(s_arm, s_arm->cpsr);
		// the program ROM as the loaded state has it (MAME device_post_load)
		if (was && (!s->has_decrypted || memcmp(key, s->encryption_table, sizeof(key)))) {
			uint8_t now[0x100];
			memcpy(now, s->encryption_table, sizeof(now));
			memcpy(s->encryption_table, key, sizeof(key));
			encrypt_back();
			memcpy(s->encryption_table, now, sizeof(now));
			was = 0;
		}
		if (!was && s->has_decrypted) {
			if (s_romboard_ram)
				igs036_decrypt_rom(s->encryption_table, (uint16_t *)s_mainrom, (int)s_mainrom_size, (int)(s_romboard_size / 2));
			else
				igs036_decrypt_rom(s->encryption_table, (uint16_t *)s_mainrom, (int)s_mainrom_size, 0);
		}
		romram_protect(s->has_decrypted);
		arm7_jit_reset();
		aic_set_lines();
	}
	return 0;
}

static int pgm2_save(void *ctx) { return pgm2_state(ctx, 0); }
static int pgm2_load(void *ctx) { return pgm2_state(ctx, 1); }
static int pgm2_supported(const char *machine)
{
	const char *init = strchr(machine, ':');
	init = init ? init + 1 : "";
	return strncmp(init, "kov3", 4) && strcmp(init, "bubucar");
}

const rbx_driver rbx_drv_pgm2 = { "IGS PGM2", pgm2_init, pgm2_exit, pgm2_reset, pgm2_frame, pgm2_save, pgm2_load,
                                  pgm2_supported, pgm2_render, pgm2_render_help, s_file_regions };

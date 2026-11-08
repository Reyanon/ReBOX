/*  Hot-B HB-PCB-A5: Super Shanghai Dragon's Eye. Ported from MAME src/mame/dataeast/sshangha.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail, Charles MacDonald, David Haywood).
    68000 at 16 MHz (IRQ 6 held at vblank), the tilemap chip 55 (its two 4 bpp playfields combined into one 8 bpp
    picture unless the video register's bit 2), two sprite chips 52 / 71 drawn raw and copied by their priority bit,
    the 146 (inputs; its address lines bit-swapped), the sound: Z80 at 4 MHz with YM2203 (its IRQ on the Z80's INT) and
    an OKI, talking to the 68000 through 8 shared words. The bootleg has the inputs at fixed addresses, no 146.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco146.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 16000000
#define Z80_CLOCK 4000000        // 16 MHz / 4 (the YM2203 too)
#define OKI_CLOCK 2000000        // 16 MHz / 8, pin 7 low
#define FRAME_MHZ 60000          // set_refresh_hz(60)
#define LINES 256                // set_size(40*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 8              // vblank 529 us
#define BLACK 0x400              // (palette_device::black_pen: a pen past the 1024 of the RAM)

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static int s_boot;                         // sshanghab
static uint16_t s_ram[0x4000];             // 0x3ec000 .. 0x3f3fff (bootleg: 0xfec000 .. 0xff3fff)
static uint16_t s_ram2[0x400];             // bootleg 0xff4000 .. 0xff47ff
static uint16_t s_vram_ext[0xc00];         // 0x206800 .. 0x207fff
static uint16_t s_spriteram[2][0x400];
static uint16_t s_sprbuf[0x800];           // bootleg 0x340000 (the original sprite RAM, a buffer there)
static uint16_t s_palram[0x800];           // 0x380000: xBGR_888, 1024 entries of two words
static uint16_t s_palext[0x1800];          // 0x381000 .. 0x383fff (unused)
static uint16_t s_rowscroll[2][0x400];
static uint16_t s_shared[8];               // sound_shared (the Z80's 0xf800 .. 0xf807: the low bytes)
static uint16_t s_video;                   // video_w: bit 2 the playfields apart
static deco16ic_t s_tile;
static decospr_t s_spr[2];
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr[2];
static int s_port_in, s_port_sys, s_port_dsw;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static uint16_t s_sbm[2][VIS_W * VIS_H];   // the sprites drawn raw (pen | colour << 4)
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x400], spriteram[2][0x400], palram[0x800], video;
	uint32_t frame;
} *s_snap;

// ---- sound: Z80, YM2203, OKI ----
static z80 s_z80;
static uint8_t s_zram[0x800];              // 0xf800 .. 0xffff (0xf800 .. 0xf807 the shared words)
static okim6295_t s_oki;
static int s_ym_irq;
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a == 0xc000 || a == 0xc001) return rbx_ym_read(0, a & 1);
	if (a == 0xc200 || a == 0xc201) return oki_read(&s_oki);
	if (a >= 0xf800 && a < 0xf808) return (uint8_t)s_shared[a & 7];   // sound_shared_r
	if (a >= 0xf808) return s_zram[a - 0xf800];
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a == 0xc000 || a == 0xc001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xc200 || a == 0xc201) oki_write(&s_oki, v);
	else if (a >= 0xf800 && a < 0xf808) s_shared[a & 7] = v;   // sound_shared_w (the word's high byte 0)
	else if (a >= 0xf808) s_zram[a - 0xf800] = v;
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

// the Z80 and the chips through one line: the YM's samples (on the worker) and the OKI held at the YM's rate
static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);   // (the YM's INT: level)
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the 146's ports ----
static int vblank(void) { return s_line >= VIS_Y0 + VIS_H && s_line < VIS_Y0 + VIS_H + VBL_LINES; }
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	rbx_port_custom(s_port_sys, 0x0008, vblank() ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }

// ioprot_r / w: the address lines 14 .. 17 to the 146's 11 .. 14, 11 .. 13 dropped
static uint16_t prot_addr(uint32_t a) { return (uint16_t)((((a >> 14) & 0xf) << 11 | (a & 0x7ff)) & 0x7fff); }

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	if (s_boot) {
		a &= 0xffffff;
		if (a >= 0x084000 && a < 0x084800) {   // sshanghab_protection16_r
			switch (a & 0x7fe) {
			case 0x050: return (uint16_t)rbx_port_read(s_port_in);
			case 0x76a: return port_b(NULL);
			case 0x0ac: return (uint16_t)rbx_port_read(s_port_dsw);
			}
			return 0;
		}
		if (a >= 0x101000 && a < 0x101010) return s_shared[(a >> 1) & 7];
		return 0;
	}
	a &= 0x3fffff;
	if (a >= 0x100000 && a < 0x100010) return s_shared[(a >> 1) & 7];
	if (a == 0x350000 || a == 0x370000) return 0xffff;   // deco_71_r (the sprite DMA done)
	if ((a >= 0x3e0000 && a < 0x3e4000) || (a >= 0x3f4000 && a < 0x3f8000)) return deco146_read(&s_prot, prot_addr(a), NULL);
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= s_boot ? 0xffffff : 0x3fffff;
	const uint32_t sh = s_boot ? 0x101000 : 0x100000;
	if (a >= sh && a < sh + 0x10) {
		uint16_t *w = &s_shared[(a >> 1) & 7];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else if (a >= 0x300000 && a < 0x300010)
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), v, mask);
	else if (a == 0x320000)
		s_video = (uint16_t)((s_video & ~mask) | (v & mask));
	else if (!s_boot && ((a >= 0x3e0000 && a < 0x3e4000) || (a >= 0x3f4000 && a < 0x3f8000)))
		deco146_write(&s_prot, prot_addr(a), v, mask, NULL);
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & ~0xf) << 8; }
// (as tattass, the base colour pf2's bank)
static uint16_t mix_cb(uint16_t p, uint16_t p2)
{
	return (uint16_t)((p2 & 0x300) ^ (((p & 0x10) << 5) | (p & 0x0f) | ((p2 & 0x0f) << 4)));
}

// MAME inefficient_copy_sprite_bitmap: the raw sprite pixels of a priority to the bitmap
static void copy_sprites(const uint16_t *src, uint16_t *bm, int pitch, uint16_t pri, uint16_t pmask, uint16_t colbase,
                         uint16_t palmask)
{
	for (int y = 0; y < VIS_H; y++) {
		const uint16_t *s = src + y * VIS_W;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			const uint16_t pix = s[x];
			if ((pix & 0xf) && (pix & pmask) == pri)
				d[x] = (uint16_t)((pix & palmask) + colbase);
		}
	}
}

static void ss_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_888 (the entry's two words, big-endian)
		const uint32_t p = (uint32_t)s_snap->palram[2 * i] << 16 | s_snap->palram[2 * i + 1];
		rbx_video.palette[i] = (p & 0xff) << 16 | (p & 0xff00) | ((p >> 16) & 0xff);
	}
	rbx_video.palette[BLACK] = 0;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t = &s_snap->tile;
	deco16ic_update(t);
	const int combine = !(s_snap->video & 4);
	// (sprites are flipped relative to the tilemaps)
	const int flip = !((deco16ic_control_r(t, 0) >> 7) & 1);
	for (int c = 0; c < 2; c++) {
		decospr_t spr = s_spr[c];
		spr.flip_screen = flip;
		memset(s_sbm[c], 0, sizeof(s_sbm[c]));
		decospr_draw(&spr, s_snap->spriteram[c], 0x400, s_sbm[c], NULL, VIS_W, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
	}
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			bm[y * pitch + x] = BLACK;
	if (combine)
		deco16ic_draw_combine(t, s_snap->rowscroll[0], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, mix_cb);
	else
		deco16ic_draw(t, 1, s_snap->rowscroll[1], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	copy_sprites(s_sbm[0], bm, pitch, 0x000, 0x000, 0x200, 0x0ff);   // low + high priority sprites 1
	copy_sprites(s_sbm[1], bm, pitch, 0x200, 0x200, 0x000, 0x0ff);   // low priority sprites 2
	if (!combine)
		deco16ic_draw(t, 0, s_snap->rowscroll[0], bm, NULL, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 0, 0xff);
	copy_sprites(s_sbm[1], bm, pitch, 0x000, 0x200, 0x000, 0x0ff);   // high priority sprites 2
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(&s_snap->tile, &s_tile, sizeof(s_tile));
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->palram, s_palram, sizeof(s_palram));
	s_snap->video = s_video;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ss_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_vram_ext, 0, sizeof(s_vram_ext));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_sprbuf, 0, sizeof(s_sprbuf));
	memset(s_palram, 0, sizeof(s_palram));
	memset(s_palext, 0, sizeof(s_palext));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_shared, 0, sizeof(s_shared));
	s_video = 0;
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	deco146_reset(&s_prot);
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_ym_irq = 0;
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void ss_exit(void);

// map a memory range at the 68000's four mirrors of the 22-bit bus (global_mask 0x3fffff)
static void map_mirrors(uint32_t start, uint32_t end, void *mem, uint32_t size, int wr)
{
	for (uint32_t m = 0; m < (s_boot ? 1u : 4u); m++)
		m68k_bus_map(start + m * 0x400000, end + m * 0x400000, mem, size, 1, wr);
}

static int ss_init(const rbx_game *g)
{
	uint32_t tsize, s1s, s2s, os;
	s_boot = !strcmp(g->set, "sshanghab");
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *spr1 = rbx_region("sprites1", &s1s),
	        *spr2 = rbx_region("sprites2", &s2s), *oki = rbx_region("oki", &os);
	if (!s_rom || !s_zrom || !tiles || !spr1 || !spr2 || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// charlayout / tilelayout: planes 8, 0, half + 8, half
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t ht = tsize * 4, h1 = s1s * 4, h2 = s2s * 4;
		const uint32_t pt[4] = { 8, 0, ht + 8, ht }, p1[4] = { 8, 0, h1 + 8, h1 }, p2[4] = { 8, 0, h2 + 8, h2 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 16 * 8 }, l16 = { 16, 16, 4, pt, x16, y16, 64 * 8 };
		const rbx_gfx_layout ls1 = { 16, 16, 4, p1, x16, y16, 64 * 8 }, ls2 = { 16, 16, 4, p2, x16, y16, 64 * 8 };
		// (the two sprite chips' ROMs are copies: one decoding for both when they are the same)
		const int same = s1s == s2s && !memcmp(spr1, spr2, s1s);
		if (rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2) ||
		    rbx_gfx_decode(&s_gfxspr[0], &ls1, spr1, s1s / 2) ||
		    (!same && rbx_gfx_decode(&s_gfxspr[1], &ls2, spr2, s2s / 2))) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ss_exit();
			return -1;
		}
		for (int c = 0; c < 2; c++) {
			memset(&s_spr[c], 0, sizeof(s_spr[c]));
			s_spr[c].gfx = c && !same ? &s_gfxspr[1] : &s_gfxspr[0];
			s_spr[c].raw = 1;   // (alloc_sprite_bitmap: raw shift 4)
			s_spr[c].raw_shift = 4;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));
	s_tile.tm[0].size = DECO_64x64;   // (the ending screen needs pf1 at 64x64)
	s_tile.tm[1].size = DECO_64x32;
	s_tile.tm[0].col_bank = 0x10;
	s_tile.tm[1].col_bank = 0x30;
	for (int i = 0; i < 2; i++) {
		s_tile.tm[i].col_mask = 0x0f;
		s_tile.tm[i].bank_cb = bank_cb;
	}
	s_tile.gfx8 = &s_gfx8;
	s_tile.gfx16 = &s_gfx16;
	deco146_init(&s_prot, 0);
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 4 MHz 0.33 (its IRQ on the Z80's INT), OKI 2 MHz 0.27, one speaker
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = Z80_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 84;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		ss_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, BLACK + 1)) {
		rbx_fail("%s: out of memory", g->set);
		ss_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 0, oki, os, 69);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	s_port_in = rbx_port_find("INPUTS");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	map_mirrors(0, 0x3ffff, s_rom, s_rom_size, 0);
	map_mirrors(0x200000, 0x201fff, s_tile.vram[0], sizeof(s_tile.vram[0]), 1);
	map_mirrors(0x202000, 0x203fff, s_tile.vram[1], sizeof(s_tile.vram[1]), 1);
	map_mirrors(0x204000, 0x2047ff, s_rowscroll[0], sizeof(s_rowscroll[0]), 1);
	map_mirrors(0x206000, 0x2067ff, s_rowscroll[1], sizeof(s_rowscroll[1]), 1);
	map_mirrors(0x206800, 0x207fff, s_vram_ext, sizeof(s_vram_ext), 1);
	map_mirrors(0x380000, 0x380fff, s_palram, sizeof(s_palram), 1);
	map_mirrors(0x381000, 0x383fff, s_palext, sizeof(s_palext), 1);
	if (s_boot) {
		m68k_bus_map(0x340000, 0x340fff, s_sprbuf, sizeof(s_sprbuf), 1, 1);
		m68k_bus_map(0x3c0000, 0x3c07ff, s_spriteram[0], sizeof(s_spriteram[0]), 1, 1);
		m68k_bus_map(0x3c0800, 0x3c0fff, s_spriteram[1], sizeof(s_spriteram[1]), 1, 1);
		m68k_bus_map(0xfec000, 0xff3fff, s_ram, sizeof(s_ram), 1, 1);
		m68k_bus_map(0xff4000, 0xff47ff, s_ram2, sizeof(s_ram2), 1, 1);
	} else {
		for (uint32_t m = 0; m < 2; m++) {   // (the sprite RAMs mirrored at 0x800)
			map_mirrors(0x340000 + m * 0x800, 0x3407ff + m * 0x800, s_spriteram[1], sizeof(s_spriteram[1]), 1);
			map_mirrors(0x360000 + m * 0x800, 0x3607ff + m * 0x800, s_spriteram[0], sizeof(s_spriteram[0]), 1);
		}
		map_mirrors(0x3ec000, 0x3f3fff, s_ram, sizeof(s_ram), 1);
	}
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	ss_reset();
	return 0;
}

static void ss_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr[0]);
	rbx_gfx_free(&s_gfxspr[1]);
	free(s_snap);
	s_snap = NULL;
}

static void ss_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank
			vblank_capture();
			m68k_bus_hold_irq(6);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			deco16ic_latch_line(&s_tile, line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int ss_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t misc[6] = { (int32_t)s_frame, s_video, s_ym_irq, (int32_t)s_oki_acc, s_oki_hold, 0 };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "vext", s_vram_ext, sizeof(s_vram_ext));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sprb", s_sprbuf, sizeof(s_sprbuf));
	rc |= rbx_state_io(ctx, "pal", s_palram, sizeof(s_palram));
	rc |= rbx_state_io(ctx, "palx", s_palext, sizeof(s_palext));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "shared", s_shared, sizeof(s_shared));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_video = (uint16_t)misc[1];
		s_ym_irq = misc[2];
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
		for (int i = 0; i < 8; i++)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ss_save(void *ctx) { return ss_state(ctx, 0); }
static int ss_load(void *ctx) { return ss_state(ctx, 1); }
static int ss_supported(const char *machine) { return !strcmp(machine, "sshangha") || !strcmp(machine, "sshanghab"); }

const rbx_driver rbx_drv_sshangha = { "DECO 16 Super Shanghai Dragon's Eye", ss_init, ss_exit, ss_reset, ss_frame,
                                      ss_save, ss_load, ss_supported, ss_render, NULL };

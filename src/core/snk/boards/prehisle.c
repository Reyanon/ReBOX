/*  SNK Prehistoric Isle in 1930. Ported from MAME src/mame/snk/prehisle.cpp (BSD-3-Clause, copyright-holders: Bryan
    McPhail, Carlos A. Lozano).
    68000 at 9 MHz (IRQ 4 held at the vblank); a BG of 16x16 tiles mapped from ROM, an FG and a text layer, 16x16
    sprites behind the text (and the FG for the upper colors) through the priority bitmap. Sound: Z80 at 4 MHz, YM3812
    (its IRQ) and a uPD7759, the latch on the NMI.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../../video/decoband.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 9000000
#define Z80_CLOCK 4000000
#define YM_CLOCK 4000000
#define UPD_CLOCK 640000
#define FRAME_MHZ 59186            // set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240)
#define LINES 264
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 224

static uint8_t *s_rom, *s_zrom;
static const uint8_t *s_bgmap;
static uint32_t s_rom_size, s_zrom_size, s_bgmap_size;
static uint16_t s_ram[0x2000];          // 0x070000 .. 0x073fff
static uint16_t s_txram[0x400];         // 0x090000 .. 0x0907ff
static uint16_t s_spriteram[0x400];     // 0x0a0000 .. 0x0a07ff
static uint16_t (*s_fgram_hp)[0x2000];  // 0x0b0000 .. 0x0b3fff (on the heap: ADR-040)
#define s_fgram (*s_fgram_hp)
static uint16_t s_pal[0x400];           // 0x0d0000 .. 0x0d07ff
static uint16_t s_bg_sx, s_bg_sy, s_fg_sx, s_fg_sy;
static uint8_t s_flip, s_invert;
static rbx_gfx s_gfxtx, s_gfxbg, s_gfxfg, s_gfxspr;
static int s_port[6];
static int s_line;
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static struct {
	uint16_t tx[0x400], spr[0x400], fg[0x2000], pal[0x400], bg_sx, bg_sy, fg_sx, fg_sy;
	uint8_t flip;
} *s_snap;
static uint16_t (*s_frame_hp)[VIS_H][VIS_W];
#define s_frame (*s_frame_hp)
static uint8_t (*s_pri_hp)[VIS_H][VIS_W];
#define s_pri (*s_pri_hp)

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch;
static int s_ym_irq;
static upd7759_t s_upd;
static int32_t *s_mix, *s_updbuf;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xf000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xf800) return s_zram[a - 0xf000];
	if (a == 0xf800) return s_latch;
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf000 && a < 0xf800) s_zram[a - 0xf000] = v;
}
static uint8_t z80_in(z80 *z, uint8_t port) { (void)z; return port == 0x00 ? rbx_ym_read(0, 0) : 0xff; }
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (port) {
	case 0x00: rbx_ym_write(0, 0, v); break;
	case 0x20: rbx_ym_write(0, 1, v); break;
	case 0x40:   // upd_port_w
		upd7759_port_w(&s_upd, v);
		upd7759_start_w(&s_upd, 0);
		upd7759_start_w(&s_upd, 1);
		break;
	case 0x80: upd7759_reset_w(&s_upd, (v >> 7) & 1); break;
	}
}

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_ym_irq, 0);
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	if (n > 0) {   // the uPD7759 at the YM's rate, 0.90
		upd7759_render(&s_upd, s_updbuf, n);
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += s_updbuf[i] * 230 / 256;
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

// ---- bus ----
static int vblank(void) { return s_line >= VIS_Y0 + VIS_H || s_line < VIS_Y0; }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	switch (a) {
	case 0x0e0010: return (uint16_t)rbx_port_read(s_port[1]);   // P2
	case 0x0e0020: return (uint16_t)rbx_port_read(s_port[2]);   // COIN
	case 0x0e0040: return (uint16_t)((rbx_port_read(s_port[0]) ^ s_invert) & 0xff);   // P1 (0x0e0041)
	case 0x0e0042: return (uint16_t)rbx_port_read(s_port[3]);   // DSW0
	case 0x0e0044:   // DSW1 and the vblank
		rbx_port_custom(s_port[4], 0x80, vblank() ? 0x80 : 0);
		return (uint16_t)rbx_port_read(s_port[4]);
	}
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static inline void combine(uint16_t *w, uint16_t v, uint16_t mask) { *w = (uint16_t)((*w & ~mask) | (v & mask)); }
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	switch (a) {
	case 0x0f0000: combine(&s_fg_sy, v, mask); break;
	case 0x0f0010: combine(&s_fg_sx, v, mask); break;
	case 0x0f0020: combine(&s_bg_sy, v, mask); break;
	case 0x0f0030: combine(&s_bg_sx, v, mask); break;
	case 0x0f0046: s_invert = v ? 0xff : 0x00; break;
	case 0x0f0060: s_flip = v & 1; break;
	case 0x0f0070:   // soundcmd_w: the latch, an NMI pulse
		s_latch = (uint8_t)v;
		z80_gen_nmi(&s_z80);
		break;
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static inline uint32_t tcoord(int v, uint32_t scroll, int flip, uint32_t mask)
{
	return (uint32_t)(flip ? 255 - v + (int)scroll : v + (int)scroll) & mask;
}

// BG (from the tile map ROM, 1024x32 tiles, opaque, priority 1) and FG (256x32, pen 15 transparent, priority 2):
// scan columns of 16x16 tiles
static void layers_line(int y)
{
	uint16_t *d = s_frame[y];
	uint8_t *pr = s_pri[y];
	const int by = y + VIS_Y0, flip = s_snap->flip;
	const uint32_t bty = tcoord(by, s_snap->bg_sy, flip, 511), fty = tcoord(by, s_snap->fg_sy, flip, 511);
	uint32_t last = 0xffffffffu, cbase = 0, hflip = 0;
	const uint8_t *trow = NULL;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = tcoord(x, s_snap->bg_sx, flip, 16383);
		if ((tx >> 4) != last) {   // (a tile looked up once for its 16 pixels)
			last = tx >> 4;
			const uint32_t i = (tx >> 4) * 32 + (bty >> 4);
			const uint32_t attr = (uint32_t)s_bgmap[rbx_wrap((2 * i + 1), s_bgmap_size)] + ((uint32_t)s_bgmap[rbx_wrap((2 * i), s_bgmap_size)] << 8);
			const uint32_t code = rbx_wrap((attr & 0x7ff), s_gfxbg.count);
			trow = s_gfxbg.pix + code * 256 + (bty & 15) * 16;
			hflip = attr & 0x800;
			cbase = 768 + (attr >> 12) * 16;
		}
		d[x] = (uint16_t)(cbase + trow[hflip ? 15 - (tx & 15) : (tx & 15)]);
		pr[x] = 1;
	}
	last = 0xffffffffu;
	for (int x = 0; x < VIS_W; x++) {
		const uint32_t tx = tcoord(x, s_snap->fg_sx, flip, 4095);
		if ((tx >> 4) != last) {   // (a tile looked up once for its 16 pixels)
			last = tx >> 4;
			const uint16_t attr = s_snap->fg[(tx >> 4) * 32 + (fty >> 4)];
			const uint32_t code = rbx_wrap((attr & 0x7ffu), s_gfxfg.count);
			const uint32_t py = (attr & 0x800) ? 15 - (fty & 15) : (fty & 15);
			trow = s_gfxfg.pix + code * 256 + py * 16;
			cbase = 512 + (uint32_t)(attr >> 12) * 16;
		}
		const uint8_t p = trow[tx & 15];
		if (p != 15) {
			d[x] = (uint16_t)(cbase + p);
			pr[x] |= 2;
		}
	}
	const uint32_t tty = (uint32_t)(flip ? 255 - by : by) & 255;
	last = 0xffffffffu;
	for (int x = 0; x < VIS_W; x++) {   // the text: 32x32 of 8x8, pen 15 transparent, priority 4
		const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
		if ((tx >> 3) != last) {   // (a character looked up once for its 8 pixels)
			last = tx >> 3;
			const uint16_t attr = s_snap->tx[(tty >> 3) * 32 + (tx >> 3)];
			const uint32_t code = rbx_wrap((attr & 0xfffu), s_gfxtx.count);
			trow = s_gfxtx.pix + code * 64 + (tty & 7) * 8;
			cbase = (uint32_t)(attr >> 12) * 16;
		}
		const uint8_t p = trow[tx & 7];
		if (p != 15) {
			d[x] = (uint16_t)(cbase + p);
			pr[x] |= 4;
		}
	}
}

static void draw_sprites(void)   // from the end; GFX_PMASK_4 (behind the text), | GFX_PMASK_2 (the FG) for colors 4+
{
	for (int offs = 1024 - 4; offs >= 0; offs -= 4) {
		const uint16_t *s = s_snap->spr + offs;
		const uint16_t attr = s[2];
		const uint32_t code = rbx_wrap((attr & 0x1fffu), s_gfxspr.count);
		const uint32_t color = s[3] >> 12;
		const uint32_t pmask = 0xf0f0u | (color < 4 ? 0 : 0xccccu) | 0x80000000u;
		int flipx = (attr & 0x4000) != 0, flipy = (attr & 0x8000) != 0;
		int sx = s[1] & 0x1ff, sy = s[0] & 0x1ff;
		if (sx & 0x100) sx = -0x100 + (sx & 0xff);
		if (sy & 0x100) sy = -0x100 + (sy & 0xff);
		if (s_snap->flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		sy -= VIS_Y0;
		if (sx >= VIS_W || sx + 16 <= 0 || sy >= VIS_H || sy + 16 <= 0)
			continue;
		const uint8_t *g = s_gfxspr.pix + code * 256;
		const uint16_t c0 = (uint16_t)(256 + color * 16);
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if ((unsigned)yy >= VIS_H)
				continue;
			const uint8_t *src = g + (flipy ? 15 - j : j) * 16;
			for (int i = 0; i < 16; i++) {
				const int xx = sx + i;
				if ((unsigned)xx >= VIS_W)
					continue;
				const uint8_t p = src[flipx ? 15 - i : i];
				if (p == 15)
					continue;
				uint8_t *pp = &s_pri[yy][xx];
				if (!((1u << (*pp & 0x1f)) & pmask))
					s_frame[yy][xx] = (uint16_t)(c0 + p);
				*pp = 31;
			}
		}
	}
}

#define BANDS 8
static int s_phase;
static void ph_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++) {
		if (s_phase == 0)
			layers_line(y);
		else
			memcpy(rbx_video.bitmap + y * rbx_video.pitch, s_frame[y], VIS_W * 2);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, ph_band };
static int ph_help(int helper) { return decoband_help(&s_bands, helper); }

static void ph_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // RGBx_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)((((p >> 12) & 15) * 17) << 16 | (((p >> 8) & 15) * 17) << 8 | (((p >> 4) & 15) * 17));
	}
	s_phase = 0;
	decoband_run(&s_bands);
	draw_sprites();
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->tx, s_txram, sizeof(s_txram));
	memcpy(s_snap->spr, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->fg, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->bg_sx = s_bg_sx;
	s_snap->bg_sy = s_bg_sy;
	s_snap->fg_sx = s_fg_sx;
	s_snap->fg_sy = s_fg_sy;
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void ph_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_txram, 0, sizeof(s_txram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_pal, 0, sizeof(s_pal));
	s_bg_sx = s_bg_sy = s_fg_sx = s_fg_sy = 0;
	s_flip = s_invert = 0;
	s_latch = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	upd7759_reset(&s_upd);
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void ph_exit(void);

static int ph_init(const rbx_game *g)
{
	uint32_t cs, bs, fs, ss, us;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_bgmap = rbx_region("bgtilemap", &s_bgmap_size);
	uint8_t *ch = rbx_region("chars", &cs), *bg = rbx_region("bgtiles", &bs), *fg = rbx_region("fgtiles", &fs);
	uint8_t *spr = rbx_region("sprites", &ss), *upd = rbx_region("upd", &us);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !s_bgmap || !s_bgmap_size || !ch || !bg || !fg || !spr || !upd) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_fgram_hp = calloc(1, sizeof(*s_fgram_hp));
	s_frame_hp = calloc(1, sizeof(*s_frame_hp));
	s_pri_hp = calloc(1, sizeof(*s_pri_hp));
	if (!s_fgram_hp || !s_frame_hp || !s_pri_hp) {
		rbx_fail("%s: out of memory", g->set);
		ph_exit();
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// gfx_8x8x4_packed_msb, gfx_8x8x4_col_2x2_group_packed_msb
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)(i * 4);
			y8[i] = (uint32_t)(i * 32);
			x16[i] = (uint32_t)(i * 4);
			x16[8 + i] = (uint32_t)(512 + i * 4);
			y16[i] = (uint32_t)(i * 32);
			y16[8 + i] = (uint32_t)(256 + i * 32);
		}
		static const uint32_t pl[4] = { 0, 1, 2, 3 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pl, x8, y8, 32 * 8 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pl, x16, y16, 128 * 8 };
		if (rbx_gfx_decode(&s_gfxtx, &l8, ch, cs) || rbx_gfx_decode(&s_gfxbg, &l16, bg, bs) ||
		    rbx_gfx_decode(&s_gfxfg, &l16, fg, fs) || rbx_gfx_decode(&s_gfxspr, &l16, spr, ss)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			ph_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // YM3812 1.0, uPD7759 0.90
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		ph_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_updbuf = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_updbuf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		ph_exit();
		return -1;
	}
	upd7759_init(&s_upd, upd, us, UPD_CLOCK, rbx_ym_rate(0));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xf0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[5] = { "P1", "P2", "COIN", "DSW0", "DSW1" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x3ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x070000, 0x073fff, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus_map(0x090000, 0x0907ff, s_txram, sizeof(s_txram), 1, 1);
	m68k_bus_map(0x0a0000, 0x0a07ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x0b0000, 0x0b3fff, s_fgram, sizeof(s_fgram), 1, 1);
	m68k_bus_map(0x0d0000, 0x0d07ff, s_pal, sizeof(s_pal), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	ph_reset();
	return 0;
}

static void ph_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	free(s_updbuf);
	s_mix = s_updbuf = NULL;
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxbg);
	rbx_gfx_free(&s_gfxfg);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_fgram_hp);
	free(s_frame_hp);
	free(s_pri_hp);
	s_fgram_hp = NULL;
	s_frame_hp = NULL;
	s_pri_hp = NULL;
}

static void ph_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // the vblank: the frame drawn, irq4_line_hold
			vblank_capture();
			m68k_bus_hold_irq(4);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int ph_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	uint16_t regs[8] = { s_bg_sx, s_bg_sy, s_fg_sx, s_fg_sy, s_flip, s_invert, s_latch, (uint16_t)s_ym_irq };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "tx", s_txram, sizeof(s_txram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "regs", regs, sizeof(regs));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= upd7759_state(&s_upd, ctx, load, "upd");
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		s_bg_sx = regs[0];
		s_bg_sy = regs[1];
		s_fg_sx = regs[2];
		s_fg_sy = regs[3];
		s_flip = (uint8_t)regs[4];
		s_invert = (uint8_t)regs[5];
		s_latch = (uint8_t)regs[6];
		s_ym_irq = regs[7];
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
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ph_save(void *ctx) { return ph_state(ctx, 0); }
static int ph_load(void *ctx) { return ph_state(ctx, 1); }
static int ph_supported(const char *machine) { return !strcmp(machine, "prehisle"); }

const rbx_driver rbx_drv_prehisle = { "SNK Prehistoric Isle", ph_init, ph_exit, ph_reset, ph_frame, ph_save, ph_load,
                                      ph_supported, ph_render, ph_help };

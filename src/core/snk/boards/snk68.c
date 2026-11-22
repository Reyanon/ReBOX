/*  SNK 68000: P.O.W., Street Smart, Ikari III and SAR - Search And Rescue. Ported from MAME src/mame/snk/snk68.cpp,
    src/mame/shared/alpha68k_spr.cpp and alpha68k_palette.cpp (BSD-3-Clause, copyright-holders: Bryan McPhail,
    Acho A. Tang, Nicola Salmoria, Angelo Salese).
    68000 at 9 MHz (IRQ 1 held at the vblank); 16x16 sprites in columns of 32 (three groups), an 8x8 text layer,
    the Alpha / Neo Geo style palette (5-bit with the dark bit, the sync color dimming all). Sound: Z80 at 4 MHz,
    YM3812 (its IRQ) and a uPD7759, the latch on the NMI.
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
#define PAL_N 2048

enum { M_POW, M_STREETSM, M_SEARCHAR };

static int s_mach;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t (*s_ram_hp)[0x2000];          // 0x040000 .. 0x043fff
#define s_ram (*s_ram_hp)
static uint16_t s_fgram[0x800];         // 0x100000 (searchar 0x200000) .. +0xfff, mirror 0x1000
static uint16_t (*s_spriteram_hp)[0x4000];    // 0x200000 (searchar 0x100000) .. +0x7fff
#define s_spriteram (*s_spriteram_hp)
static uint16_t s_pal[PAL_N];           // 0x400000 .. 0x400fff
static uint8_t s_flip, s_flip_axis, s_invert;
static uint16_t s_fg_tile_offset;
static rbx_gfx s_gfxtx, s_gfxspr;
static int s_port[7];
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static uint8_t s_lookup[32][2];   // the palette's resistor network (normal, dark)
static struct {
	uint16_t fg[0x800], spr[0x4000], pal[PAL_N], fg_tile_offset;
	uint8_t flip, flip_axis;
} *s_snap;

// ---- sound ----
static z80 s_z80;
static uint8_t s_zram[0x800];
static uint8_t s_latch, s_latch2;
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
	else if (a == 0xf800) s_latch2 = v;
}
static uint8_t z80_in(z80 *z, uint8_t port)
{
	(void)z;
	return port == 0x00 ? rbx_ym_read(0, 0) : 0xff;   // (status_r)
}
static void z80_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	switch (port) {
	case 0x00: rbx_ym_write(0, 0, v); break;
	case 0x20: rbx_ym_write(0, 1, v); break;
	case 0x40:   // d7759_write_port_0_w
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
	if (n > 0) {   // the uPD7759 at the YM's rate, 0.50
		upd7759_render(&s_upd, s_updbuf, n);
		for (int i = 0; i < n; i++)
			s_mix[s_mix_n + i] += s_updbuf[i] / 2;
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
static inline int searchar(void) { return s_mach == M_SEARCHAR; }

static uint16_t rotary(int i) { return (uint16_t)(~(1u << (rbx_port_read(s_port[5 + i]) & 0xf))); }

static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	const uint32_t fg = searchar() ? 0x200000 : 0x100000, spr = searchar() ? 0x100000 : 0x200000;
	if (a >= fg && a < fg + 0x2000) {   // (8-bit; mirror 0x1000)
		const uint16_t v = s_fgram[((a - fg) >> 1) & 0x7ff];
		return searchar() ? v : (uint16_t)(v | 0xff00);
	}
	if (a >= spr && a < spr + 0x8000) {   // spriteram_r: the MSB of every 32-bit word reads 0xff
		const uint32_t o = (a - spr) >> 1;
		return (uint16_t)(s_spriteram[o] | (o & 1 ? 0 : 0xff00));
	}
	if (a >= 0x400000 && a < 0x401000)
		return s_pal[(a - 0x400000) >> 1];
	if (a == 0x0f0000) return (uint16_t)rbx_port_read(s_port[3]);   // DSW1
	if (a == 0x0f0008) return (uint16_t)rbx_port_read(s_port[4]);   // DSW2
	if (!searchar()) {
		if (a == 0x080000)
			return (uint16_t)((rbx_port_read(s_port[1]) & 0xff) << 8 | (rbx_port_read(s_port[0]) & 0xff));   // P2, P1
		if (a == 0x0c0000) return (uint16_t)rbx_port_read(s_port[2]);   // SYSTEM
		return 0;
	}
	switch (a) {
	case 0x080000: return (uint16_t)((rbx_port_read(s_port[0]) ^ s_invert) & 0xff);
	case 0x080002: return (uint16_t)((rbx_port_read(s_port[1]) ^ s_invert) & 0xff);
	case 0x080004: return (uint16_t)((rbx_port_read(s_port[2]) ^ s_invert) & 0xff);
	case 0x0c0000: return (uint16_t)((rotary(0) << 8) & 0xff00);
	case 0x0c8000: return (uint16_t)((rotary(1) << 8) & 0xff00);
	case 0x0d0000: return (uint16_t)(((rotary(1) << 4) & 0xf000) + (rotary(0) & 0x0f00));
	case 0x0f8000: return (uint16_t)(s_latch2 << 8);
	}
	if (a >= 0x300000 && a < 0x340000) {   // the extra code bank: maincpu + 0x40000
		const uint32_t o = 0x40000 + (a - 0x300000);
		return o + 1 < s_rom_size ? *(const uint16_t *)(s_rom + o) : 0;
	}
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	const uint32_t fg = searchar() ? 0x200000 : 0x100000, spr = searchar() ? 0x100000 : 0x200000;
	if (a >= fg && a < fg + 0x2000) {
		uint16_t *w = &s_fgram[((a - fg) >> 1) & 0x7ff];
		if (!searchar())
			v |= 0xff00;   // (fg_videoram_w: data |= 0xff00)
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else if (a >= spr && a < spr + 0x8000) {   // spriteram_w
		const uint32_t o = (a - spr) >> 1;
		if (!(o & 1))
			v |= 0xff00;
		s_spriteram[o] = (uint16_t)((s_spriteram[o] & ~mask) | (v & mask));
	} else if (a >= 0x400000 && a < 0x401000) {
		uint16_t *w = &s_pal[(a - 0x400000) >> 1];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
	} else if (a == 0x080000 && (mask & 0xff00)) {   // sound_w: the latch, an NMI pulse
		s_latch = (uint8_t)(v >> 8);
		z80_gen_nmi(&s_z80);
	} else if (a == 0x0c0000 && (mask & 0x00ff)) {   // flipscreen_w (+ the text bank)
		s_flip = (v >> 3) & 1;
		s_flip_axis = (v >> 2) & 1;
		if (!searchar())
			s_fg_tile_offset = (uint16_t)((v & 0x70) << 4);
	} else if (searchar() && a == 0x080006 && (mask & 0x00ff))
		s_invert = (v & 0xff) == 0x07 ? 0xff : 0x00;
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static uint16_t (*s_frame_hp)[VIS_H][VIS_W];
#define s_frame (*s_frame_hp)

static void draw_sprites(int group)   // alpha68k_sprite_device::draw_sprites (xpos_shift 12), whole screen
{
	const uint16_t *sp = s_snap->spr;
	const uint16_t *tiledata = &sp[0x800 * group];
	const int flip = s_snap->flip;
	for (int offs = 0; offs < 0x800; offs += 0x40) {
		int mx = (sp[offs + 2 * group] & 0xff) << 4;
		int my = sp[offs + 2 * group + 1];
		mx = mx | (my >> 12);
		mx = ((mx + 16) & 0x1ff) - 16;
		my = -my;
		if (flip) {
			mx = 240 - mx;
			my = 240 - my;
		}
		for (int i = 0; i < 0x20; i++, tiledata += 2) {
			my &= 0x1ff;
			const int y0 = my;
			if (flip)
				my -= 16;
			else
				my += 16;
			if (!(y0 <= VIS_Y0 + VIS_H - 1 && y0 + 15 >= VIS_Y0))
				continue;
			uint32_t tile = tiledata[1];
			int fx, fy;
			if (s_mach == M_POW) {   // tile_callback_pow
				fx = (tile & 0x4000) != 0;
				fy = (tile & 0x8000) != 0;
				tile &= 0x3fff;
			} else if (s_snap->flip_axis) {   // tile_callback_notpow
				fx = 0;
				fy = (tile & 0x8000) != 0;
				tile &= 0x7fff;
			} else {
				fx = (tile & 0x8000) != 0;
				fy = 0;
				tile &= 0x7fff;
			}
			if (flip) {
				fx = !fx;
				fy = !fy;
			}
			tile %= s_gfxspr.count;
			if (!s_gfxspr.used[tile])
				continue;
			const uint16_t c0 = (uint16_t)((tiledata[0] & 0x7f) * 16);
			const uint8_t *g = s_gfxspr.pix + tile * 256;
			const int sy = y0 - VIS_Y0;
			for (int j = 0; j < 16; j++) {
				const int yy = sy + j;
				if ((unsigned)yy >= VIS_H)
					continue;
				const uint8_t *src = g + (fy ? 15 - j : j) * 16;
				uint16_t *d = s_frame[yy];
				for (int k = 0; k < 16; k++) {
					const int xx = mx + k;
					if ((unsigned)xx >= VIS_W)
						continue;
					const uint8_t p = src[fx ? 15 - k : k];
					if (p)
						d[xx] = (uint16_t)(c0 + p);
				}
			}
		}
	}
}

static void fg_line(int by, uint16_t *d)   // 32x32 of 8x8 (scan columns), pen 0 transparent
{
	const int flip = s_snap->flip;
	const uint32_t ty = (uint32_t)(flip ? 255 - by : by) & 255;
	for (int x = 0; x < VIS_W; x += 8) {
		const uint32_t tx = (uint32_t)(flip ? 255 - x : x) & 255;
		const uint32_t i = (tx >> 3) * 32 + (ty >> 3);
		uint32_t code, color;
		if (s_mach == M_SEARCHAR) {
			const uint16_t data = s_snap->fg[2 * i];
			code = data & 0x7ffu;
			color = (data & 0x7000u) >> 12;
		} else {
			code = s_snap->fg_tile_offset + (s_snap->fg[2 * i] & 0xffu);
			color = s_snap->fg[2 * i + 1] & 0x07u;
		}
		code %= s_gfxtx.count;
		if (!s_gfxtx.used[code])
			continue;
		const uint8_t *src = s_gfxtx.pix + code * 64 + (ty & 7) * 8;
		for (int k = 0; k < 8; k++) {
			const uint8_t p = src[flip ? 7 - k : k];
			if (p)
				d[x + k] = (uint16_t)(color * 16 + p);
		}
	}
}

#define BANDS 8
static int s_phase;
static void sk_band(int band)
{
	for (int y = VIS_H * band / BANDS; y < VIS_H * (band + 1) / BANDS; y++) {
		uint16_t *d = s_frame[y];
		if (s_phase == 0)   // the backdrop
			for (int x = 0; x < VIS_W; x++)
				d[x] = PAL_N - 1;
		else {              // (the sprites), the text
			fg_line(y + VIS_Y0, d);
			memcpy(rbx_video.bitmap + y * rbx_video.pitch, d, VIS_W * 2);
		}
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, sk_band };
static int sk_help(int helper) { return decoband_help(&s_bands, helper); }

static void sk_render(void)
{
	const uint16_t p0 = s_snap->pal[0];
	const int shift = (p0 & 0x7fff) == 0 ? 0 : 2;   // (the sync color: not black dims every entry)
	for (int i = 0; i < PAL_N; i++) {   // set_color_entry
		const uint16_t p = s_snap->pal[i];
		const int dark = p >> 15, sh = i == 0 ? 0 : shift;
		const int r = (((p >> 14) & 1) | ((p >> 7) & 0x1e)) >> sh;
		const int g = (((p >> 13) & 1) | ((p >> 3) & 0x1e)) >> sh;
		const int b = (((p >> 12) & 1) | ((p << 1) & 0x1e)) >> sh;
		rbx_video.palette[i] = (uint32_t)s_lookup[r][dark] << 16 | (uint32_t)s_lookup[g][dark] << 8 | s_lookup[b][dark];
	}
	s_phase = 0;
	decoband_run(&s_bands);
	draw_sprites(2);   // draw_sprites_all
	draw_sprites(3);
	draw_sprites(1);
	s_phase = 1;
	decoband_run(&s_bands);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->fg, s_fgram, sizeof(s_fgram));
	memcpy(s_snap->spr, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->fg_tile_offset = s_fg_tile_offset;
	s_snap->flip = s_flip;
	s_snap->flip_axis = s_flip_axis;
	rbx_video_frame_done();
}

static void rgb_lookups(void)   // alpha68k_palette create_rgb_lookups (compute_resistor_weights, normal and dark)
{
	static const double res[5] = { 3900, 2200, 1000, 470, 220 };
	static const double pd[2] = { 0, 8200 };
	double w[2][5];
	double scale = 0;
	for (int net = 0; net < 2; net++) {
		double sum = 0;
		for (int i = 0; i < 5; i++) {
			double r0 = pd[net] == 0 ? 1.0 / 1e12 : 1.0 / pd[net], r1 = 1.0 / 1e12;
			for (int j = 0; j < 5; j++) {
				if (j == i)
					r1 += 1.0 / res[j];
				else
					r0 += 1.0 / res[j];
			}
			r0 = 1.0 / r0;
			r1 = 1.0 / r1;
			double v = 255.0 * r0 / (r1 + r0);
			if (v < 0) v = 0;
			if (v > 255) v = 255;
			w[net][i] = v;
			sum += v;
		}
		if (net == 0)
			scale = 255.0 / sum;
	}
	for (int i = 0; i < 32; i++)
		for (int net = 0; net < 2; net++) {
			double s = 0;
			for (int b = 0; b < 5; b++)
				if ((i >> b) & 1)
					s += w[net][b] * scale;
			s_lookup[i][net] = (uint8_t)(int)(s + 0.5);
		}
}

// ---- init / frame ----
static void sk_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_fgram, 0, sizeof(s_fgram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	s_flip = s_flip_axis = s_invert = 0;
	s_fg_tile_offset = 0;
	s_latch = s_latch2 = 0;
	s_ym_irq = 0;
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	upd7759_reset(&s_upd);
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void sk_exit(void);

static int sk_init(const rbx_game *g)
{
	if (!s_frame_hp && !(s_frame_hp = calloc(1, sizeof(*s_frame_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_spriteram_hp && !(s_spriteram_hp = calloc(1, sizeof(*s_spriteram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	const char *m = g->machine;
	s_mach = !strcmp(m, "searchar") ? M_SEARCHAR : !strcmp(m, "streetsm") ? M_STREETSM : M_POW;
	uint32_t ts, ss, us;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	uint8_t *tx = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss), *upd = rbx_region("upd", &us);
	if (!s_rom || s_rom_size < 0x40000 || !s_zrom || !tx || !spr || !upd) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	{	// charlayout, spritelayout (RGN_FRAC(1,2))
		static uint32_t xc[8], yc[8], xs[16], ys[16];
		for (int i = 0; i < 4; i++) {
			xc[i] = (uint32_t)(8 * 8 + 3 - i);
			xc[4 + i] = (uint32_t)(3 - i);
		}
		for (int i = 0; i < 8; i++) {
			yc[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(32 * 8 + 7 - i);
			xs[8 + i] = (uint32_t)(7 - i);
		}
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)(i * 16);
		const uint32_t ht = ts * 4, hs = ss * 4;
		const uint32_t pc[4] = { 0, 4, ht, ht + 4 }, psp[4] = { 0, 8, hs, hs + 8 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, xc, yc, 16 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, psp, xs, ys, 64 * 8 };
		if (rbx_gfx_decode(&s_gfxtx, &lc, tx, ts / 2) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ss / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sk_exit();
			return -1;
		}
	}
	rgb_lookups();
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // YM3812 1.0, uPD7759 0.50
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		sk_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_updbuf = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_updbuf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_N)) {
		rbx_fail("%s: out of memory", g->set);
		sk_exit();
		return -1;
	}
	upd7759_init(&s_upd, upd, us, UPD_CLOCK, rbx_ym_rate(0));
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0xf0 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	static const char *const names[7] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2", "ROT1", "ROT2" };
	for (int i = 0; i < 7; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x3ffff, s_rom, 0x40000, 1, 0);
	m68k_bus_map(0x040000, 0x043fff, s_ram, sizeof(s_ram), 1, 1);
	if (s_mach == M_SEARCHAR && s_rom_size >= 0x80000)
		m68k_bus_map(0x300000, 0x33ffff, s_rom + 0x40000, 0x40000, 1, 0);   // (the extra code bank)
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	sk_reset();
	return 0;
}

static void sk_exit(void)
{
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	free(s_mix);
	free(s_updbuf);
	s_mix = s_updbuf = NULL;
	rbx_gfx_free(&s_gfxtx);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_frame_hp);
	s_frame_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
	free(s_spriteram_hp);
	s_spriteram_hp = NULL;
}

static void sk_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_Y0 + VIS_H) {   // the vblank: the frame drawn, irq1_line_hold
			vblank_capture();
			m68k_bus_hold_irq(1);
		}
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
}

static int sk_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	uint16_t regs[8] = { s_flip, s_flip_axis, s_invert, s_fg_tile_offset, s_latch, s_latch2, (uint16_t)s_ym_irq, 0 };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "fg", s_fgram, sizeof(s_fgram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
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
		s_flip = (uint8_t)regs[0];
		s_flip_axis = (uint8_t)regs[1];
		s_invert = (uint8_t)regs[2];
		s_fg_tile_offset = regs[3];
		s_latch = (uint8_t)regs[4];
		s_latch2 = (uint8_t)regs[5];
		s_ym_irq = regs[6];
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

static int sk_save(void *ctx) { return sk_state(ctx, 0); }
static int sk_load(void *ctx) { return sk_state(ctx, 1); }
static int sk_supported(const char *machine)
{
	return !strcmp(machine, "pow") || !strcmp(machine, "streetsm") || !strcmp(machine, "searchar");
}

const rbx_driver rbx_drv_snk68 = { "SNK 68000", sk_init, sk_exit, sk_reset, sk_frame, sk_save, sk_load, sk_supported,
                                   sk_render, sk_help };

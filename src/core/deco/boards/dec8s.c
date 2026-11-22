/*  Data East "dec8" Super Real Darwin board, from MAME src/mame/dataeast/dec8.cpp, dec8_v.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail): SRD: Super Real Darwin (and the bootleg without the MCU).
    One 6809 at 1.5 MHz (NMI at vblank), the i8751 (the command and return words; the coins on its port 3; MAME runs
    them in perfect sync: here interleaved by sixteenths of a line), a 512x256 background of 16x16 tiles (x scroll;
    four transparency groups split the pens between behind and over the sprites), a byte-coded sprite list (buffered),
    an 8x8 text layer, the DECO RM-C3 palette (144 entries). The sound: DECO 222 (a 6502 with its opcodes' bits 5
    and 6 swapped) at 1.5 MHz with YM2203 and YM3812, the latch on its NMI. Reached through the dec8 driver (dec8.c).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1500000
#define SND_CLOCK 1500000
#define MCU_CLOCK (8000000 / 12)
#define YM1_CLOCK 1500000        // YM2203
#define YM2_CLOCK 3000000        // YM3812
#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 8, 248)
#define LINES 272
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256
#define SLICES 16

static int s_boot;   // (srdarwinb: no MCU, the coins read by the CPU)
static uint8_t *s_rom, *s_bank_rom, *s_mcu_rom;
static uint32_t s_rom_size, s_bank_size, s_mcu_size;
static uint8_t s_ram[0x1800];              // 0x0000 .. 0x17ff (RAM, sprites, text, RAM, background)
static uint8_t s_sprbuf[0x200];            // (the buffered sprite RAM)
static uint8_t s_pal[0x90], s_pal_ext[0x90];
static uint8_t s_scroll[2];
static int s_bank, s_flip;
static m6809_t s_cpu;
static rbx_gfx s_gfx8, s_gfxspr, s_gfx16[4];
static uint8_t (*s_pri_buf_hp)[VIS_W * VIS_H];
#define s_pri_buf (*s_pri_buf_hp)
static int s_port_in[2], s_port_coin, s_port_dsw[2];
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac;
static struct {
	uint8_t txt[0x400], bg[0x400], spr[0x200], pal[0x90], pal_ext[0x90];
	uint8_t scroll[2];
	int flip;
} *s_snap;

#define SPR 0x0600   // (s_ram offsets)
#define TXT 0x0800
#define BG 0x1400

// ---- the i8751 ----
static mcs51_t s_mcu;
static uint8_t s_p0, s_p2;
static uint16_t s_mcu_return, s_mcu_value;

static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	if (port == 0) return s_p0;
	if (port == 3) return (uint8_t)rbx_port_read(s_port_coin);
	return 0xff;
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 0) { s_p0 = data; return; }
	if (port != 2)
		return;
	const uint8_t fall = (uint8_t)(~data & s_p2);   // srdarwin mcu_to_main_w (a single port)
	s_p2 = data;
	if (fall & 0x10) s_p0 = (uint8_t)(s_mcu_value >> 8);
	if (fall & 0x20) s_p0 = (uint8_t)(s_mcu_value & 0xff);
	if (fall & 0x40) s_mcu_return = (uint16_t)((s_mcu_return & 0xff) | (s_p0 << 8));
	if (fall & 0x80) s_mcu_return = (uint16_t)((s_mcu_return & 0xff00) | s_p0);
	if (!(data & 0x02)) mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
}

// ---- sound: DECO 222, YM2203, YM3812 (dec8_s_map) ----
static m6502_t s_snd;
static uint8_t s_sram[0x600];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;
static int s_ym_irq[2];
static int32_t *s_mix, *s_ym2buf;
static int s_mix_n, s_mix_cap, s_ym2_n, s_ym2_cap;

static void snd_irq(void) { m6502_set_irq(&s_snd, s_ym_irq[0] | s_ym_irq[1]); }
static void ym1_irq(int state) { s_ym_irq[0] = state; snd_irq(); }
static void ym2_irq(int state) { s_ym_irq[1] = state; snd_irq(); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0600) return s_sram[a];
	if (a == 0x6000) {
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	if (a >= 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0600) s_sram[a] = v;
	else if (a == 0x2000 || a == 0x2001) rbx_ym_write(0, a & 1, v);
	else if (a == 0x4000 || a == 0x4001) rbx_ym_write(1, a & 1, v);
}
static void sound_line(void)
{
	s_snd_frac += (uint64_t)SND_CLOCK * 1000;
	const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
	const int64_t cyc = (int64_t)(s_snd_frac / den);
	s_snd_frac -= (uint64_t)cyc * den;
	m6502_run_until(&s_snd, s_snd.cycles + cyc);
	s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym1_frac, YM1_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
	                          s_mix_cap - s_mix_n);
	s_ym2_n += rbx_ym_advance(1, (uint32_t)rbx_line_cycles(&s_ym2_frac, YM2_CLOCK, FRAME_MHZ, LINES), s_ym2buf + s_ym2_n,
	                          s_ym2_cap - s_ym2_n);
}
static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	rbx_ym_async_take(1, s_ym2buf, s_ym2_n);
	if (s_ym2_n > 0)
		for (int i = 0; i < s_mix_n; i++)
			s_mix[i] += s_ym2buf[(int)((int64_t)i * s_ym2_n / s_mix_n)];
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_ym2buf, 0, sizeof(int32_t) * (size_t)s_ym2_n);
	s_mix_n = s_ym2_n = 0;
}

// ---- bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)
{
	s_bank = b & 7;
	const uint32_t o = (uint32_t)s_bank * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = o + 0x4000 <= s_bank_size ? s_bank_rom + o + (uint32_t)p * 256 : NULL;
}

static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (a) {
	case 0x2000: return s_boot ? 0 : (uint8_t)(s_mcu_return >> 8);
	case 0x2001: return s_boot ? (uint8_t)rbx_port_read(s_port_coin) : (uint8_t)(s_mcu_return & 0xff);
	case 0x3800: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x3801: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 0x3802:
		rbx_port_custom(s_port_in[1], 0x40, vblank() ? 0x40 : 0);
		return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0x3803: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	}
	return 0;
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x2800 && a < 0x2890) { s_pal[a - 0x2800] = v; return; }
	if (a >= 0x3000 && a < 0x3090) { s_pal_ext[a - 0x3000] = v; return; }
	switch (a) {
	case 0x1800:   // i8751_hi_w: SECIRQ when P21 is high
		if (s_boot) return;
		s_mcu_value = (uint16_t)((s_mcu_value & 0xff) | (v << 8));
		if (s_p2 & 2)
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
		return;
	case 0x1801:
		if (!s_boot)
			s_mcu_value = (uint16_t)((s_mcu_value & 0xff00) | v);
		return;
	case 0x1804: memcpy(s_sprbuf, s_ram + SPR, sizeof(s_sprbuf)); return;   // buffered_spriteram8 write
	case 0x1805:   // control_w: the bank (bits 5-7), the scroll MSBs
		set_bank(v >> 5);
		s_scroll[0] = v & 0x0f;
		return;
	case 0x1806: s_scroll[1] = v; return;
	case 0x2000: s_latch = v; m6502_set_nmi(&s_snd, 1); return;
	case 0x2001: s_flip = v; return;
	}
}

// ---- video ----
static inline uint8_t rmc3(int v) { return (uint8_t)(0x0e * (v & 1) + 0x1f * ((v >> 1) & 1) + 0x43 * ((v >> 2) & 1) + 0x8f * ((v >> 3) & 1)); }

// a transparent sprite element with MAME's prio_transpen: drawn where (1 << pri) is not in pmask (bit 31 implied),
// then pri = 31
static void sprite(uint32_t code, int colour, int fx, int fy, int sx, int sy, uint32_t pmask, uint16_t *bm, int pitch)
{
	if (!s_gfxspr.count)
		return;
	code %= s_gfxspr.count;
	pmask |= 1u << 31;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int y = 0; y < 16; y++) {
		const int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *row = src + (fy ? 15 - y : y) * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[fx ? 15 - x : x];
			if (!p)
				continue;
			uint8_t *pr = &s_pri_buf[dy * VIS_W + dx];
			if (!((1u << (*pr & 0x1f)) & pmask))
				bm[dy * pitch + dx] = (uint16_t)(64 + colour * 8 + p);
			*pr = 31;
		}
	}
}

static void d8s_render(void)
{
	for (int i = 0; i < 144; i++) {   // deco_rgb_decoder: xxxxBBBBGGGGRRRR (the byte and its extension)
		const int raw = s_snap->pal[i] | (s_snap->pal_ext[i] << 8);
		rbx_video.palette[i] = (uint32_t)(rmc3(raw & 15) << 16 | rmc3((raw >> 4) & 15) << 8 | rmc3((raw >> 8) & 15));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip != 0;
	// the background: 32x16 of 16x16 (rows), tile bits 12-13 the colour and the transparency group: group 0 behind
	// the sprites (LAYER1, priority 1), group 3 over them (LAYER0, priority 2), groups 1-2 pens 0-7 behind, 8-15 over
	const int sx = ((s_snap->scroll[0] << 8) + s_snap->scroll[1]) & 0x1ff;
	const uint8_t *bg = s_snap->bg;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *d = bm + y * pitch;
		uint8_t *pr = s_pri_buf + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			int px = (x + sx) & 0x1ff, py = y + VIS_Y0;
			if (flip) {
				px = 511 - px;
				py = 255 - py;
			}
			const int idx = (py >> 4) * 32 + (px >> 4);
			const int tile = bg[idx * 2 + 1] | (bg[idx * 2] << 8);
			const int colour = (tile >> 12) & 3;
			const rbx_gfx *g = &s_gfx16[(tile >> 8) & 3];
			const uint32_t code = rbx_wrap((uint32_t)(tile & 0xff), g->count);
			const uint8_t p = g->pix[(code * 16 + (uint32_t)(py & 15)) * 16 + (uint32_t)(px & 15)];
			d[x] = (uint16_t)(colour * 16 + p);
			pr[x] = (uint8_t)(colour == 3 || (colour != 0 && p >= 8) ? 2 : 1);
		}
	}
	const uint8_t *sr = s_snap->spr;
	for (int offs = 0x200 - 4; offs >= 0; offs -= 4) {   // srdarwin draw_sprites
		const int colour = (sr[offs + 1] & 0x03) + ((sr[offs + 1] & 0x08) >> 1);
		const uint32_t pmask = colour == 0 ? 0xcc : 0;   // (GFX_PMASK_2: behind the foreground pens)
		const uint32_t code = (uint32_t)(sr[offs + 3] + ((sr[offs + 1] & 0xe0) << 3));
		if (!code)
			continue;
		int sy = sr[offs];
		if (sy == 0xf8)
			continue;
		int sx2 = 241 - sr[offs + 2];
		int fx = (sr[offs + 1] & 0x04) != 0;
		const int multi = (sr[offs + 1] & 0x10) != 0;
		int sy2;
		if (flip) {
			sy = 240 - sy;
			sx2 = 240 - sx2;
			fx = !fx;
			sy2 = sy - 16;
		} else
			sy2 = sy + 16;
		sprite(code, colour, fx, flip, sx2, sy, pmask, bm, pitch);
		if (multi)
			sprite(code + 1, colour, fx, flip, sx2, sy2, pmask, bm, pitch);
	}
	const uint8_t *txt = s_snap->txt;
	for (int y = 0; y < VIS_H; y++) {   // the text layer: 32x32 of 8x8 (a byte a tile), 2 bpp, colour 0, pen 0 clear
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const uint32_t tile = txt[(py >> 3) * 32 + (px >> 3)];
			const uint32_t code = rbx_wrap(tile, s_gfx8.count);
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)(128 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->txt, s_ram + TXT, sizeof(s_snap->txt));
	memcpy(s_snap->bg, s_ram + BG, sizeof(s_snap->bg));
	memcpy(s_snap->spr, s_sprbuf, sizeof(s_sprbuf));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	memcpy(s_snap->pal_ext, s_pal_ext, sizeof(s_pal_ext));
	memcpy(s_snap->scroll, s_scroll, sizeof(s_scroll));
	s_snap->flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void d8s_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_sprbuf, 0, sizeof(s_sprbuf));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_pal_ext, 0, sizeof(s_pal_ext));
	memset(s_scroll, 0, sizeof(s_scroll));
	s_flip = 0;
	set_bank(0);
	m6809_reset(&s_cpu);
	m6809_set_nmi(&s_cpu, 0);
	s_p0 = 0;
	s_p2 = 0xff;
	s_mcu_return = s_mcu_value = 0;
	if (!s_boot)
		mcs51_reset(&s_mcu);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	s_ym_irq[0] = s_ym_irq[1] = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	rbx_ym_reset(1);
	s_mix_n = s_ym2_n = 0;
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym1_frac = s_ym2_frac = 0;
}

static void d8s_exit(void);

static int d8s_init(const rbx_game *g)
{
	if (!s_pri_buf_hp && !(s_pri_buf_hp = calloc(1, sizeof(*s_pri_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_boot = !strcmp(g->machine, "srdarwinb");
	uint32_t csize, tsize, ssize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_bank_rom = rbx_region("mainbank", &s_bank_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = s_boot ? NULL : rbx_region("mcu", &s_mcu_size);
	uint8_t *chars = rbx_region("char", &csize), *tiles = rbx_region("tiles1", &tsize), *spr = rbx_region("sprites", &ssize);
	if (!s_rom || s_rom_size < 0x10000 || !s_bank_rom || !s_zrom || (!s_boot && (!s_mcu_rom || s_mcu_size < 0x1000)) ||
	    !chars || csize < 0x4000 || !tiles || tsize < 0x40000 || !spr || ssize < 0x30000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_boot)   // init_srdarwinb: the sprite ROMs' middle bytes of each four swapped
		for (uint32_t i = 0; i + 3 < 0x30000; i += 4) {
			const uint8_t b = spr[i + 1];
			spr[i + 1] = spr[i + 2];
			spr[i + 2] = b;
		}
	{	// charlayout_16k (2 bpp, the planes in the nibbles, the left half 0x2000 on), sr_sprites (3 bpp),
		// srdarwin_tiles (4 entries of 256 at 0x10000 apart, the planes in the nibbles)
		static uint32_t xc[8], y8[8], xs[16], y16[16], xt[16];
		for (int i = 0; i < 4; i++) {
			xc[i] = (uint32_t)(0x2000 * 8 + i);
			xc[4 + i] = (uint32_t)i;
			xt[i] = (uint32_t)i;
			xt[4 + i] = (uint32_t)(1024 * 8 * 8 + i);
			xt[8 + i] = (uint32_t)(16 * 8 + i);
			xt[12 + i] = (uint32_t)(16 * 8 + 1024 * 8 * 8 + i);
		}
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(16 * 8 + i);
			xs[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t pc[2] = { 0, 4 }, ps[3] = { 0x10000 * 8, 0x20000 * 8, 0 };
		static const uint32_t pt[4] = { 0x8000 * 8, 0x8000 * 8 + 4, 0, 4 };
		const rbx_gfx_layout l8 = { 8, 8, 2, pc, xc, y8, 8 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, y16, 16 * 16 };
		const rbx_gfx_layout lt = { 16, 16, 4, pt, xt, y16, 32 * 8 };
		int rc = rbx_gfx_decode(&s_gfx8, &l8, chars, 1024 * 8) || rbx_gfx_decode(&s_gfxspr, &ls, spr, 2048 * 32);
		for (int k = 0; k < 4; k++)
			rc = rc || rbx_gfx_decode(&s_gfx16[k], &lt, tiles + 0x10000 * k, 256 * 32);
		if (rc) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			d8s_exit();
			return -1;
		}
	}
	if (!s_boot) {
		mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
		s_mcu.port_in = mcu_in;
		s_mcu.port_out = mcu_out;
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2203 1.5 MHz (SSG 0.20, FM 0.40), YM3812 3 MHz 0.80
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM1_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 51;
	yc.gain[3] = 102;
	yc.irq = ym1_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM2_CLOCK;
	yc.gain[0] = 205;
	yc.irq = ym2_irq;
	rc = rc || rbx_ym_init(1, &yc) || rbx_ym_set_async(1, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_ym2_cap = (int)(rbx_ym_rate(1) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		s_ym2buf = (int32_t *)calloc((size_t)s_ym2_cap, sizeof(int32_t));
		rc = !s_mix || !s_ym2buf || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 144);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		d8s_exit();
		return -1;
	}
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.deco222 = 1;
	for (int p = 0; p < 6; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		if ((uint32_t)p * 256 + 256 <= s_zrom_size)
			s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	m6809_init(&s_cpu);
	for (int p = 0; p < 0x18; p++) {   // (RAM, sprites, text, RAM, background)
		s_cpu.rpage[p] = s_ram + p * 256;
		s_cpu.wpage[p] = s_ram + p * 256;
	}
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_coin = rbx_port_find("COIN");
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	d8s_reset();
	return 0;
}

static void d8s_exit(void)
{
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	free(s_mix);
	free(s_ym2buf);
	s_mix = s_ym2buf = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfxspr);
	for (int k = 0; k < 4; k++)
		rbx_gfx_free(&s_gfx16[k]);
	free(s_snap);
	s_snap = NULL;
	free(s_pri_buf_hp);
	s_pri_buf_hp = NULL;
}

static void d8s_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // screen_vblank on the NMI
			vblank_capture();
			m6809_set_nmi(&s_cpu, 1);
		} else if (line == VIS_Y0)
			m6809_set_nmi(&s_cpu, 0);
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		if (s_boot)
			m6809_run_until(&s_cpu, s_cpu.cycles + cyc);
		else {   // (MAME: perfect sync between the CPU and the MCU)
			const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
			const int64_t t0 = s_cpu.cycles;
			for (int k = 0; k < SLICES; k++) {
				m6809_run_until(&s_cpu, t0 + cyc * (k + 1) / SLICES);
				mcs51_execute(&s_mcu, mc * (k + 1) / SLICES - mc * k / SLICES);
			}
		}
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int d8s_state(void *ctx, int load)
{
	uint8_t cpu[M6809_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6809_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[5] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym1_frac, s_ym2_frac };
	int32_t misc[9] = { (int32_t)s_frame, s_bank, s_flip, s_p0, s_p2, s_mcu_return, s_mcu_value, s_latch,
	                    s_ym_irq[0] | s_ym_irq[1] << 1 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6809", cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "sprb", s_sprbuf, sizeof(s_sprbuf));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "palx", s_pal_ext, sizeof(s_pal_ext));
	rc |= rbx_state_io(ctx, "scroll", s_scroll, sizeof(s_scroll));
	if (!s_boot) {
		rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "mcuf", s_mcu.forced_inputs, sizeof(s_mcu.forced_inputs));
	}
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6809_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym1_frac = fr[3];
		s_ym2_frac = fr[4];
		s_frame = (uint32_t)misc[0];
		set_bank(misc[1]);
		s_flip = misc[2];
		s_p0 = (uint8_t)misc[3];
		s_p2 = (uint8_t)misc[4];
		s_mcu_return = (uint16_t)misc[5];
		s_mcu_value = (uint16_t)misc[6];
		s_latch = (uint8_t)misc[7];
		s_ym_irq[0] = misc[8] & 1;
		s_ym_irq[1] = (misc[8] >> 1) & 1;
	}
	return rc ? -1 : 0;
}

static int d8s_save(void *ctx) { return d8s_state(ctx, 0); }
static int d8s_load(void *ctx) { return d8s_state(ctx, 1); }
static int d8s_supported(const char *machine) { return !strcmp(machine, "srdarwin") || !strcmp(machine, "srdarwinb"); }

const rbx_driver rbx_drv_dec8s = { "DECO dec8", d8s_init, d8s_exit, d8s_reset, d8s_frame, d8s_save, d8s_load,
                                   d8s_supported, d8s_render, NULL };

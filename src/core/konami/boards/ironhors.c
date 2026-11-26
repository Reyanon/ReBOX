/*  Konami Iron Horse / Dai Ressya Goutou. Ported from MAME src/mame/konami/ironhors.cpp (BSD-3-Clause,
    copyright-holders: Mirko Buffoni) and the palette's resistor network of src/emu/video/resnet.cpp (BSD-3-Clause,
    copyright-holders: Couriersud). A 6809E at 1.536 MHz (its FIRQ every other frame and NMIs every 64 lines from a
    scanline timer), a 32x32 tilemap with row scroll, the 16x16 / 16x8 / 8x16 / 8x8 sprites, the colours through the
    PROMs; the sound: a Z80 at 3.072 MHz and a YM2203, a frame behind on the worker (the discrete RC filters not done:
    the routes' gains set to MAME's levels). The tilemap's flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/filter_rc.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12)
#define Z80_CLOCK 3072000u     // (18.432 MHz / 6, the YM2203's too)
#define SND_RATE 48000
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0+8, 256-8, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 8
#define VIS_W 240
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x1000           // (16 * 8 * 16 * 2 pens over 256 colours)

static uint8_t *s_rom, *s_zrom, *s_prom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_chr, *s_spr16;   // (a byte a pixel: the 8x8 characters (the 8x8 sprites too), the 16x16 sprites)
static uint32_t s_nchr, s_nspr16;
static uint32_t s_rgb[256];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static int s_port_dsw[3], s_port_p[2], s_port_system;

static struct {
	uint8_t ram0[0x100];     // 0000-00ff (0003 charbank_w, 0004 the interrupt enable, 0020-003f the row scroll)
	uint8_t ram2[0x2000];    // 2000-3fff (2000 the colour RAM, 2400 the video RAM, 3000 / 3800 the sprite RAMs)
	uint8_t charbank, spriterambank, palettebank, flip;
	uint8_t zram[0x400], soundlatch, z80_irq;
	int32_t hp_x, hp_y;   // (the output's high-pass)
	filter_rc_t flt[4];   // (the SSG channels' switched RC filters, filter_w; [3] the FM's fixed one)
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t scroll[0x20], cram[0x400], vram[0x400], spr[0x100], charbank, palettebank, flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.scroll, t.ram0 + 0x20, sizeof(s_vs.scroll));
	memcpy(s_vs.cram, t.ram2, sizeof(s_vs.cram));
	memcpy(s_vs.vram, t.ram2 + 0x400, sizeof(s_vs.vram));
	memcpy(s_vs.spr, t.ram2 + (t.spriterambank ? 0x1800 : 0x1000), sizeof(s_vs.spr));   // (bank 0 at 3800, 1 at 3000)
	s_vs.charbank = t.charbank;
	s_vs.palettebank = t.palettebank;
	s_vs.flip = t.flip;
}

static void draw_gfx(const uint8_t *src, int sz, int pen, int flipx, int flipy, int sx, int sy, uint16_t *bm, int pitch)
{
	for (int j = 0; j < sz; j++) {
		const int yy = sy + j;
		if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
			continue;
		const uint8_t *s = src + (flipy ? sz - 1 - j : j) * sz;
		uint16_t *d = bm + (yy - VIS_Y0) * pitch - VIS_X0;
		for (int i = 0; i < sz; i++) {
			const int xx = sx + i;
			if (xx < VIS_X0 || xx >= VIS_X0 + VIS_W)
				continue;
			const int p = s[flipx ? sz - 1 - i : i];
			if (p)
				d[xx] = (uint16_t)(pen + p);
		}
	}
}

static void ih_render(void)   // (screen_update)
{
	for (int p = 0; p < NPENS; p++) {   // (pen ((i & 0x100) << 3) | (j << 8) | (i & 0xff): (j << 5) | (~i & 0x100) >> 4 | PROM)
		const int i = (p & 0xff) | (((p >> 11) & 1) << 8), j = (p >> 8) & 7;
		rbx_video.palette[p] = s_rgb[(j << 5) | ((~i & 0x100) >> 4) | (s_prom[0x300 + i] & 0x0f)];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {   // (the tilemap, set_scroll_rows(32))
		uint16_t *d = bm + (y - VIS_Y0) * pitch - VIS_X0;
		const int row = y >> 3;
		const int sx = s_vs.scroll[row & 31];
		for (int x = VIS_X0; x < VIS_X0 + VIS_W;) {
			const int tx = (x + sx) & 255;
			int n = 8 - (tx & 7);
			if (n > VIS_X0 + VIS_W - x)
				n = VIS_X0 + VIS_W - x;
			const int idx = row * 32 + (tx >> 3);
			const int c = s_vs.cram[idx];
			const int code = s_vs.vram[idx] + ((c & 0x40) << 2) + ((c & 0x20) << 4) + (s_vs.charbank << 10);
			const int pen = ((c & 0x0f) + 16 * s_vs.palettebank) * 16;
			const int fx = (c & 0x10) != 0, fy = (c & 0x20) != 0;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < n; i++) {
				const int px = (tx + i) & 7;
				d[x + i] = (uint16_t)(pen + src[fx ? 7 - px : px]);
			}
			x += n;
		}
	}
	const uint8_t *sr = s_vs.spr;
	for (int offs = 0; offs < 0x100 - 0x100 % 5; offs += 5) {   // (draw_sprites)
		int sx = sr[offs + 3], sy = sr[offs + 2];
		int flipx = (sr[offs + 4] & 0x20) != 0, flipy = (sr[offs + 4] & 0x40) != 0;
		const int code = (sr[offs] << 2) + ((sr[offs + 1] & 0x03) << 10) + ((sr[offs + 1] & 0x0c) >> 2);
		const int pen = 16 * 8 * 16 + (((sr[offs + 1] & 0xf0) >> 4) + 16 * s_vs.palettebank) * 16;
		if (s_vs.flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
#define C8(c) (s_chr + (rbx_wrap((uint32_t)(c), s_nchr)) * 64)
		switch (sr[offs + 4] & 0x0c) {
		case 0x00:   // (16x16)
			draw_gfx(s_spr16 + (rbx_wrap((uint32_t)(code / 4), s_nspr16)) * 256, 16, pen, flipx, flipy, sx, sy, bm, pitch);
			break;
		case 0x04:   // (16x8)
			if (s_vs.flip)
				sy += 8;
			draw_gfx(C8(code & ~1), 8, pen, flipx, flipy, flipx ? sx + 8 : sx, sy, bm, pitch);
			draw_gfx(C8(code | 1), 8, pen, flipx, flipy, flipx ? sx : sx + 8, sy, bm, pitch);
			break;
		case 0x08:   // (8x16)
			draw_gfx(C8(code & ~2), 8, pen, flipx, flipy, sx, flipy ? sy + 8 : sy, bm, pitch);
			draw_gfx(C8(code | 2), 8, pen, flipx, flipy, sx, flipy ? sy : sy + 8, bm, pitch);
			break;
		default:   // (8x8)
			draw_gfx(C8(code), 8, pen, flipx, flipy, sx, sy, bm, pitch);
			break;
		}
#undef C8
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

static void q_put(int kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- the main CPU's bus (master_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x00e0)
		return t.ram0[a];
	switch (a) {
	case 0x0900: return port(s_port_dsw[2]);
	case 0x0a00: return port(s_port_dsw[1]);
	case 0x0b00: return port(s_port_dsw[0]);
	case 0x0b01: return port(s_port_p[1]);
	case 0x0b02: return port(s_port_p[0]);
	case 0x0b03: return port(s_port_system);
	}
	if (a >= 0x2000 && a < 0x4000)
		return t.ram2[a - 0x2000];
	if (a >= 0x4000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x00e0) {
		t.ram0[a] = v;
		if (a == 0x0003) {   // (charbank_w)
			t.charbank = v & 0x03;
			t.spriterambank = v & 0x08;
		}
		return;
	}
	if (a >= 0x2000 && a < 0x4000) {
		t.ram2[a - 0x2000] = v;
		return;
	}
	switch (a) {
	case 0x0800: q_put(Q_LATCH, v); break;
	case 0x0900: q_put(Q_IRQ, 0); break;   // (sh_irqtrigger_w: HOLD_LINE)
	case 0x0a00: t.palettebank = v & 0x07; break;   // (palettebank_w; bits 4, 5 the coin counters)
	case 0x0b00: t.flip = (~v & 0x08) != 0; break;   // (flipscreen_w)
	}
}

// ---- the sound board (slave_map, slave_io_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x4400)
		return t.zram[a - 0x4000];
	if (a == 0x8000)
		return t.soundlatch;
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400)
		t.zram[a - 0x4000] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	return p < 2 ? rbx_ym_read(0, p & 1) : 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 2)
		rbx_ym_write(0, p & 1, v);
}

static void sound_run(int n)   // (a line's samples: the Z80 for their cycles, the YM2203; mono)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = t.z80_irq;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
	t.z80_irq = (uint8_t)line;
	if (n > 64)
		n = 64;
	int32_t ym[96];
	memset(ym, 0, sizeof(ym));
	t.ym_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	// (the discrete mixer: each SSG channel through its switched 1k / 0.22 uF filter, the FM through 1k / 33 nF)
	int32_t raw[4 * 96];
	const int yn = rbx_ym_advance_outputs(0, yc, raw, 96);
	for (int i = 0; i < yn; i++)
		ym[i] = (filter_rc_step(&t.flt[0], raw[4 * i + 1]) + filter_rc_step(&t.flt[1], raw[4 * i + 2]) +
		         filter_rc_step(&t.flt[2], raw[4 * i + 3])) * 615 / 256 + filter_rc_step(&t.flt[3], raw[4 * i]) * 320 / 256;
	// (MAME's mixers: SSG x 5 through 2.2k / 1k (0.19), then 0.5k against the FM's 1k (0.5 / 0.25), output x 5)
	for (int i = 0; i < n && s_out_pos < 2048; i++) {   // (the final mixer's 4.7 uF: a high-pass of about 34 Hz)
		const int32_t x = yn > 0 ? ym[i * yn / n] : 0;
		t.hp_y = x - t.hp_x + t.hp_y - t.hp_y / 225;
		t.hp_x = x;
		s_out[s_out_pos++] = t.hp_y;
	}
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v); break;
		case Q_LATCH: t.soundlatch = (uint8_t)e->v; break;
		case Q_IRQ: t.z80_irq = 1; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void ih_exit(void);
static void ih_reset(void);

// MAME compute_resistor_weights(0, 255, -1.0, ...) for three like networks: each resistor's output alone, scaled so
// that all of them on gives 255
static void resistor_weights(const int *res, int n, int pulldown, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = pulldown ? 1.0 / pulldown : 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		double v = 255.0 * r0 / (r1 + r0);
		w[k] = v < 0 ? 0 : v > 255 ? 255 : v;
		sum += w[k];
	}
	for (int k = 0; k < n; k++)
		w[k] *= 255.0 / sum;
}

// filter_w (the YM2203's port A): bits 2, 1, 0 switch the SSG channels A, B, C's 0.22 uF in
static void filter_w(uint8_t v)
{
	const double rate = rbx_ym_rate(0);
	for (int c = 0; c < 3; c++)
		filter_rc_lowpass_3r(&t.flt[c], 1000, 1e12, 0, (v >> (2 - c)) & 1 ? 0.22e-6 : 0, rate);
}

static int ih_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t gs, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("soundcpu", &s_zrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *gfx = rbx_region("gfx", &gs);
	if (!s_rom || !s_zrom || !s_prom || ps < 0x500 || !gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb over the whole region; ironhors_spritelayout: 512 of 16x16, the right half 32 bytes on,
	// the lower 64; the first pixel in the top nibble)
	s_nchr = gs / 32;
	s_nspr16 = 512;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr16 = (uint8_t *)malloc((size_t)s_nspr16 * 256);
	if (!s_chr || !s_spr16 || gs < 512 * 128) {
		rbx_fail("%s: out of memory", g->set);
		ih_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = gfx[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = gfx[e * 32 + (uint32_t)i] & 15;
		}
	for (uint32_t e = 0; e < s_nspr16; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint8_t b = gfx[e * 128 + (uint32_t)((x >> 3) * 32 + (y >> 3) * 64 + (y & 7) * 4 + (x & 7) / 2)];
				s_spr16[e * 256 + (uint32_t)(y * 16 + x)] = (x & 1) ? b & 15 : b >> 4;
			}
	static const int res[4] = { 2000, 1000, 470, 220 };
	double w[4];
	resistor_weights(res, 4, 1000, w);
	for (int i = 0; i < 256; i++) {
		int c[3];
		for (int k = 0; k < 3; k++) {
			const uint8_t v = s_prom[i | (k << 8)];
			c[k] = (int)(w[0] * (v & 1) + w[1] * ((v >> 1) & 1) + w[2] * ((v >> 2) & 1) + w[3] * ((v >> 3) & 1) + 0.5);
		}
		s_rgb[i] = (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = Z80_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = 490;   // (the discrete mixer's levels, measured against MAME)
	yc.gain[3] = 165;
	yc.port_a_write = filter_w;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2203", g->set);
		ih_exit();
		return -1;
	}
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		ih_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	ih_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void ih_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	free(s_chr);
	free(s_spr16);
	s_chr = s_spr16 = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void ih_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	for (int c = 0; c < 3; c++)
		t.flt[c].k = 0x10000;
	filter_rc_lowpass_3r(&t.flt[3], 1000, 1e12, 0, 33e-9, rbx_ym_rate(0));   // (the FM's RC filter)
	rbx_ym_reset(0);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x20; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram2 + (p - 0x20) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x40; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 4; p++)
		s_zmap[0x40 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void ih_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {
			capture();
			rbx_video_frame_done();
		}
		// (scanline_tick: the FIRQ at 240 on even frames, else the NMI every 64 lines from 48)
		const uint8_t ie = t.ram0[0x0004];
		if (vpos == 240 && (t.frame & 1) == 0) {
			if (ie & 4)
				m6809_hold_firq(&s_cpu);
		} else if ((vpos + 16) % 64 == 0 && (ie & 1)) {
			m6809_set_nmi(&s_cpu, 1);
			m6809_set_nmi(&s_cpu, 0);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int ih_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "ih_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "ih", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "ih_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "ih_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "ih_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int ih_save(void *ctx) { return ih_state(ctx, 0); }
static int ih_load(void *ctx) { return ih_state(ctx, 1); }
static int ih_supported(const char *machine) { return !strcmp(machine, "ironhors"); }

const rbx_driver rbx_drv_ironhors = { "Konami Iron Horse", ih_init, ih_exit, ih_reset, ih_frame, ih_save, ih_load,
                                      ih_supported, ih_render, NULL };

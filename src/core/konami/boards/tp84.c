/*  Konami Time Pilot '84. Ported from MAME src/mame/konami/tp84.cpp (BSD-3-Clause, copyright-holders: Marc Lafontaine)
    and the palette's resistor network of src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud). Two
    6809Es at 1.536 MHz sharing 2 KB, line by line; a background and a foreground (its 16-pixel side columns) tilemap of
    2 bpp tiles, the sprites the sub CPU rewrites during the frame (MAME's update_partial: kept here as bands of lines
    with their sprite RAM), the colours through the PROMs; the sound: a Z80 at 3.58 MHz and three SN76489As, a frame
    behind on the worker (each chip through its RC filter). The flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/sn76496.h"
#include "../../sound/filter_rc.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12, the two 6809s)
#define Z80_CLOCK 3579545u     // (14.318 MHz / 4; the SN76489As at half that)
#define SND_RATE 48000
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define CPU_LINE (CPU_CLOCK / 60 / LINES)   // (100 clocks a line)
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x1000
#define MAX_BANDS 64

static uint8_t *s_rom, *s_srom, *s_zrom, *s_prom;
static uint32_t s_rom_size, s_srom_size, s_zrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[256];
static m6809_t s_cpu, s_sub;
static z80 s_z;
static const uint8_t *s_zmap[256];
static sn76496_t s_sn[3];
static int s_port_dsw[2], s_port_p[2], s_port_system, s_b;   // (s_b: tp84b's main map)
static int s_vpos;

static struct {
	uint8_t vram[0x1000];    // (bg video, fg video, bg colour, fg colour RAMs)
	uint8_t shared[0x800], subram[0x7a0], spr[0x60];
	uint8_t palette_bank, scroll_x, scroll_y, latch, irq_enable, sub_irq_mask;
	uint8_t zram[0x400], soundlatch, z80_irq;
	int64_t cpu_end;         // (the CPUs' line target: absolute, an instruction's overshoot not carried on)
	uint64_t z_frac, z_end, sn_frac, snd_frac;
	uint32_t frame;
	filter_rc_t flt[3];   // (each SN76489A's RC filter: filter_w)
	uint8_t state_end;
} t;

// the bands of the frame being run: the sprite RAM from each start line on (update_partial before each write)
static struct { int start; uint8_t spr[0x60]; } s_bands[MAX_BANDS];
static int s_nbands;

// ---- the drawing's snapshot ----
static struct {
	uint8_t vram[0x1000], palette_bank, scroll_x, scroll_y;
	int nbands, start[MAX_BANDS];
	uint8_t spr[MAX_BANDS][0x60];
} s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	s_vs.palette_bank = t.palette_bank;
	s_vs.scroll_x = t.scroll_x;
	s_vs.scroll_y = t.scroll_y;
	s_vs.nbands = s_nbands;
	for (int i = 0; i < s_nbands; i++) {
		s_vs.start[i] = s_bands[i].start;
		memcpy(s_vs.spr[i], s_bands[i].spr, 0x60);
	}
}

static inline uint8_t ctab(int q)   // (the pen's colour: pen ((i & 0x100) << 3) | (j << 8) | (i & 0xff))
{
	const int i = (q & 0xff) | (((q >> 11) & 1) << 8), j = (q >> 8) & 7;
	return (uint8_t)(((~i & 0x100) >> 1) | (j << 4) | (s_prom[0x300 + i] & 0x0f));
}

static void draw_layer(int fg, int x0, int x1, uint16_t *bm, int pitch)
{
	const uint8_t *vram = s_vs.vram + (fg ? 0x400 : 0), *cram = s_vs.vram + (fg ? 0xc00 : 0x800);
	const int pb = s_vs.palette_bank, sx = fg ? 0 : s_vs.scroll_x, sy = fg ? 0 : s_vs.scroll_y;
	const int cbase = ((pb & 0x07) << 6) | ((pb & 0x18) << 1);
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int ty = (y + sy) & 255;
		for (int x = x0; x <= x1;) {
			const int tx = (x + sx) & 255;
			int n = 8 - (tx & 7);
			if (n > x1 - x + 1)
				n = x1 - x + 1;
			const int idx = (ty >> 3) * 32 + (tx >> 3);
			const int c = cram[idx];
			const int code = ((c & 0x30) << 4) | vram[idx];
			const int pen = (cbase | (c & 0x0f)) * 4;
			const int fx = (c >> 6) & 1, fy = (c >> 7) & 1;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8;
			for (int i = 0; i < n; i++) {
				const int px = (tx + i) & 7;
				d[x + i] = (uint16_t)(pen + src[fx ? 7 - px : px]);
			}
			x += n;
		}
	}
}

static void tp_render(void)   // (screen_update, band by band for the sprites)
{
	for (int q = 0; q < NPENS; q++)
		rbx_video.palette[q] = s_rgb[ctab(q)];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_layer(0, 0, VIS_W - 1, bm, pitch);
	const int pbase = (s_vs.palette_bank & 0x07) << 4;
	for (int b = 0; b < s_vs.nbands; b++) {
		const int y0 = s_vs.start[b], y1 = b + 1 < s_vs.nbands ? s_vs.start[b + 1] - 1 : VIS_Y0 + VIS_H - 1;
		const uint8_t *sr = s_vs.spr[b];
		for (int offs = 0x5c; offs >= 0; offs -= 4) {   // (draw_sprites: transmask of the pens of colour palette_base)
			const int x = sr[offs], y = 240 - sr[offs + 3];
			const int color = pbase | (sr[offs + 2] & 0x0f);
			const int flipx = !(sr[offs + 2] & 0x40), flipy = (sr[offs + 2] & 0x80) != 0;
			const int pen = 2048 + color * 16;
			uint8_t trans[16];
			for (int p = 0; p < 16; p++)
				trans[p] = ctab(pen + p) == pbase;
			const uint8_t *src = s_spr + (rbx_wrap((uint32_t)sr[offs + 1], s_nspr)) * 256;
			for (int j = 0; j < 16; j++) {
				const int yy = y + j;
				if (yy < y0 || yy > y1 || yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
					continue;
				const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
				uint16_t *d = bm + (yy - VIS_Y0) * pitch;
				for (int k = 0; k < 16; k++) {
					const int xx = x + k;
					if (xx >= VIS_W)
						continue;
					const int p = s[flipx ? 15 - k : k];
					if (!trans[p])
						d[xx] = (uint16_t)(pen + p);
				}
			}
		}
	}
	draw_layer(1, 0, 15, bm, pitch);
	draw_layer(1, VIS_W - 16, VIS_W - 1, bm, pitch);
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

// ---- the main CPU's bus (tp84_cpu1_map; tp84b's at other addresses) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	const uint16_t vbase = s_b ? 0x0000 : 0x4000, io = s_b ? 0x1a00 : 0x2800;
	if (a >= vbase && a < vbase + 0x1000)
		return t.vram[a - vbase];
	if (a >= vbase + 0x1000 && a < vbase + 0x1800)
		return t.shared[a - vbase - 0x1000];
	if (a == io)
		return port(s_port_system);
	if (a == io + 0x20)
		return port(s_port_p[0]);
	if (a == io + 0x40)
		return port(s_port_p[1]);
	if (a == io + 0x60)
		return port(s_port_dsw[0]);
	if (a == (s_b ? 0x1c00 : 0x3000))
		return port(s_port_dsw[1]);
	if (a >= 0x8000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	const uint16_t vbase = s_b ? 0x0000 : 0x4000;
	if (a >= vbase && a < vbase + 0x1000) {
		t.vram[a - vbase] = v;
		return;
	}
	if (a >= vbase + 0x1000 && a < vbase + 0x1800) {
		t.shared[a - vbase - 0x1000] = v;
		return;
	}
	const uint16_t pal = s_b ? 0x1a00 : 0x2800, latch = s_b ? 0x1c00 : 0x3000, irq = s_b ? 0x1e00 : 0x3800;
	const uint16_t snd = s_b ? 0x1e80 : 0x3a00, scx = s_b ? 0x1f00 : 0x3c00, scy = s_b ? 0x1f80 : 0x3e00;
	if (a == pal)
		t.palette_bank = v;
	else if (a >= latch && a < latch + 8) {   // (the LS259: Q0 the IRQ's enable)
		const int bit = a & 7, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 0) {
			t.irq_enable = (uint8_t)now;
			if (!now)
				m6809_set_irq(&s_cpu, 0);
		}
	} else if (a == irq)
		q_put(Q_IRQ, 0);   // (sh_irqtrigger_w: HOLD_LINE)
	else if (a == snd)
		q_put(Q_LATCH, v);
	else if (a == scx)
		t.scroll_x = v;
	else if (a == scy)
		t.scroll_y = v;
}

// ---- the sub CPU (cpu2_map) ----
static uint8_t sub_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x2000)
		return (uint8_t)s_vpos;   // (scanline_r)
	if (a >= 0x6000 && a < 0x67a0)
		return t.subram[a - 0x6000];
	if (a >= 0x67a0 && a < 0x6800)
		return t.spr[a - 0x67a0];
	if (a >= 0x8000 && a < 0x8800)
		return t.shared[a - 0x8000];
	if (a >= 0xe000)
		return s_srom[rbx_wrap(a, s_srom_size)];
	return 0;
}
static void sub_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x4000) {   // (sub_irq_mask_w)
		t.sub_irq_mask = v & 1;
		if (!t.sub_irq_mask)
			m6809_set_irq(&s_sub, 0);
	} else if (a >= 0x6000 && a < 0x67a0)
		t.subram[a - 0x6000] = v;
	else if (a >= 0x67a0 && a < 0x6800) {   // (spriteram_w: update_partial(vpos) first)
		const int o = a - 0x67a0;
		if (s_vpos >= VIS_Y0 && s_vpos < VIS_Y0 + VIS_H - 1 && s_nbands > 0 && s_bands[s_nbands - 1].start != s_vpos + 1 &&
		    s_nbands < MAX_BANDS) {
			s_bands[s_nbands].start = s_vpos + 1;
			memcpy(s_bands[s_nbands].spr, t.spr, 0x60);
			s_nbands++;
		}
		t.spr[o] = v;
		if (s_nbands > 0 && s_bands[s_nbands - 1].start > s_vpos)
			s_bands[s_nbands - 1].spr[o] = v;
	} else if (a >= 0x8000 && a < 0x8800)
		t.shared[a - 0x8000] = v;
}

// ---- the sound board (audio_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x4400)
		return t.zram[a - 0x4000];
	if (a == 0x6000)
		return t.soundlatch;
	if (a == 0x8000)   // (sh_timer_r)
		return (uint8_t)((s_z.cyc / (2048 / 2)) & 0x0f);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400)
		t.zram[a - 0x4000] = v;
	else if (a >= 0xa000 && a < 0xa200) {   // (filter_w: the address lines choose the capacitors; #1's filter unused)
		const uint32_t o = a - 0xa000u;
		const double rate = sn76496_rate(&s_sn[0]);
		filter_rc_lowpass_3r(&t.flt[0], 1000, 2200, 1000, ((o >> 3) & 1 ? 47000e-12 : 0) + ((o >> 4) & 1 ? 470000e-12 : 0), rate);
		filter_rc_lowpass_3r(&t.flt[1], 1000, 2200, 1000, (o >> 7) & 1 ? 470000e-12 : 0, rate);
		filter_rc_lowpass_3r(&t.flt[2], 1000, 2200, 1000, (o >> 8) & 1 ? 470000e-12 : 0, rate);
	} else if (a == 0xc001)
		sn76496_write(&s_sn[0], v);
	else if (a == 0xc003)
		sn76496_write(&s_sn[1], v);
	else if (a == 0xc004)
		sn76496_write(&s_sn[2], v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, the SN76489As 0.75 each)
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
	if (n > 16)
		n = 16;
	int32_t a[3][64];
	memset(a, 0, sizeof(a));
	t.sn_frac += (uint64_t)(Z80_CLOCK / 2) * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	int k[3];
	for (int c = 0; c < 3; c++) {
		k[c] = sn76496_render(&s_sn[c], sc, a[c], 64);
		for (int j = 0; j < k[c]; j++)
			a[c][j] = filter_rc_step(&t.flt[c], a[c][j]);
	}
	for (int i = 0; i < n && s_out_pos < 2048; i++) {
		int32_t m = 0;
		for (int c = 0; c < 3; c++)
			m += k[c] > 0 ? a[c][i * k[c] / n] : 0;
		s_out[s_out_pos++] = m;
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
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void tp_exit(void);
static void tp_reset(void);

static int tp_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ts, ss, ps;
	s_rom = rbx_region("cpu1", &s_rom_size);
	s_srom = rbx_region("sub", &s_srom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_srom || !s_zrom || !s_prom || ps < 0x500 || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_b = !strcmp(g->machine, "tp84b");
	memset(&t, 0, sizeof(t));
	// (charlayout: 8x8 2 bpp, planes { 4, 0 }, x 4 pixels a byte, the right half 8 bytes on; spritelayout: 16x16,
	// planes { half + 4, half + 0, 4, 0 }, the 4-pixel columns 8 bytes apart, a byte a row, the lower half 32 on)
	s_nchr = ts / 16;
	s_nspr = ss / 2 / 64;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		tp_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint8_t b = tiles[e * 16 + (uint32_t)((x >> 2) * 8 + y)];
				const int q = x & 3;
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)(((b >> (3 - q)) & 1) << 1 | ((b >> (7 - q)) & 1));
			}
	const uint32_t half = ss / 2;
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = e * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
				const int q = x & 3;
				const uint8_t b1 = sprites[o], b2 = sprites[half + o];
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = (uint8_t)(((b2 >> (3 - q)) & 1) << 3 | ((b2 >> (7 - q)) & 1) << 2 |
				                                                      ((b1 >> (3 - q)) & 1) << 1 | ((b1 >> (7 - q)) & 1));
			}
	// (compute_resistor_weights(0, 255, -1.0, 4, { 1000, 470, 220, 100 }, 470): one network for the three)
	static const int res[4] = { 1000, 470, 220, 100 };
	double w[4], sum = 0;
	for (int k = 0; k < 4; k++) {
		double r0 = 1.0 / 470, r1 = 1.0 / 1e12;
		for (int j = 0; j < 4; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	for (int k = 0; k < 4; k++)
		w[k] *= 255.0 / sum;
	for (int i = 0; i < 256; i++) {
		int c[3];
		for (int k = 0; k < 3; k++) {
			const uint8_t v = s_prom[i | (k << 8)];
			c[k] = (int)(w[0] * (v & 1) + w[1] * ((v >> 1) & 1) + w[2] * ((v >> 2) & 1) + w[3] * ((v >> 3) & 1) + 0.5);
		}
		s_rgb[i] = (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
	}
	for (int i = 0; i < 3; i++)
		sn76496_init(&s_sn[i], Z80_CLOCK / 2, SN76489A_CONFIG, 192, 32, 1);   // (0.75)
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		tp_exit();
		return -1;
	}
	tp_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void tp_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void tp_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	for (int c = 0; c < 3; c++)
		t.flt[c].k = 0x10000;
	s_nbands = 0;
	for (int i = 0; i < 3; i++)
		sn76496_reset(&s_sn[i]);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
	m6809_init(&s_sub);
	s_sub.read = sub_rd;
	s_sub.write = sub_wr;
	for (int p = 0x80; p < 0x88; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = t.shared + (p - 0x80) * 256;
	for (int p = 0xe0; p < 0x100; p++)
		s_sub.rpage[p] = s_srom + ((uint32_t)p * 256) % s_srom_size;
	m6809_reset(&s_sub);
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

static void tp_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		s_vpos = vpos;
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_enable)
				m6809_set_irq(&s_cpu, 1);
			if (t.sub_irq_mask)
				m6809_set_irq(&s_sub, 1);
		}
		if (vpos == VIS_Y0) {   // (the first band: the sprite RAM at the top of the screen)
			s_nbands = 1;
			s_bands[0].start = VIS_Y0;
			memcpy(s_bands[0].spr, t.spr, 0x60);
		}
		t.cpu_end += CPU_LINE;
		m6809_run_until(&s_cpu, t.cpu_end);
		m6809_run_until(&s_sub, t.cpu_end);
		t.snd_frac += SND_RATE;
		const int ns = (int)(t.snd_frac / (60u * LINES));
		t.snd_frac -= (uint64_t)ns * (60u * LINES);
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int tp_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "tp_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "tp_sub", &s_sub, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "tp", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "tp_nb", &s_nbands, sizeof(s_nbands));
	rc |= rbx_state_io(ctx, "tp_bands", s_bands, sizeof(s_bands));
	for (int i = 0; i < 3; i++) {
		static const char *const tag[3] = { "tp_sn0", "tp_sn1", "tp_sn2" };
		rc |= rbx_state_io(ctx, tag[i], &s_sn[i], sn76496_state_size());
	}
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "tp_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "tp_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tp_z80", &z, sizeof(z));
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

static int tp_save(void *ctx) { return tp_state(ctx, 0); }
static int tp_load(void *ctx) { return tp_state(ctx, 1); }
static int tp_supported(const char *machine) { return !strcmp(machine, "tp84") || !strcmp(machine, "tp84b"); }

const rbx_driver rbx_drv_tp84 = { "Konami Time Pilot '84", tp_init, tp_exit, tp_reset, tp_frame, tp_save, tp_load,
                                  tp_supported, tp_render, NULL };

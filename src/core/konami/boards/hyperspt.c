/*  Konami Hyper Sports / Hyper Olympic '84 and Road Fighter. Ported from MAME src/mame/konami/hyperspt.cpp (BSD-3-Clause,
    copyright-holders: Chris Hardy), the sound board's src/mame/konami/trackfld_a.cpp (BSD-3-Clause, copyright-holders:
    Chris Hardy) and the palette's resistor networks of src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders:
    Couriersud). A KONAMI1 at 1.536 MHz (a plain 6809E on roadfu), a 64x32 tilemap with row scroll, wrapping sprites over
    it, the colours through the lookup PROMs; the sound: a Z80 at 3.58 MHz, an SN76489A through a latch, a DAC and (Hyper
    Sports only) a VLM5030, a frame behind on the worker. hypersptb, whose speech is a 6802 and an MSM5205, is not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/sn76496.h"
#include "../../sound/vlm5030.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12)
#define Z80_CLOCK 3579545u     // (14.318 MHz / 4; the SN76489A at half that)
#define SND_RATE 48000
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define LINE_HZ (60u * LINES)
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200            // (16 * 16 + 16 * 16)
#define ZROM_SIZE 0x4000
#define NVRAM_OFS 0x1800       // (3800-3fff in t.ram: DEFAULT_ALL_0)
#define NVRAM_SIZE 0x800

static uint8_t *s_rom, *s_zrom, *s_prom, *s_vlmrom;
static uint32_t s_rom_size, s_zrom_size, s_vlmrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static sn76496_t s_sn;
static vlm5030_t s_vlm;
static int s_konami1, s_roadf;
static int s_port_dsw[2], s_port_p[2], s_port_system;
static char s_nv_path[256];

static struct {
	uint8_t sprscroll[0x100];   // 1000-10ff (10c0 the scroll)
	uint8_t ram[0x2000];        // 2000-3fff (2000 the video RAM, 2800 the colour RAM, 3800 the battery RAM)
	uint8_t latch, irq_mask, flip;
	uint8_t zram[0x1000], soundlatch, z80_irq, sn_latch, dac;
	int16_t vlm_out;
	int32_t hp_x, hp_y;         // (the output's high-pass: the DAC idles at 0)
	int64_t cpu_end;            // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t z_frac, z_end, sn_frac, vlm_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x800], cram[0x800], spr[0x100], flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.ram, sizeof(s_vs.vram));
	memcpy(s_vs.cram, t.ram + 0x800, sizeof(s_vs.cram));
	memcpy(s_vs.spr, t.sprscroll, sizeof(s_vs.spr));
	s_vs.flip = t.flip;
}

static inline void draw_sprite(uint16_t *bm, int pitch, const uint8_t *src, const uint8_t *lut, int pen, int fx, int fy,
                               int sx, int sy)
{
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
			continue;
		const uint8_t *s = src + (fy ? 15 - j : j) * 16;
		uint16_t *d = bm + (yy - VIS_Y0) * pitch;
		for (int k = 0; k < 16; k++) {
			const int xx = sx + k;
			if (xx < 0 || xx >= VIS_W)
				continue;
			const int p = s[fx ? 15 - k : k];
			if (lut[p] & 0x0f)
				d[xx] = (uint16_t)(pen + p);
		}
	}
}

static void hs_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the sprites' 0-15, 100-1ff the characters' 16-31)
		rbx_video.palette[i] = s_rgb[s_prom[0x20 + i] & 0x0f];
		rbx_video.palette[0x100 + i] = s_rgb[(s_prom[0x120 + i] & 0x0f) | 0x10];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int ty = s_vs.flip ? 255 - y : y;   // (the flip: the tilemap's rows and columns turned, the scroll negated)
		const int row = ty >> 3;
		const int scrollx = s_vs.spr[0xc0 + row * 2] + (s_vs.spr[0xc0 + row * 2 + 1] & 1) * 256;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (s_vs.flip ? 255 - x - scrollx : x + scrollx) & 511;
			const int idx = row * 64 + (tx >> 3);
			const int attr = s_vs.cram[idx];
			const int code = s_vs.vram[idx] + ((attr & 0x80) << 1) + (s_roadf ? (attr & 0x60) << 4 : (attr & 0x40) << 3);
			const int fx = (attr & 0x10) != 0, fy = !s_roadf && (attr & 0x20);
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8;
			d[x] = (uint16_t)(0x100 + (attr & 0x0f) * 16 + src[fx ? 7 - (tx & 7) : (tx & 7)]);
		}
	}
	const uint8_t *sr = s_vs.spr;
	for (int offs = 0xc0 - 4; offs >= 0; offs -= 4) {   // (draw_sprites: transmask of the pens mapping to colour 0)
		const int sx = sr[offs + 3];
		int sy = 240 - sr[offs + 1];
		const int code = sr[offs + 2] + 8 * (sr[offs] & 0x20);
		const int color = sr[offs] & 0x0f;
		const int flipx = !(sr[offs] & 0x40);
		int flipy = (sr[offs] & 0x80) != 0;
		if (s_vs.flip) {
			sy = 240 - sy;
			flipy = !flipy;
		}
		sy += 1;   // (after the flip: a hardware "feature")
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)code, s_nspr)) * 256;
		const uint8_t *lut = s_prom + 0x20 + color * 16;
		draw_sprite(bm, pitch, src, lut, color * 16, flipx, flipy, sx, sy);
		draw_sprite(bm, pitch, src, lut, color * 16, flipx, flipy, sx - 256, sy);   // (the wraparound)
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

// ---- the main CPU's bus (common_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x1000 && a < 0x1100)
		return t.sprscroll[a - 0x1000];
	switch (a) {
	case 0x1600: return port(s_port_dsw[1]);
	case 0x1680: return port(s_port_system);
	case 0x1681: return port(s_port_p[0]);
	case 0x1682: return port(s_port_p[1]);
	case 0x1683: return port(s_port_dsw[0]);
	}
	if (a >= 0x2000 && a < 0x4000)
		return t.ram[a - 0x2000];
	if (a >= 0x4000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x1000 && a < 0x1100)
		t.sprscroll[a - 0x1000] = v;
	else if (a >= 0x2000 && a < 0x4000)
		t.ram[a - 0x2000] = v;
	else if (a >= 0x1480 && a < 0x1488) {   // (the LS259: Q0 the flip, Q1 the sound's IRQ on its rising edge, Q7 the IRQ mask)
		const int bit = a & 7, old = (t.latch >> bit) & 1, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 0)
			t.flip = (uint8_t)now;
		else if (bit == 1 && now && !old)
			q_put(Q_IRQ, 0);
		else if (bit == 7) {
			t.irq_mask = (uint8_t)now;
			if (!now)
				m6809_set_irq(&s_cpu, 0);
		}
	} else if (a == 0x1500)
		q_put(Q_LATCH, v);
}

// ---- the sound board (common_sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < ZROM_SIZE)
		return s_zrom[a];
	if (a >= 0x4000 && a < 0x5000)
		return t.zram[a - 0x4000];
	if (a == 0x6000)
		return t.soundlatch;
	if (a == 0x8000)   // (hyperspt_sh_timer_r: the Z80's clock / 1024)
		return (uint8_t)(((s_z.cyc >> 10) & 3) | (!s_roadf && vlm5030_bsy(&s_vlm) ? 4 : 0));
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x5000)
		t.zram[a - 0x4000] = v;
	else if (a == 0xe000)
		t.dac = v;
	else if (a == 0xe001)
		t.sn_latch = v;
	else if (a == 0xe002)
		sn76496_write(&s_sn, t.sn_latch);
	else if (s_roadf)   // (no VLM5030)
		return;
	else if (a == 0xa000)
		vlm5030_data_w(&s_vlm, v);
	else if (a >= 0xc000 && a < 0xe000) {   // (hyperspt_sound_w: A4 the VLM5030's ST, A5 its RST)
		vlm5030_st(&s_vlm, ((a - 0xc000) >> 4) & 1);
		vlm5030_rst(&s_vlm, ((a - 0xc000) >> 5) & 1);
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, the SN76489A 1.0, the VLM5030 1.0, the DAC 0.4)
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
	int32_t a[64];
	memset(a, 0, sizeof(a));
	t.sn_frac += (uint64_t)(Z80_CLOCK / 2) * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	const int ka = sn76496_render(&s_sn, sc, a, 64);
	const int32_t dac = (int32_t)t.dac * 2 * 128 * 40 / 100;   // (the DAC's -1..1 at 0.4)
	for (int i = 0; i < n && s_out_pos < 2048; i++) {
		if (!s_roadf) {
			t.vlm_frac += vlm5030_rate(&s_vlm);
			while (t.vlm_frac >= SND_RATE) {
				t.vlm_frac -= SND_RATE;
				int16_t s;
				vlm5030_render(&s_vlm, &s, 1);
				t.vlm_out = s;
			}
		}
		const int32_t x = (ka > 0 ? a[i * ka / n] : 0) + t.vlm_out + dac;
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
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void hs_exit(void);
static void hs_reset(void);

// MAME compute_resistor_weights(0, 255, -1.0, ...): each resistor's output alone, all the networks scaled so that
// the one with the greatest output gives 255
static double raw_weights(const int *res, int n, int pulldown, double *w)
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
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	return sum;
}

// (a layout's pixel: the planes { half + 4, half + 0, 4, 0 }, q the pixel in a byte's two nibbles)
static inline uint8_t layout_pixel(const uint8_t *gfx, uint32_t half, uint32_t o, int q)
{
	const uint8_t b1 = gfx[half + o], b2 = gfx[o];
	return (uint8_t)(((b1 >> (3 - q)) & 1) << 3 | ((b1 >> (7 - q)) & 1) << 2 | ((b2 >> (3 - q)) & 1) << 1 | ((b2 >> (7 - q)) & 1));
}

static int hs_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_prom = rbx_region("proms", &ps);
	s_vlmrom = rbx_region("vlm", &s_vlmrom_size);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	s_roadf = !strncmp(g->machine, "roadf", 5);
	s_konami1 = strcmp(g->machine, "roadfu") != 0;
	if (!s_rom || !s_zrom || s_zrom_size < ZROM_SIZE || !s_prom || ps < 0x220 || (!s_roadf && !s_vlmrom) || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (hyperspt_charlayout / roadf_charlayout: 16 bytes a character over each half of the region;
	// hyperspt_spritelayout / roadf_spritelayout: 64 bytes a sprite, the lower half 32 on)
	const uint32_t chalf = ts / 2, shalf = ss / 2;
	s_nchr = chalf / 16;
	s_nspr = shalf / 64;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		hs_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++)
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = layout_pixel(tiles, chalf, e * 16 + (uint32_t)((x >> 2) * 8 + y), x & 3);
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++)
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] =
					layout_pixel(sprites, shalf, e * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32), x & 3);
	static const int res_rg[3] = { 1000, 470, 220 }, res_b[2] = { 470, 220 };
	double wr[3], wb[2];
	const double srg = raw_weights(res_rg, 3, 1000, wr), sb = raw_weights(res_b, 2, 1000, wb);
	const double scale = 255.0 / (srg > sb ? srg : sb);
	for (int i = 0; i < 0x20; i++) {
		const uint8_t c = s_prom[i];
		const int r = (int)(scale * (wr[0] * (c & 1) + wr[1] * ((c >> 1) & 1) + wr[2] * ((c >> 2) & 1)) + 0.5);
		const int gg = (int)(scale * (wr[0] * ((c >> 3) & 1) + wr[1] * ((c >> 4) & 1) + wr[2] * ((c >> 5) & 1)) + 0.5);
		const int b = (int)(scale * (wb[0] * ((c >> 6) & 1) + wb[1] * ((c >> 7) & 1)) + 0.5);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	sn76496_init(&s_sn, Z80_CLOCK / 2, SN76489A_CONFIG, 256, 32, 1);
	if (!s_roadf)
		vlm5030_init(&s_vlm, 3579545, s_vlmrom, s_vlmrom_size);
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_p[0] = rbx_port_find(s_roadf ? "P1" : "P1_P2");
	s_port_p[1] = rbx_port_find(s_roadf ? "P2" : "P3_P4");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		hs_exit();
		return -1;
	}
	hs_reset();
	s_nv_path[0] = 0;
	{   // (the battery RAM, kept between runs)
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(t.ram + NVRAM_OFS, 1, NVRAM_SIZE, f) != NVRAM_SIZE)
					memset(t.ram + NVRAM_OFS, 0, NVRAM_SIZE);
				fclose(f);
			}
		}
	}
	rbx_aux_set_job(job);
	return 0;
}

static void hs_exit(void)
{
	rbx_aux_set_job(NULL);
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(t.ram + NVRAM_OFS, 1, NVRAM_SIZE, f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void hs_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	uint8_t nv[NVRAM_SIZE];   // (the battery RAM survives a reset; hs_init cleared it first)
	memcpy(nv, t.ram + NVRAM_OFS, NVRAM_SIZE);
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memcpy(t.ram + NVRAM_OFS, nv, NVRAM_SIZE);
	sn76496_reset(&s_sn);
	if (!s_roadf)
		vlm5030_reset(&s_vlm);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.konami1 = (uint8_t)s_konami1;
	s_cpu.rpage[0x10] = s_cpu.wpage[0x10] = t.sprscroll;
	for (int p = 0x20; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x20) * 256;
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
	for (int p = 0; p < ZROM_SIZE / 256; p++)
		s_zmap[p] = s_zrom + p * 256;
	for (int p = 0; p < 0x10; p++)
		s_zmap[0x40 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void hs_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_mask)
				m6809_set_irq(&s_cpu, 1);
		}
		t.cpu_end += CPU_CLOCK / LINE_HZ;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.snd_frac += SND_RATE;
		const int ns = (int)(t.snd_frac / LINE_HZ);
		t.snd_frac -= (uint64_t)ns * LINE_HZ;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int hs_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "hs_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "hs", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "hs_sn", &s_sn, sn76496_state_size());
	if (!s_roadf)
		rc |= vlm5030_state(&s_vlm, ctx, load, "hs_vlm");
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "hs_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "hs_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "hs_z80", &z, sizeof(z));
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

static int hs_save(void *ctx) { return hs_state(ctx, 0); }
static int hs_load(void *ctx) { return hs_state(ctx, 1); }
static int hs_supported(const char *machine)
{
	return !strcmp(machine, "hyperspt") || !strcmp(machine, "roadf") || !strcmp(machine, "roadfu");
}

const rbx_driver rbx_drv_hyperspt = { "Konami Hyper Sports", hs_init, hs_exit, hs_reset, hs_frame, hs_save, hs_load,
                                      hs_supported, hs_render, NULL };

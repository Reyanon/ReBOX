/*  Konami Track & Field. Ported from MAME src/mame/konami/trackfld.cpp, trackfld_v.cpp (BSD-3-Clause, copyright-holders:
    Chris Hardy), trackfld_a.cpp (BSD-3-Clause, copyright-holders: Chris Hardy) and the palette's resistor networks of
    src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud). A KONAMI1 at 1.536 MHz (trackfldu: a plain
    6809), a 64x32 tilemap of 32 scrolled rows, the sprites, the colours through the lookup PROMs; the sound: a Z80 at
    3.58 MHz, an SN76489A, a VLM5030 and a DAC, a frame behind on the worker (the output high-passed: the DAC idles at
    0). The flip screen and the bootleg machines not done.
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

#define PIX_CLOCK 6144000u     // (18.432 MHz / 3)
#define HTOTAL 384
#define LINES 264              // (60.61 Hz)
#define CPU_LINE 96            // (1.536 MHz a line)
#define Z80_CLOCK 3579545u     // (14.318 MHz / 4; the SN76489A at half that)
#define SND_RATE 48000
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200

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
static int s_port_dsw[2], s_port_in[2], s_port_system;
static uint8_t s_nv[0x800];   // (the battery RAM 2800-2fff: DEFAULT_ALL_0, kept between runs)
static char s_nv_path[512];

static struct {
	uint8_t ram[0x800];      // 1800-1fff (1800 sprites 2, 1840 scroll, 1c00 sprites 1, 1c40 scroll 2)
	uint8_t vram[0x1000];    // 3000-37ff the video RAM, 3800-3fff the colour RAM
	uint8_t latch, irq_mask;
	uint8_t zram[0x400], soundlatch, z80_irq, sn_latch, dac;
	int16_t vlm_out;
	int32_t hp_x, hp_y;      // (the output's high-pass)
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t z_frac, z_end, sn_frac, vlm_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t ram[0x800], vram[0x1000]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.ram, t.ram, sizeof(s_vs.ram));
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
}

static void draw_sprite(const uint8_t *src, const uint8_t *lut, int pen, int fx, int fy, int sx, int sy, uint16_t *bm, int pitch)
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

static void tf_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++)   // (palette: 0-ff the sprites, 100-1ff the tiles, with the PROM's upper 16 colours)
		rbx_video.palette[i] = s_rgb[(s_prom[0x20 + i] & 0x0f) | (i >= 0x100 ? 0x10 : 0)];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *scroll = s_vs.ram + 0x40, *scroll2 = s_vs.ram + 0x440;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int row = y >> 3;
		const int sc = scroll[row] + 256 * (scroll2[row] & 1);
		for (int x = 0; x < VIS_W;) {
			const int tx = (x + sc) & 511;
			int n = 8 - (tx & 7);
			if (n > VIS_W - x)
				n = VIS_W - x;
			const int idx = row * 64 + (tx >> 3);
			const int attr = s_vs.vram[0x800 + idx];
			const int code = s_vs.vram[idx] + 4 * (attr & 0xc0);
			const int pen = 0x100 + (attr & 0x0f) * 16;
			const int fx = (attr & 0x10) != 0, fy = (attr & 0x20) != 0;
			const uint8_t *src = s_chr + ((uint32_t)code % s_nchr) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < n; i++) {
				const int px = (tx + i) & 7;
				d[x + i] = (uint16_t)(pen + src[fx ? 7 - px : px]);
			}
			x += n;
		}
	}
	const uint8_t *spr2 = s_vs.ram, *spr = s_vs.ram + 0x400;
	for (int offs = 0x40 - 2; offs >= 0; offs -= 2) {   // (draw_sprites: transmask of the pens mapping to colour 0)
		const int attr = spr2[offs];
		const int code = spr[offs + 1] | ((attr & 1) << 8);
		const int color = attr & 0x0f;
		const int fx = !(attr & 0x40), fy = (attr & 0x80) != 0;
		const int sx = spr[offs] - 1, sy = 240 - spr2[offs + 1] + 1;
		const uint8_t *src = s_spr + ((uint32_t)code % s_nspr) * 256;
		const uint8_t *lut = s_prom + 0x20 + color * 16;
		draw_sprite(src, lut, color * 16, fx, fy, sx, sy, bm, pitch);
		draw_sprite(src, lut, color * 16, fx, fy, sx - 256, sy, bm, pitch);   // (the wraparound)
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
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

// ---- the main CPU's bus (main_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if ((a & 0xff80) == 0x1200)
		return port(s_port_dsw[1]);
	if ((a & 0xff80) == 0x1280) {
		switch (a & 3) {
		case 0: return port(s_port_system);
		case 1: return port(s_port_in[0]);
		case 2: return port(s_port_in[1]);
		case 3: return port(s_port_dsw[0]);
		}
	}
	if (a >= 0x1800 && a < 0x2000)
		return t.ram[a - 0x1800];
	if (a >= 0x2800 && a < 0x3000)
		return s_nv[a - 0x2800];
	if (a >= 0x3000 && a < 0x4000)
		return t.vram[a - 0x3000];
	if (a >= 0x6000)
		return s_rom[a % s_rom_size];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if ((a & 0xff80) == 0x1080) {   // (the LS259: Q1 the sound's IRQ on its rising edge, Q7 the IRQ mask)
		const int bit = a & 7, old = (t.latch >> bit) & 1, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 1 && now && !old)
			q_put(Q_IRQ, 0);
		else if (bit == 7) {
			t.irq_mask = (uint8_t)now;
			if (!now)
				m6809_set_irq(&s_cpu, 0);
		}
	} else if ((a & 0xff80) == 0x1100)
		q_put(Q_LATCH, v);
	else if (a >= 0x1800 && a < 0x2000)
		t.ram[a - 0x1800] = v;
	else if (a >= 0x2800 && a < 0x3000)
		s_nv[a - 0x2800] = v;
	else if (a >= 0x3000 && a < 0x4000)
		t.vram[a - 0x3000] = v;
}

// ---- the sound board (sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x6000)
		return t.zram[a & 0x3ff];
	if (a < 0x8000)
		return t.soundlatch;
	if (a < 0xa000)   // (trackfld_sh_timer_r)
		return (uint8_t)((s_z.cyc / 1024) & 0x0f);
	if (a < 0xc000)
		return 0;
	if (a < 0xe000) {   // (trackfld_SN76489a_r: the read strobes the chip too)
		sn76496_write(&s_sn, t.sn_latch);
		return 0xff;
	}
	if ((a & 7) == 2)   // (trackfld_speech_r)
		return vlm5030_bsy(&s_vlm) ? 0x10 : 0;
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x4000)
		return;
	if (a < 0x6000)
		t.zram[a & 0x3ff] = v;
	else if (a >= 0xa000 && a < 0xc000)
		t.sn_latch = v;
	else if (a >= 0xc000 && a < 0xe000)
		sn76496_write(&s_sn, t.sn_latch);
	else if (a >= 0xe000) {
		switch (a & 7) {
		case 0: t.dac = v; break;
		case 3:   // (trackfld_sound_w: A8 the ST pin, A9 the RST pin)
			vlm5030_st(&s_vlm, (a >> 8) & 1);
			vlm5030_rst(&s_vlm, (a >> 9) & 1);
			break;
		case 4: vlm5030_data_w(&s_vlm, v); break;
		}
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, then the SN76489A 1.0, the VLM5030 1.0, the DAC 0.4)
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
	const int k = sn76496_render(&s_sn, sc, a, 64);
	const int32_t dac = ((int32_t)t.dac - 128) * 102;   // (the DAC's range -1 to 1, 0.4: -13107 at 0 in MAME's WAV)
	for (int i = 0; i < n && s_out_pos < 2048; i++) {
		t.vlm_frac += vlm5030_rate(&s_vlm);
		while (t.vlm_frac >= SND_RATE) {
			t.vlm_frac -= SND_RATE;
			int16_t s;
			vlm5030_render(&s_vlm, &s, 1);
			t.vlm_out = s;
		}
		const int32_t x = (k > 0 ? a[i * k / n] : 0) + t.vlm_out + dac;
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
static void tf_exit(void);
static void tf_reset(void);

// MAME compute_resistor_weights(0, 255, -1.0, ...): each resistor's output alone, all the networks scaled so that
// the one with the greatest output gives 255
static double raw_weights(const int *res, int n, int pulldown, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = 1.0 / pulldown, r1 = 1.0 / 1e12;
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

static int tf_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_prom = rbx_region("proms", &ps);
	s_vlmrom = rbx_region("vlm", &s_vlmrom_size);
	const uint8_t *tiles = rbx_region("gfx2", &ts), *sprites = rbx_region("gfx1", &ss);
	if (!s_rom || !s_zrom || !s_prom || ps < 0x220 || !s_vlmrom || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	memset(s_nv, 0, sizeof(s_nv));
	s_cpu.konami1 = 0;
	if (!strcmp(g->set, "trackfldnz"))   // (init_trackfldnz: BITSWAP8(6, 7, 5, 4, 3, 2, 1, 0) over the program)
		for (uint32_t a = 0x6000; a < s_rom_size; a++)
			s_rom[a] = (uint8_t)((s_rom[a] & 0x3f) | ((s_rom[a] & 0x80) >> 1) | ((s_rom[a] & 0x40) << 1));
	// (gfx_8x8x4_packed_msb; spritelayout: 16x16, planes { half + 4, half + 0, 4, 0 }, the 4-pixel columns 8 bytes apart,
	// a byte a row, the lower half 32 on)
	s_nchr = ts / 32;
	s_nspr = ss / 2 / 64;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		tf_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint8_t b = tiles[e * 32 + (uint32_t)(y * 4 + (x >> 1))];
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)((x & 1) ? (b & 0x0f) : (b >> 4));
			}
	const uint32_t half = ss / 2;
#define PIX(hi, lo, q) (uint8_t)((((hi) >> (3 - (q))) & 1) << 3 | (((hi) >> (7 - (q))) & 1) << 2 | \
                                 (((lo) >> (3 - (q))) & 1) << 1 | (((lo) >> (7 - (q))) & 1))
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = e * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = PIX(sprites[half + o], sprites[o], x & 3);
			}
#undef PIX
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
	vlm5030_init(&s_vlm, Z80_CLOCK, s_vlmrom, s_vlmrom_size < 0x2000 ? s_vlmrom_size : 0x2000);
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		tf_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	const int konami1 = strcmp(g->machine, "trackfldu") != 0;
	tf_reset();
	s_cpu.konami1 = konami1;
	m6809_reset(&s_cpu);
	s_nv_path[0] = 0;
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_nv, 1, sizeof(s_nv), f) != sizeof(s_nv))
					memset(s_nv, 0, sizeof(s_nv));
				fclose(f);
			}
		}
	}
	rbx_aux_set_job(job);
	return 0;
}

static void tf_exit(void)
{
	rbx_aux_set_job(NULL);
	if (s_nv_path[0]) {
		FILE *f = fopen(s_nv_path, "wb");
		if (f) {
			fwrite(s_nv, 1, sizeof(s_nv), f);
			fclose(f);
		}
		s_nv_path[0] = 0;
	}
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
}

static void tf_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	sn76496_reset(&s_sn);
	vlm5030_reset(&s_vlm);
	const int konami1 = s_cpu.konami1;
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.konami1 = konami1;
	for (int p = 0x18; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x18) * 256;
	for (int p = 0x28; p < 0x30; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_nv + (p - 0x28) * 256;
	for (int p = 0x30; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.vram + (p - 0x30) * 256;
	for (int p = 0x60; p < 0x100; p++)
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
	for (int p = 0x40; p < 0x60; p++)
		s_zmap[p] = t.zram + (p & 3) * 256;
	s_z.read_map = s_zmap;
}

static void tf_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_mask)
				m6809_set_irq(&s_cpu, 1);
		}
		t.cpu_end += CPU_LINE;
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

static int tf_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "tf_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "tf", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "tf_nv", s_nv, sizeof(s_nv));
	rc |= rbx_state_io(ctx, "tf_sn", &s_sn, sn76496_state_size());
	rc |= vlm5030_state(&s_vlm, ctx, load, "tf_vlm");
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "tf_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "tf_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tf_z80", &z, sizeof(z));
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

static int tf_save(void *ctx) { return tf_state(ctx, 0); }
static int tf_load(void *ctx) { return tf_state(ctx, 1); }
static int tf_supported(const char *machine) { return !strcmp(machine, "trackfld") || !strcmp(machine, "trackfldu"); }

const rbx_driver rbx_drv_trackfld = { "Konami Track & Field", tf_init, tf_exit, tf_reset, tf_frame, tf_save, tf_load,
                                      tf_supported, tf_render, NULL };

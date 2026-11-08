/*  Konami Mikie / Shinnyuushain Tooru-kun. Ported from MAME src/mame/konami/mikie.cpp (BSD-3-Clause,
    copyright-holders: Allard van der Bas) and the palette's resistor network of src/emu/video/resnet.cpp (BSD-3-Clause,
    copyright-holders: Couriersud). A 6809E at 1.536 MHz, a 32x32 tilemap in two categories around the sprites, two
    sprite sets interleaved byte by byte, the colours through the PROMs; the sound: a Z80 at 3.58 MHz and two
    SN76489As, a frame behind on the worker. The flip screen not done for the tilemap.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/sn76496.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12)
#define Z80_CLOCK 3579545u     // (14.318 MHz / 4; the second SN76489A, the first at half that)
#define SND_RATE 48000
#define FRAME_MHZ 60590u       // (set_refresh_hz(60.59), set_size(256, 256), set_vblank_time(0))
#define LINES 256
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x1000           // (16 * 8 * 16 * 2 pens over 256 colours)

static uint8_t *s_rom, *s_zrom, *s_prom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_chr, *s_spr[2];   // (a byte a pixel: the characters, the two sprite sets)
static uint32_t s_nchr;
static uint32_t s_rgb[256];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static sn76496_t s_sn[2];
static int s_port_dsw[3], s_port_p[2], s_port_system;

static struct {
	uint8_t ram0[0x100];     // 0000-00ff
	uint8_t ram[0x1800];     // 2800-3fff (2800-288f the sprites, 3800 the colour RAM, 3c00 the video RAM)
	uint8_t latch, palettebank, irq_mask;
	uint8_t zram[0x400], soundlatch, z80_irq;
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, sn_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t spr[0x90], cram[0x400], vram[0x400], palettebank; } s_vs;

static void capture(void)
{
	memcpy(s_vs.spr, t.ram, sizeof(s_vs.spr));
	memcpy(s_vs.cram, t.ram + 0x1000, sizeof(s_vs.cram));
	memcpy(s_vs.vram, t.ram + 0x1400, sizeof(s_vs.vram));
	s_vs.palettebank = t.palettebank;
}

static inline uint8_t ctab(int q)   // (the pen's colour: pen ((i & 0x100) << 3) | (j << 8) | (i & 0xff))
{
	const int i = (q & 0xff) | (((q >> 11) & 1) << 8), j = (q >> 8) & 7;
	return (uint8_t)((j << 5) | ((~i & 0x100) >> 4) | (s_prom[0x300 + i] & 0x0f));
}

static void draw_tiles(int category, uint16_t *bm, int pitch)
{
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			const int idx = (y >> 3) * 32 + col;
			const int c = s_vs.cram[idx];
			if (((c >> 4) & 1) != category)
				continue;
			const int code = s_vs.vram[idx] + ((c & 0x20) << 3);
			const int pen = ((c & 0x0f) + 16 * s_vs.palettebank) * 16;
			const int fx = (c & 0x40) != 0, fy = (c & 0x80) != 0;
			const uint8_t *src = s_chr + ((uint32_t)code % s_nchr) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(pen + src[fx ? 7 - i : i]);
		}
	}
}

static void mk_render(void)   // (screen_update)
{
	for (int q = 0; q < NPENS; q++)
		rbx_video.palette[q] = s_rgb[ctab(q)];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_tiles(0, bm, pitch);
	const uint8_t *sr = s_vs.spr;
	for (int offs = 0; offs < 0x90; offs += 4) {   // (draw_sprites: pen 0 transparent)
		const int set = (sr[offs + 2] & 0x40) ? 1 : 0;
		const int code = (sr[offs + 2] & 0x3f) + ((sr[offs + 2] & 0x80) >> 1) + ((sr[offs] & 0x40) << 1);
		const int pen = 16 * 8 * 16 + ((sr[offs] & 0x0f) + 16 * s_vs.palettebank) * 16;
		const int sx = sr[offs + 3], sy = 244 - sr[offs + 1];
		const int flipx = !(sr[offs] & 0x10), flipy = (sr[offs] & 0x20) != 0;
		const uint8_t *src = s_spr[set] + (uint32_t)(code & 0xff) * 256;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int k = 0; k < 16; k++) {
				const int xx = sx + k;
				if (xx >= VIS_W)
					continue;
				const int p = s[flipx ? 15 - k : k];
				if (p)
					d[xx] = (uint16_t)(pen + p);
			}
		}
	}
	draw_tiles(1, bm, pitch);
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
	if (a < 0x0100)
		return t.ram0[a];
	switch (a) {
	case 0x2400: return port(s_port_system);
	case 0x2401: return port(s_port_p[0]);
	case 0x2402: return port(s_port_p[1]);
	case 0x2403: return port(s_port_dsw[2]);
	case 0x2500: return port(s_port_dsw[0]);
	case 0x2501: return port(s_port_dsw[1]);
	}
	if (a >= 0x2800 && a < 0x4000)
		return t.ram[a - 0x2800];
	if (a >= 0x4000)
		return s_rom[a % s_rom_size];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0100)
		t.ram0[a] = v;
	else if (a >= 0x2800 && a < 0x4000)
		t.ram[a - 0x2800] = v;
	else if (a >= 0x2000 && a < 0x2008) {   // (the LS259: Q2 the sound's IRQ on its rising edge, Q7 the IRQ mask)
		const int bit = a & 7, old = (t.latch >> bit) & 1, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 2 && now && !old)
			q_put(Q_IRQ, 0);
		else if (bit == 7) {
			t.irq_mask = (uint8_t)now;
			if (!now)
				m6809_set_irq(&s_cpu, 0);
		}
	} else if (a == 0x2200)
		t.palettebank = v & 0x07;
	else if (a == 0x2400)
		q_put(Q_LATCH, v);
}

// ---- the sound board (sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x4000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x4400)
		return t.zram[a - 0x4000];
	if (a == 0x8003)
		return t.soundlatch;
	if (a == 0x8005)   // (sh_timer_r: the Z80's clock / 512)
		return (uint8_t)(s_z.cyc / 512);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400)
		t.zram[a - 0x4000] = v;
	else if (a == 0x8002)
		sn76496_write(&s_sn[0], v);
	else if (a == 0x8004)
		sn76496_write(&s_sn[1], v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, the SN76489As 0.60 each)
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
	int32_t a[64], b[64];
	memset(a, 0, sizeof(a));
	memset(b, 0, sizeof(b));
	t.sn_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	const int ka = sn76496_render(&s_sn[0], sc / 2, a, 64), kb = sn76496_render(&s_sn[1], sc, b, 64);
	for (int i = 0; i < n && s_out_pos < 2048; i++)
		s_out[s_out_pos++] = (ka > 0 ? a[i * ka / n] : 0) + (kb > 0 ? b[i * kb / n] : 0);
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
static void mk_exit(void);
static void mk_reset(void);

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
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	for (int k = 0; k < n; k++)
		w[k] *= 255.0 / sum;
}

static int mk_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_zrom || !s_prom || ps < 0x500 || !tiles || !sprites || ss < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb; spritelayout: 256 of 16x16 at byte 0 and byte 1, planes { 0, 4 } and 0x8000 on, the
	// 4-pixel columns at bytes 32, 16, 0, 48, a row 2 bytes, the lower half 64 on)
	s_nchr = ts / 32;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr[0] = (uint8_t *)malloc(256 * 256);
	s_spr[1] = (uint8_t *)malloc(256 * 256);
	if (!s_chr || !s_spr[0] || !s_spr[1]) {
		rbx_fail("%s: out of memory", g->set);
		mk_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = tiles[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = tiles[e * 32 + (uint32_t)i] & 15;
		}
	static const int xg[4] = { 32, 16, 0, 48 };
	for (int set = 0; set < 2; set++)
		for (uint32_t e = 0; e < 256; e++)
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 16; x++) {
					const uint32_t o = (uint32_t)set + e * 128 + (uint32_t)(xg[x >> 2] + (y & 7) * 2 + (y >> 3) * 64);
					const uint8_t b1 = sprites[o], b2 = sprites[0x8000 + o];
					const int q = x & 3;
					s_spr[set][e * 256 + (uint32_t)(y * 16 + x)] = (uint8_t)(((b1 >> (7 - q)) & 1) << 3 | ((b1 >> (3 - q)) & 1) << 2 |
					                                                         ((b2 >> (7 - q)) & 1) << 1 | ((b2 >> (3 - q)) & 1));
				}
	static const int res[4] = { 2200, 1000, 470, 220 };
	double w[4];
	resistor_weights(res, 4, 470, w);
	for (int i = 0; i < 256; i++) {
		int c[3];
		for (int k = 0; k < 3; k++) {
			const uint8_t v = s_prom[i + (k << 8)];
			c[k] = (int)(w[0] * (v & 1) + w[1] * ((v >> 1) & 1) + w[2] * ((v >> 2) & 1) + w[3] * ((v >> 3) & 1) + 0.5);
		}
		s_rgb[i] = (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
	}
	sn76496_init(&s_sn[0], Z80_CLOCK / 2, SN76489A_CONFIG, 154, 32, 1);   // (0.60)
	sn76496_init(&s_sn[1], Z80_CLOCK, SN76489A_CONFIG, 154, 32, 1);
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		mk_exit();
		return -1;
	}
	rbx_audio.frame_mhz = FRAME_MHZ;
	mk_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void mk_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_chr);
	free(s_spr[0]);
	free(s_spr[1]);
	s_chr = s_spr[0] = s_spr[1] = NULL;
}

static void mk_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	sn76496_reset(&s_sn[0]);
	sn76496_reset(&s_sn[1]);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.rpage[0] = s_cpu.wpage[0] = t.ram0;
	for (int p = 0x28; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x28) * 256;
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

static void mk_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_mask)
				m6809_set_irq(&s_cpu, 1);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * 1000;
		const int64_t cyc = (int64_t)(t.cyc_frac / ((uint64_t)FRAME_MHZ * LINES));
		t.cyc_frac -= (uint64_t)cyc * FRAME_MHZ * LINES;
		t.cpu_end += cyc;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.snd_frac += (uint64_t)SND_RATE * 1000;
		const int ns = (int)(t.snd_frac / ((uint64_t)FRAME_MHZ * LINES));
		t.snd_frac -= (uint64_t)ns * FRAME_MHZ * LINES;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int mk_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "mk_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "mk", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "mk_sn0", &s_sn[0], sn76496_state_size());
	rc |= rbx_state_io(ctx, "mk_sn1", &s_sn[1], sn76496_state_size());
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "mk_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "mk_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "mk_z80", &z, sizeof(z));
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

static int mk_save(void *ctx) { return mk_state(ctx, 0); }
static int mk_load(void *ctx) { return mk_state(ctx, 1); }
static int mk_supported(const char *machine) { return !strcmp(machine, "mikie"); }

const rbx_driver rbx_drv_mikie = { "Konami Mikie", mk_init, mk_exit, mk_reset, mk_frame, mk_save, mk_load, mk_supported,
                                   mk_render, NULL };

/*  Konami / Interlogic Pandora's Palace. Ported from MAME src/mame/konami/pandoras.cpp (BSD-3-Clause,
    copyright-holders: Manuel Abadia) and the palette's resistor networks of src/emu/video/resnet.cpp (BSD-3-Clause,
    copyright-holders: Couriersud). Two 6809Es at 3.072 MHz sharing the video RAM and 2 KB, line by line (each one
    FIRQs the other; the second held in reset by the latch); a 32x32 tilemap in two categories around the sprites, the
    colours through the lookup PROMs; the sound: a Z80 at 1.79 MHz with an AY-3-8910 and an I8039 on a DAC, a frame
    behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3072000u     // (18.432 MHz / 6, the two 6809s)
#define Z80_CLOCK 1789772u     // (14.318 MHz / 8; the AY too)
#define MCS_CYCLES 477273u     // (the I8039 at 14.318 MHz / 2: 15 clocks a machine cycle)
#define SND_RATE 48000
#define LINES 256              // (set_refresh_hz(60), set_size(256, 256), set_vblank_time(0))
#define CPU_LINE (CPU_CLOCK / 60 / LINES)   // (200 clocks a line)
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200

static uint8_t *s_rom, *s_srom, *s_zrom, *s_mrom, *s_prom;
static uint32_t s_rom_size, s_srom_size, s_zrom_size, s_mrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static m6809_t s_cpu, s_sub;
static z80 s_z;
static const uint8_t *s_zmap[256];
static mcs48 s_mcs;
static ay8910_t s_ay;
static int s_port_dsw[3], s_port_p[2], s_port_system;

static struct {
	uint8_t vram[0x1800];    // 0000-17ff of both (0000 the sprites' RAM, 1000 the colour RAM, 1400 the video RAM)
	uint8_t shared[0x800];   // (A 6000, B c000)
	uint8_t latch, scrolly, firq_old_a, firq_old_b;
	uint8_t zram[0x400], soundlatch, soundlatch2, z80_irq, dac, i8039_status;
	int32_t mcs_left, hp_x, hp_y;
	int64_t cpu_end;         // (the CPUs' line target: absolute, an instruction's overshoot not carried on)
	uint64_t z_frac, z_end, mcs_frac, ay_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t spr[0x100], cram[0x400], vram[0x400], scrolly, flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.spr, t.vram + 0x800, sizeof(s_vs.spr));
	memcpy(s_vs.cram, t.vram + 0x1000, sizeof(s_vs.cram));
	memcpy(s_vs.vram, t.vram + 0x1400, sizeof(s_vs.vram));
	s_vs.scrolly = t.scrolly;
	s_vs.flip = (t.latch >> 5) & 1;   // (Q5: flip_screen_set, which only the tilemap follows)
}

static void draw_tiles(int category, uint16_t *bm, int pitch)
{
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int flip = s_vs.flip;
		const int ty = ((flip ? 255 - y : y) + s_vs.scrolly) & 255;
		for (int col = 0; col < 32; col++) {
			const int idx = (ty >> 3) * 32 + (flip ? 31 - col : col);
			const int attr = s_vs.cram[idx];
			if (((attr >> 5) & 1) != category)
				continue;
			const int code = s_vs.vram[idx] + ((attr & 0x10) << 4);
			const int pen = 0x100 + (attr & 0x0f) * 16;
			const int fx = ((attr & 0x40) != 0) ^ flip, fy = (attr & 0x80) != 0;
			const uint8_t *src = s_chr + ((uint32_t)code % s_nchr) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(pen + src[fx ? 7 - i : i]);
		}
	}
}

static void pd_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the sprites' 0-15, 100-1ff the characters' 16-31)
		rbx_video.palette[i] = s_rgb[s_prom[0x20 + i] & 0x0f];
		rbx_video.palette[0x100 + i] = s_rgb[(s_prom[0x120 + i] & 0x0f) | 0x10];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_tiles(1, bm, pitch);
	const uint8_t *sr = s_vs.spr;
	for (int offs = 0; offs < 0x100; offs += 4) {   // (draw_sprites: transmask of the pens mapping to colour 0)
		const int sx = sr[offs + 1], sy = 240 - sr[offs];
		const int color = sr[offs + 3] & 0x0f;
		const int flipx = !(sr[offs + 3] & 0x40), flipy = !(sr[offs + 3] & 0x80);
		const uint8_t *src = s_spr + ((uint32_t)sr[offs + 2] % s_nspr) * 256;
		const uint8_t *lut = s_prom + 0x20 + color * 16;
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
				if (lut[p] & 0x0f)
					d[xx] = (uint16_t)(color * 16 + p);
			}
		}
	}
	draw_tiles(0, bm, pitch);
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

// ---- the CPUs' buses (master_map, slave_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static void latch_w(int bit, int now)   // (the LS259 both CPUs write: Q0 / Q6 the IRQ enables, Q7 B's /RESET)
{
	const int old = (t.latch >> bit) & 1;
	t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
	if (bit == 0 && !now)
		m6809_set_irq(&s_cpu, 0);
	else if (bit == 6 && !now)
		m6809_set_irq(&s_sub, 0);
	else if (bit == 7 && now && !old)
		m6809_reset(&s_sub);   // (released from reset)
}

static uint8_t a_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x1800)
		return t.vram[a];
	if (a >= 0x6000 && a < 0x6800)
		return t.shared[a - 0x6000];
	if (a >= 0x4000 && (a < 0x6000 || a >= 0x8000))
		return s_rom[a % s_rom_size];
	return 0;
}
static void a_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x1800)
		t.vram[a] = v;
	else if (a >= 0x1800 && a < 0x1808)
		latch_w(a & 7, v & 1);
	else if (a == 0x1a00)
		t.scrolly = v;
	else if (a == 0x1c00)
		q_put(Q_IRQ, 0);   // (z80_irqtrigger_w: HOLD_LINE)
	else if (a == 0x1e00)
		q_put(Q_LATCH, v);
	else if (a == 0x2000) {   // (cpub_irqtrigger_w)
		if (!t.firq_old_b && v)
			m6809_hold_firq(&s_sub);
		t.firq_old_b = v;
	} else if (a >= 0x6000 && a < 0x6800)
		t.shared[a - 0x6000] = v;
}

static uint8_t b_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x1800)
		return t.vram[a];
	switch (a) {
	case 0x1800: return port(s_port_dsw[0]);
	case 0x1a00: return port(s_port_system);
	case 0x1a01: return port(s_port_p[0]);
	case 0x1a02: return port(s_port_p[1]);
	case 0x1a03: return port(s_port_dsw[2]);
	case 0x1c00: return port(s_port_dsw[1]);
	}
	if (a >= 0xc000 && a < 0xc800)
		return t.shared[a - 0xc000];
	if (a >= 0xe000)
		return s_srom[a % s_srom_size];
	return 0;
}
static void b_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x1800)
		t.vram[a] = v;
	else if (a >= 0x1800 && a < 0x1808)
		latch_w(a & 7, v & 1);
	else if (a == 0xa000) {   // (cpua_irqtrigger_w)
		if (!t.firq_old_a && v)
			m6809_hold_firq(&s_cpu);
		t.firq_old_a = v;
	} else if (a >= 0xc000 && a < 0xc800)
		t.shared[a - 0xc000] = v;
}

// ---- the sound board (sound_map; the I8039: every I/O read latch 2, P1 the DAC, P2 bit 7 low clears its IRQ) ----
static uint8_t ay_port_a(void *u) { (void)u; return t.i8039_status; }
static uint8_t ay_port_b(void *u) { (void)u; return (uint8_t)((s_z.cyc / 512) & 0x0f); }
static uint8_t mcs_ext_r(void *u, uint8_t a) { (void)u; (void)a; return t.soundlatch2; }
static void mcs_port_w(void *u, int p, uint8_t v)
{
	(void)u;
	if (p == 1)
		t.dac = v;
	else if (p == 2) {
		if (!(v & 0x80))
			mcs48_set_irq(&s_mcs, 0);
		t.i8039_status = (v & 0x20) >> 5;
	}
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x2400)
		return t.zram[a - 0x2000];
	if (a == 0x4000)
		return t.soundlatch;
	if (a == 0x6001)
		return ay8910_data_r(&s_ay);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2400)
		t.zram[a - 0x2000] = v;
	else if (a == 0x6000)
		ay8910_address_w(&s_ay, v);
	else if (a == 0x6002)
		ay8910_data_w(&s_ay, v);
	else if (a == 0x8000)
		mcs48_set_irq(&s_mcs, 1);   // (i8039_irqtrigger_w: ASSERT_LINE)
	else if (a == 0xa000)
		t.soundlatch2 = v;
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, the I8039, the AY 0.40, the DAC 0.12)
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
	t.mcs_frac += (uint64_t)MCS_CYCLES * (uint64_t)n;
	const int mc = (int)(t.mcs_frac / SND_RATE);
	t.mcs_frac -= (uint64_t)mc * SND_RATE;
	t.mcs_left += mc;
	if (t.mcs_left > 0)
		t.mcs_left -= mcs48_execute(&s_mcs, t.mcs_left);
	if (n > 64)
		n = 64;
	int32_t a1[96];
	memset(a1, 0, sizeof(a1));
	t.ay_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const uint32_t ac = (uint32_t)(t.ay_frac / SND_RATE);
	t.ay_frac -= (uint64_t)ac * SND_RATE;
	const int n1 = ay8910_render(&s_ay, ac, a1, 96);
	const int32_t dac = ((int32_t)t.dac - 128) * 256 * 12 / 100 * 2;   // (the DAC's range -1 to 1, 0.12)
	for (int i = 0; i < n && s_out_pos < 2048; i++) {   // (a high-pass of about 34 Hz: the DAC idles at 0)
		const int32_t x = (n1 > 0 ? a1[i * n1 / n] : 0) + dac;
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
static void pd_exit(void);
static void pd_reset(void);

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

static void mcs_setup(void)
{
	s_mcs.rom = s_mrom;
	s_mcs.rom_mask = 0x0fff;
	s_mcs.userdata = NULL;
	s_mcs.ext_r = mcs_ext_r;
	s_mcs.port_w = mcs_port_w;
}

static int pd_init(const rbx_game *g)
{
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("sub", &s_srom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mrom = rbx_region("mcu", &s_mrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_srom || !s_zrom || !s_mrom || s_mrom_size < 0x1000 || !s_prom || ps < 0x220 || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb; spritelayout: 16x16 4 bpp packed, 8 bytes a row, the pixels and the rows reversed)
	s_nchr = ts / 32;
	s_nspr = ss / 128;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		pd_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = tiles[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = tiles[e * 32 + (uint32_t)i] & 15;
		}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const int nib = 15 - x;
				const uint8_t b = sprites[e * 128 + (uint32_t)((15 - y) * 8 + nib / 2)];
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = (nib & 1) ? b & 15 : b >> 4;
			}
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
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, Z80_CLOCK, AY8910_LEGACY_OUTPUT | AY8910_SINGLE_OUTPUT, 102, 4, 0);   // (0.40)
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		pd_exit();
		return -1;
	}
	pd_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void pd_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
}

static void pd_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	ay8910_reset(&s_ay);
	s_ay.port_a_read = ay_port_a;
	s_ay.port_b_read = ay_port_b;
	mcs48_init(&s_mcs, MCS48_I8039);
	mcs_setup();
	mcs48_reset(&s_mcs);
	m6809_init(&s_cpu);
	s_cpu.read = a_rd;
	s_cpu.write = a_wr;
	for (int p = 0; p < 0x18; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.vram + p * 256;
	for (int p = 0x60; p < 0x68; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.shared + (p - 0x60) * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
	m6809_init(&s_sub);
	s_sub.read = b_rd;
	s_sub.write = b_wr;
	for (int p = 0; p < 0x18; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = t.vram + p * 256;
	for (int p = 0xc0; p < 0xc8; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = t.shared + (p - 0xc0) * 256;
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
	for (int p = 0; p < 0x20; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 4; p++)
		s_zmap[0x20 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void pd_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.latch & 0x01)
				m6809_set_irq(&s_cpu, 1);
			if (t.latch & 0x40)
				m6809_set_irq(&s_sub, 1);
		}
		t.cpu_end += CPU_LINE;
		m6809_run_until(&s_cpu, t.cpu_end);
		if (t.latch & 0x80)   // (B runs while its /RESET is high)
			m6809_run_until(&s_sub, t.cpu_end);
		else
			s_sub.cycles = t.cpu_end;
		t.snd_frac += SND_RATE;
		const int ns = (int)(t.snd_frac / (60u * LINES));
		t.snd_frac -= (uint64_t)ns * (60u * LINES);
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int pd_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "pd_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "pd_sub", &s_sub, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "pd", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "pd_mcs", (uint8_t *)&s_mcs + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	rc |= rbx_state_io(ctx, "pd_ay", &s_ay, ay8910_state_size());
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "pd_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "pd_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "pd_z80", &z, sizeof(z));
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
		mcs_setup();
	}
	return rc ? -1 : 0;
}

static int pd_save(void *ctx) { return pd_state(ctx, 0); }
static int pd_load(void *ctx) { return pd_state(ctx, 1); }
static int pd_supported(const char *machine) { return !strcmp(machine, "pandoras"); }

const rbx_driver rbx_drv_pandoras = { "Konami Pandora's Palace", pd_init, pd_exit, pd_reset, pd_frame, pd_save, pd_load,
                                      pd_supported, pd_render, NULL };

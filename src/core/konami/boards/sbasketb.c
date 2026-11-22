/*  Konami Super Basketball. Ported from MAME src/mame/konami/sbasketb.cpp (BSD-3-Clause, copyright-holders: Zsolt
    Vasvari), the sound board's src/mame/konami/trackfld_a.cpp (BSD-3-Clause, copyright-holders: Chris Hardy) and the
    palette's resistor network of src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud). A KONAMI1 at
    1.536 MHz (a plain 6809E on sbasketbu), a 32x32 tilemap with column scroll, sprites over it in 16 palette banks, the
    colours through the lookup PROMs; the sound: a Z80 at 3.58 MHz, an SN76489A through a latch, a VLM5030 and a DAC, a
    frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/sn76496.h"
#include "../../sound/vlm5030.h"

#include <stddef.h>
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
#define NPENS 0x1100           // (16 * 16 + 16 * 16 * 16)
#define ZROM_SIZE 0x2000

static uint8_t *s_rom, *s_zrom, *s_prom, *s_vlmrom;
static uint32_t s_rom_size, s_zrom_size, s_vlmrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[256];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static sn76496_t s_sn;
static vlm5030_t s_vlm;
static int s_konami1;
static int s_port_dsw[2], s_port_p[2], s_port_system;

static struct {
	uint8_t ram[0x1c00];     // 2000-3bff (3000 the colour RAM, 3400 the video RAM, 3800-39ff the sprites)
	uint8_t scroll, palettebank, latch, irq_mask, spriteram_select, flip;
	uint8_t zram[0x400], soundlatch, z80_irq, sn_latch, dac;
	int16_t vlm_out;
	int32_t hp_x, hp_y;      // (the output's high-pass: the DAC idles at 0)
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t z_frac, z_end, sn_frac, vlm_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t cram[0x400], vram[0x400], spr[0x100], scroll, palettebank, flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.cram, t.ram + 0x1000, sizeof(s_vs.cram));
	memcpy(s_vs.vram, t.ram + 0x1400, sizeof(s_vs.vram));
	memcpy(s_vs.spr, t.ram + 0x1800 + (t.spriteram_select ? 0x100 : 0), sizeof(s_vs.spr));
	s_vs.scroll = t.scroll;
	s_vs.palettebank = t.palettebank;
	s_vs.flip = t.flip;
}

static void sb_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: the characters 0xf0-0xff, the sprites 16 banks of the whole 256)
		rbx_video.palette[i] = s_rgb[(s_prom[0x300 + i] & 0x0f) | 0xf0];
		for (int j = 0; j < 0x10; j++)
			rbx_video.palette[0x100 + ((j << 8) | i)] = s_rgb[(j << 4) | (s_prom[0x400 + i] & 0x0f)];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int flip = s_vs.flip;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int sc = 0; sc < 32; sc++) {
			const int col = flip ? 31 - sc : sc;
			const int ty = ((flip ? 255 - y : y) + (col >= 6 ? s_vs.scroll : 0)) & 255;
			const int idx = (ty >> 3) * 32 + col;
			const int attr = s_vs.cram[idx];
			const int code = s_vs.vram[idx] + ((attr & 0x20) << 3);
			const int pen = (attr & 0x0f) * 16;
			const int fx = ((attr & 0x40) != 0) ^ flip, fy = (attr & 0x80) != 0;
			const uint8_t *src = s_chr + ((uint32_t)code % s_nchr) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[sc * 8 + i] = (uint16_t)(pen + src[fx ? 7 - i : i]);
		}
	}
	const uint8_t *sr = s_vs.spr;
	for (int offs = 0; offs < 0x100; offs += 4) {   // (draw_sprites: pen 0 transparent)
		int sx = sr[offs + 2], sy = sr[offs + 3];
		if (!sx && !sy)
			continue;
		const int code = sr[offs] | ((sr[offs + 1] & 0x20) << 3);
		const int color = ((sr[offs + 1] & 0x0f) + 16 * s_vs.palettebank) & 0xff;   // (the gfx wraps at 256 colours)
		int flipx = (sr[offs + 1] & 0x40) != 0, flipy = (sr[offs + 1] & 0x80) != 0;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			flipx = !flipx;
			flipy = !flipy;
		}
		const uint8_t *src = s_spr + ((uint32_t)code % s_nspr) * 256;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int k = 0; k < 16; k++) {
				const int xx = sx + k;
				if (xx < 0 || xx >= VIS_W)
					continue;
				const int p = s[flipx ? 15 - k : k];
				if (p)
					d[xx] = (uint16_t)(0x100 + color * 16 + p);
			}
		}
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

// ---- the main CPU's bus (main_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x3c00)
		return t.ram[a - 0x2000];
	switch (a) {
	case 0x3e00: return port(s_port_system);
	case 0x3e01: return port(s_port_p[0]);
	case 0x3e02: return port(s_port_p[1]);
	case 0x3e80: return port(s_port_dsw[1]);
	case 0x3f00: return port(s_port_dsw[0]);
	}
	if (a >= 0x6000)
		return s_rom[a % s_rom_size];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x3c00)
		t.ram[a - 0x2000] = v;
	else if (a >= 0x3c80 && a < 0x3c88) {   // (the LS259: Q0 the flip, Q1 the IRQ mask, Q5 the sprite bank)
		const int bit = a & 7, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 0)
			t.flip = (uint8_t)now;
		else if (bit == 1) {
			t.irq_mask = (uint8_t)now;
			if (!now)
				m6809_set_irq(&s_cpu, 0);
		} else if (bit == 5)
			t.spriteram_select = (uint8_t)now;
	} else if (a == 0x3c20)
		t.palettebank = v;
	else if (a == 0x3d00)
		q_put(Q_LATCH, v);
	else if (a == 0x3d80)
		q_put(Q_IRQ, 0);   // (sh_irqtrigger_w: HOLD_LINE)
	else if (a == 0x3f80)
		t.scroll = v;
}

// ---- the sound board (sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < ZROM_SIZE)
		return s_zrom[a];
	if (a >= 0x4000 && a < 0x4400)
		return t.zram[a - 0x4000];
	if (a == 0x6000)
		return t.soundlatch;
	if (a == 0x8000)   // (hyperspt_sh_timer_r: the Z80's clock / 1024)
		return (uint8_t)(((s_z.cyc >> 10) & 3) | (vlm5030_bsy(&s_vlm) ? 4 : 0));
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400)
		t.zram[a - 0x4000] = v;
	else if (a == 0xa000)
		vlm5030_data_w(&s_vlm, v);
	else if (a >= 0xc000 && a < 0xe000) {   // (hyperspt_sound_w: A4 the VLM5030's ST, A5 its RST)
		vlm5030_st(&s_vlm, ((a - 0xc000) >> 4) & 1);
		vlm5030_rst(&s_vlm, ((a - 0xc000) >> 5) & 1);
	} else if (a == 0xe000)
		t.dac = v;
	else if (a == 0xe001)
		t.sn_latch = v;
	else if (a == 0xe002)
		sn76496_write(&s_sn, t.sn_latch);
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
		t.vlm_frac += vlm5030_rate(&s_vlm);
		while (t.vlm_frac >= SND_RATE) {
			t.vlm_frac -= SND_RATE;
			int16_t s;
			vlm5030_render(&s_vlm, &s, 1);
			t.vlm_out = s;
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
static void sb_exit(void);
static void sb_reset(void);

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

static int sb_init(const rbx_game *g)
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
	if (!s_rom || !s_zrom || s_zrom_size < ZROM_SIZE || !s_prom || ps < 0x500 || !s_vlmrom || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_konami1 = strcmp(g->machine, "sbasketbu") != 0;
	// (gfx_8x8x4_packed_msb; gfx_16x16x4_packed_msb: a row of 8 bytes)
	s_nchr = ts / 32;
	s_nspr = ss / 128;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = tiles[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = tiles[e * 32 + (uint32_t)i] & 15;
		}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int i = 0; i < 128; i++) {
			s_spr[e * 256 + (uint32_t)i * 2] = sprites[e * 128 + (uint32_t)i] >> 4;
			s_spr[e * 256 + (uint32_t)i * 2 + 1] = sprites[e * 128 + (uint32_t)i] & 15;
		}
	static const int res[4] = { 2000, 1000, 470, 220 };
	double w[4];
	resistor_weights(res, 4, 1000, w);
	for (int i = 0; i < 256; i++) {
		int c[3];
		for (int k = 0; k < 3; k++) {
			const uint8_t v = s_prom[i + (k << 8)];
			c[k] = (int)(w[0] * (v & 1) + w[1] * ((v >> 1) & 1) + w[2] * ((v >> 2) & 1) + w[3] * ((v >> 3) & 1) + 0.5);
		}
		s_rgb[i] = (uint32_t)(c[0] << 16 | c[1] << 8 | c[2]);
	}
	sn76496_init(&s_sn, Z80_CLOCK / 2, SN76489A_CONFIG, 256, 32, 1);
	vlm5030_init(&s_vlm, 3579545, s_vlmrom, s_vlmrom_size);
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		sb_exit();
		return -1;
	}
	sb_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void sb_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void sb_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	sn76496_reset(&s_sn);
	vlm5030_reset(&s_vlm);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.konami1 = (uint8_t)s_konami1;
	for (int p = 0x20; p < 0x3c; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x20) * 256;
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
	for (int p = 0; p < ZROM_SIZE / 256; p++)
		s_zmap[p] = s_zrom + p * 256;
	for (int p = 0; p < 4; p++)
		s_zmap[0x40 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void sb_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_mask)
				m6809_hold_irq(&s_cpu);
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

static int sb_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "sb_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "sb", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "sb_sn", &s_sn, sn76496_state_size());
	rc |= vlm5030_state(&s_vlm, ctx, load, "sb_vlm");
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "sb_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "sb_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "sb_z80", &z, sizeof(z));
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

static int sb_save(void *ctx) { return sb_state(ctx, 0); }
static int sb_load(void *ctx) { return sb_state(ctx, 1); }
static int sb_supported(const char *machine) { return !strcmp(machine, "sbasketb") || !strcmp(machine, "sbasketbu"); }

const rbx_driver rbx_drv_sbasketb = { "Konami Super Basketball", sb_init, sb_exit, sb_reset, sb_frame, sb_save, sb_load,
                                      sb_supported, sb_render, NULL };

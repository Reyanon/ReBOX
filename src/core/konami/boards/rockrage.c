/*  Konami Rock'n Rage / Koi no Hotrock. Ported from MAME src/mame/konami/rockrage.cpp (BSD-3-Clause,
    copyright-holders: Manuel Abadia) and the devices it uses (k007342.cpp, k007420.cpp). An HD6309E at 3 MHz, a
    K007342 (tilemaps) and a K007420 (sprites), 64 colours through the lookup PROMs; the sound: a 6809E at 1.5 MHz, a
    YM2151 and a VLM5030, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/vlm5030.h"
#include "../video/k007342.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define SND_CLOCK 1500000u     // (24 MHz / 16)
#define YM_CLOCK 3579545u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 768              // (16 * 16 * 3 pens over 64 colours)

static uint8_t *s_rom, *s_srom, *s_vlmrom, *s_prom;
static uint32_t s_rom_size, s_srom_size, s_vlmrom_size;
static uint8_t *s_gfx[2], *s_opaque;   // (tiles, sprites; the sprites' elements with a visible pixel)
static uint32_t s_gfx_count[2];
static hd6309_t s_cpu;
static m6809_t s_snd;
static vlm5030_t s_vlm;
static k007342_t s_k342;
static int s_port_system, s_port_p[2], s_port_dsw[2];

static struct {
	uint8_t pal[0x80];       // 2400-247f
	uint8_t spr[0x200];      // 2000-21ff (the K007420's)
	uint8_t ram[0x2000];     // 4000-5fff
	uint8_t mainbank, vreg;
	uint8_t sram[0x800], soundlatch, snd_irq;
	int16_t vlm_out;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, s_frac, ym_frac, vlm_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x80], spr[0x200], vreg; k007342_t k342; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	s_vs.vreg = t.vreg;
	memcpy(&s_vs.k342, &s_k342, sizeof(k007342_t));
}

static void tile_cb(int layer, uint32_t bank, uint32_t *code, uint32_t *color, int *flags)
{
	(void)flags;
	const uint32_t vreg = s_vs.vreg;
	if (layer == 1)   // (doesn't use the bank: Tutankhamen's eyes blinking)
		*code |= ((*color & 0x40) << 2) | ((vreg & 0x04) << 7);
	else
		*code |= ((*color & 0x40) << 2) | ((bank & 0x03) << 10) | ((vreg & 0x04) << 7) | ((vreg & 0x08) << 9);
	*color = (uint32_t)layer * 16 + (*color & 0x0f);
}
static void sprite_cb(uint32_t *code, uint32_t *color)
{
	const uint32_t vreg = s_vs.vreg;
	*code |= ((*color & 0x40) << 2) | ((*color & 0x80) << 1) * ((vreg & 0x03) << 1);
	*code = (*code << 2) | ((*color & 0x30) >> 4);
	*color = *color & 0x0f;
}

static void rr_render(void)   // (screen_update)
{
	uint32_t col[64];
	for (int i = 0; i < 64; i++) {   // (xBGR_555, little-endian)
		const uint16_t v = (uint16_t)(s_vs.pal[2 * i] | s_vs.pal[2 * i + 1] << 8);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		col[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	for (int i = 0; i < NPENS; i++)   // (palette: the lookup tables)
		rbx_video.palette[i] = col[(s_prom[i] & 0x0f) | (i / 256) * 16];
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < VIS_H; y++)
		memset(bm + y * rbx_video.pitch, 0, VIS_W * sizeof(uint16_t));
	const int x1 = VIS_X0 + VIS_W - 1, y1 = VIS_Y0 + VIS_H - 1, pitch = rbx_video.pitch;
#define LAYER(l, cat, op) k007342_draw(&s_vs.k342, l, cat, op, 0, s_gfx[0], s_gfx_count[0], 0, tile_cb, bm, NULL, pitch, \
	                                   VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1)
	LAYER(0, 0, 1);
	LAYER(1, 0, 0);
	k007420_draw(s_vs.spr, s_vs.k342.flipscreen, s_vs.k342.wrap_y, 0x3ff, sprite_cb, s_gfx[1], s_opaque, s_gfx_count[1],
	             512, bm, pitch, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1);
	LAYER(0, 1, 0);
	LAYER(1, 1, 0);
#undef LAYER
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
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
static void map_bank(void)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x60 + p] = s_rom + rbx_wrap((0x10000u + (uint32_t)t.mainbank * 0x2000 + (uint32_t)p * 256), s_rom_size);
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x2000)
		return s_k342.ram[a];
	if (a < 0x2200)
		return t.spr[a - 0x2000];
	if (a < 0x2400)
		return s_k342.scroll_ram[a - 0x2200];
	if (a < 0x2480)
		return t.pal[a - 0x2400];
	switch (a) {
	case 0x2e00: return (uint8_t)rbx_port_read(s_port_system);
	case 0x2e01: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x2e02: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x2e03: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0x2e40: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	}
	if (a < 0x4000)
		return 0;
	if (a < 0x6000)
		return t.ram[a - 0x4000];
	if (a < 0x8000)
		return s_rom[rbx_wrap((0x10000u + (uint32_t)t.mainbank * 0x2000 + (a - 0x6000)), s_rom_size)];
	return s_rom[a];
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000)
		s_k342.ram[a] = v;
	else if (a < 0x2200)
		t.spr[a - 0x2000] = v;
	else if (a < 0x2400)
		s_k342.scroll_ram[a - 0x2200] = v;
	else if (a < 0x2480)
		t.pal[a - 0x2400] = v;
	else if (a >= 0x2600 && a < 0x2608)
		k007342_vreg_w(&s_k342, a - 0x2600, v);
	else if (a >= 0x4000 && a < 0x6000)
		t.ram[a - 0x4000] = v;
	else
		switch (a) {
		case 0x2e80: q_put(Q_LATCH, v); break;   // (the latch's data_pending: the 6809's IRQ)
		case 0x2f00: t.vreg = v; break;
		case 0x2f40:   // (bankswitch_w)
			t.mainbank = (v & 0x70) >> 4;
			map_bank();
			break;
		}
}

// ---- the sound board (sound_map) ----
static uint8_t snd_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x3000)
		return (uint8_t)vlm5030_bsy(&s_vlm);
	if (a == 0x5000) {   // (the latch read: data_pending cleared)
		t.snd_irq = 0;
		m6809_set_irq(&s_snd, 0);
		return t.soundlatch;
	}
	if (a == 0x6000 || a == 0x6001)
		return rbx_ym_read(0, a & 1);
	if (a >= 0x7000 && a < 0x7800)
		return t.sram[a - 0x7000];
	if (a >= 0x8000)
		return s_srom[rbx_wrap(a, s_srom_size)];
	return 0;
}
static void snd_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x2000)
		vlm5030_data_w(&s_vlm, v);
	else if (a == 0x4000) {   // (speech_w)
		vlm5030_rst(&s_vlm, (v >> 1) & 1);
		vlm5030_st(&s_vlm, v & 1);
	} else if (a == 0x6000 || a == 0x6001)
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0x7000 && a < 0x7800)
		t.sram[a - 0x7000] = v;
}

static void sound_run(int n)   // (a line's samples: the 6809, the YM2151 0.30 stereo, the VLM5030 0.60)
{
	if (n <= 0)
		return;
	t.s_frac += (uint64_t)SND_CLOCK * (uint64_t)n;
	const int64_t cyc = (int64_t)(t.s_frac / SND_RATE);
	t.s_frac -= (uint64_t)cyc * SND_RATE;
	t.snd_end += cyc;
	m6809_run_until(&s_snd, t.snd_end);
	if (n > 64)
		n = 64;
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++) {
		t.vlm_frac += vlm5030_rate(&s_vlm);
		while (t.vlm_frac >= SND_RATE) {
			t.vlm_frac -= SND_RATE;
			int16_t s;
			vlm5030_render(&s_vlm, &s, 1);
			t.vlm_out = s;
		}
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t v = t.vlm_out * 60 / 100;
		s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + v;
		s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + v;
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
		case Q_LATCH:
			t.soundlatch = (uint8_t)e->v;
			t.snd_irq = 1;
			m6809_set_irq(&s_snd, 1);
			break;
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
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void rr_exit(void);
static void rr_reset(void);

static int decode_gfx(int i, const uint8_t *rom, uint32_t size)   // (gfx_8x8x4_packed_msb)
{
	s_gfx_count[i] = size / 32;
	s_gfx[i] = (uint8_t *)malloc((size_t)s_gfx_count[i] * 64);
	if (!s_gfx[i])
		return -1;
	for (uint32_t n = 0; n < s_gfx_count[i]; n++)
		for (int b = 0; b < 32; b++) {
			s_gfx[i][n * 64 + b * 2] = rom[n * 32 + b] >> 4;
			s_gfx[i][n * 64 + b * 2 + 1] = rom[n * 32 + b] & 15;
		}
	return 0;
}

static int rr_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t g0, g1, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_vlmrom = rbx_region("vlm", &s_vlmrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &g0), *sprites = rbx_region("sprites", &g1);
	if (!s_rom || !s_srom || !s_vlmrom || !s_prom || ps < 0x300 || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	memset(&s_k342, 0, sizeof(s_k342));
	if (decode_gfx(0, tiles, g0) || decode_gfx(1, sprites, g1) || !(s_opaque = (uint8_t *)malloc(s_gfx_count[1]))) {
		rbx_fail("%s: out of memory", g->set);
		rr_exit();
		return -1;
	}
	for (uint32_t n = 0; n < s_gfx_count[1]; n++) {
		uint8_t o = 0;
		for (int p = 0; p < 64; p++)
			o |= s_gfx[1][n * 64 + p];
		s_opaque[n] = o != 0;
	}
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 77;   // (0.30)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		rr_exit();
		return -1;
	}
	vlm5030_init(&s_vlm, YM_CLOCK, s_vlmrom, s_vlmrom_size);
	s_port_system = rbx_port_find("SYSTEM");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		rr_exit();
		return -1;
	}
	rr_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void rr_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	for (int i = 0; i < 2; i++) {
		free(s_gfx[i]);
		s_gfx[i] = NULL;
	}
	free(s_opaque);
	s_opaque = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void rr_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(s_k342.ram, 0, sizeof(s_k342.ram));
	memset(s_k342.scroll_ram, 0, sizeof(s_k342.scroll_ram));
	k007342_reset(&s_k342);
	rbx_ym_reset(0);
	vlm5030_reset(&s_vlm);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + (uint32_t)p * 256;
	for (int p = 0x00; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_k342.ram + p * 256;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x40 + p] = s_cpu.wpage[0x40 + p] = t.ram + p * 256;
	map_bank();
	hd6309_reset(&s_cpu);
	m6809_init(&s_snd);
	s_snd.read = snd_rd;
	s_snd.write = snd_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = (uint32_t)p * 256 + 256 <= s_srom_size ? s_srom + p * 256 : NULL;
	for (int p = 0x70; p < 0x78; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.sram + (p - 0x70) * 256;
	m6809_reset(&s_snd);
}

static void rr_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the screen's update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (s_k342.int_enabled)
				hd6309_hold_irq(&s_cpu);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		hd6309_run_until(&s_cpu, t.cpu_end);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int rr_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "rr_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "rr_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "rr", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "rr_k342", &s_k342, (uint32_t)offsetof(k007342_t, state_end));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= vlm5030_state(&s_vlm, ctx, load, "rr_vlm");
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "rr_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "rr_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "rr_outr", s_out_r, sizeof(s_out_r));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		map_bank();
	}
	return rc ? -1 : 0;
}

static int rr_save(void *ctx) { return rr_state(ctx, 0); }
static int rr_load(void *ctx) { return rr_state(ctx, 1); }
static int rr_supported(const char *machine) { return !strcmp(machine, "rockrage"); }

const rbx_driver rbx_drv_rockrage = { "Konami Rock'n Rage", rr_init, rr_exit, rr_reset, rr_frame, rr_save, rr_load,
                                      rr_supported, rr_render, NULL };

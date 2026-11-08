/*  Konami Battlantis / Rack 'em Up. Ported from MAME src/mame/konami/battlnts.cpp (BSD-3-Clause, copyright-holders:
    Manuel Abadia) and the devices it uses (k007342.cpp, k007420.cpp). An HD6309E at 3 MHz, a K007342 (tilemaps) and a
    K007420 (sprites), the direct palette; the sound: a Z80 at 4 MHz and two YM3812s, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../video/k007342.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define Z80_CLOCK 4000000u     // (24 MHz / 6)
#define YM_CLOCK 3000000u      // (24 MHz / 8)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 128

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_gfx[2], *s_opaque;   // (tiles, sprites; the sprites' elements with a visible pixel)
static uint32_t s_gfx_count[2];
static hd6309_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k007342_t s_k342;
static int s_port_p[2], s_port_dsw[3];

static struct {
	uint8_t pal[0x100];      // 2400-24ff
	uint8_t spr[0x200];      // 2000-21ff (the K007420's)
	uint8_t mainbank;
	uint16_t spritebank;
	uint8_t zram[0x800], soundlatch, z80_irq;
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x100], spr[0x200]; uint16_t spritebank; k007342_t k342; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	s_vs.spritebank = t.spritebank;
	memcpy(&s_vs.k342, &s_k342, sizeof(k007342_t));
}

static void tile_cb(int layer, uint32_t bank, uint32_t *code, uint32_t *color, int *flags)
{
	(void)layer;
	(void)bank;
	(void)flags;
	*code |= ((*color & 0x0f) << 9) | ((*color & 0x40) << 2);
	*color = 0;
}
static void sprite_cb(uint32_t *code, uint32_t *color)
{
	*code |= ((*color & 0xc0) << 2) | s_vs.spritebank;
	*code = (*code << 2) | ((*color & 0x30) >> 4);
	*color = 0;
}

static void bt_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++) {   // (xBGR_555, the high byte first)
		const uint16_t v = (uint16_t)(s_vs.pal[2 * i] << 8 | s_vs.pal[2 * i + 1]);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int x1 = VIS_X0 + VIS_W - 1, y1 = VIS_Y0 + VIS_H - 1;
	k007342_draw(&s_vs.k342, 0, 0, 1, 0, s_gfx[0], s_gfx_count[0], 0, tile_cb, bm, NULL, rbx_video.pitch, VIS_X0, VIS_Y0,
	             VIS_X0, VIS_Y0, x1, y1);
	k007420_draw(s_vs.spr, s_vs.k342.flipscreen, s_vs.k342.wrap_y, 0x3ff, sprite_cb, s_gfx[1], s_opaque, s_gfx_count[1],
	             4 * 16, bm, rbx_video.pitch, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1);
	k007342_draw(&s_vs.k342, 0, 1, 1, 0, s_gfx[0], s_gfx_count[0], 0, tile_cb, bm, NULL, rbx_video.pitch, VIS_X0, VIS_Y0,
	             VIS_X0, VIS_Y0, x1, y1);
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
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = s_rom + (0x10000u + (uint32_t)t.mainbank * 0x4000 + (uint32_t)p * 256) % s_rom_size;
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
	if (a < 0x2500)
		return t.pal[a - 0x2400];
	switch (a) {
	case 0x2e00: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x2e01: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x2e02: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x2e03: return (uint8_t)rbx_port_read(s_port_dsw[2]);
	case 0x2e04: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	}
	if (a < 0x4000)
		return 0;
	if (a < 0x8000)
		return s_rom[(0x10000u + (uint32_t)t.mainbank * 0x4000 + (a - 0x4000)) % s_rom_size];
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
	else if (a < 0x2500)
		t.pal[a - 0x2400] = v;
	else if (a >= 0x2600 && a < 0x2608)
		k007342_vreg_w(&s_k342, a - 0x2600, v);
	else
		switch (a) {
		case 0x2e08:   // (bankswitch_w)
			t.mainbank = (v & 0xc0) >> 6;
			map_bank();
			break;
		case 0x2e0c: t.spritebank = (uint16_t)(1024 * (v & 1)); break;
		case 0x2e14: q_put(Q_LATCH, v); break;
		case 0x2e18: q_put(Q_IRQ, 0); break;   // (sh_irqtrigger_w: HOLD_LINE)
		}
}

// ---- the sound board (sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a == 0xa000 || a == 0xa001)
		return rbx_ym_read(0, a & 1);
	if (a == 0xc000 || a == 0xc001)
		return rbx_ym_read(1, a & 1);
	if (a == 0xe000)
		return t.soundlatch;
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a == 0xa000 || a == 0xa001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(1, a & 1, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, the two YM3812s 1.0; mono)
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
	int32_t y0[96], y1[96];
	memset(y0, 0, sizeof(y0));
	memset(y1, 0, sizeof(y1));
	t.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int n0 = rbx_ym_advance(0, yc, y0, 96), n1 = rbx_ym_advance(1, yc, y1, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++)
		s_out[s_out_pos++] = (n0 > 0 ? y0[i * n0 / n] : 0) + (n1 > 0 ? y1[i * n1 / n] : 0);
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
static void bt_exit(void);
static void bt_reset(void);

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

static int bt_init(const rbx_game *g)
{
	uint32_t g0, g1;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *tiles = rbx_region("tiles", &g0), *sprites = rbx_region("sprites", &g1);
	if (!s_rom || !s_zrom || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	memset(&s_k342, 0, sizeof(s_k342));
	if (decode_gfx(0, tiles, g0) || decode_gfx(1, sprites, g1) || !(s_opaque = (uint8_t *)malloc(s_gfx_count[1]))) {
		rbx_fail("%s: out of memory", g->set);
		bt_exit();
		return -1;
	}
	for (uint32_t n = 0; n < s_gfx_count[1]; n++) {
		uint8_t o = 0;
		for (int p = 0; p < 64; p++)
			o |= s_gfx[1][n * 64 + p];
		s_opaque[n] = o != 0;
	}
	for (int i = 0; i < 2; i++) {
		rbx_ym_config yc;
		memset(&yc, 0, sizeof(yc));
		yc.type = RBX_YM3812;
		yc.clock = YM_CLOCK;
		yc.gain[0] = 256;
		if (rbx_ym_init(i, &yc)) {
			rbx_fail("%s: YM3812", g->set);
			bt_exit();
			return -1;
		}
	}
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		bt_exit();
		return -1;
	}
	bt_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void bt_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	rbx_ym_exit(1);
	for (int i = 0; i < 2; i++) {
		free(s_gfx[i]);
		s_gfx[i] = NULL;
	}
	free(s_opaque);
	s_opaque = NULL;
}

static void bt_reset(void)
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
	rbx_ym_reset(1);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + (uint32_t)p * 256;
	for (int p = 0x00; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_k342.ram + p * 256;
	map_bank();
	hd6309_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0x80 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void bt_frame(void)
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
		hd6309_run_until(&s_cpu, t.cpu_end += cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int bt_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "bt_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "bt", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "bt_k342", &s_k342, (uint32_t)offsetof(k007342_t, state_end));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_ym_state(1, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "bt_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "bt_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "bt_z80", &z, sizeof(z));
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
		map_bank();
	}
	return rc ? -1 : 0;
}

static int bt_save(void *ctx) { return bt_state(ctx, 0); }
static int bt_load(void *ctx) { return bt_state(ctx, 1); }
static int bt_supported(const char *machine) { return !strcmp(machine, "battlnts"); }

const rbx_driver rbx_drv_battlnts = { "Konami Battlantis", bt_init, bt_exit, bt_reset, bt_frame, bt_save, bt_load,
                                      bt_supported, bt_render, NULL };

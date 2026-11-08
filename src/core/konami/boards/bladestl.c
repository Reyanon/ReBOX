/*  Konami Blades of Steel. Ported from MAME src/mame/konami/bladestl.cpp (BSD-3-Clause, copyright-holders: Manuel
    Abadia) and the devices it uses (k007342.cpp, k007420.cpp, k051733.cpp). An HD6309E at 3 MHz, a K007342 (tilemaps)
    and a K007420 (sprites), a K051733 (protection, the NMI timer), the trackballs; the sound: a 6809E at 1.5 MHz, a
    YM2203 and a uPD7759, a frame behind on the worker (the SSG's RC filters not done).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../video/k007342.h"
#include "../machine/k051733.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define SND_CLOCK 1500000u     // (24 MHz / 16)
#define YM_CLOCK 3000000u      // (24 MHz / 8)
#define UPD_CLOCK 640000u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x120            // (32 + 16 * 16 pens over 48 colours)

static uint8_t *s_rom, *s_srom, *s_upd_rom, *s_prom;
static uint32_t s_rom_size, s_srom_size, s_upd_size;
static uint8_t *s_gfx[2], *s_opaque;   // (tiles, sprites; the sprites' elements with a visible pixel)
static uint32_t s_gfx_count[2];
static hd6309_t s_cpu;
static m6809_t s_snd;
static upd7759_t s_upd;
static k007342_t s_k342;
static k051733_t s_k733;
static int s_port_coin, s_port_p[2], s_port_dsw[2], s_port_track[4];

static struct {
	uint8_t pal[0x60];       // 2400-245f
	uint8_t spr[0x200];      // 2000-21ff (the K007420's)
	uint8_t ram[0x2000];     // 4000-5fff
	uint8_t mainbank, last_track[4];
	uint16_t spritebank;
	uint8_t sram[0x800], soundlatch, snd_irq, upd_bank;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, s_frac, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x60], spr[0x200]; uint16_t spritebank; k007342_t k342; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.spr, t.spr, sizeof(s_vs.spr));
	s_vs.spritebank = t.spritebank;
	memcpy(&s_vs.k342, &s_k342, sizeof(k007342_t));
}

static void tile_cb(int layer, uint32_t bank, uint32_t *code, uint32_t *color, int *flags)
{
	(void)bank;
	(void)flags;
	*code |= ((*color & 0x0f) << 8) | ((*color & 0x40) << 6);
	*color = (uint32_t)layer;
}
static void sprite_cb(uint32_t *code, uint32_t *color)
{
	*code |= ((*color & 0xc0) << 2) + s_vs.spritebank;
	*code = (*code << 2) | ((*color & 0x30) >> 4);
	*color = *color & 0x0f;
}

static void bl_render(void)   // (screen_update)
{
	uint32_t col[48];
	for (int i = 0; i < 48; i++) {   // (xBGR_555, the high byte first)
		const uint16_t v = (uint16_t)(s_vs.pal[2 * i] << 8 | s_vs.pal[2 * i + 1]);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		col[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	for (int i = 0; i < 0x20; i++)   // (palette: the tiles direct, the sprites through the lookup PROM)
		rbx_video.palette[i] = col[i];
	for (int i = 0x20; i < NPENS; i++)
		rbx_video.palette[i] = col[(s_prom[i - 0x20] & 0x0f) | 0x20];
	uint16_t *bm = rbx_video.bitmap;
	const int x1 = VIS_X0 + VIS_W - 1, y1 = VIS_Y0 + VIS_H - 1, pitch = rbx_video.pitch;
#define LAYER(l, cat, op) k007342_draw(&s_vs.k342, l, cat, op, 0, s_gfx[0], s_gfx_count[0], 0, tile_cb, bm, NULL, pitch, \
	                                   VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1)
	LAYER(1, 0, 1);
	k007420_draw(s_vs.spr, s_vs.k342.flipscreen, s_vs.k342.wrap_y, 0x3ff, sprite_cb, s_gfx[1], s_opaque, s_gfx_count[1],
	             32, bm, pitch, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1);
	LAYER(1, 1, 1);
	LAYER(0, 0, 0);
	LAYER(0, 1, 0);
#undef LAYER
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH };
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
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x60 + p] = s_rom + ((uint32_t)t.mainbank * 0x2000 + (uint32_t)p * 256) % s_rom_size;
}

static uint8_t trackball_r(int i)
{
	const int curr = (int)(rbx_port_read(s_port_track[i]) & 0xff);
	const int delta = (curr - t.last_track[i]) & 0xff;
	t.last_track[i] = (uint8_t)curr;
	return (uint8_t)((delta & 0x80) | (curr >> 1));
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
	if (a < 0x2460)
		return t.pal[a - 0x2400];
	switch (a) {
	case 0x2e00: return (uint8_t)rbx_port_read(s_port_coin);
	case 0x2e01: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x2e02: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x2e03: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	case 0x2e40: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	}
	if (a >= 0x2f00 && a < 0x2f04)
		return trackball_r(a - 0x2f00);
	if (a >= 0x2f80 && a < 0x2fa0)
		return k051733_read(&s_k733, a - 0x2f80);
	if (a < 0x4000)
		return 0;
	if (a < 0x6000)
		return t.ram[a - 0x4000];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[a % s_rom_size];
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
	else if (a < 0x2460)
		t.pal[a - 0x2400] = v;
	else if (a >= 0x2600 && a < 0x2608)
		k007342_vreg_w(&s_k342, a - 0x2600, v);
	else if (a >= 0x2f80 && a < 0x2fa0)
		k051733_write(&s_k733, a - 0x2f80, v);
	else if (a >= 0x4000 && a < 0x6000)
		t.ram[a - 0x4000] = v;
	else if (a == 0x2e80)
		q_put(Q_LATCH, v);   // (the latch's data_pending: the 6809's IRQ)
	else if (a == 0x2f40) {   // (bankswitch_w)
		t.mainbank = (v & 0x60) >> 5;
		t.spritebank = (uint16_t)((v & 0x80) << 3);
		map_bank();
	}
}

// ---- the sound board (sound_map) ----
static void upd_bank(void)   // (set_rom_bank: 128 KB pages)
{
	const uint32_t off = (uint32_t)t.upd_bank * 0x20000;
	s_upd.rom = off < s_upd_size ? s_upd_rom + off : s_upd_rom;
	s_upd.rom_size = off < s_upd_size ? (s_upd_size - off < 0x20000 ? s_upd_size - off : 0x20000) : 0;
}
static void ym_port_a(uint8_t data) { upd7759_port_w(&s_upd, data); }
static void ym_port_b(uint8_t data)   // (port_b_w: the uPD7759's bank; bits 0-2 the SSG's filters)
{
	t.upd_bank = (data & 0x38) >> 3;
	upd_bank();
}

static uint8_t snd_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800)
		return t.sram[a];
	if (a == 0x1000 || a == 0x1001)
		return rbx_ym_read(0, a & 1);
	if (a == 0x4000)
		return upd7759_busy_r(&s_upd) ? 1 : 0;
	if (a == 0x6000)
		return t.soundlatch;
	if (a >= 0x8000)
		return s_srom[a % s_srom_size];
	return 0;
}
static void snd_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800)
		t.sram[a] = v;
	else if (a == 0x1000 || a == 0x1001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0x3000) {   // (speech_ctrl_w)
		upd7759_reset_w(&s_upd, v & 1);
		upd7759_start_w(&s_upd, (v >> 1) & 1);
	} else if (a == 0x5000) {   // (the latch's acknowledge_w)
		t.snd_irq = 0;
		m6809_set_irq(&s_snd, 0);
	}
}

static void sound_run(int n)   // (a line's samples: the 6809, the YM2203 0.45, the uPD7759 0.60; mono)
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
	int32_t ym[96], up[64];
	memset(ym, 0, sizeof(ym));
	t.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance(0, yc, ym, 96);
	upd7759_render(&s_upd, up, n);
	for (int i = 0; i < n && s_out_pos < 2048; i++)
		s_out[s_out_pos++] = (yn > 0 ? ym[i * yn / n] : 0) + up[i] * 60 / 100;
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
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void bl_exit(void);
static void bl_reset(void);

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

static int bl_init(const rbx_game *g)
{
	uint32_t g0, g1, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_upd_rom = rbx_region("upd", &s_upd_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &g0), *sprites = rbx_region("sprites", &g1);
	if (!s_rom || !s_srom || !s_upd_rom || !s_prom || ps < 0x100 || !tiles || !sprites) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	memset(&s_k342, 0, sizeof(s_k342));
	if (decode_gfx(0, tiles, g0) || decode_gfx(1, sprites, g1) || !(s_opaque = (uint8_t *)malloc(s_gfx_count[1]))) {
		rbx_fail("%s: out of memory", g->set);
		bl_exit();
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
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 115;   // (0.45)
	yc.port_a_write = ym_port_a;
	yc.port_b_write = ym_port_b;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2203", g->set);
		bl_exit();
		return -1;
	}
	upd7759_init(&s_upd, s_upd_rom, s_upd_size, UPD_CLOCK, SND_RATE);
	s_port_coin = rbx_port_find("COINSW");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	static const char *const tn[4] = { "TRACKBALL.0", "TRACKBALL.1", "TRACKBALL.2", "TRACKBALL.3" };
	for (int i = 0; i < 4; i++)
		s_port_track[i] = rbx_port_find(tn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		bl_exit();
		return -1;
	}
	bl_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void bl_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	for (int i = 0; i < 2; i++) {
		free(s_gfx[i]);
		s_gfx[i] = NULL;
	}
	free(s_opaque);
	s_opaque = NULL;
}

static void bl_reset(void)
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
	k051733_reset(&s_k733);
	rbx_ym_reset(0);
	upd_bank();
	upd7759_reset(&s_upd);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
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
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.sram + p * 256;
	m6809_reset(&s_snd);
}

static void bl_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the screen's update, vblank_w, the K051733's nmiclock_w)
			capture();
			rbx_video_frame_done();
			if (s_k342.int_enabled)
				hd6309_hold_firq(&s_cpu);
			if (k051733_nmiclock(&s_k733, 1)) {
				hd6309_set_nmi(&s_cpu, 1);
				hd6309_set_nmi(&s_cpu, 0);
			}
		} else if (vpos == VIS_Y0)
			k051733_nmiclock(&s_k733, 0);
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

static int bl_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "bl_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "bl_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "bl", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "bl_k342", &s_k342, (uint32_t)offsetof(k007342_t, state_end));
	rc |= rbx_state_io(ctx, "bl_k733", &s_k733, sizeof(s_k733));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= upd7759_state(&s_upd, ctx, load, "bl_upd");
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "bl_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "bl_out", s_out, sizeof(s_out));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		map_bank();
		upd_bank();
	}
	return rc ? -1 : 0;
}

static int bl_save(void *ctx) { return bl_state(ctx, 0); }
static int bl_load(void *ctx) { return bl_state(ctx, 1); }
static int bl_supported(const char *machine) { return !strcmp(machine, "bladestl"); }

const rbx_driver rbx_drv_bladestl = { "Konami Blades of Steel", bl_init, bl_exit, bl_reset, bl_frame, bl_save, bl_load,
                                      bl_supported, bl_render, NULL };

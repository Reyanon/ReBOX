/*  Banpresto / Bandai Ultraman (Konami hardware). Ported from MAME src/mame/konami/ultraman.cpp (BSD-3-Clause,
    copyright-holders: Manuel Abadia) and the devices it uses (k051316.cpp: Nicola Salmoria; k051960.cpp: Fabio Priuli,
    Acho A. Tang, R. Belmont). A 68000 at 12 MHz, three K051316 zoom layers, the K051960 sprites (konamigx_v.c's pre-GX
    mode: MAME's bitmap_ind16 and priority bitmap); the sound: a Z80 at 4 MHz (its NMI from the input merger), a
    YM2151, an OKI M6295, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../video/k051316.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 12000000u   // (24 MHz / 2)
#define Z80_CLOCK 4000000u     // (24 MHz / 6)
#define YM_CLOCK 4000000u
#define OKI_CLOCK 1056000u
#define OKI_RATE 8000          // (pin 7 high: clock / 132)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 16, 304, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 16
#define VIS_W 288
#define VIS_Y0 16
#define VIS_H 224

static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k051316_t s_k316[3];
static okim6295_t s_oki;
static int s_port_sys, s_port_p1, s_port_p2, s_port_dsw1, s_port_dsw2;

static struct {
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint8_t irq_enabled, bank[3];
	uint8_t zram[0x4000];
	uint8_t soundlatch, nmi_in0, nmi_in1;
	uint64_t oki_frac;
	int32_t oki_out;
	int64_t cycles_done;
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow, bank[3]; k051316_t k316[3]; } s_vs;

static void zoom_cb(int chip, int *code, int *color)   // (zoom_callback<Which>, the bank as captured)
{
	*code |= ((*color & 0x07) << 8) | (s_vs.bank[chip] << 11);
	*color = (1024 * chip) / 16 + ((*color & 0xf8) >> 3);
}

static void sprite_cb(int *code, int *color, int *priority, int *shadow)
{
	(void)code;
	*priority = (*color & 0x80) ? 0 : 0xaaaa;   // (GFX_PMASK_1)
	*color = 3072 / 16 + ((*color & 0x7e) >> 1);
	*shadow = 0;
}

static void capture(void)
{
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	memcpy(s_vs.bank, t.bank, sizeof(s_vs.bank));
	for (int i = 0; i < 3; i++)
		memcpy(&s_vs.k316[i], &s_k316[i], offsetof(k051316_t, state_end));
}

static void draw(void)   // (screen_update)
{
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	memset(p, 0, (size_t)(VIS_W * VIS_H));
	static const int order[3] = { 2, 1, 0 };
	for (int i = 0; i < 3; i++) {
		k051316_t *k = &s_vs.k316[order[i]];
		k->rom = s_k316[order[i]].rom;
		k->rom_size = s_k316[order[i]].rom_size;
		k->gfx = s_k316[order[i]].gfx;
		k->ntiles = s_k316[order[i]].ntiles;
		k->pix = s_k316[order[i]].pix;   // (the drawing's cache, the render thread's own)
		k->key = s_k316[order[i]].key;
		k->chip = order[i];
		k->bpp = s_k316[order[i]].bpp;
		k->dx = 8;
		k->dy = 0;
		k->cb = zoom_cb;
		k051316_zoom_draw(k, f, p, VIS_W, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1,
		                  i ? 0 : K051316_DRAW_OPAQUE, (uint8_t)(i == 2));
	}
	gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
}

// ---- the K051960 / K051937 ----
static inline int64_t now_cycles(void) { return t.cycles_done + m68k_cycles_run(); }

static uint8_t k051960_fetchromdata(int offset)
{
	int addr = t.k51960_romoffset + (t.k51960_rombank[0] << 8) + ((t.k51960_rombank[1] & 0x03) << 16);
	int code = (addr & 0x3ffe0) >> 5, color = ((t.k51960_rombank[1] & 0xfc) >> 2) + ((t.k51960_rombank[2] & 0x03) << 6);
	const int off1 = addr & 0x1f;
	int pri = 0, shadow = 0;
	sprite_cb(&code, &color, &pri, &shadow);
	addr = (code << 7) | (off1 << 2) | offset;
	return s_sprrom[(uint32_t)addr & (s_sprrom_size - 1)];
}
static uint8_t k051960_r(int offset)
{
	if (t.k51960_control & 0x20) {
		t.k51960_romoffset = (uint8_t)((offset & 0x3fc) >> 2);
		return k051960_fetchromdata(offset & 3);
	}
	return t.k51960_ram[offset & 0x3ff];
}
static uint8_t k051937_r(int offset)
{
	offset &= 7;
	if ((t.k51960_control & 0x20) && (offset & 4))
		return k051960_fetchromdata(offset & 3);
	if (offset == 0)
		return now_cycles() < t.k51960_busy_until;
	return 0;
}
static void k051937_w(int offset, uint8_t v)
{
	offset &= 7;
	if (offset == 0)
		t.k51960_control = v;
	else if (offset == 1)
		t.k51960_shadow = v & 0x07;
	else if (offset < 5)
		t.k51960_rombank[offset - 2] = v;
}
static void k051960_vblank(void)
{
	if (t.k51960_control & 0x10)
		return;
	memcpy(t.k51960_buffer, t.k51960_ram, sizeof(t.k51960_buffer));
	int active = 0;
	for (int i = 0; i < 0x400; i += 8)
		active += t.k51960_buffer[i] >> 7;
	const uint32_t ticks = (uint32_t)(active * 32 + (128 - active) * 18);
	t.k51960_busy_until = now_cycles() + (int64_t)((uint64_t)ticks * 4 * MAIN_CLOCK / 24000000u);
}

// ---- the sound board a frame behind on the worker ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_NMI_SET };
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

static void nmi_inputs(int in0, int in1)   // (INPUT_MERGER_ALL_HIGH to the Z80's NMI: its rising edge)
{
	const int old = t.nmi_in0 && t.nmi_in1;
	t.nmi_in0 = (uint8_t)(in0 != 0);
	t.nmi_in1 = (uint8_t)(in1 != 0);
	if (!old && t.nmi_in0 && t.nmi_in1)
		z80_gen_nmi(&s_z);
}

// ---- the main CPU's bus ----
static void gfxctrl_w(uint8_t data)
{
	for (int i = 0; i < 3; i++) {
		s_k316[i].wrap = (uint8_t)((data >> (2 * i)) & 1);
		t.bank[i] = (uint8_t)((data >> (2 * i + 1)) & 1);
	}
}
static void irq_enable_w(uint8_t data)
{
	t.irq_enabled = (uint8_t)(data >> 7);
	if (!t.irq_enabled)
		m68k_bus_set_line(4, 0);
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	switch (a) {
	case 0x1c0000: return (uint16_t)rbx_port_read(s_port_sys);
	case 0x1c0002: return (uint16_t)rbx_port_read(s_port_p1);
	case 0x1c0004: return (uint16_t)rbx_port_read(s_port_p2);
	case 0x1c0006: return (uint16_t)rbx_port_read(s_port_dsw1);
	case 0x1c0008: return (uint16_t)rbx_port_read(s_port_dsw2);
	}
	if (a >= 0x204000 && a < 0x207000)   // (umask 00ff)
		return k051316_read(&s_k316[(a - 0x204000) >> 12], (int)((a & 0xfff) >> 1));
	if (a >= 0x304000 && a < 0x304010)
		return k051937_r((int)((a - 0x304000) >> 1));
	if (a >= 0x304800 && a < 0x305000)
		return k051960_r((int)((a - 0x304800) >> 1));
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (!(mask & 0x00ff))
		return;   // (every register: the low byte)
	const uint8_t b = (uint8_t)v;
	if (a == 0x1c0010)
		irq_enable_w(b);
	else if (a == 0x1c0018)
		gfxctrl_w(b);
	else if (a == 0x1c0020)
		q_put(Q_LATCH, b);
	else if (a == 0x1c0028)
		q_put(Q_NMI_SET, 0);
	else if (a >= 0x204000 && a < 0x207000)
		k051316_write(&s_k316[(a - 0x204000) >> 12], (int)((a & 0xfff) >> 1), b);
	else if (a >= 0x207f80 && a < 0x207fe0)
		k051316_ctrl_w(&s_k316[(a - 0x207f80) >> 5], (int)((a & 0x1f) >> 1), b);
	else if (a >= 0x304000 && a < 0x304010)
		k051937_w((int)((a - 0x304000) >> 1), b);
	else if (a >= 0x304800 && a < 0x305000)
		t.k51960_ram[((a - 0x304800) >> 1) & 0x3ff] = b;
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v) { main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

// ---- the sound board (sound_map, sound_io_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0xc000)
		return t.zram[a - 0x8000];
	if (a == 0xc000)
		return t.soundlatch;
	if (a == 0xe000)
		return oki_read(&s_oki);
	if (a == 0xf000 || a == 0xf001)
		return rbx_ym_read(0, a & 1);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0xc000)
		t.zram[a - 0x8000] = v;
	else if (a == 0xd000)   // (sound_nmi_enable_w)
		nmi_inputs(t.nmi_in0, v & 1);
	else if (a == 0xe000)
		oki_write(&s_oki, v);
	else if (a == 0xf000 || a == 0xf001)
		rbx_ym_write(0, a & 1, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	(void)v;
	if (p == 0x00)   // (in_clear<0>)
		nmi_inputs(0, t.nmi_in1);
}

static void sound_run(int n)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = 0;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
	if (n > 64)
		n = 64;
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		t.oki_frac += OKI_RATE;
		while (t.oki_frac >= SND_RATE) {
			t.oki_frac -= SND_RATE;
			t.oki_out = oki_sample(&s_oki);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		if (s_out_pos < 2048) {
			s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + t.oki_out;
			s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + t.oki_out;
			s_out_pos++;
		}
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
		case Q_NMI_SET: nmi_inputs(1, t.nmi_in1); break;
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
static void um_exit(void);
static void um_reset(void);

static int um_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t oki_size, zs[3];
	static const char *const zn[3] = { "k051316_1", "k051316_2", "k051316_3" };
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_sprrom = rbx_region("k051960", &s_sprrom_size);
	const uint8_t *oki = rbx_region("oki", &oki_size);
	const uint8_t *zr[3];
	for (int i = 0; i < 3; i++)
		zr[i] = rbx_region(zn[i], &zs[i]);
	if (!s_rom || !s_zrom || !s_sprrom || !oki || !zr[0] || !zr[1] || !zr[2]) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	gx.pal_entries = 8192;
	gx.pal555 = 1;
	gx.pal555rgb = 1;   // (xRGB_555)
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	gx.pre_dim = 1;
	for (int i = 0; i < 8; i++)
		gx.pre_contrast[i] = 1.0f;
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_051960, 0, 0);
	gx_video_set_area(VIS_X0, VIS_W);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	for (int i = 0; i < 3; i++)
		if (k051316_init(&s_k316[i], i, zr[i], zs[i], 8, 0, zoom_cb)) {
			rbx_fail("%s: out of memory", g->set);
			um_exit();
			return -1;
		}
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x080000, 0x08ffff, gx.workram, 0x10000, 1, 1);
	m68k_bus_map(0x180000, 0x183fff, gx.palram, 0x4000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	oki_init(&s_oki, OKI_CLOCK, 1, oki, oki_size, 128);   // (0.50)
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		um_exit();
		return -1;
	}
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		um_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	um_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void um_exit(void)
{
	rbx_aux_set_job(NULL);
	gx_video_exit();
	for (int i = 0; i < 3; i++)
		k051316_exit(&s_k316[i]);
	rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void um_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(gx.palram, 0, sizeof(gx.palram));
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 3; i++)
		k051316_reset(&s_k316[i]);
	gfxctrl_w(0);   // (machine_reset)
	for (int i = 0; i < 5; i++)
		k051937_w(i, 0);
	oki_reset(&s_oki);
	rbx_ym_reset(0);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void um_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the sprite DMA, vblank_irq)
			gx_video_capture();
			k051960_vblank();
			if (t.irq_enabled)
				m68k_bus_set_line(4, 1);
		}
		t.cyc_frac += (uint64_t)MAIN_CLOCK * HTOTAL;
		int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		while (cyc > 0) {
			const int used = m68k_execute((int)cyc);
			t.cycles_done += used;
			cyc -= used;
		}
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
	gx.frame = t.frame;
}

static int um_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	rbx_aux_sync();
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "um_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "um_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= rbx_state_io(ctx, "um", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 3; i++) {
		static const char *const tags[3] = { "um_316a", "um_316b", "um_316c" };
		rc |= k051316_state(&s_k316[i], ctx, load, tags[i]);
	}
	rc |= rbx_state_io(ctx, "um_oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "um_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "um_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "um_outr", s_out_r, sizeof(s_out_r));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "um_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_set_context(cpu);
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

static int um_save(void *ctx) { return um_state(ctx, 0); }
static int um_load(void *ctx) { return um_state(ctx, 1); }
static int um_supported(const char *machine) { return !strcmp(machine, "ultraman"); }

const rbx_driver rbx_drv_ultraman = { "Konami Ultraman", um_init, um_exit, um_reset, um_frame, um_save, um_load,
                                      um_supported, gx_render, NULL };

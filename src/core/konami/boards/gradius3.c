/*  Konami Gradius III. Ported from MAME src/mame/konami/gradius3.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria)
    and the devices it uses (k052109.cpp, k051960.cpp: Fabio Priuli, Acho A. Tang, R. Belmont; k007232.cpp: Nicola
    Salmoria, Hiromitsu Shioya). Two 68000s at 10 MHz (cpu/m68k_multi.c: CPU B held in reset by CPU A), the K052109's
    tiles from RAM (set_char_ram: decoded again while the drawing is idle), the K051960 sprites with the priority bitmap
    (konamigx_v.c's pre-GX mode); the sound: a Z80 at 3.58 MHz, a YM2151 (its left output), a K007232, a frame behind on
    the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k_multi.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000u
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u    // (set_raw(24 MHz / 4, 384, 0, 320, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 320
#define VIS_Y0 16
#define VIS_H 224
#define SLICES 2              // (set_maximum_quantum(6000 Hz): about 2.6 lines)

static uint8_t *s_rom, *s_subrom, *s_zrom;
static uint32_t s_rom_size, s_subrom_size, s_zrom_size;
static uint8_t *s_sprrom;
static uint32_t s_sprrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k052109_t s_k109;
static k007232_t s_k232;
static uint16_t (*s_gfxram_hp)[0x10000];   // 180000 / 280000: the K052109's characters (host order words)
#define s_gfxram (*s_gfxram_hp)
static uint8_t s_dirty[0x1000];      // (the characters written since the last decoding)
static int s_any_dirty;
static int s_port_sys, s_port_p1, s_port_p2, s_port_dsw[3];

static struct {
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint8_t priority, irqAen, irqBmask, sub_run, sub_reset;
	uint8_t zram[0x800];
	uint8_t soundlatch, z80_irq, bank;
	uint64_t k232_frac;
	int32_t k232_a, k232_b;
	int64_t cycles_done;
	uint64_t cpu_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow, priority; k052109_t k109; } s_vs;
static k052109_view_t s_view;

static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	static const int layer_colorbase[3] = { 0 / 16, 512 / 16, 768 / 16 };
	(void)bank;
	(void)flags;
	(void)priority;
	const int c = *color;   // ((color & 0x02) the flip y, the 052109's own)
	*code |= ((c & 0x01) << 8) | ((c & 0x1c) << 7);
	*color = layer_colorbase[layer] + ((c & 0xe0) >> 5);
}

static void sprite_cb(int *code, int *color, int *priority, int *shadow)
{
	enum { PM1 = 0xaaaa, PM2 = 0xcccc, PM4 = 0xf0f0 };   // (GFX_PMASK_1, 2, 4)
	static const int primask[2][4] = { { PM1 | PM4, PM1, PM1 | PM4, PM1 | PM2 | PM4 }, { PM2 | PM4, PM4, 0, PM1 | PM2 | PM4 } };
	(void)shadow;
	const int c = *color;
	*priority = primask[s_vs.priority & 1][(c & 0x60) >> 5];
	*code |= (c & 0x01) << 13;
	*color = 256 / 16 + ((c & 0x1e) >> 1);
}

static void capture(void)
{
	if (s_any_dirty) {   // (the drawing is idle here: the changed characters decoded again)
		for (uint32_t i = 0; i < 0x1000; i++)
			if (s_dirty[i]) {
				s_dirty[i] = 0;
				k052109_charram_decode(&s_k109, s_gfxram, i);
			}
		s_any_dirty = 0;
	}
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	s_vs.priority = t.priority;
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
}

static void draw(void)   // (screen_update)
{
	s_vs.k109.rom = NULL;
	s_vs.k109.rom_size = 0;
	s_vs.k109.gfx = s_k109.gfx;
	s_vs.k109.ntiles = s_k109.ntiles;
	s_vs.k109.colors = s_k109.colors;
	s_vs.k109.cb = tile_cb;
	// (the kludge that enforces the char banks)
	k052109_write(&s_vs.k109, 0x1d80, 0x10);
	k052109_write(&s_vs.k109, 0x1f00, 0x32);
	k052109_prepare(&s_vs.k109, &s_view, VIS_X0 + (VIS_X0 + VIS_W - 1) + 1, VIS_Y0 + (VIS_Y0 + VIS_H - 1) + 1);
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	const int x1 = VIS_X0 + VIS_W - 1, y1 = VIS_Y0 + VIS_H - 1;
	memset(p, 0, (size_t)(VIS_W * VIS_H));
	static const int order[2][3] = { { 1, 2, 0 }, { 0, 1, 2 } };
	static const uint8_t prio[2][3] = { { 2, 4, 1 }, { 1, 2, 4 } };
	const int pr = s_vs.priority & 1;
	for (int i = 0; i < 3; i++)
		k052109_draw(&s_vs.k109, &s_view, order[pr][i], i ? 0 : K052109_DRAW_OPAQUE, prio[pr][i], f, p, VIS_W, VIS_X0,
		             VIS_Y0, VIS_X0, VIS_Y0, x1, y1);
	gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
}

// ---- the K051960 / K051937 (CPU B's) ----
static inline int64_t now_cycles(void) { return t.cycles_done + m68k_cycles_run(); }

static uint8_t k051960_fetchromdata(int offset)
{
	int addr = t.k51960_romoffset + (t.k51960_rombank[0] << 8) + ((t.k51960_rombank[1] & 0x03) << 16);
	int code = (addr & 0x3ffe0) >> 5, color = ((t.k51960_rombank[1] & 0xfc) >> 2) + ((t.k51960_rombank[2] & 0x03) << 6);
	const int off1 = addr & 0x1f;
	int pri = 0, shadow = 0;
	const uint8_t keep = s_vs.priority;
	s_vs.priority = t.priority;
	sprite_cb(&code, &color, &pri, &shadow);
	s_vs.priority = keep;
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
	t.k51960_busy_until = now_cycles() + (int64_t)((uint64_t)ticks * 4 * CPU_CLOCK / 24000000u);
}

// ---- the sound board a frame behind on the worker ----
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

// ---- the CPUs' buses ----
static void gfxram_w(uint32_t o, uint16_t v, uint16_t mask)   // (o: the word)
{
	const uint16_t old = s_gfxram[o];
	s_gfxram[o] = (uint16_t)((old & ~mask) | (v & mask));
	if (old != s_gfxram[o]) {
		s_dirty[o >> 4] = 1;
		s_any_dirty = 1;
	}
}
static void k052109_hw_w(int o, uint16_t v, uint16_t mask)   // (k052109_halfword_w)
{
	if (mask & 0x00ff)
		k052109_write(&s_k109, o, (uint8_t)v);
	else
		k052109_write(&s_k109, o, (uint8_t)(v >> 8));
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	switch (a) {
	case 0x0c8000: return (uint16_t)rbx_port_read(s_port_sys);
	case 0x0c8002: return (uint16_t)rbx_port_read(s_port_p1);
	case 0x0c8004: return (uint16_t)rbx_port_read(s_port_p2);
	case 0x0c8006: return (uint16_t)rbx_port_read(s_port_dsw[2]);
	case 0x0d0000: return (uint16_t)rbx_port_read(s_port_dsw[0]);
	case 0x0d0002: return (uint16_t)rbx_port_read(s_port_dsw[1]);
	}
	if (a >= 0x14c000 && a < 0x154000)
		return k052109_read(&s_k109, (int)((a - 0x14c000) >> 1));
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
	if (a == 0x0c0000) {   // (cpuA_ctrl_w)
		if (mask & 0xff00) {
			const int d = v >> 8;
			t.priority = (uint8_t)((d >> 2) & 1);
			const uint8_t run = (uint8_t)((d >> 3) & 1);   // (bit 3: CPU B out of reset)
			if (run && !t.sub_run)
				t.sub_reset = 1;
			t.sub_run = run;
			t.irqAen = (uint8_t)((d >> 5) & 1);
		}
	} else if (a == 0x0d8000) {   // (cpuB_irqtrigger_w)
		if (t.irqBmask & 4)
			m68k_multi_hold_irq(1, 4);
	} else if (a == 0x0e8000 && (mask & 0xff00))
		q_put(Q_LATCH, (uint8_t)(v >> 8));
	else if (a == 0x0f0000)
		q_put(Q_IRQ, 0);
	else if (a >= 0x14c000 && a < 0x154000)
		k052109_hw_w((int)((a - 0x14c000) >> 1), v, mask);
	else if (a >= 0x180000 && a < 0x1a0000)
		gfxram_w((a - 0x180000) >> 1, v, mask);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v) { main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

static uint16_t sub_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x24c000 && a < 0x254000)
		return k052109_read(&s_k109, (int)((a - 0x24c000) >> 1));
	if (a >= 0x2c0000 && a < 0x2c0010)   // (umask 00ff)
		return k051937_r((int)((a - 0x2c0000) >> 1));
	if (a >= 0x2c0800 && a < 0x2c1000)
		return k051960_r((int)((a - 0x2c0800) >> 1));
	return 0;
}
static uint8_t sub_r8(uint32_t a)
{
	if (!(a & 1) && ((a >= 0x2c0000 && a < 0x2c0010) || (a >= 0x2c0800 && a < 0x2c1000)))
		return 0;
	const uint16_t w = sub_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void sub_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a == 0x140000) {   // (cpuB_irqenable_w)
		if (mask & 0xff00)
			t.irqBmask = (uint8_t)((v >> 8) & 7);
	} else if (a >= 0x24c000 && a < 0x254000)
		k052109_hw_w((int)((a - 0x24c000) >> 1), v, mask);
	else if (a >= 0x280000 && a < 0x2a0000)
		gfxram_w((a - 0x280000) >> 1, v, mask);
	else if (a >= 0x2c0000 && a < 0x2c0010) {
		if (mask & 0x00ff)
			k051937_w((int)((a - 0x2c0000) >> 1), (uint8_t)v);
	} else if (a >= 0x2c0800 && a < 0x2c1000) {
		if (mask & 0x00ff)
			t.k51960_ram[((a - 0x2c0800) >> 1) & 0x3ff] = (uint8_t)v;
	}
}
static void sub_w16(uint32_t a, uint16_t v) { sub_w16m(a, v, 0xffff); }
static void sub_w8(uint32_t a, uint8_t v) { sub_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00); }

// ---- the sound board (sound_map) ----
static void k007232_port(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232, 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232, 1, 0, (data & 0x0f) * 0x11);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xf000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a >= 0xf800)
		return t.zram[a - 0xf800];
	if (a == 0xf010)
		return t.soundlatch;
	if (a >= 0xf020 && a < 0xf02e)
		return k007232_read(&s_k232, a - 0xf020);
	if (a == 0xf030 || a == 0xf031)
		return rbx_ym_read(0, a & 1);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf800)
		t.zram[a - 0xf800] = v;
	else if (a == 0xf000) {   // (sound_bank_w)
		t.bank = v;
		k007232_set_bank(&s_k232, v & 3, (v >> 2) & 3);
	} else if (a >= 0xf020 && a < 0xf02e)
		k007232_write(&s_k232, a - 0xf020, v);
	else if (a == 0xf030 || a == 0xf031)
		rbx_ym_write(0, a & 1, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)
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
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		t.k232_frac += CHIP_CLOCK / 128;
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			k007232_sample(&s_k232, &t.k232_a, &t.k232_b);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t m = (k >= 0 ? yl[k] : 0) + (t.k232_a + t.k232_b) / 5;   // (YM: the left output only; K007232 0.20)
		if (s_out_pos < 2048)
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
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void g3_exit(void);
static void g3_reset(void);

static int g3_init(const rbx_game *g)
{
	if (!s_gfxram_hp && !(s_gfxram_hp = calloc(1, sizeof(*s_gfxram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t pcm_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_subrom = rbx_region("sub", &s_subrom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_sprrom = rbx_region("k051960", &s_sprrom_size);
	const uint8_t *pcm = rbx_region("k007232", &pcm_size);
	if (!s_rom || !s_subrom || !s_zrom || !s_sprrom || !pcm) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	gx.pal_entries = 2048;
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
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_GRADIUS3, 0, 0);
	gx_video_set_area(VIS_X0, VIS_W);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	memset(s_gfxram, 0, sizeof(s_gfxram));
	memset(s_dirty, 0, sizeof(s_dirty));
	s_any_dirty = 0;
	if (k052109_init_charram(&s_k109, s_gfxram, sizeof(s_gfxram), 2048, tile_cb) || m68k_multi_init(2)) {
		rbx_fail("%s: out of memory", g->set);
		g3_exit();
		return -1;
	}
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_bus_swap16(s_subrom, s_subrom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	for (int c = 0; c < 2; c++) {
		m68k_bus_reset_map();
		if (!c) {
			m68k_bus_map(0x000000, 0x03ffff, s_rom, s_rom_size, 1, 0);
			m68k_bus_map(0x040000, 0x043fff, gx.workram, 0x4000, 1, 1);
			m68k_bus_map(0x080000, 0x080fff, gx.palram, 0x1000, 1, 1);
			m68k_bus_map(0x100000, 0x103fff, gx.workram + 0x8000, 0x4000, 1, 1);
			m68k_bus_map(0x180000, 0x19ffff, s_gfxram, sizeof(s_gfxram), 1, 0);
		} else {
			m68k_bus_map(0x000000, 0x0fffff, s_subrom, s_subrom_size, 1, 0);
			m68k_bus_map(0x100000, 0x103fff, gx.workram + 0x4000, 0x4000, 1, 1);
			m68k_bus_map(0x200000, 0x203fff, gx.workram + 0x8000, 0x4000, 1, 1);
			m68k_bus_map(0x280000, 0x29ffff, s_gfxram, sizeof(s_gfxram), 1, 0);
			// (gfxrom_r: rom[2o + 1] << 8 | rom[2o], the ROM's bytes as little endian words, the host's order)
			m68k_bus_map(0x400000, 0x5fffff, s_sprrom, s_sprrom_size, 1, 0);
		}
		m68k_bus.read8 = c ? sub_r8 : main_r8;
		m68k_bus.read16 = c ? sub_r16 : main_r16;
		m68k_bus.write8 = c ? sub_w8 : main_w8;
		m68k_bus.write16 = c ? sub_w16 : main_w16;
		m68k_multi_capture(c);
	}
	m68k_multi_finish();
	k007232_init(&s_k232, pcm, pcm_size, k007232_port);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = 256;   // (add_route(0, 1.0), add_route(1, 0.0))
	yc.gain[1] = 0;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		g3_exit();
		return -1;
	}
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_dsw[2] = rbx_port_find("DSW3");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		g3_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	g3_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void g3_exit(void)
{
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	m68k_multi_exit();
	rbx_ym_exit(0);
	free(s_gfxram_hp);
	s_gfxram_hp = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void g3_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(gx.palram, 0, sizeof(gx.palram));
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(s_gfxram, 0, sizeof(s_gfxram));
	for (uint32_t i = 0; i < 0x1000; i++)
		k052109_charram_decode(&s_k109, s_gfxram, i);
	memset(s_dirty, 0, sizeof(s_dirty));
	s_any_dirty = 0;
	k052109_reset(&s_k109);
	for (int i = 0; i < 5; i++)
		k051937_w(i, 0);
	k007232_reset(&s_k232);
	rbx_ym_reset(0);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0xf0 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0xf8 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
	m68k_multi_reset();   // (CPU B held in reset: not run until CPU A lets it go)
	for (int c = 0; c < 2; c++)
		m68k_multi_set_asserted(c, 0);
}

static void g3_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the sprite DMA, cpuA_interrupt, sub_scanline 240)
			gx_video_capture();
			k051960_vblank();
			k052109_vblank(&s_k109, 1);
			if (t.irqAen)
				m68k_multi_hold_irq(0, 2);
			if (t.irqBmask & 1)
				m68k_multi_hold_irq(1, 1);
		}
		if (vpos == VIS_Y0) {
			k052109_vblank(&s_k109, 0);
			if (t.irqBmask & 2)   // (sprite end DMA)
				m68k_multi_hold_irq(1, 2);
		}
		t.cpu_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int cyc = (int)(t.cpu_frac / PIX_CLOCK);
		t.cpu_frac -= (uint64_t)cyc * PIX_CLOCK;
		for (int k = 0; k < SLICES; k++) {
			const int n = cyc * (k + 1) / SLICES - cyc * k / SLICES;
			for (int c = 0; c < 2; c++) {
				if (c && t.sub_reset) {   // (CPU B out of reset: from its vectors)
					t.sub_reset = 0;
					m68k_multi_reset_cpu(1);
				}
				if (c && !t.sub_run)
					continue;
				m68k_multi_switch(c);
				int left = n;
				while (left > 0) {
					const int used = m68k_execute(left);
					if (c)
						t.cycles_done += used;
					left -= used;
				}
			}
			m68k_multi_switch(0);
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

static int g3_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	uint32_t asserted[2] = { m68k_multi_asserted(0), m68k_multi_asserted(1) };
	rc |= m68k_multi_state(ctx, load);
	rc |= rbx_state_io(ctx, "g3_irq", asserted, sizeof(asserted));
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= rbx_state_io(ctx, "g3_gfxram", s_gfxram, sizeof(s_gfxram));
	rc |= k052109_state(&s_k109, ctx, load, "g3_k109");
	rc |= rbx_state_io(ctx, "g3", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= k007232_state(&s_k232, ctx, load, "g3_k232");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "g3_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "g3_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "g3_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_multi_set_asserted(0, asserted[0]);
		m68k_multi_set_asserted(1, asserted[1]);
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		for (uint32_t i = 0; i < 0x1000; i++)   // (device_post_load: mark_all_dirty)
			k052109_charram_decode(&s_k109, s_gfxram, i);
		memset(s_dirty, 0, sizeof(s_dirty));
		s_any_dirty = 0;
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

static int g3_save(void *ctx) { return g3_state(ctx, 0); }
static int g3_load(void *ctx) { return g3_state(ctx, 1); }
static int g3_supported(const char *machine) { return !strcmp(machine, "gradius3"); }

const rbx_driver rbx_drv_gradius3 = { "Konami Gradius III", g3_init, g3_exit, g3_reset, g3_frame, g3_save, g3_load,
                                      g3_supported, gx_render, NULL };

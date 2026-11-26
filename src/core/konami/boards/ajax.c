/*  Konami Ajax / Typhoon. Ported from MAME src/mame/konami/ajax.cpp (BSD-3-Clause, copyright-holders: Manuel Abadia)
    and the devices it uses (k052109.cpp, k051960.cpp, k051316.cpp, k007232.cpp). The Konami CPU at 3 MHz (cpu/konami)
    with the K051960 sprites, an HD6309E at 3 MHz with the K052109 tilemaps and the K051316 (7 bpp), 8 KB of RAM
    shared; the sprites through konamigx_v.c's pre-GX mode (MAME's bitmap_ind16 and priority bitmap); the sound: a Z80
    at 3.58 MHz, a YM2151 and two K007232s, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/konami/konami.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../video/k051316.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (the Konami CPU: 24 MHz / 2, its internal divider of 4; the HD6309E 24 MHz / 8)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u    // (the YM2151 and the K007232s)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0+8, 320, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 8
#define VIS_W 312
#define VIS_Y0 16
#define VIS_H 224
#define NPAL 2048
#define PMASK_1 0xaa           // (GFX_PMASK_1, 2, 4)
#define PMASK_2 0xcc
#define PMASK_4 0xf0

static uint8_t *s_rom, *s_srom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_srom_size, s_zrom_size, s_sprrom_size;
static konami_t s_cpu;
static hd6309_t s_sub;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k052109_t s_k109;
static k051316_t s_k316;
static k007232_t s_k232[2];
static int s_port_p[2], s_port_dsw[3], s_port_system;

static struct {
	uint8_t pal8[0x1000];    // 1000-1fff (xBGR_555, the high byte first)
	uint8_t shared[0x2000];  // 2000-3fff of both CPUs
	uint8_t ram[0x2000];     // 4000-5fff
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint8_t mainbank, subbank, priority, firq_enable;
	uint32_t rand;   // (machine().rand() of the 0000-003f reads: a 32-bit LCG here)
	uint8_t zram[0x800], soundlatch, z80_irq;
	uint64_t k232_frac;
	int32_t k232_out[2][2];   // (the K007232s' last samples, held between theirs)
	int64_t cpu_end, sub_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot (screen_update) ----
static struct { uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow, priority; k052109_t k109; k051316_t k316; } s_vs;
static k052109_view_t s_view;

static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	(void)flags;
	(void)priority;
	static const int base[3] = { 1024 / 16, 0 / 16, 512 / 16 };
	const int c = *color;
	*code |= ((c & 0x0f) << 8) | (bank << 12);
	*color = base[layer] + ((c & 0xf0) >> 4);
}

static void sprite_cb(int *code, int *color, int *priority, int *shadow)
{
	(void)code;
	(void)shadow;
	const int c = *color;   // (over zoom: bit 4 0 has priority; over B: bit 5 0; over A: bit 6 1; never over F)
	*priority = ((c & 0x10) ? PMASK_4 : 0) | ((~c & 0x40) ? PMASK_2 : 0) | ((c & 0x20) ? PMASK_1 : 0);
	*color = 256 / 16 + (c & 0x0f);
}

static void zoom_cb(int chip, int *code, int *color)
{
	(void)chip;
	*code |= (*color & 0x07) << 8;
	*color = 768 / 128 + ((*color & 0x08) >> 3);
}

static void capture(void)
{
	for (int i = 0; i < gx.pal_entries; i++)
		((uint16_t *)gx.palram)[i] = (uint16_t)(t.pal8[2 * i] << 8 | t.pal8[2 * i + 1]);
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	s_vs.priority = t.priority;
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
	memcpy(&s_vs.k316, &s_k316, offsetof(k051316_t, state_end));
}

static void draw(void)   // (screen_update)
{
	s_vs.k109.rom = s_k109.rom;
	s_vs.k109.rom_size = s_k109.rom_size;
	s_vs.k109.gfx = s_k109.gfx;
	s_vs.k109.ntiles = s_k109.ntiles;
	s_vs.k109.colors = s_k109.colors;
	s_vs.k109.cb = tile_cb;
	k052109_prepare(&s_vs.k109, &s_view, VIS_X0 + (VIS_X0 + VIS_W - 1) + 1, VIS_Y0 + (VIS_Y0 + VIS_H - 1) + 1);
	k051316_t *z = &s_vs.k316;
	z->rom = s_k316.rom;
	z->rom_size = s_k316.rom_size;
	z->gfx = s_k316.gfx;
	z->ntiles = s_k316.ntiles;
	z->pix = s_k316.pix;   // (the drawing's cache, the render thread's own)
	z->key = s_k316.key;
	z->cb = s_k316.cb;
	z->bpp = s_k316.bpp;
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	const int x1 = VIS_X0 + VIS_W - 1, y1 = VIS_Y0 + VIS_H - 1;
	memset(p, 0, (size_t)(VIS_W * VIS_H));
	gx_pre_fill_pen(2 * NPAL);   // (the palette's black pen)
#define LAYER(l, pr) k052109_draw(&s_vs.k109, &s_view, l, 0, pr, f, p, VIS_W, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1)
#define ZOOM() k051316_zoom_draw(z, f, p, VIS_W, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1, 0, 4)
	LAYER(2, 1);
	if (s_vs.priority) {
		ZOOM();
		LAYER(1, 2);
	} else {
		LAYER(1, 2);
		ZOOM();
	}
	gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 0);
	LAYER(0, 0);
#undef ZOOM
#undef LAYER
}

// ---- the K051960 / K051937 (as konami8b.c's) ----
static inline int64_t now_cycles(void) { return s_cpu.cycles; }

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
	if (offset == 0) {   // (the IRQ cleared when its enable goes off)
		if (~v & t.k51960_control & 1)
			konami_set_irq(&s_cpu, 0);
		t.k51960_control = v;
	} else if (offset == 1)
		t.k51960_shadow = v & 0x07;
	else if (offset < 5)
		t.k51960_rombank[offset - 2] = v;
}
static void k051960_vblank(void)
{
	if (t.k51960_control & 1)   // (the vblank IRQ to the Konami CPU)
		konami_set_irq(&s_cpu, 1);
	if (t.k51960_control & 0x10)
		return;
	memcpy(t.k51960_buffer, t.k51960_ram, sizeof(t.k51960_buffer));
	int active = 0;
	for (int i = 0; i < 0x400; i += 8)
		active += t.k51960_buffer[i] >> 7;
	const uint32_t ticks = (uint32_t)(active * 32 + (128 - active) * 18);
	t.k51960_busy_until = now_cycles() + (int64_t)((uint64_t)ticks * 4 * CPU_CLOCK / 24000000u);
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
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
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static void map_mainbank(void)
{
	const uint32_t e = t.mainbank;   // (entries 0-3 the ROM's first 32 KB, 4-11 its second half)
	const uint32_t base = rbx_wrap((e < 4 ? e * 0x2000u : 0x10000u + (e - 4) * 0x2000u), s_rom_size);
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x60 + p] = s_rom + base + (uint32_t)p * 256;
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a <= 0x01c0) {   // (ls138_f10_r)
		switch ((a & 0x01c0) >> 6) {
		case 0x04: return port(s_port_p[1]);
		case 0x06:
			if (a & 0x02)
				return port(s_port_dsw[a & 1]);
			return (a & 1) ? port(s_port_p[0]) : port(s_port_system);
		case 0x07: return port(s_port_dsw[2]);
		case 0x00:   // (machine().rand(): the game waits on these reads for a non-zero value)
			t.rand = t.rand * 1103515245u + 12345u;
			return (uint8_t)(t.rand >> 16);
		default: return 0;
		}
	}
	if (a >= 0x0800 && a < 0x0808)
		return k051937_r(a - 0x0800);
	if (a >= 0x0c00 && a < 0x1000)
		return k051960_r(a - 0x0c00);
	if (a >= 0x1000 && a < 0x2000)
		return t.pal8[a - 0x1000];
	if (a >= 0x2000 && a < 0x4000)
		return t.shared[a - 0x2000];
	if (a >= 0x4000 && a < 0x6000)
		return t.ram[a - 0x4000];
	if (a >= 0x6000 && a < 0x8000)
		return s_cpu.rpage[a >> 8][a & 0xff];
	if (a >= 0x8000)
		return s_rom[a];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a <= 0x01c0) {   // (ls138_f10_w)
		switch ((a & 0x01c0) >> 6) {
		case 0x00:
			if (!a && t.firq_enable)   // (NSFIRQ; the other offsets the watchdog)
				hd6309_hold_firq(&s_sub);
			break;
		case 0x01: q_put(Q_IRQ, 0); break;   // (the Z80's IRQ: HOLD_LINE)
		case 0x02: q_put(Q_LATCH, v); break;
		case 0x03:   // (main_bankswitch_w)
			t.priority = (v >> 3) & 1;
			t.mainbank = (uint8_t)((!(v & 0x80) ? 4 : 0) + (v & 0x07));
			map_mainbank();
			break;
		}
		return;
	}
	if (a >= 0x0800 && a < 0x0808)
		k051937_w(a - 0x0800, v);
	else if (a >= 0x0c00 && a < 0x1000)
		t.k51960_ram[a - 0x0c00] = v;
	else if (a >= 0x1000 && a < 0x2000)
		t.pal8[a - 0x1000] = v;
	else if (a >= 0x2000 && a < 0x4000)
		t.shared[a - 0x2000] = v;
	else if (a >= 0x4000 && a < 0x6000)
		t.ram[a - 0x4000] = v;
}

// ---- the sub CPU's bus (sub_map) ----
static void map_subbank(void)
{
	const uint32_t base = rbx_wrap(((uint32_t)t.subbank * 0x2000u), s_srom_size);
	for (int p = 0; p < 0x20; p++)
		s_sub.rpage[0x80 + p] = s_srom + base + (uint32_t)p * 256;
}

static uint8_t sub_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800)
		return k051316_read(&s_k316, a);
	if (a >= 0x1000 && a < 0x1800)
		return k051316_rom_r(&s_k316, a - 0x1000);
	if (a >= 0x2000 && a < 0x4000)
		return t.shared[a - 0x2000];
	if (a >= 0x4000 && a < 0x8000)
		return k052109_read(&s_k109, a - 0x4000);
	if (a >= 0x8000 && a < 0xa000)
		return s_srom[rbx_wrap(((uint32_t)t.subbank * 0x2000u + (a - 0x8000)), s_srom_size)];
	if (a >= 0xa000)
		return s_srom[rbx_wrap((0x12000u + (a - 0xa000)), s_srom_size)];
	return 0;
}

static void sub_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800)
		k051316_write(&s_k316, a, v);
	else if (a >= 0x0800 && a < 0x0810)
		k051316_ctrl_w(&s_k316, a - 0x0800, v);
	else if (a == 0x1800) {   // (sub_bankswitch_w)
		k052109_set_rmrd_line(&s_k109, (v >> 6) & 1);
		s_k316.wrap = (v >> 5) & 1;
		t.firq_enable = (v >> 4) & 1;
		t.subbank = v & 0x0f;
		map_subbank();
	} else if (a >= 0x2000 && a < 0x4000)
		t.shared[a - 0x2000] = v;
	else if (a >= 0x4000 && a < 0x8000) {
		k052109_write(&s_k109, a - 0x4000, v);
		hd6309_set_irq(&s_sub, s_k109.irq_out);
	}
}

// ---- the sound board (sound_map) ----
static void k007232_port0(uint8_t data)   // (volume_callback0)
{
	k007232_set_volume(&s_k232[0], 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232[0], 1, 0, (data & 0x0f) * 0x11);
}
static void k007232_port1(uint8_t data)   // (volume_callback1)
{
	k007232_set_volume(&s_k232[1], 1, (data & 0x0f) * 0x11 / 2, (data >> 4) * 0x11 / 2);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a >= 0xa000 && a < 0xa00e)
		return k007232_read(&s_k232[0], a - 0xa000);
	if (a >= 0xb000 && a < 0xb00e)
		return k007232_read(&s_k232[1], a - 0xb000);
	if (a == 0xc000 || a == 0xc001)
		return rbx_ym_read(0, a & 1);
	if (a == 0xe000)
		return t.soundlatch;
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000)
		return;
	if (a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a == 0x9000) {   // (sound_bank_w)
		k007232_set_bank(&s_k232[0], (v >> 1) & 1, v & 1);
		k007232_set_bank(&s_k232[1], (v >> 4) & 3, (v >> 2) & 3);
	} else if (a >= 0xa000 && a < 0xa00e)
		k007232_write(&s_k232[0], a - 0xa000, v);
	else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232[1], a - 0xb000, v);
	else if (a == 0xb80c)   // (k007232_extvol_w)
		k007232_set_volume(&s_k232[1], 0, (v & 0x0f) * 0x11 / 2, (v & 0x0f) * 0x11 / 2);
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(0, a & 1, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples (a line's): the Z80, the YM2151 1.0 stereo, the K007232 #0 0.20 to both sides, the #1's outputs 0.50 left
// and right
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
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++) {
		t.k232_frac += CHIP_CLOCK / 128;   // (their rate: clock / 128)
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			for (int k = 0; k < 2; k++)
				k007232_sample(&s_k232[k], &t.k232_out[k][0], &t.k232_out[k][1]);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t c0 = (t.k232_out[0][0] + t.k232_out[0][1]) / 5;
		s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + c0 + t.k232_out[1][0] / 2;
		s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + c0 + t.k232_out[1][1] / 2;
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
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void aj_exit(void);
static void aj_reset(void);

static int aj_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t tile_size, zsize, p0size, p1size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("sub", &s_srom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("k052109", &tile_size);
	s_sprrom = rbx_region("k051960", &s_sprrom_size);
	const uint8_t *zrom = rbx_region("k051316", &zsize);
	const uint8_t *pcm0 = rbx_region("k007232_1", &p0size), *pcm1 = rbx_region("k007232_2", &p1size);
	if (!s_rom || !s_srom || !s_zrom || !tiles || !s_sprrom || !zrom || !pcm0 || !pcm1) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&t, 0, sizeof(t));
	gx.pal_entries = NPAL;
	gx.pal555 = 1;
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
	if (k052109_init(&s_k109, tiles, tile_size, NPAL, tile_cb) ||
	    k051316_init_bpp(&s_k316, 0, 7, zrom, zsize, 0, 0, zoom_cb)) {
		rbx_fail("%s: out of memory", g->set);
		aj_exit();
		return -1;
	}
	k007232_init(&s_k232[0], pcm0, p0size, k007232_port0);
	k007232_init(&s_k232[1], pcm1, p1size, k007232_port1);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		aj_exit();
		return -1;
	}
	s_port_system = rbx_port_find("SYSTEM");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		aj_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	aj_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void aj_exit(void)
{
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	k051316_exit(&s_k316);
	rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void map_pages(void)   // (the ROM and the RAM as direct pages; the windows with handlers left out)
{
	memset(s_cpu.rpage, 0, sizeof(s_cpu.rpage));
	memset(s_cpu.wpage, 0, sizeof(s_cpu.wpage));
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + (uint32_t)p * 256;
	for (int p = 0x10; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.pal8 + (p - 0x10) * 256;
	for (int p = 0x20; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.shared + (p - 0x20) * 256;
	for (int p = 0x40; p < 0x60; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x40) * 256;
	map_mainbank();
	memset(s_sub.rpage, 0, sizeof(s_sub.rpage));
	memset(s_sub.wpage, 0, sizeof(s_sub.wpage));
	for (int p = 0; p < 8; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = s_k316.ram + p * 256;
	for (int p = 0x20; p < 0x40; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = t.shared + (p - 0x20) * 256;
	for (int p = 0xa0; p < 0x100; p++)
		s_sub.rpage[p] = s_srom + 0x12000 + (uint32_t)(p - 0xa0) * 256;
	map_subbank();
}

static void aj_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	k052109_reset(&s_k109);
	k051316_reset(&s_k316);
	for (int i = 0; i < 2; i++)
		k007232_reset(&s_k232[i]);
	rbx_ym_reset(0);
	konami_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.set_lines = NULL;
	s_cpu.ctx = NULL;
	hd6309_init(&s_sub);
	s_sub.read = sub_rd;
	s_sub.write = sub_wr;
	s_sub.ctx = NULL;
	map_pages();
	konami_reset(&s_cpu);
	hd6309_reset(&s_sub);
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

static void aj_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the K051960's IRQ and sprite DMA, the K052109's IRQ)
			gx_video_capture();
			k051960_vblank();
			k052109_vblank(&s_k109, 1);
			hd6309_set_irq(&s_sub, s_k109.irq_out);
		}
		if (vpos == VIS_Y0) {
			k052109_vblank(&s_k109, 0);
			hd6309_set_irq(&s_sub, s_k109.irq_out);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		konami_run_until(&s_cpu, t.cpu_end += cyc);
		hd6309_run_until(&s_sub, t.sub_end += cyc);
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

static int aj_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "aj_cpu", &s_cpu, KONAMI_STATE_SIZE);
	rc |= rbx_state_io(ctx, "aj_sub", &s_sub, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k052109_state(&s_k109, ctx, load, "aj_k109");
	rc |= k051316_state(&s_k316, ctx, load, "aj_k316");
	rc |= rbx_state_io(ctx, "aj", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= k007232_state(&s_k232[0], ctx, load, "aj_k232a");
	rc |= k007232_state(&s_k232[1], ctx, load, "aj_k232b");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "aj_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "aj_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "aj_outr", s_out_r, sizeof(s_out_r));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "aj_z80", &z, sizeof(z));
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
		map_mainbank();
		map_subbank();
	}
	return rc ? -1 : 0;
}

static int aj_save(void *ctx) { return aj_state(ctx, 0); }
static int aj_load(void *ctx) { return aj_state(ctx, 1); }
static int aj_supported(const char *machine) { return !strcmp(machine, "ajax"); }

const rbx_driver rbx_drv_ajax = { "Konami Ajax", aj_init, aj_exit, aj_reset, aj_frame, aj_save, aj_load, aj_supported,
                                  gx_render, NULL };

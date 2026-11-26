/*  Konami Chequered Flag. Ported from MAME src/mame/konami/chqflag.cpp (BSD-3-Clause, copyright-holders: Nicola
    Salmoria, Manuel Abadia) and the devices it uses (k051960.cpp, k051316.cpp, k051733.cpp, k007232.cpp). The Konami
    CPU at 3 MHz, the K051960 sprites (its vblank IRQ and its NMI every 32 lines), two K051316s (4 bpp; 8 bpp with a layer
    mask), the accelerator and the wheel through an ADC0804; the sprites through konamigx_v.c's pre-GX mode (MAME's
    bitmap_ind16 and priority bitmap, the shadows and highlights); the sound: a Z80 at 3.58 MHz, a YM2151 and two
    K007232s, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/konami/konami.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../machine/k051733.h"
#include "../video/k051316.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 2, the CPU's internal divider of 4)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u    // (the YM2151 and the K007232s)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 320-16, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 304
#define VIS_Y0 16
#define VIS_H 224
#define NPAL 1024
#define PMASK_1 0xaa           // (GFX_PMASK_1)

static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static konami_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k051316_t s_k316[2];
static k051733_t s_k733;
static k007232_t s_k232[2];
static int s_port_dsw[2], s_port_in[5];

static struct {
	uint8_t ram[0x1000];     // 0000-0fff
	uint8_t ram1000[0x1000]; // 1000-1fff (the view's RAM)
	uint8_t pal8[0x800];     // 1800-1fff with the other view (xBGR_555, the high byte first)
	uint8_t k51960_ram[0x400], k51960_buffer[0x400];
	uint8_t k51960_control, k51960_shadow, k51960_rombank[3], k51960_romoffset;
	int64_t k51960_busy_until;
	uint8_t mainbank, view1000, readroms, last_vreg, analog_ctrl;
	float contrast, shadow_factor, highlight_factor;   // (vreg_w's palette: 1, 0 (the default), 0 (the default))
	int32_t nmi_lines;   // (lines to the K051960's next NMI: every 32 lines from vpos 0 at power on)
	uint8_t zram[0x800], soundlatch, soundlatch2, ym_irq;
	int32_t zline;       // (the Z80's IRQ: soundlatch2's data pending)
	uint64_t k232_frac;
	int32_t k232_out[2][2];
	int64_t cpu_end;     // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot (screen_update) ----
static struct {
	uint8_t k51960_buffer[0x400], k51960_control, k51960_shadow;
	float contrast, shadow_factor, highlight_factor;
	k051316_t k316[2];
} s_vs;

static void sprite_cb(int *code, int *color, int *priority, int *shadow)
{
	(void)code;
	(void)shadow;
	*priority = (*color & 0x10) ? 0 : PMASK_1;
	*color = *color & 0x0f;
}

static void zoom_cb(int chip, int *code, int *color)
{
	const int c = *color;
	if (chip == 0) {   // (zoom_callback_1: colours 256 / 16 on)
		*code |= (c & 0x03) << 8;
		*color = 256 / 16 + ((c & 0x3c) >> 2);
	} else {   // (zoom_callback_2: 512 / 256 on)
		*code |= (c & 0x0f) << 8;
		*color = 512 / 256 + ((c & 0x10) >> 4);
	}
}

static void capture(void)
{
	for (int i = 0; i < gx.pal_entries; i++)
		((uint16_t *)gx.palram)[i] = (uint16_t)(t.pal8[2 * i] << 8 | t.pal8[2 * i + 1]);
	memcpy(s_vs.k51960_buffer, t.k51960_buffer, sizeof(s_vs.k51960_buffer));
	s_vs.k51960_control = t.k51960_control;
	s_vs.k51960_shadow = t.k51960_shadow;
	s_vs.contrast = t.contrast;
	s_vs.shadow_factor = t.shadow_factor;
	s_vs.highlight_factor = t.highlight_factor;
	for (int i = 0; i < 2; i++)
		memcpy(&s_vs.k316[i], &s_k316[i], offsetof(k051316_t, state_end));
}

static void draw(void)   // (screen_update)
{
	for (int i = 0; i < 2; i++) {
		k051316_t *z = &s_vs.k316[i];
		const k051316_t *k = &s_k316[i];
		z->rom = k->rom;
		z->rom_size = k->rom_size;
		z->gfx = k->gfx;
		z->ntiles = k->ntiles;
		z->pix = k->pix;   // (the drawing's cache, the render thread's own)
		z->key = k->key;
		z->cb = k->cb;
		z->chip = k->chip;
		z->dx = k->dx;
		z->dy = k->dy;
		z->bpp = k->bpp;
		z->layermask = k->layermask;
	}
	// (the palette: pens 512-1023 at the background's brightness, the shadow group shadows or highlights by the
	// K051937's shadow mode)
	for (int i = 0; i < 8; i++)
		gx.pre_contrast[i] = (i == 2 || i == 3) ? s_vs.contrast : 1.0f;
	gx.pre_highlight = s_vs.k51960_shadow & 1;
	gx.pre_shadow_factor = s_vs.shadow_factor;
	gx.pre_highlight_factor = s_vs.highlight_factor;
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	const int x1 = VIS_X0 + VIS_W - 1, y1 = VIS_Y0 + VIS_H - 1;
	memset(p, 0, (size_t)(VIS_W * VIS_H));
#define ZOOM(n, fl, pr) k051316_zoom_draw(&s_vs.k316[n], f, p, VIS_W, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, x1, y1, fl, pr)
	ZOOM(1, K051316_DRAW_LAYER1, 0);
	ZOOM(1, K051316_DRAW_LAYER0, 1);
	gx_pre_sprites_051960(s_vs.k51960_buffer, s_vs.k51960_control, s_vs.k51960_shadow, -1, -1, sprite_cb, 1);
	ZOOM(0, 0, 0);
#undef ZOOM
}

// ---- the K051960 / K051937 (as konami8b.c's) ----
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
		return s_cpu.cycles < t.k51960_busy_until;
	return 0;
}
static void k051937_w(int offset, uint8_t v)
{
	offset &= 7;
	if (offset == 0) {   // (the interrupts cleared when their enables go off)
		if (~v & t.k51960_control & 1)
			konami_set_irq(&s_cpu, 0);
		if (~v & t.k51960_control & 4)
			konami_set_nmi(&s_cpu, 0);
		t.k51960_control = v;
	} else if (offset == 1)
		t.k51960_shadow = v & 0x07;   // (bit 0 the palette's shadow mode)
	else if (offset < 5)
		t.k51960_rombank[offset - 2] = v;
}
static void k051960_vblank(void)
{
	if (t.k51960_control & 1)
		konami_set_irq(&s_cpu, 1);
	if (t.k51960_control & 0x10)
		return;
	memcpy(t.k51960_buffer, t.k51960_ram, sizeof(t.k51960_buffer));
	int active = 0;
	for (int i = 0; i < 0x400; i += 8)
		active += t.k51960_buffer[i] >> 7;
	const uint32_t ticks = (uint32_t)(active * 32 + (128 - active) * 18);
	t.k51960_busy_until = s_cpu.cycles + (int64_t)((uint64_t)ticks * 4 * CPU_CLOCK / 24000000u);
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latches queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_LATCH2 };
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

static void map_bank(void)
{
	const uint32_t base = rbx_wrap(((uint32_t)t.mainbank * 0x4000u), s_rom_size);
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = s_rom + base + (uint32_t)p * 256;
}
static void map_view(void)   // (1000-1fff: the RAM, or the K051316 #0 and the palette)
{
	for (int p = 0x10; p < 0x20; p++) {
		uint8_t *m = t.view1000 ? (p < 0x18 ? NULL : t.pal8 + (p - 0x18) * 256) : t.ram1000 + (p - 0x10) * 256;
		s_cpu.rpage[p] = s_cpu.wpage[p] = m;
	}
	if (t.view1000 && !t.readroms)
		for (int p = 0x10; p < 0x18; p++)
			s_cpu.rpage[p] = s_k316[0].ram + (p - 0x10) * 256;
}

static void vreg_w(uint8_t v)
{
	t.readroms = (v >> 4) & 1;
	if ((v ^ t.last_vreg) & 0x88) {   // (bits 3, 7: the night and rain stages' dimming and highlights)
		static const float bg_brightness[4] = { 1.0f, 1.0f, 0.75f, 0.75f };
		static const float highlight_factor[4] = { 1.1f, 1.15f, 1.25f, 1.45f };
		const int index = ((v >> 3) & 1) | ((v >> 7) << 1);
		t.contrast = bg_brightness[index];
		t.shadow_factor = 0.75f;
		t.highlight_factor = highlight_factor[index];
	}
	t.last_vreg = v;
	map_view();
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x1000)
		return t.ram[a];
	if (a < 0x2000) {
		if (!t.view1000)
			return t.ram1000[a - 0x1000];
		if (a < 0x1800)
			return t.readroms ? k051316_rom_r(&s_k316[0], a - 0x1000) : k051316_read(&s_k316[0], a - 0x1000);
		return t.pal8[a - 0x1800];
	}
	if (a < 0x2008)
		return k051937_r(a - 0x2000);
	if (a >= 0x2400 && a < 0x2800)
		return k051960_r(a - 0x2400);
	if (a >= 0x2800 && a < 0x3000)
		return t.readroms ? k051316_rom_r(&s_k316[1], a - 0x2800) : k051316_read(&s_k316[1], a - 0x2800);
	switch (a) {
	case 0x3100: return port(s_port_dsw[0]);
	case 0x3200: return port(s_port_in[1]);
	case 0x3201: return port(s_port_in[0]);
	case 0x3203: return port(s_port_dsw[1]);
	case 0x3701: return port(s_port_in[2]);
	case 0x3702: return port(s_port_in[3 + (t.analog_ctrl & 1)]);   // (the ADC0804: the accelerator or the wheel)
	}
	if (a >= 0x3400 && a < 0x3420)
		return k051733_read(&s_k733, a - 0x3400);
	if (a >= 0x4000 && a < 0x8000)
		return s_rom[rbx_wrap(((uint32_t)t.mainbank * 0x4000u + (a - 0x4000)), s_rom_size)];
	if (a >= 0x8000)
		return s_rom[rbx_wrap((0x48000u + (a - 0x8000)), s_rom_size)];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x1000)
		t.ram[a] = v;
	else if (a < 0x2000) {
		if (!t.view1000)
			t.ram1000[a - 0x1000] = v;
		else if (a < 0x1800)
			k051316_write(&s_k316[0], a - 0x1000, v);
		else
			t.pal8[a - 0x1800] = v;
	} else if (a < 0x2008)
		k051937_w(a - 0x2000, v);
	else if (a >= 0x2400 && a < 0x2800)
		t.k51960_ram[a - 0x2400] = v;
	else if (a >= 0x2800 && a < 0x3000)
		k051316_write(&s_k316[1], a - 0x2800, v);
	else if (a >= 0x3400 && a < 0x3420)
		k051733_write(&s_k733, a - 0x3400, v);
	else if (a >= 0x3500 && a < 0x3510)
		k051316_ctrl_w(&s_k316[0], a - 0x3500, v);
	else if (a >= 0x3600 && a < 0x3610)
		k051316_ctrl_w(&s_k316[1], a - 0x3600, v);
	else
		switch (a) {
		case 0x3000: q_put(Q_LATCH, v); break;
		case 0x3001: q_put(Q_LATCH2, v); break;   // (its data pending: the Z80's IRQ)
		case 0x3002:   // (bankswitch_w)
			if ((v & 0x1f) < 0x50000 / 0x4000)
				t.mainbank = v & 0x1f;
			t.view1000 = (v >> 5) & 1;
			map_bank();
			map_view();
			break;
		case 0x3003: vreg_w(v); break;
		case 0x3700: t.analog_ctrl = v; break;   // (select_analog_ctrl_w)
		}
}

// ---- the sound board (sound_map) ----
static void k007232_port0(uint8_t d)   // (volume_callback0)
{
	k007232_set_volume(&s_k232[0], 0, (d & 0x0f) * 0x11 / 2, (d >> 4) * 0x11 / 2);
}
static void k007232_port1(uint8_t d)   // (volume_callback1)
{
	k007232_set_volume(&s_k232[1], 0, (d >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232[1], 1, 0, (d & 0x0f) * 0x11);
}
static void ym_irq(int state)   // (the YM2151's IRQ to the Z80's NMI)
{
	if (state && !t.ym_irq)
		z80_gen_nmi(&s_z);
	t.ym_irq = (uint8_t)(state != 0);
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
	if (a == 0xd000)
		return t.soundlatch;
	if (a == 0xe000) {
		t.zline = 0;   // (soundlatch2 read: its data pending cleared)
		return t.soundlatch2;
	}
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000)
		return;
	if (a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a == 0x9000) {   // (k007232_bankswitch_w)
		k007232_set_bank(&s_k232[0], (v >> 4) & 3, (v >> 6) & 3);
		k007232_set_bank(&s_k232[1], v & 3, (v >> 2) & 3);
	} else if (a >= 0xa000 && a < 0xa00e)
		k007232_write(&s_k232[0], a - 0xa000, v);
	else if (a == 0xa01c)   // (k007232_extvolume_w)
		k007232_set_volume(&s_k232[0], 1, (v & 0x0f) * 0x11 / 2, (v >> 4) * 0x11 / 2);
	else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232[1], a - 0xb000, v);
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(0, a & 1, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples (a line's): the Z80 (its IRQ a level), the YM2151 1.0 stereo, the K007232 #0's outputs 0.20 left and
// right, the #1's 0.20 to both
static void sound_run(int n)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, (int *)&t.zline, 0);   // (the latch's read drops the line)
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
		const int32_t c1 = (t.k232_out[1][0] + t.k232_out[1][1]) / 5;
		s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + t.k232_out[0][0] / 5 + c1;
		s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + t.k232_out[0][1] / 5 + c1;
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
		case Q_LATCH2:
			t.soundlatch2 = (uint8_t)e->v;
			t.zline = 1;
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
static void cq_exit(void);
static void cq_reset(void);

static int cq_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t z0size, z1size, p0size, p1size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_sprrom = rbx_region("k051960", &s_sprrom_size);
	const uint8_t *z0 = rbx_region("k051316_1", &z0size), *z1 = rbx_region("k051316_2", &z1size);
	const uint8_t *pcm0 = rbx_region("k007232_1", &p0size), *pcm1 = rbx_region("k007232_2", &p1size);
	if (!s_rom || !s_zrom || !s_sprrom || !z0 || !z1 || !pcm0 || !pcm1) {
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
	if (k051316_init_bpp(&s_k316[0], 0, 4, z0, z0size, 7, 0, zoom_cb) ||
	    k051316_init_bpp(&s_k316[1], 1, 8, z1, z1size, 7, 0, zoom_cb)) {
		rbx_fail("%s: out of memory", g->set);
		cq_exit();
		return -1;
	}
	s_k316[1].layermask = 0xc0;
	k007232_init(&s_k232[0], pcm0, p0size, k007232_port0);
	k007232_init(&s_k232[1], pcm1, p1size, k007232_port1);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		cq_exit();
		return -1;
	}
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	static const char *const in[5] = { "IN0", "IN1", "IN2", "IN3", "IN4" };
	for (int i = 0; i < 5; i++)
		s_port_in[i] = rbx_port_find(in[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		cq_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	cq_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void cq_exit(void)
{
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k051316_exit(&s_k316[0]);
	k051316_exit(&s_k316[1]);
	rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void cq_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.contrast = 1.0f;
	t.nmi_lines = LINES - (VIS_Y0 + VIS_H);   // (the frame starts at the vblank: vpos 0 that many lines on)
	for (int i = 0; i < 2; i++) {
		k051316_reset(&s_k316[i]);
		k007232_reset(&s_k232[i]);
	}
	s_k316[1].wrap = 1;   // (set_wrap(1))
	k051733_reset(&s_k733);
	rbx_ym_reset(0);
	konami_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.set_lines = NULL;
	s_cpu.ctx = NULL;
	memset(s_cpu.rpage, 0, sizeof(s_cpu.rpage));
	memset(s_cpu.wpage, 0, sizeof(s_cpu.wpage));
	for (int p = 0; p < 0x10; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + 0x48000 + (uint32_t)(p - 0x80) * 256;
	map_bank();
	vreg_w(0);   // (machine_reset)
	konami_reset(&s_cpu);
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

static void cq_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the K051960's IRQ and sprite DMA)
			gx_video_capture();
			k051960_vblank();
		}
		if (--t.nmi_lines < 0) {   // (nmi_scanline: every 32 lines)
			t.nmi_lines = 31;
			if (t.k51960_control & 4)
				konami_set_nmi(&s_cpu, 1);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		konami_run_until(&s_cpu, t.cpu_end += cyc);
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

static int cq_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "cq_cpu", &s_cpu, KONAMI_STATE_SIZE);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k051316_state(&s_k316[0], ctx, load, "cq_k316a");
	rc |= k051316_state(&s_k316[1], ctx, load, "cq_k316b");
	rc |= rbx_state_io(ctx, "cq", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "cq_k733", &s_k733, sizeof(s_k733));
	rc |= k007232_state(&s_k232[0], ctx, load, "cq_k232a");
	rc |= k007232_state(&s_k232[1], ctx, load, "cq_k232b");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "cq_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "cq_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "cq_outr", s_out_r, sizeof(s_out_r));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "cq_z80", &z, sizeof(z));
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
		map_view();
	}
	return rc ? -1 : 0;
}

static int cq_save(void *ctx) { return cq_state(ctx, 0); }
static int cq_load(void *ctx) { return cq_state(ctx, 1); }
static int cq_supported(const char *machine) { return !strcmp(machine, "chqflag"); }

const rbx_driver rbx_drv_chqflag = { "Konami Chequered Flag", cq_init, cq_exit, cq_reset, cq_frame, cq_save, cq_load,
                                     cq_supported, gx_render, NULL };

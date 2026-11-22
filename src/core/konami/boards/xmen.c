/*  Konami X-Men (2 and 4 players). Ported from MAME src/mame/konami/xmen.cpp (BSD-3-Clause, copyright-holders:
    Nicola Salmoria) and the devices it uses (k053251.cpp: Nicola Salmoria; k054321.cpp: Olivier Galibert;
    eepromser.cpp: Aaron Giles). A 68000 at 16 MHz, the K052109 tilemaps (video/k052109.c), the K053246/K053247 sprites
    and the K053251 priorities and colour bases (drawn by konamigx_v.c's pre-GX mode: MAME's bitmap_ind16 with the
    priority bitmap), an ER5911 EEPROM; the sound: a Z80 at 8 MHz with a banked ROM, a YM2151, a K054539, the K054321
    latches (a frame behind on the worker, as moo.c's).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k054539.h"
#include "../../sound/rbx_ym.h"
#include "../video/k052109.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 16000000u
#define Z80_CLOCK 8000000u
#define YM_CLOCK 4000000u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u   // (set_raw(24 MHz / 4, 384, 0+8, 320-8, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 8
#define VIS_W 304
#define VIS_Y0 16
#define VIS_H 224

static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k054539_t s_k539;
static k052109_t s_k109;
static eepser_t s_eeprom;
static int s_port_p13, s_port_p24, s_port_eep;
static int32_t s_gain_l, s_gain_r;

static struct {
	uint8_t k53251[16];
	uint8_t zram[0x2000];
	uint8_t latch[3], k54321_active, k54321_volume, z80_irq, sound_bank;
	uint8_t irq5_enable, sound_irq, mute;
	uint16_t control;
	int64_t cycles_done;
	uint64_t cyc_frac, z_frac, z_end, snd_frac, ym_frac;
	uint32_t frame;
	uint8_t state_end;
} x;

// ---- the drawing's snapshot (screen_update) ----
static struct { uint8_t k53251[16]; k052109_t k109; } s_vs;
static k052109_view_t s_view;
static int s_layer_colorbase[3], s_layerpri[3], s_sprite_colorbase;

static int k53251_palette_index(const uint8_t *r, int ci)   // (k053251_device::reset_indexes)
{
	switch (ci) {
	case 0: return 32 * (r[9] & 3);
	case 1: return 32 * (r[9] >> 2 & 3);
	case 2: return 32 * (r[9] >> 4 & 3);
	case 3: return 16 * (r[10] & 7);
	default: return 16 * (r[10] >> 3 & 7);
	}
}

// ---- the K054321 ----
static void k54321_volume(void)
{
	const double vol = pow(2.0, ((double)x.k54321_volume - 40.0) / 10.0);
	s_gain_l = (x.k54321_active & 2) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
	s_gain_r = (x.k54321_active & 1) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
}
static void k54321_main_w(int r, uint8_t v)
{
	switch (r) {
	case 0x0: x.k54321_active = v; k54321_volume(); break;
	case 0x2: x.k54321_volume = 0; k54321_volume(); break;
	case 0x3:
		if (v && x.k54321_volume < 64) {
			x.k54321_volume++;
			k54321_volume();
		}
		break;
	case 0x6: x.latch[0] = v; break;
	case 0x7: x.latch[1] = v; break;
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux), as moo.c's ----
#define Q_MAX 2048
enum { Q_ADV, Q_K54321, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static uint8_t s_pub;   // (latch 2 as of the last job)
static volatile int s_job_frame;

static void q_put(int kind, int arg, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].arg = (uint16_t)arg;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

static uint8_t k54321_main_r(int r) { return r == 0xa ? s_pub : r == 0x8 ? 0 : 0xff; }

// ---- the video callbacks (xmen_state) ----
static void tile_cb(int layer, int bank, int *code, int *color, int *flags, int *priority)
{
	(void)bank;
	(void)code;
	(void)flags;
	(void)priority;
	if (layer == 0)
		*color = s_layer_colorbase[layer] + ((*color & 0xf0) >> 4);
	else
		*color = s_layer_colorbase[layer] + ((*color & 0x7c) >> 2);
}

static void sprite_cb(int *code, int *color, int *primask)
{
	(void)code;
	const int pri = (*color & 0x00e0) >> 4;
	if (pri <= s_layerpri[2])
		*primask = 0;
	else if (pri <= s_layerpri[1])
		*primask = 0xf0;
	else if (pri <= s_layerpri[0])
		*primask = 0xf0 | 0xcc;
	else
		*primask = 0xf0 | 0xcc | 0xaa;
	*color = s_sprite_colorbase + (*color & 0x001f);
}

static void capture(void)
{
	memcpy(s_vs.k53251, x.k53251, sizeof(s_vs.k53251));
	memcpy(&s_vs.k109, &s_k109, offsetof(k052109_t, state_end));
}

static void draw(void)   // (screen_update)
{
	static const int ci[3] = { 3, 0, 2 };   // (K053251 CI3, CI0, CI2)
	const uint8_t *r = s_vs.k53251;
	const int bg_colorbase = k53251_palette_index(r, 4);
	s_sprite_colorbase = k53251_palette_index(r, 1);
	int layer[3];
	for (int i = 0; i < 3; i++) {
		s_layer_colorbase[i] = k53251_palette_index(r, ci[i]);
		layer[i] = i;
		s_layerpri[i] = r[ci[i]];   // (get_priority)
	}
	for (int a = 0; a < 2; a++)   // (konami_sortlayers3: swapped when pri[a] < pri[b])
		for (int b = a + 1; b < 3; b++)
			if (s_layerpri[a] < s_layerpri[b]) {
				int t = s_layerpri[a]; s_layerpri[a] = s_layerpri[b]; s_layerpri[b] = t;
				t = layer[a]; layer[a] = layer[b]; layer[b] = t;
			}
	s_vs.k109.rom = s_k109.rom;   // (the configuration with the snapshot's state)
	s_vs.k109.rom_size = s_k109.rom_size;
	s_vs.k109.gfx = s_k109.gfx;
	s_vs.k109.ntiles = s_k109.ntiles;
	s_vs.k109.colors = s_k109.colors;
	s_vs.k109.cb = tile_cb;
	k052109_prepare(&s_vs.k109, &s_view, VIS_X0 + (VIS_X0 + VIS_W - 1) + 1, VIS_Y0 + (VIS_Y0 + VIS_H - 1) + 1);
	gx_pre_fill_pen(16 * bg_colorbase + 1);   // (note the '+1' in the background colour)
	uint16_t *f = gx_pre_frame16();
	uint8_t *p = gx_pre_pri();
	static const uint8_t prio[3] = { 1, 2, 4 };
	for (int i = 0; i < 3; i++)
		k052109_draw(&s_vs.k109, &s_view, layer[i], 0, prio[i], f, p, VIS_W, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0,
		             VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1);
	gx_pre_sprites();
}

// ---- the main CPU's bus ----
static inline int64_t now_cycles(void) { return x.cycles_done + m68k_cycles_run(); }

static void irq_lines(void)   // (the K052109's IRQ: IRQ 3; IRQ 5 asserted at line 0 while enabled)
{
	m68k_bus_set_line(3, s_k109.irq_out);
}

static uint8_t k053246_r(int offset)
{
	if (!gx.k55673.objcha_line)
		return 0;
	const uint8_t *r = gx.k55673.kx46_regs;
	uint32_t addr = (uint32_t)r[6] << 17 | (uint32_t)r[7] << 9 | (uint32_t)r[4] << 1 | (uint32_t)((offset & 1) ^ 1);
	addr &= s_sprrom_size - 1;
	return s_sprrom[addr];
}

static void control_w(uint16_t v, uint16_t mask)
{
	if (mask & 0x00ff) {   // (the EEPROM: bit 2 data, bit 3 clock, bit 4 cs; bit 5 IRQ 5 enable)
		const int64_t t = now_cycles();
		eepser_di_write(&s_eeprom, v >> 2 & 1);
		eepser_cs_write(&s_eeprom, v >> 4 & 1, t);
		eepser_clk_write(&s_eeprom, v >> 3 & 1, t);
		x.irq5_enable = (v >> 5) & 1;
		if (!x.irq5_enable)
			m68k_bus_set_line(5, 0);
	}
	if (mask & 0xff00) {   // (bit 8 OBJCHA, bit 9 RMRD, bit 10 the sound IRQ's flip-flop, bit 11 mute)
		gx.k55673.objcha_line = (v >> 8) & 1;
		k052109_set_rmrd_line(&s_k109, (v >> 9) & 1);
		const int irq = (v >> 10) & 1;
		if (!x.sound_irq && irq)
			q_put(Q_IRQ, 0, 0);
		x.sound_irq = (uint8_t)irq;
		x.mute = !((v >> 11) & 1);
	}
	x.control = (uint16_t)((v & mask) | (x.control & ~mask));
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x101000)
		return k055673_ram_r(&gx.k55673, (int)((a - 0x100000) >> 1));
	if (a == 0x10a000)
		return (uint16_t)rbx_port_read(s_port_p24);
	if (a == 0x10a002)
		return (uint16_t)rbx_port_read(s_port_p13);
	if (a == 0x10a004) {
		rbx_port_custom(s_port_eep, 0xc0,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1) << 6);
		return (uint16_t)rbx_port_read(s_port_eep);
	}
	if (a == 0x10a00c)
		return (uint16_t)(k053246_r(0) << 8 | k053246_r(1));
	if (a >= 0x108040 && a < 0x108060)
		return k54321_main_r((int)((a - 0x108040) >> 1));
	if (a >= 0x18c000 && a < 0x198000)   // (the K052109: umask 00ff)
		return k052109_read(&s_k109, (int)((a - 0x18c000) >> 1));
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
	if (a >= 0x100000 && a < 0x101000)
		k055673_ram_w(&gx.k55673, (int)((a - 0x100000) >> 1), v, mask);
	else if (a == 0x108000)
		control_w(v, mask);
	else if (a >= 0x108020 && a < 0x108028) {
		if (mask & 0xff00) k055673_k053246_w(&gx.k55673, (int)(a - 0x108020), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k055673_k053246_w(&gx.k55673, (int)(a - 0x108020) + 1, (uint8_t)v);
	} else if (a >= 0x108040 && a < 0x108060) {   // (the K054321: umask 00ff)
		if (mask & 0x00ff)
			q_put(Q_K54321, (int)((a - 0x108040) >> 1), (uint8_t)v);
	} else if (a >= 0x108060 && a < 0x108080) {   // (the K053251: umask 00ff)
		if (mask & 0x00ff)
			x.k53251[((a - 0x108060) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x18c000 && a < 0x198000) {
		if (mask & 0x00ff) {
			k052109_write(&s_k109, (int)((a - 0x18c000) >> 1), (uint8_t)v);
			irq_lines();
		}
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (sound_map) ----
static void z_bank(void)
{
	const uint32_t base = (uint32_t)(x.sound_bank & 7) * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = base + (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + base + (uint32_t)p * 256 : NULL;
}
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xc000) {
		const uint32_t o = (uint32_t)(x.sound_bank & 7) * 0x4000 + (a - 0x8000);
		return o < s_zrom_size ? s_zrom[o] : 0xff;
	}
	if (a < 0xe000)
		return x.zram[a - 0xc000];
	if (a < 0xe230)
		return k054539_read(&s_k539, a - 0xe000);
	if ((a & ~0x0401) == 0xe800)   // (the YM2151, mirror 0x400)
		return rbx_ym_read(0, a & 1);
	if (a == 0xf002)
		return x.latch[0];
	if (a == 0xf003)
		return x.latch[1];
	return 0xff;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xe000)
		x.zram[a - 0xc000] = v;
	else if (a >= 0xe000 && a < 0xe230)
		k054539_write(&s_k539, a - 0xe000, v);
	else if ((a & ~0x0401) == 0xe800)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0xf000)
		x.latch[2] = v;
	else if (a == 0xf800) {
		x.sound_bank = v;
		z_bank();
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples (a line's): the Z80 for their cycles, the K054539, the YM2151 for its clocks (its samples spread over the
// line's, the nearest earlier); both chips to the speaker inverted, the YM2151 at 0.20, at the K054321's volume
static void sound_run(int n)
{
	if (n <= 0)
		return;
	x.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(x.z_frac / SND_RATE);
	x.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = x.z80_irq;
	x.z_end += cyc;
	z80_run(&s_z, (unsigned long)x.z_end, &line, 1);
	x.z80_irq = (uint8_t)line;
	int32_t l[64], r[64], yl[96], yr[96];
	if (n > 64)
		n = 64;
	memset(l, 0, sizeof(int32_t) * (size_t)n);
	memset(r, 0, sizeof(int32_t) * (size_t)n);
	k054539_render(&s_k539, l, r, n);
	x.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(x.ym_frac / SND_RATE);
	x.ym_frac -= (uint64_t)yc * SND_RATE;
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t hl = k >= 0 ? yl[k] : 0, hr = k >= 0 ? yr[k] : 0;
		if (s_out_pos < 2048) {
			s_out_l[s_out_pos] = x.mute ? 0 : (int32_t)(((int64_t)r[i] + hr) * s_gain_l >> 12);
			s_out_r[s_out_pos] = x.mute ? 0 : (int32_t)(((int64_t)l[i] + hl) * s_gain_r >> 12);
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
		case Q_K54321: k54321_main_w(e->arg, (uint8_t)e->v); break;
		case Q_IRQ: x.z80_irq = 1; break;
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
	s_pub = x.latch[2];
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void xm_exit(void);
static void xm_reset(void);

static int xm_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t pcm_size, tile_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *pcm = rbx_region("k054539", &pcm_size);
	const uint8_t *tiles = rbx_region("k052109", &tile_size);
	s_sprrom = rbx_region("k053246", &s_sprrom_size);
	if (!s_rom || !s_zrom || !pcm || !tiles || !s_sprrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&x, 0, sizeof(x));
	gx.pal_entries = 2048;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_053247, -43, -2);
	gx_video_set_area(VIS_X0, VIS_W);
	gx_video_set_y0(VIS_Y0);
	gx_video_set_height(VIS_H);
	if (k052109_init(&s_k109, tiles, tile_size, 2048, tile_cb)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x101000, 0x101fff, gx.workram + 0x10000, 0x1000, 1, 1);
	m68k_bus_map(0x104000, 0x104fff, gx.palram, 0x1000, 1, 1);
	m68k_bus_map(0x110000, 0x113fff, gx.workram, 0x4000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	k054539_init(&s_k539, pcm, pcm_size);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 51;   // (0.20)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		return -1;
	}
	eepser_init(&s_eeprom, EEPSER_ER5911, 128, 8, MAIN_CLOCK);
	{
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
	}
	eepser_nv_load(&s_eeprom, g->set);   // (the contents kept between runs)
	s_port_p13 = rbx_port_find("P1_P3");
	s_port_p24 = rbx_port_find("P2_P4");
	s_port_eep = rbx_port_find("EEPROM");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		xm_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	xm_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void xm_exit(void)
{
	eepser_nv_save(&s_eeprom);
	rbx_aux_set_job(NULL);
	gx_video_exit();
	k052109_exit(&s_k109);
	rbx_ym_exit(0);
	free(s_q_hp);
	s_q_hp = NULL;
}

static void xm_reset(void)
{
	rbx_aux_sync();
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(gx.palram, 0, sizeof(gx.palram));
	memset(gx.k55673.ram, 0, sizeof(gx.k55673.ram));
	memset(&x, 0, offsetof(__typeof__(x), state_end));
	k052109_reset(&s_k109);
	k54321_volume();
	k054539_reset(&s_k539);
	rbx_ym_reset(0);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0x80 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 0x20; p++)
		s_zmap[0xc0 + p] = x.zram + p * 256;
	z_bank();
	s_z.read_map = s_zmap;
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_pub = 0;
	s_job_frame = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	control_w(0, 0xffff);   // (machine_reset)
}

static void xm_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, the K052109's IRQ)
			gx_video_capture();
			k052109_vblank(&s_k109, 1);
			irq_lines();
		}
		if (vpos == VIS_Y0)   // (the vblank's end: the K052109 latches its scroll)
			k052109_vblank(&s_k109, 0);
		if (vpos == 0 && x.irq5_enable && (gx.k55673.kx46_regs[5] & 0x10))   // (scanline 0: the sprite DMA's end)
			m68k_bus_set_line(5, 1);
		x.cyc_frac += (uint64_t)MAIN_CLOCK * HTOTAL;
		int64_t cyc = (int64_t)(x.cyc_frac / PIX_CLOCK);
		x.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		while (cyc > 0) {
			const int used = m68k_execute((int)cyc);
			x.cycles_done += used;
			cyc -= used;
		}
		x.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(x.snd_frac / PIX_CLOCK);
		x.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, 0, (uint32_t)ns);
	}
	sound_frame();
	x.frame++;
	gx.frame = x.frame;
}

static int xm_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	rbx_aux_sync();
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "xm_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "xm_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k055673_state(&gx.k55673, ctx, load);
	rc |= k052109_state(&s_k109, ctx, load, "xm_k109");
	rc |= rbx_state_io(ctx, "xm", &x, (uint32_t)offsetof(__typeof__(x), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "xm_eeprom");
	rc |= k054539_state(&s_k539, ctx, load, "xm_k539_");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qv[2] = { s_out_n, s_pub };
	rc |= rbx_state_io(ctx, "xm_q", qv, sizeof(qv));
	rc |= rbx_state_io(ctx, "xm_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "xm_outr", s_out_r, sizeof(s_out_r));
	if (load && !rc) {
		s_out_n = qv[0] >= 0 && qv[0] <= 2048 ? qv[0] : 0;
		s_pub = (uint8_t)qv[1];
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "xm_z80", &z, sizeof(z));
	if (!rc && load) {
		m68k_set_context(cpu);
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
		z_bank();
		k54321_volume();
	}
	return rc ? -1 : 0;
}

static int xm_save(void *ctx) { return xm_state(ctx, 0); }
static int xm_load(void *ctx) { return xm_state(ctx, 1); }
static int xm_supported(const char *machine) { return !strcmp(machine, "xmen"); }

const rbx_driver rbx_drv_xmen = { "Konami X-Men", xm_init, xm_exit, xm_reset, xm_frame, xm_save, xm_load,
                                  xm_supported, gx_render, NULL };

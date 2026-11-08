/*  Konami G.I. Joe. Ported from MAME src/mame/konami/gijoe.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert)
    and the devices it uses (k053251.cpp: Nicola Salmoria; k054321.cpp: Olivier Galibert; eepromser.cpp: Aaron Giles).
    A 68000 at 16 MHz, the K056832 tilemaps (4 bpp, with its AVAC tile banks), the K053246/K053247 sprites, the K053251
    priorities and colour bases (drawn by konamigx_v.c's pre-GX mode: MAME's bitmap_ind16 with the priority bitmap),
    an ER5911 EEPROM; the sound: a Z80 at 8 MHz, a K054539, the K054321 latches.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k054539.h"
#include "../video/k056832.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 16000000u
#define Z80_CLOCK 8000000u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u   // (set_raw(24 MHz / 4, 384, 24, 312, 262, 16, 240))
#define HTOTAL 384
#define LINES 262
#define VIS_H 224
#define DMA_CYCLES (MAIN_CLOCK / 1000000 * 384)   // (JOE_DMADELAY: 42.7 + 341.3 us)

static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k054539_t s_k539;
static eepser_t s_eeprom;
static int s_port_start, s_port_sys, s_port_p12, s_port_p34;
static int32_t s_gain_l, s_gain_r;

static struct {
	uint16_t spriteram[0x800];   // 100000: the object DMA's source
	uint8_t k53251[16];
	uint8_t zram[0x800];
	uint8_t latch[3], k54321_active, k54321_volume;
	uint8_t nmi_prev, z80_irq;
	uint16_t control2;
	int64_t cycles_done, dma_due;
	uint64_t cyc_frac, z_frac, z_end, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} j;

// ---- the drawing's snapshot and state (screen_update) ----
static struct { uint8_t k53251[16]; } s_vs;
static int s_layer_colorbase[4], s_layer_pri[4], s_sprite_colorbase, s_avac_bits[4];

// ---- the K053251 (k053251_device: get_priority, reset_indexes' palette indexes) ----
static int k53251_palette_index(const uint8_t *r, int ci)
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
	const double vol = pow(2.0, ((double)j.k54321_volume - 40.0) / 10.0);
	s_gain_l = (j.k54321_active & 2) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
	s_gain_r = (j.k54321_active & 1) ? (int32_t)(vol * 4096.0 + 0.5) : 0;
}
static void k54321_main_w(int r, uint8_t v)
{
	switch (r) {
	case 0x0: j.k54321_active = v; k54321_volume(); break;
	case 0x2: j.k54321_volume = 0; k54321_volume(); break;
	case 0x3:
		if (v && j.k54321_volume < 64) {
			j.k54321_volume++;
			k54321_volume();
		}
		break;
	case 0x6: j.latch[0] = v; break;
	case 0x7: j.latch[1] = v; break;
	}
}
static uint8_t k54321_main_r(int r) { return r == 0xa ? j.latch[2] : r == 0x8 ? 0 : 0xff; }

// ---- the video callbacks (gijoe_state) ----
static void tile_cb(int layer, int *code, int *color, int *flip, int *category)
{
	(void)flip;
	(void)category;
	int tile = *code;
	if (tile >= 0xf000 && tile <= 0xf4ff) {
		tile &= 0x0fff;
		if (tile < 0x0310)
			tile |= s_avac_bits[0];
		else if (tile < 0x0470)
			tile |= s_avac_bits[1];
		else
			tile |= s_avac_bits[2];
		*code = tile;
	}
	*color = (*color >> 2 & 0x0f) | s_layer_colorbase[layer];
}

static void sprite_cb(int *code, int *color, int *primask)
{
	(void)code;
	const int pri = (*color & 0x03e0) >> 4;
	if (pri <= s_layer_pri[3])
		*primask = 0;
	else if (pri <= s_layer_pri[2])
		*primask = 0xff00;
	else if (pri <= s_layer_pri[1])
		*primask = 0xff00 | 0xf0f0;
	else if (pri <= s_layer_pri[0])
		*primask = 0xff00 | 0xf0f0 | 0xcccc;
	else
		*primask = 0xff00 | 0xf0f0 | 0xcccc | 0xaaaa;
	*color = s_sprite_colorbase | (*color & 0x001f);
}

static void capture(void) { memcpy(s_vs.k53251, j.k53251, sizeof(s_vs.k53251)); }

static void draw(void)   // (screen_update)
{
	k056832_t *k = (k056832_t *)gx_pre_k056832();
	const int vrc_mode = k->regs[0x04 / 2] & 7, vrc_new = k->regs[0x38 / 2];   // (read_avac)
	if (vrc_mode) {
		s_avac_bits[0] = vrc_new << 4 & 0xf000;
		s_avac_bits[1] = vrc_new & 0xf000;
		s_avac_bits[2] = vrc_new << 8 & 0xf000;
		s_avac_bits[3] = vrc_new << 12 & 0xf000;
	} else
		s_avac_bits[0] = s_avac_bits[1] = s_avac_bits[2] = s_avac_bits[3] = 0xf000;
	s_sprite_colorbase = k53251_palette_index(s_vs.k53251, 0);
	for (int i = 0; i < 4; i++)
		s_layer_colorbase[i] = k53251_palette_index(s_vs.k53251, i + 1);
	if (k056832_word_r(k, 0x14) == 2) {   // (the layer offsets: layer A's x-scroll 2, the 832 standard)
		static const int o[4] = { 2, 4, 6, 8 };
		for (int l = 0; l < 4; l++)
			k056832_set_layer_offs(k, l, o[l], 0);
	} else {
		static const int o[4] = { 0, 8, 14, 16 };
		for (int l = 0; l < 4; l++)
			k056832_set_layer_offs(k, l, o[l], 0);
	}
	int layer[4] = { 0, 1, 2, 3 };
	s_layer_pri[0] = 0;
	s_layer_pri[1] = s_vs.k53251[2];   // (get_priority: CI2, CI3, CI4)
	s_layer_pri[2] = s_vs.k53251[3];
	s_layer_pri[3] = s_vs.k53251[4];
	for (int a = 0; a < 3; a++)   // (konami_sortlayers4: swapped when pri[a] <= pri[b])
		for (int b = a + 1; b < 4; b++)
			if (s_layer_pri[a] <= s_layer_pri[b]) {
				int t = s_layer_pri[a]; s_layer_pri[a] = s_layer_pri[b]; s_layer_pri[b] = t;
				t = layer[a]; layer[a] = layer[b]; layer[b] = t;
			}
	gx_pre_fill();
	gx_pre_tilemap(layer[0], 1);
	gx_pre_tilemap(layer[1], 2);
	gx_pre_tilemap(layer[2], 4);
	gx_pre_tilemap(layer[3], 8);
	gx_pre_sprites();
}

// ---- the main CPU's bus ----
static inline int64_t now_cycles(void) { return j.cycles_done + m68k_cycles_run(); }

static void object_dma(void)   // (gijoe_state::object_dma: the live sprites to the K053247's RAM, the rest cleared)
{
	uint16_t *dst_head = gx.k55673.ram, *dst_tail = gx.k55673.ram + 255 * 8;
	for (const uint16_t *src = j.spriteram; src <= j.spriteram + 255 * 8; src += 8) {
		if (*src & 0x8000) {
			memcpy(dst_head, src, 0x10);
			dst_head += 8;
		} else {
			*dst_tail = 0;
			dst_tail -= 8;
		}
	}
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

static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x100000 && a < 0x101000)
		return j.spriteram[(a - 0x100000) >> 1];
	if (a >= 0x120000 && a < 0x124000)
		return k056832_ram_word_r(&gx.k56832, (int)((a & 0x1fff) >> 1));
	if (a >= 0x130000 && a < 0x132000) {   // (rom_word_r)
		const k056832_t *k = &gx.k56832;
		const uint32_t addr = 0x2000u * (uint32_t)k->cur_gfx_banks + (a - 0x130000);
		return (uint16_t)((addr + 1 < k->rom_size ? k->rom[addr + 1] : 0) | (addr < k->rom_size ? k->rom[addr] : 0) << 8);
	}
	if (a >= 0x1c0000 && a < 0x1c0020)
		return k54321_main_r((int)((a - 0x1c0000) >> 1));
	if (a == 0x1e0000)
		return (uint16_t)rbx_port_read(s_port_p12);
	if (a == 0x1e0002)
		return (uint16_t)rbx_port_read(s_port_p34);
	if (a == 0x1e4000)
		return (uint16_t)rbx_port_read(s_port_sys);
	if (a == 0x1e4002) {
		rbx_port_custom(s_port_start, 0x300,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1) << 8);
		return (uint16_t)rbx_port_read(s_port_start);
	}
	if (a == 0x1e8000)
		return j.control2;
	if (a == 0x1f0000)
		return (uint16_t)(k053246_r(0) << 8 | k053246_r(1));
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
	if (a >= 0x100000 && a < 0x101000) {
		uint16_t *p = &j.spriteram[(a - 0x100000) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if (a >= 0x110000 && a < 0x110008) {
		if (mask & 0xff00) k055673_k053246_w(&gx.k55673, (int)(a - 0x110000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k055673_k053246_w(&gx.k55673, (int)(a - 0x110000) + 1, (uint8_t)v);
	} else if (a >= 0x120000 && a < 0x124000)
		k056832_ram_word_w(&gx.k56832, (int)((a & 0x1fff) >> 1), v, mask);
	else if (a >= 0x160000 && a < 0x160008)
		k056832_b_word_w(&gx.k56832, (int)((a - 0x160000) >> 1), v, mask);
	else if (a >= 0x1a0000 && a < 0x1a0020) {   // (K053251 write: umask 00ff)
		if (mask & 0x00ff)
			j.k53251[((a - 0x1a0000) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (a >= 0x1b0000 && a < 0x1b0040)
		k056832_word_w(&gx.k56832, (int)((a - 0x1b0000) >> 1), v, mask);
	else if (a >= 0x1c0000 && a < 0x1c0020) {
		if (mask & 0x00ff)
			k54321_main_w((int)((a - 0x1c0000) >> 1), (uint8_t)v);
	} else if (a == 0x1d0000)
		j.z80_irq = 1;   // (sound_irq_w)
	else if (a == 0x1e8000 && (mask & 0x00ff)) {   // (control2_w: the EEPROM lines, the IRQ enables, OBJCHA)
		const int64_t t = now_cycles();
		eepser_di_write(&s_eeprom, v & 1);
		eepser_cs_write(&s_eeprom, v >> 1 & 1, t);
		eepser_clk_write(&s_eeprom, v >> 2 & 1, t);
		j.control2 = (uint16_t)(v & 0xff);
		gx.k55673.objcha_line = (v & 0x40) != 0;
	}
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xf000)
		return a < s_zrom_size ? s_zrom[a] : 0xff;
	if (a < 0xf800)
		return j.zram[a - 0xf000];
	if (a < 0xfa30)
		return k054539_read(&s_k539, a - 0xf800);
	if (a == 0xfc02)
		return j.latch[0];
	if (a == 0xfc03)
		return j.latch[1];
	return 0xff;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xf000 && a < 0xf800)
		j.zram[a - 0xf000] = v;
	else if (a >= 0xf800 && a < 0xfa30)
		k054539_write(&s_k539, a - 0xf800, v);
	else if (a == 0xfc00)
		j.latch[2] = v;
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void k054539_timer(void *user, int state)   // (timer_handler: the Z80's NMI line)
{
	(void)user;
	if (state && !j.nmi_prev)
		z80_gen_nmi(&s_z);
	j.nmi_prev = (uint8_t)state;
}

static void sound_run(int n)
{
	for (int i = 0; i < n; i++) {
		j.z_frac += Z80_CLOCK;
		const unsigned long cyc = (unsigned long)(j.z_frac / SND_RATE);
		j.z_frac -= (uint64_t)cyc * SND_RATE;
		int line = j.z80_irq;
		j.z_end += cyc;
		z80_run(&s_z, (unsigned long)j.z_end, &line, 1);
		j.z80_irq = (uint8_t)line;
		int32_t l = 0, r = 0;
		k054539_render(&s_k539, &l, &r, 1);
		// (the chip's outputs to the speaker inverted, at the K054321's gains)
		rbx_audio_put_stereo((int32_t)((int64_t)r * s_gain_l >> 12), (int32_t)((int64_t)l * s_gain_r >> 12));
	}
}

// ---- init / frame ----
static void gj_exit(void);
static void gj_reset(void);

static int gj_init(const rbx_game *g)
{
	uint32_t pcm_size, tile_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *pcm = rbx_region("k054539", &pcm_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	s_sprrom = rbx_region("k053246", &s_sprrom_size);
	if (!s_rom || !s_zrom || !pcm || !tiles || !s_sprrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&j, 0, sizeof(j));
	gx.pal_entries = 2048;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_tilecb = tile_cb;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	k056832_init(&gx.k56832, tiles, tile_size, K056832_BPP_4, 1, 0);
	gx.k56832.linemap_enabled = 1;   // (set_linemap_enable(true))
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_053247, -37, 20);
	gx_video_set_area(24, 288);
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x180000, 0x18ffff, gx.workram, 0x10000, 1, 1);
	m68k_bus_map(0x190000, 0x190fff, gx.palram, 0x1000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	k054539_init(&s_k539, pcm, pcm_size);
	s_k539.timer_cb = k054539_timer;
	eepser_init(&s_eeprom, EEPSER_ER5911, 128, 8, MAIN_CLOCK);
	{
		uint32_t esz;
		const uint8_t *e = rbx_region("eeprom", &esz);
		if (e)
			eepser_default(&s_eeprom, e, esz);
	}
	eepser_nv_load(&s_eeprom, g->set);   // (the contents kept between runs)
	s_port_start = rbx_port_find("START");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_p12 = rbx_port_find("P1_P2");
	s_port_p34 = rbx_port_find("P3_P4");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(288, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		gj_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	gj_reset();
	return 0;
}

static void gj_exit(void)
{
	eepser_nv_save(&s_eeprom);
	gx_video_exit();
}

static void gj_reset(void)
{
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(j.zram, 0, sizeof(j.zram));
	memset(j.latch, 0, sizeof(j.latch));
	memset(j.k53251, 0, sizeof(j.k53251));
	j.control2 = 0;
	j.nmi_prev = j.z80_irq = 0;
	j.cycles_done = 0;
	j.dma_due = -1;
	j.cyc_frac = j.z_frac = j.z_end = j.snd_frac = 0;
	j.frame = 0;
	j.k54321_active = j.k54321_volume = 0;
	k54321_volume();
	k054539_reset(&s_k539);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	for (int p = 0; p < 256; p++)
		s_zmap[p] = p < 0xf0 && (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0xf0 + p] = j.zram + p * 256;
	s_z.read_map = s_zmap;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void gj_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (line + 240) % LINES;   // (the frame from the vblank start)
		if (line == 0) {
			gx_video_capture();
			// (interrupt: the vblank, masked by the K056832's IRQ enable 0)
			if (gx.k56832.regs[0x06 / 2] & 1) {
				if (gx.k55673.kx46_regs[5] & 0x10) {   // (k053246_is_irq_enabled: the object DMA, IRQ 6 at its end)
					object_dma();
					j.dma_due = j.cycles_done + DMA_CYCLES;
				}
				if (j.control2 & 0x80)
					m68k_bus_hold_irq(5);
			}
		}
		(void)vpos;
		j.cyc_frac += (uint64_t)MAIN_CLOCK * HTOTAL;
		int64_t cyc = (int64_t)(j.cyc_frac / PIX_CLOCK);
		j.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		while (cyc > 0) {
			int run = (int)cyc;
			if (j.dma_due >= 0 && j.dma_due - j.cycles_done < run)
				run = j.dma_due - j.cycles_done > 0 ? (int)(j.dma_due - j.cycles_done) : 0;
			if (run > 0) {
				const int used = m68k_execute(run);
				j.cycles_done += used;
				cyc -= used;
			}
			if (j.dma_due >= 0 && j.cycles_done >= j.dma_due) {   // (dmaend_callback)
				j.dma_due = -1;
				if (j.control2 & 0x20)
					m68k_bus_hold_irq(6);
			}
		}
		j.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(j.snd_frac / PIX_CLOCK);
		j.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		sound_run(ns);
	}
	j.frame++;
	gx.frame = j.frame;
}

static int gj_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "gj_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "gj_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k056832_state(&gx.k56832, ctx, load);
	rc |= k055673_state(&gx.k55673, ctx, load);
	rc |= rbx_state_io(ctx, "gj", &j, (uint32_t)offsetof(__typeof__(j), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "gj_eeprom");
	rc |= k054539_state(&s_k539, ctx, load, "gj_k539_");
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "gj_z80", &z, sizeof(z));
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
		k54321_volume();
	}
	return rc ? -1 : 0;
}

static int gj_save(void *ctx) { return gj_state(ctx, 0); }
static int gj_load(void *ctx) { return gj_state(ctx, 1); }
static int gj_supported(const char *machine) { return !strcmp(machine, "gijoe"); }

const rbx_driver rbx_drv_gijoe = { "Konami G.I. Joe", gj_init, gj_exit, gj_reset, gj_frame, gj_save, gj_load,
                                   gj_supported, gx_render, NULL };

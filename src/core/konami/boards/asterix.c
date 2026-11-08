/*  Konami Asterix. Ported from MAME src/mame/konami/asterix.cpp (BSD-3-Clause, copyright-holders: Olivier Galibert)
    and the devices it uses (k053244_k053245.cpp: David Haywood; k053251.cpp: Nicola Salmoria; eepromser.cpp: Aaron
    Giles). A 68000 at 12 MHz, the K056832 tilemaps (4 bpp, its tile banks through an 8-bit window), the K053244/K053245
    sprites, the K053251 priorities and colour bases (drawn by konamigx_v.c's pre-GX mode, MAME's bitmap_ind16), the
    protection's copy command, an ER5911 EEPROM; the sound: a Z80 at 6 MHz, a YM2151, a K053260 (its SH1 output the
    Z80's NMI, re-armed by the Z80).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../machine/eepromser.h"
#include "../../sound/k053260.h"
#include "../../sound/rbx_ym.h"
#include "../video/k056832.h"
#include "../video/k055673.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 12000000u
#define Z80_CLOCK 6000000u
#define CHIP_CLOCK 4000000u    // (the YM2151 and the K053260: 32 MHz / 8)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 16, 304, 262, 16, 240))
#define HTOTAL 384
#define LINES 262
#define VIS_H 224

static uint8_t *s_rom, *s_zrom, *s_sprrom;
static uint32_t s_rom_size, s_zrom_size, s_sprrom_size;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k053260_t s_k260;
static eepser_t s_eeprom;
static int s_port_in0, s_port_in1;

static struct {
	uint16_t k245_ram[0x400], k245_buffer[0x400], k245_extra[0x400];
	uint8_t k244_regs[16];
	uint8_t k53251[16];
	uint8_t zram[0x800];
	uint16_t prot[2], spritebank, control2;
	uint8_t z80_irq, nmi_line;
	int64_t nmi_due;          // (the Z80 cycle of the next NMI: the K053260's SH1 rising after the arm, -1 none)
	uint32_t rombank;
	int64_t cycles_done;
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} a;

// ---- the drawing's snapshot and state (screen_update) ----
static struct { uint8_t k53251[16], k244_regs[16]; uint16_t buffer[0x400], spritebank; } s_vs;
static int s_layer_colorbase[4], s_layerpri[3], s_sprite_colorbase, s_tilebanks[4], s_spritebanks[4];

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

// ---- the video callbacks (asterix_state) ----
static void tile_cb(int layer, int *code, int *color, int *flip, int *category)
{
	(void)category;
	const int c = *code;
	*flip = (c & 0x1000) ? 1 : 0;   // (flags = TILE_FLIPX or 0)
	*color = (s_layer_colorbase[layer] + ((c & 0xe000) >> 13)) & 0x7f;
	*code = (c & 0x03ff) | s_tilebanks[(c >> 10) & 3];
}

static void sprite_cb(int *code, int *color, int *primask)
{
	const int pri = (*color & 0x00e0) >> 2;
	if (pri <= s_layerpri[2])
		*primask = 0;
	else if (pri <= s_layerpri[1])
		*primask = 0xf0;
	else if (pri <= s_layerpri[0])
		*primask = 0xf0 | 0xcc;
	else
		*primask = 0xf0 | 0xcc | 0xaa;
	*color = s_sprite_colorbase | (*color & 0x001f);
	*code = (*code & 0xfff) | s_spritebanks[(*code >> 12) & 3];
}

static void capture(void)
{
	memcpy(s_vs.k53251, a.k53251, sizeof(s_vs.k53251));
	memcpy(s_vs.k244_regs, a.k244_regs, sizeof(s_vs.k244_regs));
	memcpy(s_vs.buffer, a.k245_buffer, sizeof(s_vs.buffer));
	s_vs.spritebank = a.spritebank;
}

static void draw(void)   // (screen_update)
{
	k056832_t *k = (k056832_t *)gx_pre_k056832();
	int sdx;
	if (k056832_word_r(k, 0) & 0x10) {
		for (int l = 0; l < 4; l++)
			k056832_set_layer_offs(k, l, -7 + 2 * l - 177, 0);
		sdx = -3 + 6;
	} else {
		for (int l = 0; l < 4; l++)
			k056832_set_layer_offs(k, l, -7 + 2 * l, 0);
		sdx = -3;
	}
	for (int bank = 0; bank < 4; bank++) {   // (get_lookup: the tile banks)
		int res = (k->regs[0x1c] >> (bank << 2)) & 0x0f;
		if (k->uses_tile_banks)
			res |= k->cur_tile_bank << 4;
		s_tilebanks[bank] = res << 10;
	}
	s_spritebanks[0] = (s_vs.spritebank << 12) & 0x7000;   // (reset_spritebank)
	s_spritebanks[1] = (s_vs.spritebank << 9) & 0x7000;
	s_spritebanks[2] = (s_vs.spritebank << 6) & 0x7000;
	s_spritebanks[3] = (s_vs.spritebank << 3) & 0x7000;
	s_sprite_colorbase = k53251_palette_index(s_vs.k53251, 1);
	static const int ci[4] = { 0, 2, 3, 4 };
	for (int plane = 0; plane < 4; plane++)
		s_layer_colorbase[plane] = k53251_palette_index(s_vs.k53251, ci[plane]);
	int layer[3] = { 0, 1, 3 };
	s_layerpri[0] = s_vs.k53251[0];   // (get_priority: CI0, CI2, CI4)
	s_layerpri[1] = s_vs.k53251[2];
	s_layerpri[2] = s_vs.k53251[4];
	for (int x = 0; x < 2; x++)   // (konami_sortlayers3: swapped when pri[a] < pri[b])
		for (int y = x + 1; y < 3; y++)
			if (s_layerpri[x] < s_layerpri[y]) {
				int t = s_layerpri[x]; s_layerpri[x] = s_layerpri[y]; s_layerpri[y] = t;
				t = layer[x]; layer[x] = layer[y]; layer[y] = t;
			}
	gx_pre_fill_pen(0);
	gx_pre_tilemap(layer[0], 1);
	gx_pre_tilemap(layer[1], 2);
	gx_pre_tilemap(layer[2], 4);
	gx_pre_sprites_053245(s_vs.buffer, s_vs.k244_regs, sdx, -1, -1);
	gx_pre_tilemap(2, 0);
}

// ---- the K053244 ----
static uint8_t k053244_r(int offset)
{
	offset &= 0xf;
	if (offset == 6)
		memcpy(a.k245_buffer, a.k245_ram, sizeof(a.k245_buffer));   // (update_buffer)
	else if (offset == 7)
		for (int i = 0; i < 0x400; i += 8)   // (clear_buffer)
			a.k245_buffer[i] = 0;
	else if (offset >= 0x0c) {
		uint32_t addr = a.rombank << 19 | (uint32_t)(a.k244_regs[11] & 7) << 18 | (uint32_t)a.k244_regs[8] << 10 |
		                (uint32_t)a.k244_regs[9] << 2 | (uint32_t)((offset & 3) ^ 1);
		return s_sprrom[addr & (s_sprrom_size - 1)];
	}
	return 0;
}
static void k053244_w(int offset, uint8_t v)
{
	offset &= 0xf;
	a.k244_regs[offset] = v;
	if (offset == 6)
		memcpy(a.k245_buffer, a.k245_ram, sizeof(a.k245_buffer));
	else if (offset == 7)
		for (int i = 0; i < 0x400; i += 8)
			a.k245_buffer[i] = 0;
}

// ---- the sound board a frame behind on the worker (rbx_aux), as moo.c's: the host's writes queued with the samples
// before them; the host reads the K053260's replies as of the last finished frame ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
static int s_out_n, s_out_pos;
static uint8_t s_pub[2];   // (the K053260's ports 2 and 3 as of the last job)
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

// ---- the main CPU's bus ----
static inline int64_t now_cycles(void) { return a.cycles_done + m68k_cycles_run(); }

static void protection_w(int offset, uint16_t v, uint16_t mask)   // (asterix_state::protection_w)
{
	a.prot[offset & 1] = (uint16_t)((v & mask) | (a.prot[offset & 1] & ~mask));
	if ((offset & 1) != 1)
		return;
	const uint32_t cmd = (uint32_t)a.prot[0] << 16 | a.prot[1];
	if (cmd >> 24 != 0x64)
		return;
	const uint32_t base = cmd & 0xffffff;
	uint32_t param1 = (uint32_t)(uint16_t)m68k_read_memory_16(base) << 16 | (uint16_t)m68k_read_memory_16(base + 2);
	uint32_t param2 = (uint32_t)(uint16_t)m68k_read_memory_16(base + 4) << 16 | (uint16_t)m68k_read_memory_16(base + 6);
	if (param1 >> 24 == 0x22) {
		int size = (int)(param2 >> 24);
		param1 &= 0xffffff;
		param2 &= 0xffffff;
		for (; size >= 0; size--, param1 += 2, param2 += 2)
			m68k_write_memory_16(param2, (uint16_t)m68k_read_memory_16(param1));
	}
}

static uint16_t main_r16(uint32_t ad)
{
	ad &= 0xffffff;
	if (ad >= 0x180000 && ad < 0x180800)
		return a.k245_ram[(ad - 0x180000) >> 1];
	if (ad >= 0x180800 && ad < 0x181000)
		return a.k245_extra[(ad - 0x180800) >> 1];
	if (ad >= 0x200000 && ad < 0x200010)   // (the K053244 on both bytes)
		return (uint16_t)(k053244_r((int)(ad - 0x200000)) << 8 | k053244_r((int)(ad - 0x200000) + 1));
	if (ad >= 0x300000 && ad < 0x300020)   // (umask 00ff)
		return k053244_r((int)((ad - 0x300000) >> 1));
	if (ad == 0x380000)
		return (uint16_t)rbx_port_read(s_port_in0);
	if (ad == 0x380002) {
		rbx_port_custom(s_port_in1, 0x300,
		                (uint32_t)(eepser_do_read(&s_eeprom) | eepser_ready_read(&s_eeprom, now_cycles()) << 1) << 8);
		return (uint16_t)rbx_port_read(s_port_in1);
	}
	if (ad >= 0x380200 && ad < 0x380204)   // (the K053260's main side: umask 00ff)
		return (((ad - 0x380200) >> 1) & 1) ? s_pub[1] : s_pub[0];   // (main_read: ports 2 / 3, the last frame's)
	if (ad >= 0x400000 && ad < 0x401000)
		return k056832_ram_half_word_r(&gx.k56832, (int)((ad - 0x400000) >> 1));
	if (ad >= 0x420000 && ad < 0x422000) {   // (old_rom_word_r)
		const k056832_t *k = &gx.k56832;
		const uint32_t addr = 0x2000u * (uint32_t)k->cur_gfx_banks + (ad - 0x420000);
		return (uint16_t)((addr + 1 < k->rom_size ? k->rom[addr + 1] : 0) | (addr < k->rom_size ? k->rom[addr] : 0) << 8);
	}
	return 0;
}
static uint8_t main_r8(uint32_t ad)
{
	const uint16_t w = main_r16(ad & ~1u);
	return (uint8_t)((ad & 1) ? w : w >> 8);
}

static void main_w16m(uint32_t ad, uint16_t v, uint16_t mask)
{
	ad &= 0xffffff;
	if (ad >= 0x180000 && ad < 0x180800) {
		uint16_t *p = &a.k245_ram[(ad - 0x180000) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if (ad >= 0x180800 && ad < 0x181000) {
		uint16_t *p = &a.k245_extra[(ad - 0x180800) >> 1];
		*p = (uint16_t)((v & mask) | (*p & ~mask));
	} else if (ad >= 0x200000 && ad < 0x200010) {
		if (mask & 0xff00) k053244_w((int)(ad - 0x200000), (uint8_t)(v >> 8));
		if (mask & 0x00ff) k053244_w((int)(ad - 0x200000) + 1, (uint8_t)v);
	} else if (ad >= 0x300000 && ad < 0x300020) {
		if (mask & 0x00ff)
			k053244_w((int)((ad - 0x300000) >> 1), (uint8_t)v);
	} else if (ad == 0x380100 && (mask & 0x00ff)) {   // (control2_w: the EEPROM lines, the tile bank)
		const int64_t t = now_cycles();
		eepser_di_write(&s_eeprom, v & 1);
		eepser_cs_write(&s_eeprom, v >> 1 & 1, t);
		eepser_clk_write(&s_eeprom, v >> 2 & 1, t);
		k056832_set_tile_bank(&gx.k56832, (v & 0x20) >> 5);
		a.control2 = v;
	} else if (ad >= 0x380200 && ad < 0x380204) {
		if (mask & 0x00ff)
			q_put(Q_LATCH, (int)((ad - 0x380200) >> 1), (uint8_t)v);
	} else if (ad == 0x380300)
		q_put(Q_IRQ, 0, 0);   // (sound_irq_w)
	else if (ad == 0x380400) {   // (spritebank_w: the K053244's ROM bank too)
		a.spritebank = (uint16_t)((v & mask) | (a.spritebank & ~mask));
		a.rombank = a.spritebank & 7;
	} else if (ad >= 0x380500 && ad < 0x380520) {
		if (mask & 0x00ff)
			a.k53251[((ad - 0x380500) >> 1) & 0xf] = (uint8_t)(v & 0x3f);
	} else if (ad >= 0x380700 && ad < 0x380708)
		k056832_b_word_w(&gx.k56832, (int)((ad - 0x380700) >> 1), v, mask);
	else if (ad >= 0x380800 && ad < 0x380804)
		protection_w((int)((ad - 0x380800) >> 1), v, mask);
	else if (ad >= 0x400000 && ad < 0x401000)
		k056832_ram_half_word_w(&gx.k56832, (int)((ad - 0x400000) >> 1), v, mask);
	else if (ad >= 0x440000 && ad < 0x440040)
		k056832_word_w(&gx.k56832, (int)((ad - 0x440000) >> 1), v, mask);
}
static void main_w16(uint32_t ad, uint16_t v) { main_w16m(ad, v, 0xffff); }
static void main_w8(uint32_t ad, uint8_t v)
{
	main_w16m(ad & ~1u, (ad & 1) ? v : (uint16_t)(v << 8), (ad & 1) ? 0x00ff : 0xff00);
}

// ---- the sound board (sound_map) ----

static uint8_t z_rd(void *u, uint16_t ad)
{
	(void)u;
	if (ad < 0xf000)
		return ad < s_zrom_size ? s_zrom[ad] : 0xff;
	if (ad < 0xf800)
		return a.zram[ad - 0xf000];
	if (ad == 0xf801)
		return rbx_ym_read(0, 1);
	if (ad >= 0xfa00 && ad < 0xfa30)
		return k053260_read(&s_k260, ad - 0xfa00);
	return 0xff;
}
static void z_wr(void *u, uint16_t ad, uint8_t v)
{
	(void)u;
	if (ad >= 0xf000 && ad < 0xf800)
		a.zram[ad - 0xf000] = v;
	else if (ad == 0xf801)
		rbx_ym_write(0, 1, v);
	else if (ad == 0xfe00)
		rbx_ym_write(0, 0, v);
	else if (ad >= 0xfa00 && ad < 0xfa30)
		k053260_write(&s_k260, ad - 0xfa00, v);
	else if (ad == 0xfc00) {   // (sound_arm_nmi_w: NMI cleared, blocked 4 cycles; then the next SH1 rise asserts it)
		a.nmi_line = 0;
		const uint64_t tb = ((uint64_t)s_z.cyc + 4) * CHIP_CLOCK / Z80_CLOCK + 1;
		const uint64_t rise = tb + k053260_sh1_next_rise(tb) - 1;
		a.nmi_due = (int64_t)((rise * Z80_CLOCK + CHIP_CLOCK - 1) / CHIP_CLOCK);
		z80_break = 1;
	}
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// n samples (a line's): the Z80 (up to its NMIs), the K053260 and the YM2151 (its samples spread over the line's)
static void sound_run(int n)
{
	if (n <= 0)
		return;
	a.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(a.z_frac / SND_RATE);
	a.z_frac -= (uint64_t)cyc * SND_RATE;
	a.z_end += cyc;
	for (;;) {
		unsigned long end = (unsigned long)a.z_end;
		if (a.nmi_due >= 0 && (unsigned long)a.nmi_due < end)
			end = (unsigned long)a.nmi_due;
		int line = a.z80_irq;
		z80_break = 0;
		if ((long)(s_z.cyc - end) < 0)
			z80_run(&s_z, end, &line, 1);
		a.z80_irq = (uint8_t)line;
		z80_break = 0;
		if (a.nmi_due >= 0 && (long)(s_z.cyc - (unsigned long)a.nmi_due) >= 0) {   // (z80_nmi_w: SH1's rise)
			a.nmi_due = -1;
			if (!a.nmi_line) {
				a.nmi_line = 1;
				z80_gen_nmi(&s_z);
			}
			continue;
		}
		if ((long)(s_z.cyc - (unsigned long)a.z_end) >= 0)
			break;
	}
	int32_t l[64], r[64], yl[96], yr[96];
	if (n > 64)
		n = 64;
	memset(l, 0, sizeof(int32_t) * (size_t)n);
	memset(r, 0, sizeof(int32_t) * (size_t)n);
	k053260_render(&s_k260, l, r, n);
	a.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(a.ym_frac / SND_RATE);
	a.ym_frac -= (uint64_t)yc * SND_RATE;
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n; i++) {
		const int k = yn > 0 ? i * yn / n : -1;
		// (the YM2151 at 1.0, the K053260 at 0.75)
		if (s_out_pos < 2048) {
			s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + l[i] * 3 / 4;
			s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + r[i] * 3 / 4;
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
		case Q_LATCH: k053260_main_write(&s_k260, e->arg, (uint8_t)e->v); break;
		case Q_IRQ: a.z80_irq = 1; break;
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
	s_pub[0] = k053260_main_read(&s_k260, 0);
	s_pub[1] = k053260_main_read(&s_k260, 1);
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void ax_exit(void);
static void ax_reset(void);

static int ax_init(const rbx_game *g)
{
	uint32_t pcm_size, tile_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *pcm = rbx_region("k053260", &pcm_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	s_sprrom = rbx_region("k053244", &s_sprrom_size);
	if (!s_rom || !s_zrom || !pcm || !tiles || !s_sprrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&a, 0, sizeof(a));
	gx.pal_entries = 2048;
	gx.pal555 = 1;
	gx.tilecb = GX_TILECB_PRE;
	gx.mixmode = GX_MIXMODE_PRE;
	gx.pre_tilecb = tile_cb;
	gx.pre_sprcb = sprite_cb;
	gx.pre_capture = capture;
	gx.pre_draw = draw;
	gx.pre_screen_w = HTOTAL;
	k056832_init(&gx.k56832, tiles, tile_size, K056832_BPP_4, 1, 1);
	k055673_init(&gx.k55673, s_sprrom, s_sprrom_size, K055673_LAYOUT_053245, 0, 0);
	gx_video_set_area(16, 288);
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x100000, 0x107fff, gx.workram, 0x8000, 1, 1);
	m68k_bus_map(0x280000, 0x280fff, gx.palram, 0x1000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	k053260_init(&s_k260, pcm, pcm_size, CHIP_CLOCK);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
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
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(288, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		ax_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, b = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (b << 3 | b >> 2));
	}
	ax_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void ax_exit(void)
{
	eepser_nv_save(&s_eeprom);
	rbx_aux_set_job(NULL);
	gx_video_exit();
	rbx_ym_exit(0);
}

static void ax_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_pub[0] = s_pub[1] = 0;
	s_job_frame = 0;
	memset(gx.workram, 0, sizeof(gx.workram));
	memset(a.zram, 0, sizeof(a.zram));
	memset(a.k53251, 0, sizeof(a.k53251));
	memset(a.k244_regs, 0, sizeof(a.k244_regs));
	a.prot[0] = a.prot[1] = 0;
	a.spritebank = 0;
	a.rombank = 0;
	a.control2 = 0;
	a.z80_irq = 0;
	a.cycles_done = 0;
	a.cyc_frac = a.z_frac = a.z_end = a.ym_frac = a.snd_frac = 0;
	a.frame = 0;
	k053260_reset(&s_k260);
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
		s_zmap[0xf0 + p] = a.zram + p * 256;
	s_z.read_map = s_zmap;
	a.nmi_due = -1;
	a.nmi_line = 1;   // (machine_reset: the NMI asserted)
	z80_gen_nmi(&s_z);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void ax_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == 0) {   // (the vblank start: interrupt, masked by the K056832's IRQ enable 0)
			gx_video_capture();
			if (gx.k56832.regs[0x06 / 2] & 1)
				m68k_bus_hold_irq(5);
		}
		a.cyc_frac += (uint64_t)MAIN_CLOCK * HTOTAL;
		const int cyc = (int)(a.cyc_frac / PIX_CLOCK);
		a.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		a.cycles_done += m68k_execute(cyc);
		a.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(a.snd_frac / PIX_CLOCK);
		a.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, 0, (uint32_t)ns);
	}
	sound_frame();
	a.frame++;
	gx.frame = a.frame;
}

static int ax_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	rbx_aux_sync();   // (the job done: the board at the frame's end, its samples waiting)
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "ax_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "ax_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k056832_state(&gx.k56832, ctx, load);
	rc |= rbx_state_io(ctx, "ax", &a, (uint32_t)offsetof(__typeof__(a), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "ax_eeprom");
	rc |= k053260_state(&s_k260, ctx, load, "ax_k260_");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qv[3] = { s_out_n, s_pub[0], s_pub[1] };
	rc |= rbx_state_io(ctx, "ax_q", qv, sizeof(qv));
	rc |= rbx_state_io(ctx, "ax_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "ax_outr", s_out_r, sizeof(s_out_r));
	if (load && !rc) {
		s_out_n = qv[0] >= 0 && qv[0] <= 2048 ? qv[0] : 0;
		s_pub[0] = (uint8_t)qv[1];
		s_pub[1] = (uint8_t)qv[2];
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
	}
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "ax_z80", &z, sizeof(z));
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
	}
	return rc ? -1 : 0;
}

static int ax_save(void *ctx) { return ax_state(ctx, 0); }
static int ax_load(void *ctx) { return ax_state(ctx, 1); }
static int ax_supported(const char *machine) { return !strcmp(machine, "asterix"); }

const rbx_driver rbx_drv_asterix = { "Konami Asterix", ax_init, ax_exit, ax_reset, ax_frame, ax_save, ax_load,
                                     ax_supported, gx_render, NULL };

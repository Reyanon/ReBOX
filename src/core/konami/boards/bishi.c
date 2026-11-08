/*  Konami Bishi Bashi Champ hardware. Ported from MAME src/mame/konami/bishi.cpp (BSD-3-Clause, copyright-holders:
    R. Belmont). A 68000 at 12 MHz, the K056832 tilemaps (8 bpp), the K055555 priorities and the K054338 background
    colour (drawn by konamigx_v.c, GX_MIXMODE_LAYERS), a YMZ280B. No sprites.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../sound/ymz280b.h"
#include "../video/k056832.h"
#include "konamigx.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define MAIN_CLOCK 12000000u
#define LINES 256          // (set_size(64*8, 32*8), 60 Hz)
#define VIS_H 224

static uint8_t *s_rom;
static uint32_t s_rom_size;
static ymz280b_t s_ymz;
static int32_t *s_ymz_l, *s_ymz_r;
static int s_ymz_cap, s_ymz_n;
static uint32_t s_ymz_line0, s_ymz_line_n;
static int s_port_dsw, s_port_sys, s_port_in;
static struct {
	uint16_t control, control2;
	int32_t line, line_cyc;
	uint64_t cyc_frac, snd_frac;
	uint8_t state_end;
} b;

// ---- the YMZ280B, rendered up to the CPU's time before each access ----
static void ymz_to(int n)
{
	if (n > s_ymz_cap)
		n = s_ymz_cap;
	if (n > s_ymz_n) {
		ymz280b_render(&s_ymz, s_ymz_l + s_ymz_n, s_ymz_r + s_ymz_n, n - s_ymz_n);
		s_ymz_n = n;
	}
}
static void ymz_sync(void)
{
	const int d = m68k_cycles_run();
	ymz_to((int)(s_ymz_line0 + (b.line_cyc > 0 ? (uint32_t)((uint64_t)(d < b.line_cyc ? d : b.line_cyc) * s_ymz_line_n / (uint32_t)b.line_cyc) : 0)));
}
static void ymz_irq(void *user, int state)
{
	(void)user;
	m68k_bus_set_line(1, state);   // (irq_handler: M68K_IRQ_1)
}

// ---- the main CPU's bus ----
static uint16_t main_r16(uint32_t a)
{
	a &= 0xffffff;
	if (a == 0x800000)
		return b.control;
	if (a == 0x800004)
		return (uint16_t)rbx_port_read(s_port_dsw);
	if (a == 0x800006)
		return (uint16_t)rbx_port_read(s_port_sys);
	if (a == 0x800008)
		return (uint16_t)rbx_port_read(s_port_in);
	if (a >= 0x880000 && a < 0x880004) {   // (the YMZ280B: umask ff00)
		ymz_sync();
		return (uint16_t)(ymz280b_read(&s_ymz, (int)((a - 0x880000) >> 1)) << 8);
	}
	if (a >= 0xa00000 && a < 0xa02000)
		return k056832_ram_word_r(&gx.k56832, (int)((a - 0xa00000) >> 1));
	if (a >= 0xb04000 && a < 0xb04800)   // (palette_mirror_r: a bug in the RAM / ROM test)
		return *(const uint16_t *)(gx.palram + (a - 0xb04000));
	if (a >= 0xc00000 && a < 0xc02000) {   // (K056832_rom_r: bishi_rom_word_r)
		const uint32_t offset = (a - 0xc00000) >> 1;
		uint32_t o = (offset >> 1) * 8;
		if (offset & 1)
			o++;
		if (b.control2 & 0x1000)
			o += 4;
		const k056832_t *k = &gx.k56832;
		const uint32_t addr = 0x4000u * (uint32_t)k->cur_gfx_banks + o;
		return (uint16_t)((addr + 2 < k->rom_size ? k->rom[addr + 2] : 0) | (addr < k->rom_size ? k->rom[addr] : 0) << 8);
	}
	return 0xffff;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}

static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a == 0x800000)
		b.control = (uint16_t)((v & mask) | (b.control & ~mask));
	else if (a >= 0x810000 && a < 0x810004)   // (the bank of the character ROM test)
		b.control2 = (uint16_t)((v & mask) | (b.control2 & ~mask));
	else if (a >= 0x830000 && a < 0x830040)
		k056832_word_w(&gx.k56832, (int)((a - 0x830000) >> 1), v, mask);
	else if (a >= 0x840000 && a < 0x840008)
		k056832_b_word_w(&gx.k56832, (int)((a - 0x840000) >> 1), v, mask);
	else if (a >= 0x850000 && a < 0x850020) {
		uint16_t *r = &gx.k54338[(a - 0x850000) >> 1];
		*r = (uint16_t)((v & mask) | (*r & ~mask));
	} else if (a >= 0x870000 && a < 0x870100)   // (K055555_word_w)
		gx.k55555[((a - 0x870000) >> 1) & 0x7f] = (uint8_t)(mask == 0x00ff ? v : v >> 8);
	else if (a >= 0x880000 && a < 0x880004) {
		if (mask & 0xff00) {
			ymz_sync();
			ymz280b_write(&s_ymz, (int)((a - 0x880000) >> 1), (uint8_t)(v >> 8));
		}
	} else if (a >= 0xa00000 && a < 0xa02000)
		k056832_ram_word_w(&gx.k56832, (int)((a - 0xa00000) >> 1), v, mask);
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- init / frame ----
static void bishi_exit(void);
static void bishi_reset(void);

static int bishi_init(const rbx_game *g)
{
	uint32_t tile_size, ysize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *tiles = rbx_region("k056832", &tile_size);
	const uint8_t *ymz = rbx_region("ymz", &ysize);
	if (!s_rom || !tiles || !ymz) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&gx, 0, sizeof(gx));
	memset(&b, 0, sizeof(b));
	gx.mw = 0;
	gx.pal_entries = 4096;
	gx.pal_bgr = 1;
	gx.tilecb = GX_TILECB_BISHI;
	gx.mixmode = GX_MIXMODE_LAYERS;
	k056832_init(&gx.k56832, tiles, tile_size, K056832_BPP_8, 1, 0);
	gx_video_set_area(29, 288);   // (set_visarea(29, 29+288-1, 16, 16+224-1))
	m68k_init();
	m68k_bus_swap16(s_rom, s_rom_size);
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x400000, 0x407fff, gx.workram, 0x8000, 1, 1);
	m68k_bus_map(0xb00000, 0xb03fff, gx.palram, 0x4000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	ymz280b_init(&s_ymz, 16934400, ymz, ysize, ymz_irq, NULL);
	const uint32_t rate = ymz280b_rate(&s_ymz);
	s_ymz_cap = (int)(rate / 50) + 64;
	s_ymz_l = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	s_ymz_r = (int32_t *)calloc((size_t)s_ymz_cap, sizeof(int32_t));
	s_port_dsw = rbx_port_find("DSW");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_in = rbx_port_find("INPUTS");
	if (!s_ymz_l || !s_ymz_r || rbx_audio_setup_stereo(rate) || rbx_video_setup(288, VIS_H, 0x10000) || gx_video_init()) {
		rbx_fail("%s: out of memory", g->set);
		bishi_exit();
		return -1;
	}
	for (int i = 0; i < 0x10000; i++) {   // (the bitmap: RGB 565)
		const int r = (i >> 11) & 31, gg = (i >> 5) & 63, bb = i & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (gg << 2 | gg >> 4) << 8 | (bb << 3 | bb >> 2));
	}
	// (video_start: no layer association, the layer offsets)
	k056832_set_layer_association(&gx.k56832, 0);
	static const int offs[4] = { -2, 2, 4, 6 };
	for (int l = 0; l < 4; l++)
		k056832_set_layer_offs(&gx.k56832, l, offs[l], 0);
	bishi_reset();
	return 0;
}

static void bishi_exit(void)
{
	gx_video_exit();
	free(s_ymz_l);
	free(s_ymz_r);
	s_ymz_l = s_ymz_r = NULL;
}

static void bishi_reset(void)
{
	memset(gx.workram, 0, sizeof(gx.workram));
	b.control = b.control2 = 0;
	b.cyc_frac = b.snd_frac = 0;
	gx.frame = 0;
	s_ymz_n = 0;
	ymz280b_reset(&s_ymz);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void bishi_frame(void)
{
	const uint32_t rate = ymz280b_rate(&s_ymz);
	for (int line = 0; line < LINES; line++) {
		b.line = (line + 240) % LINES;   // (the frame from the vblank start)
		if (line == 0)
			gx_video_capture();
		if (b.control & 0x800) {   // (scanline: vblank-out IRQ 3 at 240, vblank-in IRQ 4 at 0)
			if (b.line == 240)
				m68k_bus_hold_irq(3);
			if (b.line == 0)
				m68k_bus_hold_irq(4);
		}
		b.cyc_frac += MAIN_CLOCK;
		const int cyc = (int)(b.cyc_frac / (60 * LINES));
		b.cyc_frac -= (uint64_t)cyc * (60 * LINES);
		b.snd_frac += rate;
		s_ymz_line0 = (uint32_t)s_ymz_n;
		s_ymz_line_n = (uint32_t)(b.snd_frac / (60 * LINES));
		b.snd_frac -= (uint64_t)s_ymz_line_n * (60 * LINES);
		b.line_cyc = cyc;
		m68k_execute(cyc);
		ymz_to((int)(s_ymz_line0 + s_ymz_line_n));
	}
	for (int i = 0; i < s_ymz_n; i++)
		rbx_audio_put_stereo(s_ymz_l[i], s_ymz_r[i]);
	memset(s_ymz_l, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	memset(s_ymz_r, 0, sizeof(int32_t) * (size_t)s_ymz_n);
	s_ymz_n = 0;
	gx.frame++;
}

static int bishi_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	int rc = 0;
	if ((uint32_t)m68k_context_size() > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	rc |= rbx_state_io(ctx, "bishi_cpu", cpu, (uint32_t)m68k_context_size());
	rc |= rbx_state_io(ctx, "bishi_bus", &m68k_bus.irq_pending, sizeof(uint32_t) * 2);
	rc |= rbx_state_io(ctx, "gx", &gx, (uint32_t)offsetof(gx_t, state_end));
	rc |= k056832_state(&gx.k56832, ctx, load);
	rc |= rbx_state_io(ctx, "bishi", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "bishi_ymz", &s_ymz, (uint32_t)YMZ280B_STATE_SIZE);
	if (!rc && load)
		m68k_set_context(cpu);
	return rc ? -1 : 0;
}

static int bishi_save(void *ctx) { return bishi_state(ctx, 0); }
static int bishi_load(void *ctx) { return bishi_state(ctx, 1); }
static int bishi_supported(const char *machine) { return !strcmp(machine, "bishi") || !strcmp(machine, "bishi2p"); }

const rbx_driver rbx_drv_bishi = { "Konami Bishi Bashi", bishi_init, bishi_exit, bishi_reset, bishi_frame, bishi_save,
                                   bishi_load, bishi_supported, gx_render, NULL };

/*  Data East 18 Holes Pro Golf. Ported from MAME src/mame/dataeast/progolf.cpp (BSD-3-Clause, copyright-holders:
    Angelo Salese, Roberto Zandona').
    A DECO 222 at 1.32 MHz (no vblank interrupt: it polls; the coins on its NMI), a 128x32 scrolled map of 8x8 tiles
    (the tile ROM readable through two windows over the map RAM), a three-plane 256x256 framebuffer written through a
    plane-select "RMW" window, an MC6845 (timing only), a 6502 at 441 kHz with two AY-3-8910 and a latch on its IRQ.
    Not supported (MAME: not working): progolfa.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/ay8910.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define XTAL 10595000
#define FRAME_MHZ 57000          // set_refresh_hz(57), set_size(256, 256), the whole frame visible
#define LINES 256
#define VBL_LINES 45             // vblank 3072 us (after the frame's end)

static uint8_t *s_rom, *s_srom, *s_gfx;
static uint32_t s_rom_size, s_srom_size, s_gfx_size;
static uint8_t s_ram[0x6000], s_vram[0x1000], s_fb[0x6000], s_sram[0x1000];
static uint8_t s_plane, s_view0, s_view1, s_scroll_lo, s_scroll_hi, s_flip;
static uint8_t s_latch;
static int s_latch_pending, s_coin_nmi;
static int s_port_vbl, s_port_p1, s_port_p2, s_port_coins, s_port_dsw1, s_port_dsw2;
static int s_line;
static uint32_t s_frame;
static int64_t s_cpu_target, s_snd_target;
static uint64_t s_cpu_frac, s_snd_frac, s_ay_frac;
static m6502_t s_cpu, s_snd;
static ay8910_t s_ay[2];
static int32_t *s_mix;
static int s_mix_cap;
static uint32_t s_rgb[96];
static rbx_gfx s_gfxchar;
static struct {
	uint8_t vram[0x1000], fb[0x6000], scroll_lo, scroll_hi;
} *s_snap;

// ---- sound ----
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x1000) return s_sram[a];
	if (a >= 0x4000 && a < 0x5000) return ay8910_data_r(&s_ay[0]);
	if (a >= 0x6000 && a < 0x7000) return ay8910_data_r(&s_ay[1]);
	if (a >= 0x8000 && a < 0x9000) return s_latch;
	if (a >= 0xf000) return s_srom[a];
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x1000) s_sram[a] = v;
	else if (a >= 0x4000 && a < 0x5000) ay8910_data_w(&s_ay[0], v);
	else if (a >= 0x5000 && a < 0x6000) ay8910_address_w(&s_ay[0], v);
	else if (a >= 0x6000 && a < 0x7000) ay8910_data_w(&s_ay[1], v);
	else if (a >= 0x7000 && a < 0x8000) ay8910_address_w(&s_ay[1], v);
	else if (a >= 0x8000 && a < 0x9000) {   // acknowledge_w (separate acknowledge)
		s_latch_pending = 0;
		m6502_set_irq(&s_snd, 0);
	}
}

// ---- main bus ----
static uint8_t charram_r(uint16_t o)   // the planes not deselected ORed
{
	uint8_t res = 0;
	const uint8_t plane = s_plane ^ 7;
	for (int i = 0; i < 3; i++)
		if (!((plane >> i) & 1))
			res |= s_fb[o | (i * 0x2000)];
	return res;
}
static void charram_w(uint16_t o, uint8_t v)
{
	const uint8_t plane = s_plane ^ 7;
	for (int i = 0; i < 3; i++) {
		const uint16_t f = (uint16_t)(o | (i * 0x2000));
		if (s_plane == 0)
			s_fb[f] = 0;
		else if (!((plane >> i) & 1))
			s_fb[f] = v;
	}
}

static uint8_t main_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x6000) return s_ram[a];
	if (a < 0x8000) return charram_r((uint16_t)(a - 0x6000));
	if (a < 0x8800) return s_view0 ? s_gfx[(s_view0 - 1) * 0x1000 + (a - 0x8000)] : s_vram[a - 0x8000];
	if (a < 0x9000) return s_view1 ? s_gfx[(s_view1 - 1) * 0x1000 + 0x800 + (a - 0x8800)] : s_vram[a - 0x8000];
	switch (a) {
	case 0x9000: return (uint8_t)rbx_port_read(s_port_coins);
	case 0x9200: return (uint8_t)rbx_port_read(s_port_p1);
	case 0x9400: return (uint8_t)rbx_port_read(s_port_p2);
	case 0x9600:
		rbx_port_custom(s_port_vbl, 0x80, s_line < VBL_LINES ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_vbl);
	case 0x9800: return (uint8_t)rbx_port_read(s_port_dsw1);
	case 0x9a00: return (uint8_t)rbx_port_read(s_port_dsw2);
	}
	return a >= 0xb000 ? s_rom[a] : 0;
}
static void main_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x6000) s_ram[a] = v;
	else if (a < 0x8000) charram_w((uint16_t)(a - 0x6000), v);
	else if (a < 0x9000) s_vram[a - 0x8000] = v;
	else if (a == 0x9000) {   // video_bank_w: the plane select, the windows' ROM banks
		s_plane = v & 7;
		const int sel = (v >> 6) & 1, bank = (v >> 4) & 3;
		s_view0 = (uint8_t)(sel ? 0 : bank);
		s_view1 = (uint8_t)(sel ? bank : 0);
	} else if (a == 0x9200) s_scroll_hi = v;
	else if (a == 0x9400) s_scroll_lo = v;
	else if (a == 0x9600) s_flip = v & 1;
	else if (a == 0x9a00) {   // the latch (its pending data: the sound IRQ)
		s_latch = v;
		s_latch_pending = 1;
		m6502_set_irq(&s_snd, 1);
	}
}

// the coins (and the service) on the NMI: MAME's coin_inserted (newval ? CLEAR : ASSERT)
static void coins(void)
{
	const uint32_t c = rbx_port_read(s_port_coins), d = rbx_port_read(s_port_dsw1);
	const int nmi = (c & 0xc0) != 0xc0 || (d & 0x40);
	if (nmi != s_coin_nmi)
		m6502_set_nmi(&s_cpu, nmi);
	s_coin_nmi = nmi;
}

// ---- video ----
static uint16_t (*s_scr_hp)[256 * 256];
#define s_scr (*s_scr_hp)

static void pg_render(void)
{
	for (int i = 0; i < 96; i++)
		rbx_video.palette[i] = s_rgb[i];
	const int scroll = s_snap->scroll_lo | ((s_snap->scroll_hi & 3) << 8);
	for (int x = 0; x < 128; x++)   // the map, opaque, colour 1 (twice: the wrap-around)
		for (int y = 0; y < 32; y++) {
			const uint8_t *t = s_gfxchar.pix + (size_t)(s_snap->vram[x * 32 + y] % s_gfxchar.count) * 64;
			for (int w = 0; w < 2; w++) {
				const int sx = 256 - x * 8 + scroll - w * 1024;
				if (sx <= -8 || sx >= 256)
					continue;
				for (int yy = 0; yy < 8; yy++)
					for (int xx = 0; xx < 8; xx++) {
						const int dx = sx + xx;
						if (dx >= 0 && dx < 256)
							s_scr[(y * 8 + yy) * 256 + dx] = (uint16_t)(8 + t[yy * 8 + xx]);
					}
			}
		}
	const uint8_t *fb = s_snap->fb;   // the framebuffer: 8x8 cells, pens 1-7 over the map
	for (int y = 0; y < 256; y += 8)
		for (int x = 0; x < 256; x += 8) {
			const uint32_t o = (uint32_t)(((y >> 3) + (x >> 3) * 32) << 3);
			for (int xi = 0; xi < 8; xi++) {
				const int rx = 256 - x + xi;
				if (rx > 255)
					continue;
				for (int yi = 0; yi < 8; yi++) {
					const int pen = ((fb[o + yi] >> (7 - xi)) & 1) | ((fb[o + yi + 0x2000] >> (7 - xi)) & 1) << 1 |
					                ((fb[o + yi + 0x4000] >> (7 - xi)) & 1) << 2;
					if (pen)
						s_scr[(y + yi) * 256 + rx] = (uint16_t)pen;
				}
			}
		}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < 256; y++)
		memcpy(bm + y * pitch, s_scr + y * 256, 256 * 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->fb, s_fb, sizeof(s_fb));
	s_snap->scroll_lo = s_scroll_lo;
	s_snap->scroll_hi = s_scroll_hi;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void pg_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_fb, 0, sizeof(s_fb));
	memset(s_sram, 0, sizeof(s_sram));
	s_plane = s_view0 = s_view1 = s_scroll_lo = s_scroll_hi = s_flip = 0;
	s_latch = 0;
	s_latch_pending = 0;
	s_coin_nmi = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	s_line = 0;
	s_frame = 0;
	s_cpu_target = s_snd_target = 0;
	s_cpu_frac = s_snd_frac = s_ay_frac = 0;
}

static void pg_exit(void);

static int pg_init(const rbx_game *g)
{
	if (!s_scr_hp && !(s_scr_hp = calloc(1, sizeof(*s_scr_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_gfx = rbx_region("gfx1", &s_gfx_size);
	const uint8_t *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !s_gfx || s_gfx_size < 0x3000 || !prom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// gfx_8x8x3_planar (RGN_FRAC(1,3))
		static uint32_t x8[8], y8[8];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
		}
		const uint32_t t = s_gfx_size / 3 * 8;
		const uint32_t p3[3] = { 2 * t, t, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 3, p3, x8, y8, 8 * 8 };
		if (rbx_gfx_decode(&s_gfxchar, &l8, s_gfx, s_gfx_size / 3)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			return -1;
		}
	}
	for (int i = 0; i < 96 && (uint32_t)i < ps; i++) {   // palette_init
		const uint8_t p = prom[i];
		const int r = 0x21 * (p & 1) + 0x47 * ((p >> 1) & 1) + 0x97 * ((p >> 2) & 1);
		const int gg = 0x21 * ((p >> 3) & 1) + 0x47 * ((p >> 4) & 1) + 0x97 * ((p >> 5) & 1);
		const int b = 0x47 * ((p >> 6) & 1) + 0x97 * ((p >> 7) & 1);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_snap = calloc(1, sizeof(*s_snap));
	// AY-3-8910 1.32 MHz 0.23 each (MAME's default legacy output)
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, XTAL / 8, AY8910_LEGACY_OUTPUT, 59, 4, 0);
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_snap || !s_mix || rbx_audio_setup(ay8910_rate(&s_ay[0])) || rbx_video_setup(256, 256, 96)) {
		rbx_fail("%s: out of memory", g->set);
		pg_exit();
		return -1;
	}
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.deco222 = 1;
	s_cpu.read = main_r;
	s_cpu.write = main_w;
	for (int p = 0xb0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + p * 256;
	for (int p = 0; p < 0x60; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	m6502_init(&s_snd, M6502_NMOS);
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	s_port_vbl = rbx_port_find("VBLANK");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_coins = rbx_port_find("COINS");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_dsw2 = rbx_port_find("DSW2");
	pg_reset();
	return 0;
}

static void pg_exit(void)
{
	rbx_gfx_free(&s_gfxchar);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_scr_hp);
	s_scr_hp = NULL;
}

static void pg_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coins();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 0)   // (the frame's end: the screen update, vblank from here)
			vblank_capture();
		s_cpu_target += rbx_line_cycles(&s_cpu_frac, XTAL / 8, FRAME_MHZ, LINES);
		m6502_run_until(&s_cpu, s_cpu_target);
		s_snd_target += rbx_line_cycles(&s_snd_frac, XTAL / 24, FRAME_MHZ, LINES);
		m6502_run_until(&s_snd, s_snd_target);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, XTAL / 8, FRAME_MHZ, LINES);
		const int n0 = ay8910_render(&s_ay[0], clocks, s_mix, s_mix_cap);
		const int n1 = ay8910_render(&s_ay[1], clocks, s_mix, s_mix_cap);
		const int n = n0 > n1 ? n0 : n1;
		for (int k = 0; k < n; k++) {
			rbx_audio_put(s_mix[k]);
			s_mix[k] = 0;
		}
	}
	s_frame++;
}

static int pg_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	int64_t tg[2] = { s_cpu_target, s_snd_target };
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_ay_frac };
	int32_t misc[12] = { (int32_t)s_frame, s_plane, s_view0, s_view1, s_scroll_lo, s_scroll_hi, s_flip, s_latch,
	                     s_latch_pending, s_coin_nmi, 0, 0 };
	uint8_t ay[2][256];
	const uint32_t ays = ay8910_state_size();
	if (!load)
		for (int i = 0; i < 2; i++)
			memcpy(ay[i], &s_ay[i], ays);
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "snd", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "fb", s_fb, sizeof(s_fb));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "tg", tg, sizeof(tg));
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		for (int i = 0; i < 2; i++)
			memcpy(&s_ay[i], ay[i], ays);
		s_cpu_target = tg[0];
		s_snd_target = tg[1];
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ay_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_plane = (uint8_t)misc[1];
		s_view0 = (uint8_t)misc[2];
		s_view1 = (uint8_t)misc[3];
		s_scroll_lo = (uint8_t)misc[4];
		s_scroll_hi = (uint8_t)misc[5];
		s_flip = (uint8_t)misc[6];
		s_latch = (uint8_t)misc[7];
		s_latch_pending = misc[8];
		s_coin_nmi = misc[9];
	}
	return rc ? -1 : 0;
}

static int pg_save(void *ctx) { return pg_state(ctx, 0); }
static int pg_load(void *ctx) { return pg_state(ctx, 1); }
static int pg_supported(const char *machine) { return !strcmp(machine, "progolf"); }

const rbx_driver rbx_drv_progolf = { "DECO Pro Golf", pg_init, pg_exit, pg_reset, pg_frame, pg_save, pg_load,
                                     pg_supported, pg_render, NULL };

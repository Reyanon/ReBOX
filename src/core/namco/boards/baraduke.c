/*  Namco Baraduke / Metro-Cross: an MC6809E, an HD63701 (sound and inputs), one CUS42/CUS43 tilemap pair, a fixed
    text layer, the CUS39/CUS48 sprites and the CUS30.
    Ported from MAME 0.289 src/mame/namco/baraduke.cpp and namco_cus4xtmap.cpp (BSD-3-Clause, copyright-holders:
    Manuel Abadia, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the lamps, the coin counters / lockout, the watchdog.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/namco_wsg.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 288
#define VIS_Y0 16
#define VIS_H 224
#define FRAME_MHZ 60606         // 6.144 MHz / (384 x 264)
#define FRAME_CYCLES 25344      // (the CPUs' clocks a frame)
#define CPU_CLOCK 1536000       // 49.152 MHz / 32 (and the MCU's E clock: / 8 / 4)
#define SND_RATE 48000

// ---- the set's configuration ----
static uint8_t *s_cpu1, *s_mcurom, *s_mcusub, *s_chars, *s_tiles, *s_sprrom, *s_proms;
static uint32_t s_cpu1_size, s_mcurom_size, s_mcusub_size, s_chars_size, s_tiles_size, s_sprrom_size, s_proms_size;
static uint8_t *s_txpix, *s_tilepix, *s_sprpix;   // decoded, a byte a pixel
static uint32_t s_tx_count, s_spr_count;
static uint32_t s_pal[2048];
static int s_port_in[3], s_port_dsw[2], s_port_edge;

static m6809_t s_cpu;
static m6801 s_mcu;
static namco_wsg_t s_wsg;

// ---- the board's state (saved) ----
static struct {
	uint8_t copy_sprites, inport_sel, pad[2];
	uint16_t xscroll[2];
	uint8_t yscroll[2];
	int64_t t;
	uint64_t snd_frac;
	uint8_t spr[0x2000];
	uint8_t vram[0x2000];
	uint8_t text[0x800];
	uint8_t mcuram[0x800];
} b;

// ---- the drawing's copy ----
typedef struct {
	uint8_t vram[0x2000];
	uint8_t spr[0x800];   // (0x1800-0x1fff)
	uint8_t text[0x800];
	uint16_t xscroll[2];
	uint8_t yscroll[2];
} snap_t;
static snap_t *s_snap;
static uint8_t *s_pri;
static int32_t s_mix[16];   // (a timeslice's samples)

static int s_sync;   // (the main CPU asked for a sync: its timeslice ends after this instruction)

// ---- the main CPU (main_map) ----
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x4000 && a < 0x4400) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	return 0;
}

static void scroll_w(int layer, int off, uint8_t v)
{
	switch (off) {
	case 0: b.xscroll[layer] = (uint16_t)((b.xscroll[layer] & 0xff) | (v << 8)); break;
	case 1: b.xscroll[layer] = (uint16_t)((b.xscroll[layer] & 0xff00) | v); break;
	default: b.yscroll[layer] = v; break;
	}
}

static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x2000) {
		b.spr[a] = v;
		if (a == 0x1ff2)   // (the sprite chip buffers the list at the next vblank)
			b.copy_sprites = 1;
	} else if (a >= 0x4000 && a < 0x4400)
		namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a == 0x8800) {   // irq_ack_w (MAME: a synced input line change, which ends the timeslice here)
		m6809_set_irq(&s_cpu, 0);
		s_sync = 1;
	} else if (a >= 0xb000 && a <= 0xb002)
		scroll_w(0, a - 0xb000, v);
	else if (a >= 0xb004 && a <= 0xb006)
		scroll_w(1, a - 0xb004, v);
}

// ---- the MCU (mcu_map) ----
static uint8_t mcu_r(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x1000 && a < 0x1400) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	if (a >= 0x8000 && a < 0xc000) return s_mcusub[rbx_wrap(a - 0x8000, s_mcusub_size)];
	if (a >= 0xc000 && a < 0xc800) return b.mcuram[a - 0xc000];
	if (a >= 0xf000) return s_mcurom[a & 0xfff];
	return 0;
}

static void mcu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x1000 && a < 0x1400) namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a >= 0xc000 && a < 0xc800) b.mcuram[a - 0xc000] = v;
}

static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	if (port != 1)
		return 0xff;
	const uint8_t d0 = (uint8_t)rbx_port_read(s_port_dsw[0]), d1 = (uint8_t)rbx_port_read(s_port_dsw[1]);
	switch (b.inport_sel) {   // inputport_r
	case 0: return (uint8_t)((d0 & 0xf8) >> 3);
	case 1: return (uint8_t)(((d0 & 0x07) << 2) | ((d1 & 0xc0) >> 6));
	case 2: return (uint8_t)((d1 & 0x3e) >> 1);
	case 3: return (uint8_t)(((d1 & 0x01) << 4) | (rbx_port_read(s_port_edge) & 0x0f));
	case 4: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 5: return (uint8_t)rbx_port_read(s_port_in[2]);
	case 6: return (uint8_t)rbx_port_read(s_port_in[1]);
	default: return 0xff;
	}
}

static void mcu_port_w(void *u, int port, uint8_t data, uint8_t ddr)
{
	(void)u;
	(void)ddr;
	if (port == 1 && (data & 0xe0) == 0x60)   // inputport_select_w (0xc0: the coin counters and lockout)
		b.inport_sel = data & 7;
}

// ---- drawing ----
static int flip_of(const snap_t *sn) { return sn->spr[0x7f6] & 1; }

// one line of a CUS42/43 layer (set_offset(-26, -227, -9, 9), adj { 0, 2 }; the visible area 0-287 x 16-239)
static void draw_tilemap(const snap_t *sn, int layer, int opaque, int prival, int y, uint16_t *d, uint8_t *pri)
{
	const int dx = -26 + (layer ? 2 : 0), flip = flip_of(sn);
	const int ty = flip ? (230 - y - sn->yscroll[layer]) & 0xff : (y + VIS_Y0 + 9 + sn->yscroll[layer]) & 0xff;
	const int sx = flip ? 514 + dx - sn->xscroll[layer] : sn->xscroll[layer] - dx;
	const uint8_t *vram = sn->vram + (layer << 12) + (ty >> 3) * 128;
	const uint8_t *gfx = s_tilepix + (uint32_t)layer * 1024 * 64 + (uint32_t)(ty & 7) * 8;
	for (int x = 0; x < VIS_W; x++) {
		const int tx = (flip ? sx - x : sx + x) & 0x1ff;
		const uint8_t *e = vram + (tx >> 3) * 2;
		const uint8_t pen = gfx[(uint32_t)(e[0] + ((e[1] & 3) << 8)) * 64 + (uint32_t)(tx & 7)];
		if (opaque || pen != 7) {
			d[x] = (uint16_t)(e[1] * 8 + pen);
			pri[x] |= (uint8_t)prival;
		}
	}
}

// the 36 x 28 text layer (tx_tilemap_scan), transparent pen 3
static void draw_text(const snap_t *sn, int y, uint16_t *d)
{
	const int flip = flip_of(sn);
	const int ly = flip ? 223 - y : y;
	const int row = ly >> 3;
	for (int x = 0; x < VIS_W; x++) {
		const int lx = flip ? VIS_W - 1 - x : x;
		const int r = row + 2, c = (lx >> 3) - 2;
		const int idx = (c & 0x20) ? r + ((c & 0x1f) << 5) : c + (r << 5);
		const uint8_t pen = s_txpix[rbx_wrap(sn->text[idx], s_tx_count) * 64 + (uint32_t)((ly & 7) * 8 + (lx & 7))];
		if (pen != 3)
			d[x] = (uint16_t)((((sn->text[idx + 0x400] << 2) & 0x1ff) << 2) + pen);
	}
}

static void draw_sprites(const snap_t *sn)
{
	static const int gfx_offs[2][2] = { { 0, 1 }, { 2, 3 } };
	const uint8_t *spr = sn->spr;   // (0x1800-based)
	const int xoffs = spr[0x7f5] - 256 * (spr[0x7f4] & 1), yoffs = spr[0x7f7];
	const int flip = flip_of(sn);
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int o = 0x800 - 32; o >= 0; o -= 16) {   // (the last is not a sprite)
		const uint8_t *s = spr + o;
		const uint32_t pmask = ((s[10] & 1) ? 0 : 0xccccu) | 0x80000000u;   // (GFX_PMASK_2; a sprite over a sprite: never)
		const int attr1 = s[10], attr2 = s[14];
		int color = s[12];
		int sx = s[13] + (color & 1) * 256, sy = 240 - s[15];
		int flipx = (attr1 >> 5) & 1, flipy = attr2 & 1;
		const int sizex = attr1 >> 7, sizey = (attr2 >> 2) & 1;
		int sprite = s[11] * 4;
		if ((attr1 & 0x10) && !sizex) sprite += 1;
		if ((attr2 & 0x10) && !sizey) sprite += 2;
		color >>= 1;
		sx += xoffs;
		sy -= yoffs;
		sy -= 16 * sizey;
		if (flip) {
			sx = 496 + 3 - 16 * sizex - sx;
			sy = 240 - 16 * sizey - sy;
			flipx ^= 1;
			flipy ^= 1;
		}
		for (int yy = 0; yy <= sizey; yy++)
			for (int xx = 0; xx <= sizex; xx++) {
				const uint32_t code = rbx_wrap((uint32_t)(sprite + gfx_offs[yy ^ (sizey * flipy)][xx ^ (sizex * flipx)]), s_spr_count);
				const uint8_t *tile = s_sprpix + code * 256;
				const int x0 = -71 + ((sx + 16 * xx) & 0x1ff), y0 = 1 + ((sy + 16 * yy) & 0xff) - VIS_Y0;
				for (int py = 0; py < 16; py++) {
					const int ay = y0 + py;
					if (ay < 0 || ay >= VIS_H)
						continue;
					const uint8_t *src = tile + (flipy ? 15 - py : py) * 16;
					uint16_t *d = bm + ay * pitch;
					uint8_t *pri = s_pri + ay * VIS_W;
					for (int px = 0; px < 16; px++) {
						const int ax = x0 + px;
						if (ax < 0 || ax >= VIS_W)
							continue;
						const uint8_t pen = src[flipx ? 15 - px : px];
						if (pen == 15)
							continue;
						if (!((1u << (pri[ax] & 0x1f)) & pmask))
							d[ax] = (uint16_t)(color * 16 + pen);
						pri[ax] = 31;
					}
				}
			}
	}
}

static void bd_render(void)
{
	const snap_t *sn = s_snap;
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int back = ((sn->xscroll[0] & 0x0e00) >> 9) == 6 ? 1 : 0;
	memset(s_pri, 0, VIS_W * VIS_H);
	for (int y = 0; y < VIS_H; y++) {
		draw_tilemap(sn, back, 1, 1, y, bm + y * pitch, s_pri + y * VIS_W);
		draw_tilemap(sn, back ^ 1, 0, 2, y, bm + y * pitch, s_pri + y * VIS_W);
	}
	draw_sprites(sn);
	for (int y = 0; y < VIS_H; y++)
		draw_text(sn, y, bm + y * pitch);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	memcpy(s_snap->spr, b.spr + 0x1800, sizeof(s_snap->spr));
	memcpy(s_snap->text, b.text, sizeof(b.text));
	memcpy(s_snap->xscroll, b.xscroll, sizeof(b.xscroll));
	memcpy(s_snap->yscroll, b.yscroll, sizeof(b.yscroll));
	rbx_video_frame_done();
}

// ---- frame ----
// MAME's scheduling: timeslices of the 6 kHz quantum (256 clocks) from the last sync point; a synced input line change
// (the main CPU's IRQ acknowledge) ends the timeslice where it happens. The main CPU runs first, then the MCU up to the
// same time. MAME starts the machine at the vblank's start (line 240), so a frame here runs from there to the next
#define QUANTUM 256
static void bd_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	const int64_t end = b.t + FRAME_CYCLES;
	while (b.t < end) {
		int64_t target = b.t + QUANTUM < end ? b.t + QUANTUM : end;
		s_sync = 0;
		while (s_cpu.cycles < target) {
			m6809_step(&s_cpu);
			if (s_sync) {
				if (s_cpu.cycles < target)
					target = s_cpu.cycles;
				break;
			}
		}
		m6801_execute(&s_mcu, (int)(target - b.t));
		b.snd_frac += (uint64_t)(target - b.t);
		b.t = target;
		const int k = (int)(b.snd_frac / (CPU_CLOCK / SND_RATE));   // (48 kHz: a sample every 32 clocks)
		if (k > 0) {
			b.snd_frac -= (uint64_t)k * (CPU_CLOCK / SND_RATE);
			memset(s_mix, 0, (size_t)k * sizeof(s_mix[0]));
			namco_wsg_render(&s_wsg, s_mix, NULL, k, SND_RATE);
			for (int i = 0; i < k; i++)
				rbx_audio_put(s_mix[i]);
		}
	}
	// vblank: the frame drawn, then the sprites copied and the IRQs
	vblank_capture();
	if (b.copy_sprites) {
		for (int i = 0x1800; i < 0x2000; i += 16)
			for (int j = 10; j < 16; j++)
				b.spr[i + j] = b.spr[i + j - 6];
		b.copy_sprites = 0;
	}
	m6809_set_irq(&s_cpu, 1);
	m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, M6801_HOLD_LINE);
}

// ---- init / reset ----
static void bd_reset(void)
{
	b.copy_sprites = b.inport_sel = 0;
	memset(b.xscroll, 0, sizeof(b.xscroll));
	memset(b.yscroll, 0, sizeof(b.yscroll));
	b.t = 0;
	b.snd_frac = 0;
	m6809_reset(&s_cpu);
	s_cpu.cycles = 4;   // (MAME: the reset vector fetch, INTERRUPT_VECTOR, takes 4 clocks)
	m6801_reset(&s_mcu);
	namco_wsg_init(&s_wsg, NAMCO_WSG_TYPE_CUS30, 49152000 / 2048, 0, NULL, 256);
}

static void bd_exit(void);

static void map_pages(m6809_t *c, int first, int n, uint8_t *mem, int writable)
{
	for (int p = 0; p < n; p++) {
		c->rpage[first + p] = mem + p * 256;
		c->wpage[first + p] = writable ? mem + p * 256 : NULL;
	}
}

static int bd_init(const rbx_game *g)
{
	s_cpu1 = rbx_region("maincpu", &s_cpu1_size);
	s_mcurom = rbx_region("mcu", &s_mcurom_size);
	s_mcusub = rbx_region("mcusub", &s_mcusub_size);
	s_chars = rbx_region("chars", &s_chars_size);
	s_tiles = rbx_region("tiles", &s_tiles_size);
	s_sprrom = rbx_region("sprites", &s_sprrom_size);
	s_proms = rbx_region("proms", &s_proms_size);
	if (!s_cpu1 || s_cpu1_size < 0x10000 || !s_mcurom || s_mcurom_size < 0x1000 || !s_mcusub || !s_mcusub_size || !s_chars ||
	    s_chars_size < 16 || !s_tiles || s_tiles_size < 0x10000 || !s_sprrom || s_sprrom_size < 128 || !s_proms || s_proms_size < 0x1000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// the palette: 2048 colours (2.2k / 1k / 470 / 220 ohm, scaled to 255)
	{
		const double g4[4] = { 1.0 / 2200, 1.0 / 1000, 1.0 / 470, 1.0 / 220 };
		const double sum = g4[0] + g4[1] + g4[2] + g4[3];
		for (int i = 0; i < 2048; i++) {
			const int bits[3] = { s_proms[i | 0x800] & 0xf, s_proms[i] & 0xf, s_proms[i] >> 4 };
			int rgb[3];
			for (int k = 0; k < 3; k++) {
				double v = 0;
				for (int j = 0; j < 4; j++)
					if (bits[k] & (1 << j))
						v += 255.0 * g4[j] / sum;
				rgb[k] = (int)(v + 0.5);
				if (rgb[k] > 255) rgb[k] = 255;
			}
			s_pal[i] = (uint32_t)(rgb[0] << 16 | rgb[1] << 8 | rgb[2]);
		}
	}
	// init_baraduke: unpack the third tile ROM (0x8000-0xbfff) into the 0x8000-0xffff plane
	{
		uint8_t *rom = s_tiles + 0x8000;
		for (int i = 0x2000; i < 0x4000; i++) {
			rom[i + 0x2000] = rom[i];
			rom[i + 0x4000] = (uint8_t)(rom[i] << 4);
		}
		for (int i = 0; i < 0x2000; i++)
			rom[i + 0x2000] = (uint8_t)(rom[i] << 4);
	}
	s_tx_count = s_chars_size / 16;
	s_spr_count = s_sprrom_size / 128;
	s_txpix = (uint8_t *)malloc((size_t)s_tx_count * 64);
	s_tilepix = (uint8_t *)malloc((size_t)2 * 1024 * 64);
	s_sprpix = (uint8_t *)malloc((size_t)s_spr_count * 256);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	s_pri = (uint8_t *)calloc(1, VIS_W * VIS_H);
	if (!s_txpix || !s_tilepix || !s_sprpix || !s_snap || !s_pri) {
		bd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (uint32_t t = 0; t < s_tx_count; t++)   // text_layout: 2bpp, planes at bits 0 / 4, x 0-3 in the second 8 bytes
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint8_t v = s_chars[t * 16 + (x < 4 ? 8 : 0) + (uint32_t)y];
				const int k = x & 3;
				s_txpix[t * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)((((v >> (7 - k)) & 1) << 1) | ((v >> (3 - k)) & 1));
			}
	for (uint32_t gn = 0; gn < 2; gn++)   // tile_layout: 3bpp, the third plane 0x8000 on
		for (uint32_t t = 0; t < 1024; t++)
			for (int y = 0; y < 8; y++)
				for (int x = 0; x < 8; x++) {
					const uint32_t base = gn * 0x4000 + t * 16 + (uint32_t)(y * 2 + (x >> 2));
					const int k = x & 3;
					const uint8_t v = s_tiles[base], h = s_tiles[0x8000 + base];
					s_tilepix[(gn * 1024 + t) * 64 + (uint32_t)(y * 8 + x)] =
						(uint8_t)((((h >> (7 - k)) & 1) << 2) | (((v >> (7 - k)) & 1) << 1) | ((v >> (3 - k)) & 1));
				}
	for (uint32_t t = 0; t < s_spr_count; t++)   // gfx_16x16x4_packed_msb
		for (int i = 0; i < 128; i++) {
			const uint8_t v = s_sprrom[t * 128 + (uint32_t)i];
			s_sprpix[t * 256 + (uint32_t)i * 2] = v >> 4;
			s_sprpix[t * 256 + (uint32_t)i * 2 + 1] = v & 15;
		}
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dsw[0] = rbx_port_find("DSWA");
	s_port_dsw[1] = rbx_port_find("DSWB");
	s_port_edge = rbx_port_find("EDGE");

	memset(&b, 0, sizeof(b));
	m6809_init(&s_cpu);
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	map_pages(&s_cpu, 0x00, 0x1f, b.spr, 1);
	map_pages(&s_cpu, 0x1f, 0x01, b.spr + 0x1f00, 0);   // (0x1ff2: the sprite copy request)
	map_pages(&s_cpu, 0x20, 0x20, b.vram, 1);
	map_pages(&s_cpu, 0x48, 0x08, b.text, 1);
	for (int p = 0x60; p < 0x100; p++)
		s_cpu.rpage[p] = s_cpu1 + p * 256;
	memset(&s_mcu, 0, sizeof(s_mcu));
	s_mcu.read = mcu_r;
	s_mcu.write = mcu_w;
	s_mcu.port_r = mcu_port_r;
	s_mcu.port_w = mcu_port_w;
	s_mcu.rom = s_mcurom;
	s_mcu.rom_base = 0xf000;
	s_mcu.rom_size = 0x1000;
	s_mcu.type = M6801_TYPE_HD63701V0;
	m6801_init(&s_mcu);
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 2048)) {
		bd_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	bd_reset();
	return 0;
}

static void bd_exit(void)
{
	free(s_txpix);
	free(s_tilepix);
	free(s_sprpix);
	free(s_snap);
	free(s_pri);
	s_txpix = s_tilepix = s_sprpix = NULL;
	s_snap = NULL;
	s_pri = NULL;
}

static int bd_state(void *ctx, int load)
{
	(void)load;
	int rc = 0;
	rc |= rbx_state_io(ctx, "board", &b, sizeof(b));
	rc |= rbx_state_io(ctx, "cpu", &s_cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mcu", m6801_state_data(&s_mcu), m6801_state_size());
	rc |= rbx_state_io(ctx, "wsg", &s_wsg, (uint32_t)NAMCO_WSG_STATE_SIZE);
	return rc;
}

static int bd_save(void *ctx) { return bd_state(ctx, 0); }
static int bd_load(void *ctx) { return bd_state(ctx, 1); }
static int bd_supported(const char *machine) { return !strcmp(machine, "baraduke"); }

const rbx_driver rbx_drv_baraduke = { "Namco Baraduke", bd_init, bd_exit, bd_reset, bd_frame, bd_save, bd_load,
                                      bd_supported, bd_render, NULL };

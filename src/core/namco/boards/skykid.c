/*  Namco Sky Kid / Dragon Buster: an MC6809E, an HD63701 (sound and inputs), a scrolling background, a fixed text
    layer, the Mappy-type sprites and the CUS30.
    Ported from MAME 0.289 src/mame/namco/skykid.cpp (BSD-3-Clause, copyright-holders: Manuel Abadia).
    This file is part of ReBOX. GPL-2.0-or-later.

    Not done: the LEDs, the coin counters / lockout, the watchdog.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/namco_wsg.h"

#include <stdlib.h>
#include <string.h>

#define VIS_W 288
#define VIS_H 224
#define LINES 264               // (the frame cut into 264 slices like the other Namco boards of the time; no raster)
#define FRAME_MHZ 60606
#define CPU_CLOCK 1536000       // 49.152 MHz / 32 (and the MCU's E clock: / 8 / 4)
#define SLICES 4
#define SND_RATE 48000

// ---- the set's configuration ----
static uint8_t *s_cpu1, *s_mcurom, *s_mcusub, *s_chars, *s_tiles, *s_sprrom, *s_proms;
static uint32_t s_cpu1_size, s_mcurom_size, s_mcusub_size, s_chars_size, s_tiles_size, s_sprrom_size, s_proms_size;
static uint8_t *s_txpix, *s_bgpix, *s_sprpix;   // decoded, a byte a pixel
static uint32_t s_tx_count, s_bg_count, s_spr_count;
static uint32_t s_pal[0x500];
static uint8_t s_spr_trans[64];   // a sprite colour's transparent pens (transpen_mask(.., 0xff))
static int s_port_pl[2], s_port_dsw[2], s_port_button2, s_port_system;

static m6809_t s_cpu;
static m6801 s_mcu;
static namco_wsg_t s_wsg;

// ---- the board's state (saved) ----
static struct {
	uint8_t priority, flip, rombank, main_irq_mask, mcu_irq_mask, mcu_reset, inport_sel, pad;
	uint16_t scroll_x, scroll_y;
	int64_t t;
	uint64_t cpu_frac, snd_frac;
	uint8_t vram[0x1000];
	uint8_t text[0x800];
	uint8_t spr[0x1800];
	uint8_t mcuram[0x800];
} b;

// ---- the drawing's copy ----
typedef struct {
	uint8_t vram[0x1000];
	uint8_t text[0x800];
	uint8_t spr[0x1800];
	uint16_t scroll_x, scroll_y;
	uint8_t priority, flip;
} snap_t;
static snap_t *s_snap;
static int32_t s_mix[SND_RATE / 50 / LINES + 8];

// ---- the main CPU (main_map) ----
static void rom_bank(void)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu1 + 0x10000 + (uint32_t)b.rombank * 0x2000 + (uint32_t)p * 256;
}

static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x6800 && a < 0x6c00) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	return 0;
}

static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x6000 && a < 0x6100)
		b.scroll_y = (uint16_t)(a - 0x6000);
	else if (a >= 0x6200 && a < 0x6400)
		b.scroll_x = (uint16_t)(a - 0x6200);
	else if (a >= 0x6800 && a < 0x6c00)
		namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a >= 0x7000 && a < 0x8000) {   // irq_1_ctrl_w
		b.main_irq_mask = !(a & 0x800);
		if (!b.main_irq_mask)
			m6809_set_irq(&s_cpu, 0);
	} else if (a >= 0x8000 && a < 0x9000) {   // subreset_w
		const uint8_t hold = (a & 0x800) != 0;
		if (hold != b.mcu_reset) {
			b.mcu_reset = hold;
			m6801_set_reset(&s_mcu, hold);
		}
	} else if (a >= 0x9000 && a < 0xa000) {   // bankswitch_w
		b.rombank = !(a & 0x800);
		rom_bank();
	} else if (a == 0xa000 || a == 0xa001) {   // flipscreen_priority_w
		b.priority = v;
		b.flip = a & 1;
	}
}

// ---- the MCU (mcu_map) ----
static uint8_t mcu_r(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x1000 && a < 0x1400) return namco_cus30_amap_r(&s_wsg, a & 0x3ff);
	if (a >= 0x8000 && a < 0xa000) return s_mcusub[rbx_wrap(a - 0x8000, s_mcusub_size)];
	if (a >= 0xc000 && a < 0xc800) return b.mcuram[a - 0xc000];
	if (a >= 0xf000) return s_mcurom[a & 0xfff];
	return 0;
}

static void mcu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x1000 && a < 0x1400)
		namco_cus30_amap_w(&s_wsg, a & 0x3ff, v);
	else if (a >= 0x4000 && a < 0x8000) {   // irq_2_ctrl_w
		b.mcu_irq_mask = !(a & 0x2000);
		if (!b.mcu_irq_mask)
			m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, 0);
	} else if (a >= 0xc000 && a < 0xc800)
		b.mcuram[a - 0xc000] = v;
}

static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	if (port != 1)
		return 0xff;
	const uint8_t d0 = (uint8_t)rbx_port_read(s_port_dsw[0]), d1 = (uint8_t)rbx_port_read(s_port_dsw[1]);
	switch (b.inport_sel) {   // inputport_r
	case 0: return (uint8_t)((d1 & 0xf8) >> 3);
	case 1: return (uint8_t)(((d1 & 0x07) << 2) | ((d0 & 0xc0) >> 6));
	case 2: return (uint8_t)((d0 & 0x3e) >> 1);
	case 3: return (uint8_t)(((d0 & 0x01) << 4) | (rbx_port_read(s_port_button2) & 0x0f));
	case 4: return (uint8_t)rbx_port_read(s_port_system);
	case 5: return (uint8_t)rbx_port_read(s_port_pl[1]);
	case 6: return (uint8_t)rbx_port_read(s_port_pl[0]);
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
static void draw_bg(const snap_t *sn, int y, uint16_t *d)
{
	const int flip = sn->flip;
	const int ty = flip ? (223 - y + 7 - sn->scroll_y) & 0xff : (y + sn->scroll_y + 25) & 0xff;
	const int sx = flip ? 287 + 189 - (sn->scroll_x ^ 1) : sn->scroll_x + 35;
	const uint8_t *row = sn->vram + (ty >> 3) * 64;
	const uint8_t *gfx = s_bgpix + (uint32_t)(ty & 7) * 8;
	for (int x = 0; x < VIS_W; x++) {
		const int tx = (flip ? sx - x : sx + x) & 0x1ff;
		const uint8_t code = row[tx >> 3], attr = row[(tx >> 3) + 0x800];
		const uint32_t c = rbx_wrap(code + ((uint32_t)(attr & 1) << 8), s_bg_count);
		const int color = ((attr & 0x7e) >> 1) | ((attr & 1) << 6);
		d[x] = (uint16_t)(0x100 + color * 4 + gfx[c * 64 + (uint32_t)(tx & 7)]);
	}
}

// the 36 x 28 text layer (tx_tilemap_scan), transparent pen 0; cat: the category drawn (-1 all, else cat only or,
// with skip, every other one)
static void draw_text(const snap_t *sn, int y, uint16_t *d, int cat, int skip)
{
	const int flip = sn->flip;
	const int ly = flip ? 223 - y : y;
	const int row = ly >> 3;
	const int py = flip ? 7 - (ly & 7) : ly & 7;
	int last = -1, hide = 0;
	uint32_t cbase = 0;
	const uint8_t *trow = NULL;
	for (int x = 0; x < VIS_W; x++) {
		const int lx = flip ? VIS_W - 1 - x : x;
		if ((lx >> 3) != last) {   // (a character looked up once for its 8 pixels)
			last = lx >> 3;
			const int r = row + 2, c = (lx >> 3) - 2;
			const int idx = (c & 0x20) ? r + ((c & 0x1f) << 5) : c + (r << 5);
			const uint8_t code = sn->text[idx];
			hide = cat >= 0 && ((code >> 4) == cat) == skip;
			// (the second set holds the flipped characters: the tile itself is not flipped back)
			trow = s_txpix + rbx_wrap(code | (flip ? 0x100u : 0u), s_tx_count) * 64 + (uint32_t)(py * 8);
			cbase = (uint32_t)(sn->text[idx + 0x400] & 0x3f) * 4;
		}
		if (hide)
			continue;
		const uint8_t pen = trow[flip ? 7 - (lx & 7) : lx & 7];
		if (pen)
			d[x] = (uint16_t)(cbase + pen);
	}
}

static void draw_sprites(const snap_t *sn)
{
	static const int gfx_offs[2][2] = { { 0, 1 }, { 2, 3 } };
	const uint8_t *s1 = sn->spr + 0x780, *s2 = s1 + 0x800, *s3 = s2 + 0x800;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int offs = 0; offs < 0x80; offs += 2) {
		int sprite = s1[offs] + ((s3[offs] & 0x80) << 1);
		const int color = s1[offs + 1] & 0x3f;
		const int sx = s2[offs + 1] + 0x100 * (s3[offs + 1] & 1) - 71;
		int sy = 256 - s2[offs] - 7;
		int flipx = s3[offs] & 1, flipy = (s3[offs] & 2) >> 1;
		const int sizex = (s3[offs] & 4) >> 2, sizey = (s3[offs] & 8) >> 3;
		sprite &= ~sizex;
		sprite &= ~(sizey << 1);
		if (sn->flip) {
			flipx ^= 1;
			flipy ^= 1;
		}
		sy -= 16 * sizey;
		sy = (sy & 0xff) - 32;
		const uint8_t trans = s_spr_trans[color];
		const uint16_t cbase = (uint16_t)(0x300 + color * 8);
		for (int yy = 0; yy <= sizey; yy++)
			for (int xx = 0; xx <= sizex; xx++) {
				const uint32_t code = rbx_wrap((uint32_t)(sprite + gfx_offs[yy ^ (sizey * flipy)][xx ^ (sizex * flipx)]), s_spr_count);
				const uint8_t *tile = s_sprpix + code * 256;
				const int x0 = sx + 16 * xx, y0 = sy + 16 * yy;
				for (int py = 0; py < 16; py++) {
					const int ay = y0 + py;
					if (ay < 0 || ay >= VIS_H)
						continue;
					const uint8_t *src = tile + (flipy ? 15 - py : py) * 16;
					uint16_t *d = bm + ay * pitch;
					for (int px = 0; px < 16; px++) {
						const int ax = x0 + px;
						if (ax < 0 || ax >= VIS_W)
							continue;
						const uint8_t pen = src[flipx ? 15 - px : px];
						if (!((trans >> pen) & 1))
							d[ax] = (uint16_t)(cbase + pen);
					}
				}
			}
	}
}

static void sk_render(void)
{
	const snap_t *sn = s_snap;
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		draw_bg(sn, y, bm + y * pitch);
	if (sn->priority & 0x04) {   // (the text layer's priority: one category under the sprites)
		const int pri = sn->priority >> 4;
		for (int y = 0; y < VIS_H; y++)
			draw_text(sn, y, bm + y * pitch, pri, 0);
		draw_sprites(sn);
		for (int y = 0; y < VIS_H; y++)
			draw_text(sn, y, bm + y * pitch, pri, 1);
	} else {
		draw_sprites(sn);
		for (int y = 0; y < VIS_H; y++)
			draw_text(sn, y, bm + y * pitch, -1, 0);
	}
}

static void vblank(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	memcpy(s_snap->text, b.text, sizeof(b.text));
	memcpy(s_snap->spr, b.spr, sizeof(b.spr));
	s_snap->scroll_x = b.scroll_x;
	s_snap->scroll_y = b.scroll_y;
	s_snap->priority = b.priority;
	s_snap->flip = b.flip;
	rbx_video_frame_done();
	if (b.main_irq_mask)
		m6809_set_irq(&s_cpu, 1);
	if (b.mcu_irq_mask)
		m6801_set_irq_line(&s_mcu, M6801_IRQ1_LINE, 1);
}

// ---- frame ----
static void sk_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		for (int sl = 0; sl < SLICES; sl++) {
			const int n = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES * SLICES);
			b.t += n;
			m6809_run_until(&s_cpu, b.t);
			m6801_execute(&s_mcu, n);
		}
		const int k = rbx_line_cycles(&b.snd_frac, SND_RATE, FRAME_MHZ, LINES);
		if (k > 0) {
			memset(s_mix, 0, (size_t)k * sizeof(s_mix[0]));
			namco_wsg_render(&s_wsg, s_mix, NULL, k, SND_RATE);
			for (int i = 0; i < k; i++)
				rbx_audio_put(s_mix[i]);
		}
	}
	vblank();   // (no vblank period: at the end of the frame)
}

// ---- init / reset ----
static void sk_reset(void)
{
	b.priority = b.flip = b.rombank = 0;
	b.main_irq_mask = b.mcu_irq_mask = b.mcu_reset = b.inport_sel = 0;
	b.scroll_x = b.scroll_y = 0;
	b.t = 0;
	b.cpu_frac = b.snd_frac = 0;
	rom_bank();
	m6809_reset(&s_cpu);
	s_cpu.cycles = 4;   // (MAME: the reset vector fetch, INTERRUPT_VECTOR, takes 4 clocks)
	m6801_set_reset(&s_mcu, 0);
	m6801_reset(&s_mcu);
	namco_wsg_init(&s_wsg, NAMCO_WSG_TYPE_CUS30, 49152000 / 2048, 0, NULL, 256);
}

static void sk_exit(void);

static void map_pages(m6809_t *c, int first, int n, uint8_t *mem)
{
	for (int p = 0; p < n; p++)
		c->rpage[first + p] = c->wpage[first + p] = mem + p * 256;
}

// 2bpp, planes at bits 0 / 4 (MSB first); xoff: the bit offsets of the 8 pixels' bytes, row: a row's bytes
static void decode_2bpp(const uint8_t *rom, uint32_t count, uint8_t *pix, const int xbyte[8], int row)
{
	for (uint32_t t = 0; t < count; t++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint8_t v = rom[t * 16 + (uint32_t)(xbyte[x] + y * row)];
				const int k = x & 3;
				pix[t * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)((((v >> (7 - k)) & 1) << 1) | ((v >> (3 - k)) & 1));
			}
}

static int sk_init(const rbx_game *g)
{
	s_cpu1 = rbx_region("maincpu", &s_cpu1_size);
	s_mcurom = rbx_region("mcu", &s_mcurom_size);
	s_mcusub = rbx_region("mcusub", &s_mcusub_size);
	s_chars = rbx_region("chars", &s_chars_size);
	s_tiles = rbx_region("tiles", &s_tiles_size);
	s_sprrom = rbx_region("sprites", &s_sprrom_size);
	s_proms = rbx_region("proms", &s_proms_size);
	if (!s_cpu1 || s_cpu1_size < 0x14000 || !s_mcurom || s_mcurom_size < 0x1000 || !s_mcusub || !s_mcusub_size || !s_chars ||
	    s_chars_size < 16 || !s_tiles || s_tiles_size < 16 || !s_sprrom || s_sprrom_size < 0x10000 || !s_proms || s_proms_size < 0x700) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	// the palette: 256 colours (4 bits a gun), the text's pens direct, the tiles' and sprites' through the lookup PROM
	{
		uint32_t rgb[256];
		for (int i = 0; i < 256; i++)
			rgb[i] = (uint32_t)((s_proms[i] & 15) * 0x11) << 16 | (uint32_t)((s_proms[i + 0x100] & 15) * 0x11) << 8 |
			         (uint32_t)((s_proms[i + 0x200] & 15) * 0x11);
		for (int i = 0; i < 0x100; i++)
			s_pal[i] = rgb[i];
		for (int i = 0x100; i < 0x500; i++)
			s_pal[i] = rgb[s_proms[0x300 + i - 0x100]];
		for (int c = 0; c < 64; c++) {
			s_spr_trans[c] = 0;
			for (int p = 0; p < 8; p++)
				if (s_proms[0x300 + 0x200 + c * 8 + p] == 0xff)
					s_spr_trans[c] |= (uint8_t)(1 << p);
		}
	}
	// init_skykid: unpack the third sprite ROM
	{
		uint8_t *rom = s_sprrom + 0x4000;
		for (int i = 0; i < 0x2000; i++) {
			rom[i + 0x4000] = rom[i];
			rom[i + 0x6000] = (uint8_t)(rom[i] >> 4);
			rom[i] = rom[i + 0x2000];
		}
	}
	s_tx_count = s_chars_size / 16;
	s_bg_count = s_tiles_size / 16;
	s_spr_count = s_sprrom_size / 2 / 64;
	s_txpix = (uint8_t *)malloc((size_t)s_tx_count * 64);
	s_bgpix = (uint8_t *)malloc((size_t)s_bg_count * 64);
	s_sprpix = (uint8_t *)malloc((size_t)s_spr_count * 256);
	s_snap = (snap_t *)calloc(1, sizeof(snap_t));
	if (!s_txpix || !s_bgpix || !s_sprpix || !s_snap) {
		sk_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	{
		static const int tx_x[8] = { 8, 8, 8, 8, 0, 0, 0, 0 }, bg_x[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
		decode_2bpp(s_chars, s_tx_count, s_txpix, tx_x, 1);   // text_layout
		decode_2bpp(s_tiles, s_bg_count, s_bgpix, bg_x, 2);   // tile_layout
	}
	{   // sprite_layout: 16x16, 3bpp, planes { half + 4, 0, 4 }, 4-pixel columns of 8 bytes
		const uint32_t half = s_sprrom_size / 2;
		for (uint32_t t = 0; t < s_spr_count; t++)
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 16; x++) {
					const uint32_t o = t * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
					const int k = x & 3;
					const uint8_t v = s_sprrom[o], h = s_sprrom[half + o];
					s_sprpix[t * 256 + (uint32_t)(y * 16 + x)] =
						(uint8_t)((((h >> (3 - k)) & 1) << 2) | (((v >> (7 - k)) & 1) << 1) | ((v >> (3 - k)) & 1));
				}
	}
	s_port_dsw[0] = rbx_port_find("DSWA");
	s_port_dsw[1] = rbx_port_find("DSWB");
	s_port_button2 = rbx_port_find("BUTTON2");
	s_port_system = rbx_port_find("SYSTEM");
	s_port_pl[0] = rbx_port_find("P1");
	s_port_pl[1] = rbx_port_find("P2");

	memset(&b, 0, sizeof(b));
	m6809_init(&s_cpu);
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	map_pages(&s_cpu, 0x20, 0x10, b.vram);
	map_pages(&s_cpu, 0x40, 0x08, b.text);
	map_pages(&s_cpu, 0x48, 0x18, b.spr);
	for (int p = 0x80; p < 0x100; p++)
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
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, 0x500)) {
		sk_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	sk_reset();
	return 0;
}

static void sk_exit(void)
{
	free(s_txpix);
	free(s_bgpix);
	free(s_sprpix);
	free(s_snap);
	s_txpix = s_bgpix = s_sprpix = NULL;
	s_snap = NULL;
}

static int sk_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "board", &b, sizeof(b));
	rc |= rbx_state_io(ctx, "cpu", &s_cpu, M6809_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mcu", m6801_state_data(&s_mcu), m6801_state_size());
	rc |= rbx_state_io(ctx, "wsg", &s_wsg, (uint32_t)NAMCO_WSG_STATE_SIZE);
	if (load && !rc)
		rom_bank();
	return rc;
}

static int sk_save(void *ctx) { return sk_state(ctx, 0); }
static int sk_load(void *ctx) { return sk_state(ctx, 1); }
static int sk_supported(const char *machine) { return !strcmp(machine, "skykid"); }

const rbx_driver rbx_drv_skykid = { "Namco Sky Kid", sk_init, sk_exit, sk_reset, sk_frame, sk_save, sk_load,
                                    sk_supported, sk_render, NULL };

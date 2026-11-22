/*  Toaplan 1 boards. Ported from MAME src/mame/toaplan/toaplan1.cpp, toaplan_bcu.cpp, toaplan_fcu.cpp,
    toaplan_video_controller.cpp (BSD-3-Clause, copyright-holders: Darren Olafson, Quench).
    A 68000 at 10 MHz; the BCU (four 512x512 tilemaps of 8x8 tiles, 16 priorities a tile), the FCU (sprites of 8x8
    pieces, the list buffered at the vblank), the video controller (two palettes, the IRQ enable); a Z80 at 3.5 MHz with
    a YM3812 reads the inputs and shares 2 KB with the 68000. 450x270 at 7 MHz (282 lines: outzone).
    (demonwld: TMS32010 DSP, vimana: HD647180 - not here.)
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 3500000
#define VIS_W 320
#define VIS_H 240

enum { M_TRUXTON, M_HELLFIRE, M_ZEROWING, M_OUTZONE, M_COUNT };
static const char *const s_mnames[M_COUNT] = { "truxton", "hellfire", "zerowing", "outzone" };

enum { IO_FCU, IO_BCU, IO_VCTRL, IO_SHARED, IO_TOFS, IO_SNDRST, IO_NOP };
typedef struct { uint32_t start, end; uint8_t kind; } io_range;
#define IO_END { 0, 0, 0xff }
static const io_range s_io_truxton[] = { { 0x0c0000, 0x0c0007, IO_FCU }, { 0x100000, 0x10001f, IO_BCU },
	{ 0x140000, 0x147fff, IO_VCTRL }, { 0x180000, 0x180fff, IO_SHARED }, { 0x1c0000, 0x1c0007, IO_TOFS },
	{ 0x1d0000, 0x1d0001, IO_SNDRST }, IO_END };
static const io_range s_io_hellfire[] = { { 0x080000, 0x087fff, IO_VCTRL }, { 0x0c0000, 0x0c0fff, IO_SHARED },
	{ 0x100000, 0x10001f, IO_BCU }, { 0x140000, 0x140007, IO_FCU }, { 0x180000, 0x180007, IO_TOFS },
	{ 0x180008, 0x180009, IO_SNDRST }, IO_END };
static const io_range s_io_zerowing[] = { { 0x0c0000, 0x0c0007, IO_TOFS }, { 0x400000, 0x407fff, IO_VCTRL },
	{ 0x440000, 0x440fff, IO_SHARED }, { 0x480000, 0x48001f, IO_BCU }, { 0x4c0000, 0x4c0007, IO_FCU }, IO_END };
static const io_range s_io_outzone[] = { { 0x100000, 0x100007, IO_FCU }, { 0x140000, 0x140fff, IO_SHARED },
	{ 0x200000, 0x20001f, IO_BCU }, { 0x300000, 0x307fff, IO_VCTRL }, { 0x340000, 0x340007, IO_TOFS }, IO_END };

static int s_m;
static const io_range *s_io;
static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static int s_lines, s_vis_y0;          // total lines; the first line shown (16 on hellfire / zerowing)
static uint32_t s_frame_mhz;
static int s_port_vblank, s_port_p1, s_port_p2, s_port_sys, s_port_dswa, s_port_dswb, s_port_tjump;
static const uint8_t *s_tiles, *s_sprites;   // 8x8, 4 bpp packed (even pixel in the low nibble), 32 bytes each
static uint32_t s_ntiles, s_nsprites;

static struct toaplan1_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x4000];
	uint16_t pal[2][0x400];
	uint32_t vram[4][0x1000];      // BCU (one 32-bit word a tile)
	uint16_t ram_offs, scrollx[4], scrolly[4], offsetx, offsety;
	uint8_t bcu_flip, fcu_flip, intenable, vblank;
	uint16_t spr[0x400], sprsize[0x40], spr_buf[0x400], sprsize_buf[0x40], spr_offs;
	uint8_t shram[0x800];
	uint8_t z80_hold;
	int32_t line;
	uint32_t frame;
	uint64_t cpu_frac, z80_frac, ym_frac;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)
static z80 s_z80;
static uint8_t s_zram_dummy;
static const uint8_t *s_zmap[256];
static int s_ym_irq;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct {
	uint32_t vram[4][0x1000];
	uint16_t scrollx[4], scrolly[4], offsetx, offsety;
	uint16_t spr[0x400], sprsize[0x40];
	uint16_t pal[2][0x400];
} *s_snap;
static uint8_t s_pri[VIS_W * VIS_H];

// ---- graphics: tilelayout (RGN_FRAC(1,2), planes { half + 8, half, 8, 0 }) to 4 bpp packed ----
static int decode_gfx(uint8_t *region, uint32_t size, const uint8_t **out, uint32_t *count)
{
	const uint32_t half = size / 2, tiles = half / 16;
	uint8_t *o = (uint8_t *)malloc(tiles * 32 ? tiles * 32 : 1);
	if (!o)
		return -1;
	for (uint32_t t = 0; t < tiles; t++)
		for (int r = 0; r < 8; r++) {
			const uint8_t b0 = region[t * 16 + (uint32_t)r * 2], b1 = region[t * 16 + (uint32_t)r * 2 + 1];
			const uint8_t c0 = region[half + t * 16 + (uint32_t)r * 2], c1 = region[half + t * 16 + (uint32_t)r * 2 + 1];
			for (int x = 0; x < 8; x += 2) {
				uint8_t px[2];
				for (int k = 0; k < 2; k++) {
					const int bit = 7 - (x + k);
					px[k] = (uint8_t)(((c1 >> bit) & 1) << 3 | ((c0 >> bit) & 1) << 2 | ((b1 >> bit) & 1) << 1 | ((b0 >> bit) & 1));
				}
				o[t * 32 + (uint32_t)r * 4 + (uint32_t)(x >> 1)] = (uint8_t)(px[0] | px[1] << 4);
			}
		}
	memcpy(region, o, tiles * 32);
	free(o);
	*out = region;
	*count = tiles;
	return 0;
}
static inline uint8_t gpix(const uint8_t *g, uint32_t t, int x, int y)
{
	const uint8_t v = g[t * 32 + (uint32_t)(y * 4 + (x >> 1))];
	return (x & 1) ? (v >> 4) : (v & 15);
}

static void z_start(void);

// ---- the 68000's devices ----
static int vpos(void) { return b.line; }
static int in_vblank(void) { const int v = vpos(); return v < s_vis_y0 || v >= s_vis_y0 + VIS_H; }

static const io_range *io_find(uint32_t a)
{
	for (const io_range *r = s_io; r->kind != 0xff; r++)
		if (a >= r->start && a <= r->end)
			return r;
	return NULL;
}

static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	const io_range *r = io_find(a);
	if (!r)
		return 0;
	const uint32_t o = a - r->start;
	switch (r->kind) {
	case IO_FCU:
		switch (o) {
		case 0: return (uint16_t)in_vblank();   // frame_done_r: the screen's vblank
		case 2: return b.spr_offs;
		case 4: return b.spr[b.spr_offs & 0x3ff];
		case 6: return b.sprsize[b.spr_offs & 0x3f];
		}
		return 0;
	case IO_BCU: {
		if (o == 2)
			return b.ram_offs;
		if (o == 4 || o == 6) {
			const int layer = b.ram_offs >> 12, offs = b.ram_offs & 0xfff;
			return layer < 4 ? (uint16_t)(b.vram[layer][offs] >> (o == 4 ? 16 : 0)) : 0;
		}
		if (o >= 0x10) {
			const int k = (int)((o - 0x10) >> 1);
			return (k & 1) ? b.scrolly[k >> 1] : b.scrollx[k >> 1];
		}
		return 0;
	}
	case IO_VCTRL:
		if (o == 0) {   // the VBLANK port (its custom bit the screen's vblank)
			rbx_port_custom(s_port_vblank, 0xffffffffu, in_vblank() ? 0xffffffffu : 0);
			return (uint16_t)rbx_port_read(s_port_vblank);
		}
		if (o >= 0x4000)
			return b.pal[(o >> 13) & 1][(o >> 1) & 0x3ff];
		return 0;
	case IO_SHARED:
		return b.shram[(o >> 1) & 0x7ff];
	default:
		return 0;
	}
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a & ~1u);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xfffffe;
	const io_range *r = io_find(a);
	if (!r)
		return;
	const uint32_t o = a - r->start;
#define COMBINE(x) ((x) = (uint16_t)((v & mask) | ((x) & ~mask)))
	switch (r->kind) {
	case IO_FCU:
		if (o == 2)
			COMBINE(b.spr_offs);
		else if (o == 4) {
			COMBINE(b.spr[b.spr_offs & 0x3ff]);
			if (b.spr_offs < 0x400)
				b.spr_offs++;
		} else if (o == 6) {
			COMBINE(b.sprsize[b.spr_offs & 0x3f]);
			if (b.spr_offs < 0x40)
				b.spr_offs++;
		}
		break;
	case IO_BCU:
		if (o == 0) {
			if (mask & 0xff)
				b.bcu_flip = v & 1;
		} else if (o == 2)
			COMBINE(b.ram_offs);
		else if (o == 4 || o == 6) {
			const int layer = b.ram_offs >> 12, offs = b.ram_offs & 0xfff;
			if (layer < 4) {
				const int sh = o == 4 ? 16 : 0;
				b.vram[layer][offs] = (b.vram[layer][offs] & ~((uint32_t)mask << sh)) | ((uint32_t)(v & mask) << sh);
			}
		} else if (o >= 0x10) {
			const int k = (int)((o - 0x10) >> 1);
			if (k < 8) {
				if (k & 1)
					COMBINE(b.scrolly[k >> 1]);
				else
					COMBINE(b.scrollx[k >> 1]);
			}
		}
		break;
	case IO_VCTRL:
		if (o == 2 && (mask & 0xff))
			b.intenable = (uint8_t)v;
		else if (o >= 0x4000)
			COMBINE(b.pal[(o >> 13) & 1][(o >> 1) & 0x3ff]);
		break;
	case IO_SHARED:
		if (mask & 0xff)
			b.shram[(o >> 1) & 0x7ff] = (uint8_t)v;
		break;
	case IO_TOFS:
		if (o == 0)
			COMBINE(b.offsetx);
		else if (o == 2)
			COMBINE(b.offsety);
		else if (o == 6 && (mask & 0xff00))
			b.fcu_flip = (v >> 15) & 1;   // (the byte at 6: bit 7)
		break;
	case IO_SNDRST:   // reset_sound_w(0): the Z80 and the YM reset
		if ((mask & 0xff) && !(v & 0xff)) {
			rbx_ym_reset(0);
			z_start();
		}
		break;
	}
#undef COMBINE
}
static void main_w16(uint32_t a, uint16_t v) { main_w16m(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	main_w16m(a & ~1u, (a & 1) ? v : (uint16_t)(v << 8), (a & 1) ? 0x00ff : 0xff00);
}

// ---- the Z80 (sound_map, the boards' I/O maps) ----
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0x8000 && a < 0x8800)
		return b.shram[a - 0x8000];
	return 0xff;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		b.shram[a - 0x8000] = v;
}
static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff; }
static uint8_t z_in(z80 *z, uint8_t port)
{
	(void)z;
	switch (s_m) {
	case M_TRUXTON:
		switch (port) {
		case 0x00: return port8(s_port_p1);
		case 0x10: return port8(s_port_p2);
		case 0x20: return port8(s_port_sys);
		case 0x40: return port8(s_port_dswa);
		case 0x50: return port8(s_port_dswb);
		case 0x60: case 0x61: return rbx_ym_read(0, port & 1);
		case 0x70: return port8(s_port_tjump);
		}
		break;
	case M_HELLFIRE:
		switch (port) {
		case 0x00: return port8(s_port_dswa);
		case 0x10: return port8(s_port_dswb);
		case 0x20: return port8(s_port_tjump);
		case 0x40: return port8(s_port_p1);
		case 0x50: return port8(s_port_p2);
		case 0x60: return port8(s_port_sys);
		case 0x70: case 0x71: return rbx_ym_read(0, port & 1);
		}
		break;
	case M_ZEROWING:
		switch (port) {
		case 0x00: return port8(s_port_p1);
		case 0x08: return port8(s_port_p2);
		case 0x20: return port8(s_port_dswa);
		case 0x28: return port8(s_port_dswb);
		case 0x80: return port8(s_port_sys);
		case 0x88: return port8(s_port_tjump);
		case 0xa8: case 0xa9: return rbx_ym_read(0, port & 1);
		}
		break;
	default:   // outzone
		switch (port) {
		case 0x00: case 0x01: return rbx_ym_read(0, port & 1);
		case 0x08: return port8(s_port_dswa);
		case 0x0c: return port8(s_port_dswb);
		case 0x10: return port8(s_port_sys);
		case 0x14: return port8(s_port_p1);
		case 0x18: return port8(s_port_p2);
		case 0x1c: return port8(s_port_tjump);
		}
		break;
	}
	return 0xff;
}
static void z_out(z80 *z, uint8_t port, uint8_t v)
{
	(void)z;
	const int ym = s_m == M_TRUXTON ? 0x60 : s_m == M_HELLFIRE ? 0x70 : s_m == M_ZEROWING ? 0xa8 : 0x00;
	if ((port & 0xfe) == ym)
		rbx_ym_write(0, port & 1, v);
}
static void ym_irq(int state) { s_ym_irq = state; }
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}

// ---- drawing: bands ----
#define BANDS 8
static int s_band_next, s_band_left, s_band_active, s_band_helpers;

// the four layers in one pass a layer: MAME draws layer 0 opaque first (priority 0), then for priorities 1..15 the
// layers 3, 2, 1, 0; so a pixel keeps the greatest key = priority << 2 | (3 - layer) of the opaque ones
static int8_t s_key[VIS_W * VIS_H];
static void draw_layer(int layer, int y0, int y1)
{
	const int dx = -0x1ef - 6 + 2 * layer, dy = -0x101;
	const int sx = (s_snap->scrollx[layer] >> 7) - (int16_t)s_snap->offsetx;
	const int sy = (s_snap->scrolly[layer] >> 7) - (int16_t)s_snap->offsety;
	const uint32_t *vram = s_snap->vram[layer];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = y0; y < y1; y++) {
		const int ly = (y + s_vis_y0 + sy - dy) & 511;
		uint16_t *d = bm + y * pitch;
		int8_t *kp = s_key + y * VIS_W;
		for (int x = 0; x < VIS_W;) {
			const int lx = (x + sx - dx) & 511;
			int n = 8 - (lx & 7);
			if (n > VIS_W - x)
				n = VIS_W - x;
			const uint32_t t = vram[(ly >> 3) * 64 + (lx >> 3)];
			const int cat = (t & 0x8000) ? 16 : (int)((t >> 28) & 0xf);
			const uint32_t code = (t & 0x7fff) % (s_ntiles ? s_ntiles : 1);
			const uint16_t cb = (uint16_t)(((t >> 16) & 0x3f) << 4);
			if (layer == 0) {   // (the background: every pixel, then its own priority if shown)
				for (int i = 0, px = lx & 7; i < n; i++, px++) {
					const uint8_t pen = gpix(s_tiles, code, px, ly & 7);
					d[x + i] = (uint16_t)(cb | pen);
					kp[x + i] = (int8_t)(pen && cat >= 1 && cat <= 15 ? cat << 2 | 3 : -1);
				}
			} else if (cat >= 1 && cat <= 15) {
				const int k = cat << 2 | (3 - layer);
				for (int i = 0, px = lx & 7; i < n; i++, px++) {
					const uint8_t pen = gpix(s_tiles, code, px, ly & 7);
					if (pen && k > kp[x + i]) {
						d[x + i] = (uint16_t)(cb | pen);
						kp[x + i] = (int8_t)k;
					}
				}
			}
			x += n;
		}
		uint8_t *pp = s_pri + y * VIS_W;
		if (layer == 3)
			for (int x = 0; x < VIS_W; x++)
				pp[x] = (uint8_t)(kp[x] < 0 ? 0 : kp[x] >> 2);
	}
}

static void draw_sprites(int y0, int y1)
{
	const uint16_t *src = s_snap->spr, *size = s_snap->sprsize;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int offs = 0x400 - 4; offs >= 0; offs -= 4) {
		if (src[offs] & 0x8000)
			continue;
		const uint16_t attrib = src[offs + 1];
		const int priority = (attrib >> 12) & 0xf;
		uint32_t sprite = src[offs] & 0x7fff;
		const uint16_t color = (uint16_t)(0x400 + ((attrib & 0x3f) << 4));
		const int sp = (attrib >> 6) & 0x3f;
		const int sizex = (size[sp] & 0x0f) * 8, sizey = ((size[sp] >> 4) & 0x0f) * 8;
		int sx_base = (src[offs + 2] >> 7) & 0x1ff, sy_base = (src[offs + 3] >> 7) & 0x1ff;
		if (sx_base >= 0x180) sx_base -= 0x200;
		if (sy_base >= 0x180) sy_base -= 0x200;
		sy_base -= s_vis_y0;   // (the bitmap's line 0 is the first shown)
		for (int dim_y = 0; dim_y < sizey; dim_y += 8) {
			const int sy = sy_base + dim_y;
			for (int dim_x = 0; dim_x < sizex; dim_x += 8, sprite++) {
				const int sx = sx_base + dim_x;
				if (sy + 8 <= y0 || sy >= y1 || sx + 8 <= 0 || sx >= VIS_W || !s_nsprites)
					continue;
				const uint32_t code = sprite % s_nsprites;
				for (int yy = 0; yy < 8; yy++) {
					const int dy = sy + yy;
					if (dy < y0 || dy >= y1)
						continue;
					uint16_t *d = bm + dy * pitch;
					uint8_t *pp = s_pri + dy * VIS_W;
					for (int xx = 0; xx < 8; xx++) {
						const int dx = sx + xx;
						if (dx < 0 || dx >= VIS_W)
							continue;
						const uint8_t pen = gpix(s_sprites, code, xx, yy);
						// prio_transpen with pri_mask ~0 << priority: over pixels below that priority; then 0x1f
						if (pen && pp[dx] < priority) {
							d[dx] = (uint16_t)(color | pen);
							pp[dx] = 0x1f;
						}
					}
				}
			}
		}
	}
}

static void run_band(int band)
{
	const int y0 = VIS_H * band / BANDS, y1 = VIS_H * (band + 1) / BANDS;
	for (int l = 0; l < 4; l++)
		draw_layer(l, y0, y1);
	draw_sprites(y0, y1);
}

static void t1_render(void)
{
	for (int p = 0; p < 2; p++)
		for (int i = 0; i < 0x400; i++) {   // xBGR_555
			const uint16_t c = s_snap->pal[p][i];
			const int r = c & 31, g = (c >> 5) & 31, bb = (c >> 10) & 31;
			rbx_video.palette[p * 0x400 + i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bb << 3 | bb >> 2));
		}
	__atomic_store_n(&s_band_next, 0, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_left, BANDS, __ATOMIC_SEQ_CST);
	__atomic_store_n(&s_band_active, 1, __ATOMIC_SEQ_CST);
	for (;;) {
		const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
		if (bnd >= BANDS)
			break;
		run_band(bnd);
		__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
	}
	while (__atomic_load_n(&s_band_left, __ATOMIC_ACQUIRE) > 0)
		;
	__atomic_store_n(&s_band_active, 0, __ATOMIC_SEQ_CST);
	while (__atomic_load_n(&s_band_helpers, __ATOMIC_ACQUIRE) > 0)
		;
}

static int t1_render_help(int helper)
{
	const int one = helper & 0x100;
	helper &= 0xff;
	if (helper < 1 || helper > 2 || !__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		return 0;
	__atomic_add_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	int did = 0;
	if (__atomic_load_n(&s_band_active, __ATOMIC_SEQ_CST))
		for (;;) {
			const int bnd = __atomic_fetch_add(&s_band_next, 1, __ATOMIC_ACQ_REL);
			if (bnd >= BANDS)
				break;
			run_band(bnd);
			__atomic_sub_fetch(&s_band_left, 1, __ATOMIC_ACQ_REL);
			did = 1;
			if (one)
				break;
		}
	__atomic_sub_fetch(&s_band_helpers, 1, __ATOMIC_SEQ_CST);
	return did;
}

// the vblank: the screen drawn (VIDEO_UPDATE_BEFORE_VBLANK), the FCU's list buffered, the IRQ
static void vblank(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	memcpy(s_snap->scrollx, b.scrollx, sizeof(b.scrollx));
	memcpy(s_snap->scrolly, b.scrolly, sizeof(b.scrolly));
	s_snap->offsetx = b.offsetx;
	s_snap->offsety = b.offsety;
	memcpy(s_snap->spr, b.spr_buf, sizeof(b.spr_buf));
	memcpy(s_snap->sprsize, b.sprsize_buf, sizeof(b.sprsize_buf));
	memcpy(s_snap->pal, b.pal, sizeof(b.pal));
	rbx_video_frame_done();
	memcpy(b.spr_buf, b.spr, sizeof(b.spr));
	memcpy(b.sprsize_buf, b.sprsize, sizeof(b.sprsize));
	if (b.intenable)
		m68k_bus_hold_irq(4);
}

static void t1_frame(void)
{
	rbx_audio.frame_mhz = s_frame_mhz;
	for (int line = 0; line < s_lines; line++) {
		b.line = line;
		if (line == s_vis_y0 + VIS_H)
			vblank();
		m68k_execute(rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, s_frame_mhz, s_lines));
		const int zc = rbx_line_cycles(&b.z80_frac, Z80_CLOCK, s_frame_mhz, s_lines);
		z80_run(&s_z80, s_z80.cyc + (unsigned long)zc, &s_ym_irq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, Z80_CLOCK, s_frame_mhz, s_lines);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	b.frame++;
}

static void t1_reset(void)
{
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	s_ym_irq = 0;
	z_start();
	rbx_ym_reset(0);
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void t1_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
}

static int t1_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = (struct toaplan1_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = -1;
	for (int i = 0; i < M_COUNT; i++)
		if (!strcmp(g->machine, s_mnames[i]))
			s_m = i;
	if (s_m < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	uint32_t ts = 0, ss = 0;
	uint8_t *tiles = rbx_region("tiles", &ts), *spr = rbx_region("sprites", &ss);
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	if (!s_rom || !s_zrom || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (decode_gfx(tiles, ts, &s_tiles, &s_ntiles) || decode_gfx(spr, ss, &s_sprites, &s_nsprites)) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	(void)s_zram_dummy;
	memset(&b, 0, sizeof(b));
	// timing: 7 MHz, 450 a line; 270 lines (outzone 282); hellfire / zerowing show lines 16 .. 255
	s_lines = s_m == M_OUTZONE ? 282 : 270;
	s_vis_y0 = (s_m == M_HELLFIRE || s_m == M_ZEROWING) ? 16 : 0;
	s_frame_mhz = (uint32_t)(7000000ull * 1000 / (450ull * (uint32_t)s_lines));
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
	switch (s_m) {
	case M_TRUXTON:
		m68k_bus_map(0, 0x3ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x080000, 0x083fff, b.ram, 0x4000, 1, 1);
		s_io = s_io_truxton;
		break;
	case M_HELLFIRE:
		m68k_bus_map(0, 0x3ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x040000, 0x047fff, b.ram, 0x8000, 1, 1);
		s_io = s_io_hellfire;
		break;
	case M_ZEROWING:
		m68k_bus_map(0, 0xffff, s_rom, 0x10000, 1, 0);
		if (s_rom_size > 0x40000)
			m68k_bus_map(0x040000, 0x07ffff, s_rom + 0x40000, s_rom_size - 0x40000 < 0x40000 ? s_rom_size - 0x40000 : 0x40000, 1, 0);
		m68k_bus_map(0x080000, 0x087fff, b.ram, 0x8000, 1, 1);
		s_io = s_io_zerowing;
		break;
	default:
		m68k_bus_map(0, 0x3ffff, s_rom, s_rom_size, 1, 0);
		m68k_bus_map(0x240000, 0x243fff, b.ram, 0x4000, 1, 1);
		s_io = s_io_outzone;
		break;
	}
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	s_port_vblank = rbx_port_find("VBLANK");
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dswa = rbx_port_find("DSWA");
	s_port_dswb = rbx_port_find("DSWB");
	s_port_tjump = rbx_port_find("TJUMP");
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = Z80_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		t1_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x800)) {
		rbx_fail("%s: out of memory", g->set);
		t1_exit();
		return -1;
	}
	t1_reset();
	return 0;
}

static int t1_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	z80 z = s_z80;
	int32_t ymirq = s_ym_irq;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "t1", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ymirq", &ymirq, sizeof(ymirq));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z_read;
		z.write_byte = z_write;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_ym_irq = ymirq;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int t1_save(void *ctx) { return t1_state(ctx, 0); }
static int t1_load(void *ctx) { return t1_state(ctx, 1); }
static int t1_supported(const char *machine)
{
	for (int i = 0; i < M_COUNT; i++)
		if (!strcmp(machine, s_mnames[i]))
			return 1;
	return 0;
}

const rbx_driver rbx_drv_toaplan1 = { "Toaplan 1", t1_init, t1_exit, t1_reset, t1_frame, t1_save, t1_load,
                                      t1_supported, t1_render, t1_render_help };

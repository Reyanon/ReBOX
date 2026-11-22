/*  Video System boards of 1990-1992 (and their bootlegs). Ported from MAME 0.289 src/mame/vsystem/pspikes.cpp,
    vsystem_spr2.cpp (VS8904 / VS8905 sprites), vsystem_gga.cpp (BSD-3-Clause, copyright-holders: Nicola Salmoria,
    David Haywood, AJR).
    A 68000 at 10 MHz; one or two 8x8 tile layers (512x256 / 512x512, row scroll on the first), one or two VS8904 /
    VS8905 sprite chips (16x16 tiles through a look-up RAM, zoomed); a palette of 2048 / 1024 xRGB 555. Sound: a Z80
    with a YM2610 (Power Spikes, Spinal Breakers, Karate Blazers, Turbo Force, Aero Fighters), and the bootlegs' sound
    (OKI only, YM2151, YM3812 with an OKI or a uPD7759). The C7-01 GGA (video timing) is written but changes nothing.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../../video/decoband.h"
#include "../video/vsystem_gfx.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

enum { M_PSPIKES, M_PSPIKESB, M_SPIKES91, M_PSPIKESC, M_KICKBALL, M_WBBC97, M_KARATBLZ, M_KARATBLZBL, M_SPINLBRK,
       M_TURBOFRC, M_AEROFGTB, M_AERFBOOT, M_AERFBOO2, M_COUNT };
static const char *const s_mnames[M_COUNT] = { "pspikes", "pspikesb", "spikes91", "pspikesc", "kickball", "wbbc97",
	"karatblz", "karatblzbl", "spinlbrk", "turbofrc", "aerofgtb", "aerfboot", "aerfboo2" };

// the sound CPU's maps
enum { Z_NONE, Z_BANKED_A, Z_BANKED_B, Z_AERFBOOT, Z_WBBC97, Z_KARATBLZBL, Z_KICKBALL, Z_SPIKES91 };

typedef struct {
	uint32_t fps_mhz;
	int lines, vx0, vx1, vy0, vy1;
	uint32_t z80_clock;
	int zmap, pal_n, spr_bytes;
} machine_t;

static const machine_t s_machines[M_COUNT] = {
	[M_PSPIKES]    = { 61310, 256, 4, 355, 0, 239, 5000000, Z_BANKED_A, 2048, 0x400 },
	[M_PSPIKESB]   = { 60000, 256, 4, 355, 0, 239, 0, Z_NONE, 2048, 0xc00 },
	[M_SPIKES91]   = { 60000, 256, 0, 319, 4, 227, 3000000, Z_SPIKES91, 2048, 0x1000 },
	[M_PSPIKESC]   = { 60000, 256, 4, 355, 0, 239, 0, Z_NONE, 2048, 0xc00 },
	[M_KICKBALL]   = { 60000, 256, 4, 355, 0, 239, 4000000, Z_KICKBALL, 2048, 0x400 },
	[M_WBBC97]     = { 60000, 512, 14, 355, 0, 239, 4000000, Z_WBBC97, 2048, 0x400 },
	[M_KARATBLZ]   = { 60000, 256, 8, 359, 0, 239, 4000000, Z_BANKED_A, 1024, 0x800 },
	[M_KARATBLZBL] = { 60000, 256, 8, 359, 0, 239, 4000000, Z_KARATBLZBL, 1024, 0x800 },
	[M_SPINLBRK]   = { 60000, 256, 8, 359, 0, 239, 5000000, Z_BANKED_A, 1024, 0x800 },
	[M_TURBOFRC]   = { 61310, 256, 0, 351, 0, 239, 5000000, Z_BANKED_A, 1024, 0x800 },
	[M_AEROFGTB]   = { 60000, 256, 12, 331, 0, 223, 4000000, Z_BANKED_B, 1024, 0x800 },
	[M_AERFBOOT]   = { 60000, 256, 12, 331, 0, 223, 4000000, Z_AERFBOOT, 1024, 0x4000 },
	[M_AERFBOO2]   = { 60000, 256, 12, 331, 0, 223, 0, Z_NONE, 1024, 0x800 },
};

#define CPU_CLOCK 10000000
#define BANDS 8

static int s_m;
static const machine_t *s_mc;
static uint32_t s_amask;          // the 68000's address mask (karatblz, turbofrc: global_mask(0xfffff))
static uint8_t *s_rom, *s_zrom, *s_zbank_rom, *s_oki_rom, *s_upd_rom;
static uint32_t s_rom_size, s_zrom_size, s_zbank_size, s_oki_size, s_upd_size;
static const uint8_t *s_lookrom;  // spinlbrk / spikes91 sprite look-up ROM (big-endian words)
static uint32_t s_lookrom_n;
static uint32_t s_look_n[2];      // the look-up RAMs' words (code % n)
static vsg_t s_tiles[2], s_sprg[2];
static int s_port_in[4], s_port_dsw[2];
static okim6295_t s_oki;
static int s_has_oki, s_has_ym, s_has_upd;
static upd7759_t s_upd;
static z80 s_z80;
static int s_zirq;
static const uint8_t *s_zmap[256];
static int32_t *s_mix_l, *s_mix_r, *s_tmp;
static int s_mix_n, s_mix_cap;
static uint32_t s_out_rate;
static int64_t s_cyc_base;

// ---- the state (saved) ----
static struct pspikes_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x8000];          // work RAM (64 KB)
	uint16_t ram2[0x2000];         // the second work RAM (16 KB at 0xf8000)
	uint16_t ram3[0x400];          // aerfboot 0xfc000
	uint16_t vram[2][0x1000];
	uint16_t look[2][0x8000];      // sprite look-up RAMs
	uint16_t spr[0x2000];
	uint16_t raster[0x800];
	uint16_t pal[0x800];
	uint16_t tx[0x1000];           // spikes91 text layer
	uint16_t bitmap[0x20000];      // wbbc97
	int32_t rowscroll[2][512];     // the tilemaps' scroll values (kept between frames as MAME's)
	int32_t colscroll[2];
	uint16_t bank[4], scrollx[2], scrolly[2], bitmap_enable;
	uint8_t gfxbank[8];
	uint8_t flip, charpal, sprpal, spikes91_lookup;
	uint8_t latch, pending, zbank, okibank;
	uint8_t zram[0x800];
	uint64_t cpu_frac, z80_frac, z80_target, ym_frac, out_frac;
	uint32_t oki_acc;
	int32_t oki_hold;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

// ---- drawing snapshot ----
typedef struct {
	int16_t sx, sy;
	uint8_t gfx, flags;           // flags: 1 flipx, 2 flipy, 4 priority
	uint16_t color;
	uint32_t code, zx, zy, pmask;
} op_t;
#define MAX_OPS 40000
static struct {
	uint32_t pens[2048];
	uint16_t vram[2][0x1000];
	int32_t rowscroll[2][512];
	vsg_layer lay[2];
	int nlay;
	int bitmap_on;
	uint16_t bitmap[0x20000];
	uint16_t raster[0x80];
	int nops;
	op_t ops[MAX_OPS];
} *s_snap;
static uint16_t s_pal_conv[0x800];
static int s_pal_valid;
static uint8_t *s_pri;
static uint32_t *s_bmpens;   // wbbc97: the bitmap's 32768 colours (GRB 555)
static int s_bmpens_done;

static inline int pal5(int v) { return (v << 3) | (v >> 2); }

// ---- sound latch / the Z80 kept up with the 68000 ----
static void z80_catchup(void)
{
	if (!s_mc->z80_clock)
		return;
	const uint64_t now = b.z80_target + (uint64_t)rbx_muldiv((uint32_t)m68k_cycles_run(), s_mc->z80_clock / 1000, CPU_CLOCK / 1000);
	if ((long)(s_z80.cyc - (unsigned long)now) < 0)
		z80_run(&s_z80, (unsigned long)now, &s_zirq, 0);
}

static void latch_w(uint8_t v)
{
	z80_catchup();
	b.latch = v;
	switch (s_mc->zmap) {
	case Z_KARATBLZBL:   // (soundlatch_w: the NMI pulsed on each write)
		b.pending = 1;
		z80_gen_nmi(&s_z80);
		break;
	case Z_SPIKES91:     // (data pending: the IRQ line)
		b.pending = 1;
		s_zirq = 1;
		break;
	default:             // (data pending: the NMI line, taken on its rising edge)
		if (!b.pending)
			z80_gen_nmi(&s_z80);
		b.pending = 1;
		break;
	}
}
static uint8_t pending_r(void)
{
	z80_catchup();
	return b.pending;
}
static uint8_t latch_read_clear(void)   // (generic_latch_8 without the separate acknowledge)
{
	b.pending = 0;
	if (s_mc->zmap == Z_SPIKES91)
		s_zirq = 0;
	return b.latch;
}

// ---- the OKI's banks ----
static uint8_t oki_read_bank(void *u, uint32_t a)
{
	(void)u;
	uint32_t off = a;
	if (a >= 0x20000) {
		if (s_m == M_PSPIKESB || s_m == M_PSPIKESC)   // (oki_map: 0x20000 fixed, the bank from the ROM's start)
			off = (uint32_t)b.okibank * 0x20000 + (a - 0x20000);
		else if (s_m == M_AERFBOOT)                     // (init_banked_oki: the banks from 0x20000)
			off = 0x20000 + (uint32_t)b.okibank * 0x20000 + (a - 0x20000);
	}
	return off < s_oki_size ? s_oki_rom[off] : 0;
}

// ---- video registers ----
static void pspikes_gfxbank_w(uint8_t d) { b.gfxbank[0] = (d & 0xf0) >> 4; b.gfxbank[1] = d & 0x0f; }
static void kickball_gfxbank_w(uint8_t d) { b.gfxbank[0] = (d & 0x0f) & ~1; b.gfxbank[1] = d & 0x0f; }
static void karatblz_gfxbank_w(uint8_t d) { b.gfxbank[0] = d & 0x01; b.gfxbank[1] = (d & 0x08) >> 3; }
static void spinlbrk_gfxbank_w(uint8_t d) { b.gfxbank[0] = d & 0x07; b.gfxbank[1] = (d & 0x38) >> 3; }
static void turbofrc_gfxbank_w(int off, uint16_t d, uint16_t m)
{
	b.bank[off] = (uint16_t)((b.bank[off] & ~m) | (d & m));
	const uint16_t v = b.bank[off];
	for (int i = 0; i < 4; i++)
		b.gfxbank[4 * off + i] = (v >> (4 * i)) & 0x0f;
}
static void palette_bank_w(uint8_t d)
{
	b.sprpal = d & 0x03;
	b.charpal = (d & 0x1c) >> 2;
	b.flip = (d >> 7) & 1;
}
static inline void combine(uint16_t *p, uint16_t d, uint16_t m) { *p = (uint16_t)((*p & ~m) | (d & m)); }

// ---- the 68000's I/O ----
static uint16_t port(int p) { return p >= 0 ? (uint16_t)rbx_port_read(p) : 0xffff; }

static uint16_t io_r(uint32_t a)
{
	switch (s_m) {
	case M_PSPIKES: case M_PSPIKESB: case M_SPIKES91: case M_PSPIKESC: case M_KICKBALL: case M_WBBC97:
		switch (a) {
		case 0xfff000: return port(s_port_in[0]);
		case 0xfff002: return port(s_port_in[1]);
		case 0xfff004: return port(s_port_dsw[0]);
		case 0xfff006:
			if (s_m == M_PSPIKESB || s_m == M_PSPIKESC)
				return oki_read(&s_oki);
			return pending_r();
		}
		break;
	case M_KARATBLZ: case M_KARATBLZBL:
		switch (a) {
		case 0xff000: return port(s_port_in[0]);
		case 0xff002: return port(s_port_in[1]);
		case 0xff004: return port(s_port_in[2]);
		case 0xff006: return port(s_port_in[3]);
		case 0xff008: return port(s_port_dsw[0]);
		case 0xff00a: return pending_r();
		}
		break;
	case M_SPINLBRK:
		switch (a) {
		case 0xfff000: return port(s_port_in[0]);
		case 0xfff002: return port(s_port_in[1]);
		case 0xfff004: return port(s_port_dsw[0]);
		}
		break;
	default: {   // turbofrc_map (0xff000), aerofgtb_map and the bootlegs (0xfe000)
		const uint32_t base = s_m == M_TURBOFRC ? 0xff000 : 0xfe000;
		if (a < base || a >= base + 0x10)
			break;
		switch (a - base) {
		case 0x0: return port(s_port_in[0]);
		case 0x2: return port(s_port_in[1]);
		case 0x4: return port(s_port_dsw[0]);
		case 0x6:
			if (s_m == M_AERFBOO2)
				return (uint16_t)(oki_read(&s_oki) << 8);
			if (s_m == M_AERFBOOT)
				break;
			return pending_r();
		case 0x8: return s_m == M_TURBOFRC ? port(s_port_in[2]) : port(s_port_dsw[1]);
		}
		break;
	}
	}
	return 0;
}

static void io_w(uint32_t a, uint16_t d, uint16_t m)
{
	switch (s_m) {
	case M_PSPIKES: case M_PSPIKESB: case M_SPIKES91: case M_PSPIKESC: case M_KICKBALL: case M_WBBC97:
		switch (a) {
		case 0xfff000:
			if ((m & 0x00ff) && s_m != M_PSPIKESB && s_m != M_SPIKES91)
				palette_bank_w((uint8_t)d);
			return;
		case 0xfff002:
			if ((m & 0x00ff) && s_m != M_PSPIKESB)
				(s_m == M_KICKBALL ? kickball_gfxbank_w : pspikes_gfxbank_w)((uint8_t)d);
			return;
		case 0xfff004: combine(&b.scrolly[0], d, m); return;
		case 0xfff006:
			if (m & 0x00ff) {
				if (s_m == M_PSPIKESB || s_m == M_PSPIKESC)
					oki_write(&s_oki, (uint8_t)d);
				else
					latch_w((uint8_t)d);
			}
			return;
		case 0xfff008:
			if (s_m == M_PSPIKESB)
				b.okibank = d & 3;
			else if (s_m == M_SPIKES91)
				b.spikes91_lookup = d & 1;
			return;
		case 0xfff00e:
			if (s_m == M_WBBC97)
				combine(&b.bitmap_enable, d, m);
			return;
		}
		return;
	case M_KARATBLZ: case M_KARATBLZBL:
		switch (a) {
		case 0xff000: if (m & 0xff00) b.flip = (uint8_t)(d >> 15); return;
		case 0xff002: if (m & 0xff00) karatblz_gfxbank_w((uint8_t)(d >> 8)); return;
		case 0xff006: if (m & 0x00ff) latch_w((uint8_t)d); return;
		case 0xff008: combine(&b.scrollx[0], d, m); return;
		case 0xff00a: combine(&b.scrolly[0], d, m); return;
		case 0xff00c: combine(&b.scrollx[1], d, m); return;
		case 0xff00e: combine(&b.scrolly[1], d, m); return;
		}
		return;
	case M_SPINLBRK:
		switch (a) {
		case 0xfff000:
			if (m & 0xff00)
				b.flip = (uint8_t)(d >> 15);
			if (m & 0x00ff)
				spinlbrk_gfxbank_w((uint8_t)d);
			return;
		case 0xfff002: combine(&b.scrollx[1], d, m); return;
		case 0xfff006: if (m & 0x00ff) latch_w((uint8_t)d); return;
		}
		return;
	default: {
		const uint32_t base = s_m == M_TURBOFRC ? 0xff000 : 0xfe000;
		if (a < base || a >= base + 0x20)
			return;
		switch (a - base) {
		case 0x0:
			if ((m & 0x00ff) && (s_m == M_TURBOFRC || s_m == M_AEROFGTB))
				b.flip = (uint8_t)((d >> 7) & 1);
			return;
		case 0x2: combine(&b.scrolly[0], d, m); return;
		case 0x4: combine(&b.scrollx[1], d, m); return;
		case 0x6: combine(&b.scrolly[1], d, m); return;
		case 0x8: turbofrc_gfxbank_w(0, d, m); return;
		case 0xa: turbofrc_gfxbank_w(1, d, m); return;
		case 0xe:
			if (m & 0xff00) {
				if (s_m == M_AERFBOO2)
					oki_write(&s_oki, (uint8_t)(d >> 8));
				else
					latch_w((uint8_t)(d >> 8));
			}
			return;
		}
		return;
	}
	}
}

// the regions the page table does not take (read-only / write-only ones, the I/O)
static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	if (a & ~s_amask)   // (a mirror of global_mask(0xfffff): the first one's map, page table or handlers)
		return (uint16_t)m68k_read_memory_16(a & s_amask);
	return io_r(a);
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w(uint32_t a, uint16_t d, uint16_t m)
{
	a &= 0xfffffe;
	if (a & ~s_amask) {   // (a mirror: as main_r16)
		if (m == 0xffff)
			m68k_write_memory_16(a & s_amask, d);
		else if (m == 0x00ff)
			m68k_write_memory_8((a & s_amask) | 1, d & 0xff);
		else
			m68k_write_memory_8(a & s_amask, d >> 8);
		return;
	}
	if (s_m == M_PSPIKESB && a >= 0xffd000 && a < 0xffe000) {   // the raster RAM, 0xffd200 also the tile banks
		combine(&b.raster[(a & 0xfff) >> 1], d, m);
		if (a == 0xffd200) {   // pspikesb_gfxbank_w (the bus's data, not the combined word)
			b.gfxbank[0] = (uint8_t)((d & 0xf000) >> 12);
			b.gfxbank[1] = (uint8_t)((d & 0x0f00) >> 8);
		}
		return;
	}
	io_w(a, d, m);
}
static void main_w16(uint32_t a, uint16_t v) { main_w(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_w(a, v, 0x00ff);
	else
		main_w(a, (uint16_t)(v << 8), 0xff00);
}

static void mapm(uint32_t start, uint32_t end, void *mem, uint32_t size, int r, int w)
{
	const uint32_t step = s_amask + 1;
	// (ranges not of whole pages: only in the first mirror, the others through the handlers)
	const uint32_t top = ((start | (end + 1)) & 0xfff) ? step : 0x1000000;
	for (uint32_t o = 0; o < top; o += step)
		m68k_bus_map(start + o, end + o, mem, size, r, w);
}

static void map_rom(uint32_t end)
{
	const uint32_t n = end + 1 < s_rom_size ? end + 1 : s_rom_size;
	mapm(0, n - 1, s_rom, n, 1, 0);
}

static void map_mem(void)
{
	m68k_bus_reset_map();
	switch (s_m) {
	case M_PSPIKES: case M_PSPIKESB: case M_SPIKES91: case M_PSPIKESC: case M_KICKBALL: case M_WBBC97:
		if (s_m == M_WBBC97) {
			map_rom(0x3fffff);
			mapm(0x500000, 0x50ffff, b.ram, 0x10000, 1, 1);
			mapm(0x600000, 0x605fff, b.look[0], 0x6000, 1, 1);
			mapm(0xa00000, 0xa3ffff, b.bitmap, 0x40000, 1, 1);
		} else {
			map_rom(s_m == M_KICKBALL ? 0x7ffff : 0x3ffff);
			mapm(0x100000, 0x10ffff, b.ram, 0x10000, 1, 1);
			if (s_m == M_KICKBALL)
				mapm(0x200000, 0x20ffff, b.look[0], 0x10000, 1, 1);
			else
				mapm(0x200000, 0x203fff, b.look[0], 0x4000, 1, 1);
		}
		mapm(0xff8000, 0xff8fff, b.vram[0], 0x1000, 1, 1);
		if (s_m == M_SPIKES91)
			mapm(0xffa000, 0xffbfff, b.tx, 0x2000, 1, 1);
		if (s_m == M_PSPIKES || s_m == M_KICKBALL || s_m == M_WBBC97)
			mapm(0xffc000, 0xffc3ff, b.spr, 0x400, 0, 1);   // (write only)
		else
			mapm(0xffc000, 0xffc000 + (uint32_t)s_mc->spr_bytes - 1, b.spr, (uint32_t)s_mc->spr_bytes, 1, 1);
		mapm(0xffd000, 0xffdfff, b.raster, 0x1000, 1, s_m != M_PSPIKESB);
		mapm(0xffe000, 0xffefff, b.pal, 0x1000, 1, 1);
		break;
	case M_KARATBLZ: case M_KARATBLZBL:
		map_rom(0x7ffff);
		mapm(0x080000, 0x081fff, b.vram[0], 0x2000, 1, 1);
		mapm(0x082000, 0x083fff, b.vram[1], 0x2000, 1, 1);
		mapm(0x0a0000, 0x0affff, b.look[0], 0x10000, 1, 1);
		mapm(0x0b0000, 0x0bffff, b.look[1], 0x10000, 1, 1);
		mapm(0x0c0000, 0x0cffff, b.ram, 0x10000, 1, 1);
		mapm(0x0f8000, 0x0fbfff, b.ram2, 0x4000, 1, 1);
		mapm(0x0fc000, 0x0fc7ff, b.spr, 0x800, 1, 1);
		mapm(0x0fe000, 0x0fe7ff, b.pal, 0x800, 1, 1);
		break;
	case M_SPINLBRK:
		map_rom(0x3ffff);
		mapm(0x080000, 0x080fff, b.vram[0], 0x1000, 1, 1);
		mapm(0x082000, 0x082fff, b.vram[1], 0x1000, 1, 1);
		mapm(0xff8000, 0xffbfff, b.ram, 0x4000, 1, 1);
		mapm(0xffc000, 0xffc7ff, b.spr, 0x800, 1, 1);
		mapm(0xffd000, 0xffd1ff, b.raster, 0x200, 1, 1);
		mapm(0xffe000, 0xffe7ff, b.pal, 0x800, 1, 1);
		break;
	case M_TURBOFRC:
		map_rom(0xbffff);
		mapm(0x0c0000, 0x0cffff, b.ram, 0x10000, 1, 1);
		mapm(0x0d0000, 0x0d1fff, b.vram[0], 0x2000, 1, 1);
		mapm(0x0d2000, 0x0d3fff, b.vram[1], 0x2000, 1, 1);
		mapm(0x0e0000, 0x0e3fff, b.look[0], 0x4000, 1, 1);
		mapm(0x0e4000, 0x0e7fff, b.look[1], 0x4000, 1, 1);
		mapm(0x0f8000, 0x0fbfff, b.ram2, 0x4000, 1, 1);
		mapm(0x0fc000, 0x0fc7ff, b.spr, 0x800, 1, 1);
		mapm(0x0fd000, 0x0fdfff, b.raster, 0x1000, 1, 1);
		mapm(0x0fe000, 0x0fe7ff, b.pal, 0x800, 1, 1);
		break;
	default:   // aerofgtb_map, aerfboot_map, aerfboo2_map
		map_rom(0x7ffff);
		mapm(0x0c0000, 0x0cffff, b.ram, 0x10000, 1, 1);
		mapm(0x0d0000, 0x0d1fff, b.vram[0], 0x2000, 1, 1);
		mapm(0x0d2000, 0x0d3fff, b.vram[1], 0x2000, 1, 1);
		mapm(0x0e0000, 0x0e3fff, b.look[0], 0x4000, 1, 1);
		mapm(0x0e4000, 0x0e7fff, b.look[1], 0x4000, 1, 1);
		mapm(0x0f8000, 0x0fbfff, b.ram2, 0x4000, 1, 1);
		if (s_m == M_AERFBOOT) {
			mapm(0x0fc000, 0x0fc7ff, b.ram3, 0x800, 1, 1);
			mapm(0x108000, 0x10bfff, b.spr, 0x4000, 1, 1);
		} else
			mapm(0x0fc000, 0x0fc7ff, b.spr, 0x800, 1, 1);
		mapm(0x0fd000, 0x0fd7ff, b.pal, 0x800, 1, 1);
		mapm(0x0ff000, 0x0fffff, b.raster, 0x1000, 1, 1);
		break;
	}
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
}

// ---- the Z80 ----
static void z_bank(void)
{
	if (s_mc->zmap != Z_BANKED_A && s_mc->zmap != Z_BANKED_B)
		return;
	for (int p = 0x80; p < 0x100; p++) {
		const uint32_t o = (uint32_t)b.zbank * 0x8000 + (uint32_t)(p - 0x80) * 256;
		s_zmap[p] = o + 256 <= s_zbank_size ? s_zbank_rom + o : NULL;
	}
}
static void z_setup_map(void)
{
	memset(s_zmap, 0, sizeof(s_zmap));
	int rom_pages;
	switch (s_mc->zmap) {
	case Z_BANKED_A: case Z_BANKED_B: rom_pages = 0x78; break;
	case Z_AERFBOOT: rom_pages = 0x80; break;
	case Z_SPIKES91: rom_pages = 0xe0; break;
	default: rom_pages = 0xf0; break;
	}
	for (int p = 0; p < rom_pages && (uint32_t)p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	z_bank();
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	switch (s_mc->zmap) {
	case Z_BANKED_A: case Z_BANKED_B:
		if (a >= 0x7800 && a < 0x8000) return b.zram[a - 0x7800];
		break;
	case Z_AERFBOOT:
		if (a >= 0x8000 && a < 0x8800) return b.zram[a - 0x8000];
		if (a == 0x9800) return oki_read(&s_oki);
		if (a == 0xa000) return latch_read_clear();
		break;
	case Z_WBBC97:
		if (a >= 0xf000 && a < 0xf800) return b.zram[a - 0xf000];
		if (a == 0xf800) return oki_read(&s_oki);
		if (a == 0xfc20) return latch_read_clear();
		break;
	case Z_KARATBLZBL: case Z_KICKBALL:
		if (a >= 0xf000 && a < 0xf800) return b.zram[a - 0xf000];
		if (a == 0xf800) return s_mc->zmap == Z_KICKBALL ? b.latch : latch_read_clear();
		break;
	case Z_SPIKES91:
		if (a >= 0xf800) return b.zram[a - 0xf800];
		if (a == 0xe000 || a == 0xe001) return rbx_ym_read(0, a & 1);
		if (a == 0xe800) return latch_read_clear();
		break;
	}
	return 0;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	switch (s_mc->zmap) {
	case Z_BANKED_A: case Z_BANKED_B:
		if (a >= 0x7800 && a < 0x8000) b.zram[a - 0x7800] = v;
		break;
	case Z_AERFBOOT:
		if (a >= 0x8000 && a < 0x8800) b.zram[a - 0x8000] = v;
		else if (a == 0x9000) { if (v & 4) b.okibank = v & 3; }   // (aerfboot_okim6295_banking_w)
		else if (a == 0x9800) oki_write(&s_oki, v);
		break;
	case Z_WBBC97:
		if (a >= 0xf000 && a < 0xf800) b.zram[a - 0xf000] = v;
		else if (a == 0xf800) oki_write(&s_oki, v);
		else if (a == 0xf810 || a == 0xf811) rbx_ym_write(0, a & 1, v);
		break;
	case Z_KARATBLZBL: case Z_KICKBALL:
		if (a >= 0xf000 && a < 0xf800) b.zram[a - 0xf000] = v;
		break;
	case Z_SPIKES91:
		if (a >= 0xf800) b.zram[a - 0xf800] = v;
		else if (a == 0xe000 || a == 0xe001) rbx_ym_write(0, a & 1, v);
		break;
	}
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (s_mc->zmap) {
	case Z_BANKED_A:
		if (p == 0x14) return b.latch;
		if (p >= 0x18 && p <= 0x1b) return rbx_ym_read(0, p & 3);
		break;
	case Z_BANKED_B:
		if (p <= 0x03) return rbx_ym_read(0, p & 3);
		if (p == 0x0c) return b.latch;
		break;
	case Z_KARATBLZBL:
		if (p == 0x00) return rbx_ym_read(0, 0);
		break;
	case Z_KICKBALL:
		if (p == 0x00) return rbx_ym_read(0, 0);
		if (p == 0x40) return oki_read(&s_oki);
		break;
	}
	return 0;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (s_mc->zmap) {
	case Z_BANKED_A:
		if (p == 0x00) { b.zbank = v & 3; z_bank(); }
		else if (p == 0x14) b.pending = 0;
		else if (p >= 0x18 && p <= 0x1b) rbx_ym_write(0, p & 3, v);
		break;
	case Z_BANKED_B:
		if (p <= 0x03) rbx_ym_write(0, p & 3, v);
		else if (p == 0x04) { b.zbank = v & 3; z_bank(); }
		else if (p == 0x08) b.pending = 0;
		break;
	case Z_KARATBLZBL:
		if (p == 0x00) rbx_ym_write(0, 0, v);
		else if (p == 0x20) rbx_ym_write(0, 1, v);
		else if (p == 0x40) { upd7759_port_w(&s_upd, v); upd7759_start_w(&s_upd, 0); upd7759_start_w(&s_upd, 1); }
		else if (p == 0x80) upd7759_reset_w(&s_upd, (v >> 7) & 1);
		break;
	case Z_KICKBALL:
		if (p == 0x00) rbx_ym_write(0, 0, v);
		else if (p == 0x20) rbx_ym_write(0, 1, v);
		else if (p == 0x40) oki_write(&s_oki, v);
		else if (p == 0xc0) b.pending = 0;
		break;
	}
}
static void ym_irq(int state)
{
	if (s_mc->zmap != Z_SPIKES91)
		s_zirq = state;
}
static void z_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}

// ---- sprites: the draw list ----
static int s_vx0, s_vx1, s_vy0, s_vy1;
static void op_add(int gfx, uint32_t code, uint32_t color, int fx, int fy, int sx, int sy, uint32_t zx, uint32_t zy,
                   int use_pri, uint32_t pmask)
{
	const int dw = (int)((zx * 16 + 0x8000) >> 16), dh = (int)((zy * 16 + 0x8000) >> 16);
	if (sx > s_vx1 || sx + dw - 1 < s_vx0 || sy > s_vy1 || sy + dh - 1 < s_vy0 || dw < 1 || dh < 1)
		return;
	if (s_snap->nops >= MAX_OPS)
		return;
	op_t *o = &s_snap->ops[s_snap->nops++];
	o->sx = (int16_t)sx;
	o->sy = (int16_t)sy;
	o->gfx = (uint8_t)gfx;
	o->flags = (uint8_t)((fx ? 1 : 0) | (fy ? 2 : 0) | (use_pri ? 4 : 0));
	o->color = (uint16_t)color;
	o->code = code;
	o->zx = zx;
	o->zy = zy;
	o->pmask = pmask;
}

// a chip's tile code through its look-up (pspikes_tile_callback, pspikes_tile2_callback, spinbrk_tile_callback)
enum { LOOK_RAM0, LOOK_RAM1, LOOK_NONE, LOOK_ROM };
static inline uint32_t look(int how, uint32_t code)
{
	switch (how) {
	case LOOK_RAM0: return b.look[0][code % s_look_n[0]];
	case LOOK_RAM1: return b.look[1][code % s_look_n[1]];
	case LOOK_ROM: return s_lookrom_n ? rbx_be16(s_lookrom + 2 * (code % s_lookrom_n)) : 0;
	default: return code;
	}
}

// vsystem_spr2_device::draw_sprites_common (pritype 0: turbo force etc., 1: spinlbrk)
static void spr2_draw(int gfx, const uint16_t *ram, int palbank, int pri_param, int flip, int pritype, int xoffs, int yoffs,
                      int how)
{
	int first = 4 * ram[0x1fe];
	first &= 0x1ff;
	if (first > 0x200 - 4)
		first = 0x200 - 4;
	for (int a = 0x200 - 8; a != first - 4; a -= 4) {
		const uint16_t *r = ram + a;
		if (!(r[2] & 0x0080))
			continue;
		int oy = r[0] & 0x01ff, zoomy = (r[0] & 0xf000) >> 12;
		int ox = r[1] & 0x01ff, zoomx = (r[1] & 0xf000) >> 12;
		const int xsize = (r[2] & 0x0700) >> 8, flipx = (r[2] >> 11) & 1;
		const int ysize = (r[2] & 0x7000) >> 12, flipy = (r[2] >> 15) & 1;
		uint32_t color = r[2] & 0x000f;
		const uint32_t pri = r[2] & 0x0010;
		uint32_t map = r[3];
		ox += xoffs;
		oy += yoffs;
		if ((int)(pri >> 4) != pri_param)
			continue;
		const uint32_t usepri = pritype == 0 ? (pri ? 0 : 2) : (pri ? 2 : 0);
		int fx = flipx, fy = flipy;
		if (flip) {
			ox = 308 - ox;
			oy = 208 - oy;
			fx = !fx;
			fy = !fy;
		}
		color += 16 * (uint32_t)palbank;
		zoomx = 32 - zoomx;
		zoomy = 32 - zoomy;
		const uint32_t zx = (uint32_t)zoomx << 11, zy = (uint32_t)zoomy << 11;
		for (int y = 0; y <= ysize; y++) {
			const int cy = flipy ? ysize - y : y;
			const int sy = ((oy + zoomy * (flip ? -cy : cy) / 2 + 16) & 0x1ff) - 16;
			for (int x = 0; x <= xsize; x++) {
				const int cx = flipx ? xsize - x : x;
				const int sx = ((ox + zoomx * (flip ? -cx : cx) / 2 + 16) & 0x1ff) - 16;
				const uint32_t curr = look(how, map++);
				op_add(gfx, curr, color, fx, fy, sx, sy, zx, zy, 1, usepri);
				op_add(gfx, curr, color, fx, fy, sx - 0x200, sy, zx, zy, 1, usepri);
				op_add(gfx, curr, color, fx, fy, sx, sy - 0x200, zx, zy, 1, usepri);
				op_add(gfx, curr, color, fx, fy, sx - 0x200, sy - 0x200, zx, zy, 1, usepri);
			}
			if (xsize == 2) map += 1;
			if (xsize == 4) map += 3;
			if (xsize == 5) map += 2;
			if (xsize == 6) map += 1;
		}
	}
}

// pspikesb_draw_sprites
static void pspikesb_sprites(void)
{
	const uint16_t *s = b.spr;
	for (int i = 4; i < s_mc->spr_bytes / 2; i += 4) {
		if (s[i + 3 - 4] & 0x8000)
			break;
		const int xpos = (s[i + 2] & 0x1ff) - 34;
		const int ypos = 256 - (s[i + 3 - 4] & 0x1ff) - 33;
		const uint32_t code = s[i + 0] & 0x1fff;
		const int flipx = (s[i + 1] >> 11) & 1;
		const uint32_t color = s[i + 1] & 0x000f;
		op_add(1, code, color, flipx, 0, xpos, ypos, 0x10000, 0x10000, 0, 0);
		op_add(1, code, color, flipx, 0, xpos, ypos + 512, 0x10000, 0x10000, 0, 0);
	}
}

// spikes91_draw_sprites and the text layer of screen_update_spikes91
static void spikes91_sprites(void)
{
	const uint16_t *s = b.spr;
	b.sprpal = 1;
	for (int i = s_mc->spr_bytes / 2 - 4; i >= 4; i -= 4) {
		uint32_t code = s[i + 0] & 0x1fff;
		if (!code)
			continue;
		const int xpos = (s[i + 2] & 0x01ff) - 16;
		const int ypos = 256 - (s[i + 1] & 0x00ff) - 26;
		const int flipx = (s[i + 3] >> 15) & 1;
		const uint32_t color = (s[i + 3] & 0x00f0) >> 4;
		code |= (uint32_t)b.spikes91_lookup << 13;
		const uint32_t t = s_lookrom_n ? rbx_be16(s_lookrom + 2 * (code % s_lookrom_n)) : 0;
		op_add(1, t, color, flipx, 0, xpos, ypos, 0x10000, 0x10000, 0, 0);
		op_add(1, t, color, flipx, 0, xpos, ypos + 512, 0x10000, 0x10000, 0, 0);
	}
	int count = 0;
	for (int y = 0; y < 32; y++)
		for (int x = 0; x < 64; x++, count++) {
			const uint16_t v = b.tx[count];
			op_add(0, v & 0x1fff, (v & 0xe000) >> 13, 0, 0, x * 8 + 24, y * 8 + 8, 0x10000, 0x10000, 0, 0);
		}
}

// aerfboot_draw_sprites (gfx 2: "gfx2", 3: "spritegfx")
static void aerfboot_sprites(void)
{
	const uint16_t *s = b.spr;
	for (int pass = 0; pass < 2; pass++) {
		int last = (((int)b.raster[(pass ? 0x402 : 0x404) / 2] << 5) - 0x8000) / 2;
		if (last < 0)
			last = 0;
		const int top = pass ? (s_mc->spr_bytes / 2) / 2 - 4 : s_mc->spr_bytes / 2 - 4;
		for (int a = top; a >= last; a -= 4) {
			const int ox = s[a + 1] & 0x01ff, oy = s[a + 0] & 0x01ff;
			const int flipx = (s[a + 2] >> 11) & 1, flipy = (s[a + 2] >> 15) & 1;
			const uint32_t color = s[a + 2] & 0x000f;
			const int zoomx = 32 + ((s[a + 1] & 0xf000) >> 12), zoomy = 32 + ((s[a + 0] & 0xf000) >> 12);
			const int pri = (s[a + 2] >> 4) & 1;
			uint32_t code = s[a + 3] & 0x1fff;
			if (!(s[a + 2] & 0x40))
				code |= 0x2000;
			const int sy = ((oy + 16 - 1) & 0x1ff) - 16, sx = ((ox + 16 + 3) & 0x1ff) - 16;
			op_add(code >= 0x1000 ? 0 : 1, code, color, flipx, flipy, sx, sy, (uint32_t)zoomx << 11, (uint32_t)zoomy << 11,
			       1, pri ? 0 : 2);
		}
	}
}

// aerfboo2_draw_sprites (gfx 2 + chip)
static void aerfboo2_sprites(int chip, int chip_disabled_pri)
{
	const uint16_t *s = b.spr;
	const int base = chip * 0x200;
	for (int a = base + 0x200 - 4; a >= base; a -= 4) {
		if (!(s[a + 2] & 0x80))
			continue;
		const int pri = (s[a + 2] >> 4) & 1;
		if (chip_disabled_pri && !pri)
			continue;
		if (!chip_disabled_pri && pri)
			continue;
		const int ox = s[a + 1] & 0x01ff, xsize = (s[a + 2] & 0x0700) >> 8;
		const int zoomx = 32 - ((s[a + 1] & 0xf000) >> 12);
		const int oy = s[a + 0] & 0x01ff, ysize = (s[a + 2] & 0x7000) >> 12;
		const int zoomy = 32 - ((s[a + 0] & 0xf000) >> 12);
		const int flipx = (s[a + 2] >> 11) & 1, flipy = (s[a + 2] >> 15) & 1;
		const uint32_t color = (s[a + 2] & 0x000f) + 16 * (uint32_t)b.sprpal;
		uint32_t map = s[a + 3];
		for (int y = 0; y <= ysize; y++) {
			const int sy = flipy ? ((oy + zoomy * (ysize - y) / 2 + 16) & 0x1ff) - 16 : ((oy + zoomy * y / 2 + 16) & 0x1ff) - 16;
			for (int x = 0; x <= xsize; x++) {
				const int sx = flipx ? ((ox + zoomx * (xsize - x) / 2 + 16) & 0x1ff) - 16 : ((ox + zoomx * x / 2 + 16) & 0x1ff) - 16;
				const uint32_t code = b.look[chip][map % s_look_n[chip]];
				op_add(chip, code, color, flipx, flipy, sx, sy, (uint32_t)zoomx << 11, (uint32_t)zoomy << 11, 1, pri ? 0 : 2);
				map++;
			}
			if (xsize == 2) map += 1;
			if (xsize == 4) map += 3;
			if (xsize == 5) map += 2;
			if (xsize == 6) map += 1;
		}
	}
}

// ---- the layers (the screen updates' tilemap set-up) ----
static void layer_init(int l, int cols, int rows, uint16_t code_mask, int bank_shift, int bank_mask, int color_shift,
                       int color_add, int opaque, int priority, int scrollrows)
{
	vsg_layer *y = &s_snap->lay[l];
	memset(y, 0, sizeof(*y));
	y->vram = s_snap->vram[l];
	y->gfx = &s_tiles[l];
	y->cols = cols;
	y->rows = rows;
	y->code_mask = code_mask;
	y->bank_shift = bank_shift;
	y->bank_mask = bank_mask;
	y->color_shift = color_shift;
	y->color_add = color_add;
	y->opaque = opaque;
	y->priority = (uint8_t)priority;
	y->flip = b.flip ? 3 : 0;
	y->scrollrows = scrollrows;
	y->rowscroll = s_snap->rowscroll[l];
	y->colscroll = b.colscroll[l];
	y->xextent = s_mc->vx1 + s_mc->vx0 + 1;
	y->yextent = s_mc->vy1 + s_mc->vy0 + 1;
}

static void build_frame(void)
{
	const int flip = b.flip;
	s_snap->nops = 0;
	s_snap->nlay = 0;
	s_snap->bitmap_on = 0;
	switch (s_m) {
	case M_PSPIKES: case M_PSPIKESB: case M_SPIKES91: case M_PSPIKESC: case M_KICKBALL: case M_WBBC97: {
		const int scrolly = b.scrolly[0];
		for (int i = 0; i < 256; i++) {
			int32_t v;
			if (s_m == M_PSPIKESB) v = b.raster[i] + 22;
			else if (s_m == M_SPIKES91) v = b.raster[i + 0x01f0 / 2] + 0x96 + 0x16;
			else v = b.raster[i];
			b.rowscroll[0][(i + scrolly) & 0xff] = v;
		}
		b.colscroll[0] = scrolly;
		layer_init(0, 64, 32, 0x0fff, 12, 1, 13, 8 * b.charpal, s_m != M_WBBC97 || !b.bitmap_enable, 0, 256);
		for (int i = 0; i < 2; i++)
			s_snap->lay[0].bank[i] = (uint32_t)b.gfxbank[i] << 12;
		s_snap->nlay = 1;
		if (s_m == M_WBBC97 && b.bitmap_enable) {
			s_snap->bitmap_on = 1;
			memcpy(s_snap->bitmap, b.bitmap, sizeof(b.bitmap));
			memcpy(s_snap->raster, b.raster, sizeof(s_snap->raster));
		}
		if (s_m == M_PSPIKESB)
			pspikesb_sprites();
		else if (s_m == M_SPIKES91)
			spikes91_sprites();
		else {
			spr2_draw(0, b.spr, b.sprpal, 1, flip, 0, 0, 0, LOOK_RAM0);
			spr2_draw(0, b.spr, b.sprpal, 0, flip, 0, 0, 0, LOOK_RAM0);
		}
		break;
	}
	case M_KARATBLZ: case M_KARATBLZBL:
		b.rowscroll[0][0] = b.scrollx[0] - 8;
		b.colscroll[0] = b.scrolly[0];
		b.rowscroll[1][0] = b.scrollx[1] - 4;
		b.colscroll[1] = b.scrolly[1];
		layer_init(0, 64, 64, 0x1fff, 0, 0, 13, 0, 1, 0, 1);
		layer_init(1, 64, 64, 0x1fff, 0, 0, 13, 0, 0, 0, 1);
		s_snap->lay[0].bank[0] = (uint32_t)b.gfxbank[0] << 13;
		s_snap->lay[1].bank[0] = (uint32_t)b.gfxbank[1] << 13;
		s_snap->nlay = 2;
		spr2_draw(1, b.spr + 0x200, b.sprpal, 1, flip, 0, 0, 0, LOOK_RAM1);
		spr2_draw(1, b.spr + 0x200, b.sprpal, 0, flip, 0, 0, 0, LOOK_RAM1);
		spr2_draw(0, b.spr, b.sprpal, 1, flip, 0, 0, 0, LOOK_RAM0);
		spr2_draw(0, b.spr, b.sprpal, 0, flip, 0, 0, 0, LOOK_RAM0);
		break;
	case M_SPINLBRK:
		for (int i = 0; i < 256; i++)
			b.rowscroll[0][i & 0x1ff] = b.raster[i] - 8;
		b.rowscroll[1][0] = b.scrollx[1] - 4;
		layer_init(0, 64, 64, 0x0fff, 0, 0, 12, 0, 1, 0, 512);
		layer_init(1, 64, 64, 0x1fff, 0, 0, 13, 0, 0, 1, 1);
		s_snap->lay[0].bank[0] = (uint32_t)b.gfxbank[0] << 12;
		s_snap->lay[1].bank[0] = (uint32_t)b.gfxbank[1] << 13;
		s_snap->nlay = 2;
		spr2_draw(0, b.spr, b.sprpal, 0, flip, 1, 0, 0, LOOK_NONE);
		spr2_draw(0, b.spr, b.sprpal, 1, flip, 1, 0, 0, LOOK_NONE);
		spr2_draw(1, b.spr + 0x200, b.sprpal, 0, flip, 1, 0, 0, LOOK_ROM);
		spr2_draw(1, b.spr + 0x200, b.sprpal, 1, flip, 1, 0, 0, LOOK_ROM);
		break;
	default: {   // screen_update_turbofrc (turbofrc, aerofgtb), _aerfboot, _aerfboo2
		const int scrolly = b.scrolly[0] + 2;
		int32_t rs, l1x, l1y, l0y;
		if (s_m == M_AERFBOOT) {
			rs = b.raster[7] + 174; l0y = scrolly; l1x = b.scrollx[1] + 172; l1y = b.scrolly[1] + 2;
		} else if (s_m == M_AERFBOO2) {
			rs = b.raster[7] - 11; l0y = scrolly; l1x = b.scrollx[1] - 7; l1y = b.scrolly[1] + 2;
		} else {
			rs = b.raster[7] - 11 - (flip ? 188 : 0);
			l0y = scrolly - (flip ? 2 : 0);
			l1x = b.scrollx[1] - (flip ? 185 : 7);
			l1y = b.scrolly[1] + (flip ? 0 : 2);
		}
		for (int i = 0; i < 256; i++)
			b.rowscroll[0][(i + scrolly) & 0x1ff] = rs;
		b.colscroll[0] = l0y;
		b.rowscroll[1][0] = l1x;
		b.colscroll[1] = l1y;
		for (int l = 0; l < 2; l++) {
			layer_init(l, 64, 64, 0x07ff, 11, 3, 13, 0, l == 0, l, l == 0 ? 512 : 1);
			for (int i = 0; i < 4; i++)
				s_snap->lay[l].bank[i] = (uint32_t)b.gfxbank[l * 4 + i] << 11;
			if (s_m == M_AEROFGTB)
				s_snap->lay[l].dx = s_snap->lay[l].dx_flipped = 1;
		}
		s_snap->nlay = 2;
		if (s_m == M_AERFBOOT)
			aerfboot_sprites();
		else if (s_m == M_AERFBOO2) {
			aerfboo2_sprites(1, -1);
			aerfboo2_sprites(1, 0);
			aerfboo2_sprites(0, -1);
			aerfboo2_sprites(0, 0);
		} else {
			const int xo = s_m == M_AEROFGTB ? 3 : 0, yo = s_m == M_AEROFGTB ? -1 : 0;
			spr2_draw(1, b.spr + 0x200, b.sprpal, 1, flip, 0, xo, yo, LOOK_RAM1);
			spr2_draw(1, b.spr + 0x200, b.sprpal, 0, flip, 0, xo, yo, LOOK_RAM1);
			spr2_draw(0, b.spr, b.sprpal, 1, flip, 0, xo, yo, LOOK_RAM0);
			spr2_draw(0, b.spr, b.sprpal, 0, flip, 0, xo, yo, LOOK_RAM0);
		}
		break;
	}
	}
	for (int l = 0; l < s_snap->nlay; l++) {
		memcpy(s_snap->vram[l], b.vram[l], sizeof(b.vram[l]));
		memcpy(s_snap->rowscroll[l], b.rowscroll[l], sizeof(b.rowscroll[l]));
	}
}

static void vblank(void)
{
	rbx_video_wait_idle();
	const int n = s_mc->pal_n;
	for (int i = 0; i < n; i++) {   // xRGB 555
		const uint16_t v = b.pal[i];
		if (s_pal_valid && s_pal_conv[i] == v)
			continue;
		s_pal_conv[i] = v;
		s_snap->pens[i] = (uint32_t)(pal5((v >> 10) & 31) << 16 | pal5((v >> 5) & 31) << 8 | pal5(v & 31));
	}
	s_pal_valid = 1;
	build_frame();
	rbx_video_frame_done();
}

// ---- render ----
static void band(int j)
{
	const int h = s_mc->vy1 - s_mc->vy0 + 1, w = s_mc->vx1 - s_mc->vx0 + 1;
	const int y0 = s_mc->vy0 + h * j / BANDS, y1 = s_mc->vy0 + h * (j + 1) / BANDS - 1;
	if (y1 < y0)
		return;
	const int pitch = rbx_video.pitch;
	for (int y = y0; y <= y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - s_mc->vy0) * pitch;
		uint8_t *p = s_pri + (y - s_mc->vy0) * pitch;
		memset(p, 0, (size_t)w);
		if (s_snap->bitmap_on) {   // wbbc97 draw_bitmap
			const int r = s_snap->raster[y & 0x7f];
			for (int x = s_mc->vx0; x <= s_mc->vx1; x++) {
				const int bx = (x - 10 + r) & 0x1ff;
				d[x - s_mc->vx0] = (uint16_t)(2048 + ((s_snap->bitmap[(16 + y * 512 + bx) & 0x1ffff] >> 1) & 0x7fff));
			}
		}
		for (int l = 0; l < s_snap->nlay; l++)
			vsg_layer_line(&s_snap->lay[l], y, s_mc->vx0, s_mc->vx1, d, p);
	}
	const vsg_target t = { rbx_video.bitmap, s_pri, pitch, s_mc->vx0, s_mc->vy0, s_mc->vx0, s_mc->vx1, y0, y1 };
	const op_t *o = s_snap->ops;
	for (int i = 0; i < s_snap->nops; i++, o++) {
		if (o->sy > y1 || o->sy + 48 < y0)
			continue;   // (no sprite tile is taller than 47 lines)
		vsg_zoom(&s_sprg[o->gfx], &t, o->code, o->color, o->flags & 1, (o->flags >> 1) & 1, o->sx, o->sy, o->zx, o->zy,
		         (o->flags >> 2) & 1, o->pmask);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, band };
static int ps_help(int helper) { return decoband_help(&s_bands, helper); }

static void ps_render(void)
{
	memcpy(rbx_video.palette, s_snap->pens, sizeof(uint32_t) * (size_t)s_mc->pal_n);
	if (s_bmpens && !s_bmpens_done) {
		memcpy(rbx_video.palette + 2048, s_bmpens, sizeof(uint32_t) * 0x8000);
		s_bmpens_done = 1;
	}
	decoband_run(&s_bands);
}

// ---- sound ----
static void sound_line(void)
{
	int n;
	int32_t *l = s_mix_l + s_mix_n, *r = s_mix_r + s_mix_n;
	const int room = s_mix_cap - s_mix_n;
	if (s_has_ym == 2) {   // YM2610: both sides
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, 8000000, s_mc->fps_mhz, s_mc->lines);
		n = rbx_ym_advance_lr(0, clocks, l, r, room);
	} else if (s_has_ym) {
		const uint32_t clk = s_mc->zmap == Z_SPIKES91 ? 3000000 : s_mc->zmap == Z_WBBC97 ? 3579545 : 4000000;
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, clk, s_mc->fps_mhz, s_mc->lines);
		n = rbx_ym_advance(0, clocks, l, room);
	} else {
		n = rbx_line_cycles(&b.out_frac, s_out_rate, s_mc->fps_mhz, s_mc->lines);
		if (n > room)
			n = room;
	}
	if (n <= 0)
		return;
	if (s_has_oki)
		for (int i = 0; i < n; i++) {
			b.oki_acc += s_oki.rate;
			while (b.oki_acc >= s_out_rate) {
				b.oki_acc -= s_out_rate;
				b.oki_hold = oki_sample(&s_oki);
			}
			l[i] += b.oki_hold;
		}
	if (s_has_upd) {
		upd7759_render(&s_upd, s_tmp, n);
		for (int i = 0; i < n; i++)
			l[i] += s_tmp[i];
	}
	s_mix_n += n;
}

// ---- the frame ----
static void ps_frame(void)
{
	const machine_t *mc = s_mc;
	rbx_audio.frame_mhz = mc->fps_mhz;
	const int vbl = mc->vy1 + 1;
	for (int line = 0; line < mc->lines; line++) {
		if (line == vbl) {
			vblank();
			m68k_bus_hold_irq(s_m == M_AERFBOO2 ? 2 : 1);
		}
		const int mc_cyc = rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, mc->fps_mhz, mc->lines);
		s_cyc_base += m68k_execute(mc_cyc);
		if (mc->z80_clock) {
			b.z80_target += (uint64_t)rbx_line_cycles(&b.z80_frac, mc->z80_clock, mc->fps_mhz, mc->lines);
			z80_run(&s_z80, (unsigned long)b.z80_target, &s_zirq, 0);
		}
		sound_line();
	}
	if (s_has_ym == 1)
		rbx_ym_async_take(0, s_mix_l, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_has_ym == 2 ? s_mix_r[i] : s_mix_l[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void ps_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	map_mem();
	if (s_mc->z80_clock) {
		z_setup_map();
		z_start();
		if (s_has_ym)
			rbx_ym_reset(0);
	}
	s_zirq = 0;
	if (s_has_oki)
		oki_reset(&s_oki);
	if (s_has_upd)
		upd7759_reset(&s_upd);
	s_pal_valid = 0;
	s_bmpens_done = 0;
	s_mix_n = 0;
	s_cyc_base = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void ps_exit(void)
{
	rbx_video_wait_idle();
	m68k_set_mame_timing(0);
	if (s_has_ym)
		rbx_ym_exit(0);
	s_has_ym = 0;
	for (int i = 0; i < 2; i++) {
		vsg_free(&s_tiles[i]);
		vsg_free(&s_sprg[i]);
	}
	free(s_snap);
	s_snap = NULL;
	free(s_pri);
	s_pri = NULL;
	free(s_bmpens);
	s_bmpens = NULL;
	free(s_mix_l);
	free(s_mix_r);
	free(s_tmp);
	s_mix_l = s_mix_r = s_tmp = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
}

static int machine_of(const char *machine)
{
	for (int i = 0; i < M_COUNT; i++) {
		const size_t l = strlen(s_mnames[i]);
		if (!strncmp(machine, s_mnames[i], l) && (machine[l] == ':' || !machine[l]))
			return i;
	}
	return -1;
}
static int ps_supported(const char *machine) { return machine_of(machine) >= 0; }

// the layouts of the bootlegs (RGN_FRAC worked out for the region's size in bits)
static void lay_frac4(vsg_layout *l, int w, uint32_t bits, int rev)   // planes at the region's quarters
{
	for (int p = 0; p < 4; p++)
		l->planeoffs[p] = (uint32_t)(rev ? 3 - p : p) * (bits / 4);
	l->w = l->h = w;
	l->planes = 4;
}

static int decode_gfx(void)
{
	uint32_t s1 = 0, s2 = 0, s3 = 0, s4 = 0;
	uint8_t *g1 = rbx_region("gfx1", &s1), *g2 = rbx_region("gfx2", &s2);
	uint8_t *g3 = rbx_region("spritegfx", &s3), *g4 = rbx_region("gfx4", &s4);
	vsg_layout l;
	memset(&l, 0, sizeof(l));
	switch (s_m) {
	case M_PSPIKES: case M_PSPIKESC: case M_WBBC97:
		if (!g1 || !g2) return -1;
		return vsg_packed(&s_tiles[0], g1, s1, 8, 0, 0, 64) || vsg_packed(&s_sprg[0], g2, s2, 16, 0, 1024, 64);
	case M_KICKBALL:
		if (!g1 || !g2) return -1;
		for (uint32_t i = 0; i < 0x80000 && i < s2; i++) {   // init_kickball: bits 5 and 6 of a ROM swapped
			const uint8_t v = g2[i];
			g2[i] = (uint8_t)((v & 0x9f) | ((v & 0x20) << 1) | ((v & 0x40) >> 1));
		}
		if (vsg_packed(&s_tiles[0], g1, s1, 8, 0, 0, 64))
			return -1;
		{   // kickball_spritelayout
			static const uint32_t xo[16] = { 6, 7, 4, 5, 2, 3, 0, 1, 14, 15, 12, 13, 10, 11, 8, 9 };
			lay_frac4(&l, 16, s2 * 8, 0);
			memcpy(l.xoffs, xo, sizeof(xo));
			for (int i = 0; i < 16; i++) l.yoffs[i] = (uint32_t)i * 16;
			l.charinc = 32 * 8;
			return vsg_decode(&s_sprg[0], &l, g2, s2, 0, (s2 * 8 / l.charinc) / 4, 1024, 64);
		}
	case M_PSPIKESB: case M_SPIKES91:
		if (!g1 || !g2) return -1;
		for (uint32_t i = 0; i < s2; i++)   // (ROMREGION_INVERT)
			g2[i] = (uint8_t)~g2[i];
		if (s_m == M_PSPIKESB)
			for (uint32_t i = 0; i < s1; i++)
				g1[i] = (uint8_t)~g1[i];
		lay_frac4(&l, 8, s1 * 8, 0);   // pspikesb_charlayout
		for (int i = 0; i < 8; i++) { l.xoffs[i] = (uint32_t)i; l.yoffs[i] = (uint32_t)i * 8; }
		l.charinc = 8 * 8;
		if (vsg_decode(&s_tiles[0], &l, g1, s1, 0, (s1 * 8 / l.charinc) / 4, 0, 64))
			return -1;
		// (spikes91: gfx 0 for the text layer, drawn as sprites: the same element set)
		if (s_m == M_SPIKES91 && vsg_decode(&s_sprg[0], &l, g1, s1, 0, (s1 * 8 / l.charinc) / 4, 0, 64))
			return -1;
		memset(&l, 0, sizeof(l));
		if (s_m == M_PSPIKESB) {   // pspikesb_spritelayout
			lay_frac4(&l, 16, s2 * 8, 0);
			for (int i = 0; i < 8; i++) { l.xoffs[i] = (uint32_t)i; l.xoffs[8 + i] = 16 * 8 + (uint32_t)i; }
			for (int i = 0; i < 16; i++) l.yoffs[i] = (uint32_t)i * 8;
			l.charinc = 32 * 8;
		} else {                   // gfx_16x16x4_planar
			lay_frac4(&l, 16, s2 * 8, 1);
			for (int i = 0; i < 16; i++) { l.xoffs[i] = (uint32_t)i; l.yoffs[i] = (uint32_t)i * 16; }
			l.charinc = 16 * 16;
		}
		return vsg_decode(&s_sprg[1], &l, g2, s2, 0, (s2 * 8 / l.charinc) / 4, 1024, 64);
	case M_AERFBOOT: {
		if (!g1 || !g2 || !g3) return -1;
		static const uint32_t xo[8] = { 7, 6, 5, 4, 3, 2, 1, 0 };   // aerfboot_charlayout
		lay_frac4(&l, 8, s1 * 8, 1);
		memcpy(l.xoffs, xo, sizeof(xo));
		for (int i = 0; i < 8; i++) l.yoffs[i] = (uint32_t)i * 8;
		l.charinc = 8 * 8;
		const uint32_t total = (s1 * 8 / l.charinc) / 8;
		if (vsg_decode(&s_tiles[0], &l, g1, s1, 0, total, 0, 16) || vsg_decode(&s_tiles[1], &l, g1, s1, 0x20000, total, 256, 16))
			return -1;
		for (int k = 0; k < 2; k++) {   // aerfboot_spritelayout: "gfx2" (gfx 2), "spritegfx" (gfx 3)
			const uint32_t sz = k ? s3 : s2, f = sz * 8 / 2;
			const uint32_t xs[16] = { 2 * 4, 3 * 4, f + 2 * 4, f + 3 * 4, 0 * 4, 1 * 4, f + 0 * 4, f + 1 * 4,
			                          6 * 4, 7 * 4, f + 6 * 4, f + 7 * 4, 4 * 4, 5 * 4, f + 4 * 4, f + 5 * 4 };
			memset(&l, 0, sizeof(l));
			l.w = l.h = 16;
			l.planes = 4;
			for (int p = 0; p < 4; p++) l.planeoffs[p] = (uint32_t)p;
			memcpy(l.xoffs, xs, sizeof(xs));
			for (int i = 0; i < 16; i++) l.yoffs[i] = (uint32_t)i * 32;
			l.charinc = 64 * 8;
			if (vsg_decode(&s_sprg[k], &l, k ? g3 : g2, sz, 0, (sz * 8 / l.charinc) / 2, k ? 768 : 512, 16))
				return -1;
		}
		return 0;
	}
	case M_AERFBOO2: {
		if (!g1 || !g2 || !g3) return -1;
		for (int k = 0; k < 2; k++) {   // aerfboo2_charlayout
			const uint32_t sz = k ? s2 : s1, f = sz * 8 / 2;
			const uint32_t xs[8] = { 1 * 4, 0 * 4, f + 1 * 4, f + 0 * 4, 3 * 4, 2 * 4, f + 3 * 4, f + 2 * 4 };
			memset(&l, 0, sizeof(l));
			l.w = l.h = 8;
			l.planes = 4;
			for (int p = 0; p < 4; p++) l.planeoffs[p] = (uint32_t)p;
			memcpy(l.xoffs, xs, sizeof(xs));
			for (int i = 0; i < 8; i++) l.yoffs[i] = (uint32_t)i * 16;
			l.charinc = 32 * 4;
			if (vsg_decode(&s_tiles[k], &l, k ? g2 : g1, sz, 0, (sz * 8 / l.charinc) / 2, k ? 256 : 0, 16))
				return -1;
		}
		static const uint32_t xs[16] = { 28, 24, 20, 16, 12, 8, 4, 0, 60, 56, 52, 48, 44, 40, 36, 32 };   // aerfboo2_spritelayout
		memset(&l, 0, sizeof(l));
		l.w = l.h = 16;
		l.planes = 4;
		for (int p = 0; p < 4; p++) l.planeoffs[p] = (uint32_t)p;
		memcpy(l.xoffs, xs, sizeof(xs));
		for (int i = 0; i < 16; i++) l.yoffs[i] = (uint32_t)i * 64;
		l.charinc = 16 * 64;
		const uint32_t total = (s3 * 8 / l.charinc) / 2;
		return vsg_decode(&s_sprg[0], &l, g3, s3, 0, total, 512, 16) || vsg_decode(&s_sprg[1], &l, g3, s3, 0x100000, total, 768, 16);
	}
	default:   // gfx_turbofrc, gfx_turbofrc_spr1 / _spr2
		if (!g1 || !g2 || !g3 || !g4) return -1;
		return vsg_packed(&s_tiles[0], g1, s1, 8, 0, 0, 16) || vsg_packed(&s_tiles[1], g2, s2, 8, 0, 256, 16) ||
		       vsg_packed(&s_sprg[0], g3, s3, 16, 0, 512, 16) || vsg_packed(&s_sprg[1], g4, s4, 16, 0, 768, 16);
	}
}

static int ps_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = (struct pspikes_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_m = machine_of(g->machine);
	if (s_m < 0) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_mc = &s_machines[s_m];
	s_amask = (s_m == M_KARATBLZ || s_m == M_KARATBLZBL || s_m == M_TURBOFRC) ? 0xfffff : 0xffffff;
	s_vx0 = s_mc->vx0; s_vx1 = s_mc->vx1; s_vy0 = s_mc->vy0; s_vy1 = s_mc->vy1;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_zbank_rom = rbx_region("soundbank", &s_zbank_size);
	s_oki_rom = rbx_region("oki", &s_oki_size);
	s_upd_rom = rbx_region("upd", &s_upd_size);
	uint32_t lsz = 0;
	s_lookrom = rbx_region("sprlookuprom", &lsz);
	s_lookrom_n = s_lookrom ? lsz / 2 : 0;
	if (!s_rom || (s_mc->z80_clock && !s_zrom)) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	memset(s_tiles, 0, sizeof(s_tiles));
	memset(s_sprg, 0, sizeof(s_sprg));
	if (decode_gfx()) {
		rbx_fail("%s: graphics missing or out of memory", g->set);
		ps_exit();
		return -1;
	}
	switch (s_m) {   // the look-up RAMs' sizes
	case M_KICKBALL: case M_KARATBLZ: case M_KARATBLZBL: s_look_n[0] = s_look_n[1] = 0x8000; break;
	case M_WBBC97: s_look_n[0] = s_look_n[1] = 0x3000; break;
	default: s_look_n[0] = s_look_n[1] = 0x2000; break;
	}
	static const char *const in_tags[4] = { "IN0", "IN1", "IN2", "IN3" };
	for (int i = 0; i < 4; i++)
		s_port_in[i] = rbx_port_find(in_tags[i]);
	s_port_dsw[0] = rbx_port_find("DSW");
	if (s_port_dsw[0] < 0)
		s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	// sound
	s_has_oki = s_has_ym = s_has_upd = 0;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.irq = ym_irq;
	switch (s_mc->zmap) {
	case Z_BANKED_A: case Z_BANKED_B: {
		uint32_t asz = 0, bsz = 0;
		uint8_t *pa = rbx_region("ymsnd:adpcma", &asz), *pb = rbx_region("ymsnd:adpcmb", &bsz);
		yc.type = RBX_YM2610;
		yc.clock = 8000000;
		yc.adpcma = pa;
		yc.adpcma_size = asz;
		yc.adpcmb = pb ? pb : pa;   // (no delta-T region: the ADPCM-A one)
		yc.adpcmb_size = pb ? bsz : asz;
		yc.gain[0] = 192;
		yc.gain[1] = yc.gain[2] = s_m == M_AEROFGTB ? 224 : 256;   // (aerofgtb's routes: both sides from the FM's right)
		s_has_ym = 2;
		if (!s_zbank_rom) {
			rbx_fail("%s: ROM regions missing", g->set);
			ps_exit();
			return -1;
		}
		break;
	}
	case Z_SPIKES91:
		yc.type = RBX_YM2151;
		yc.clock = 3000000;
		yc.gain[0] = yc.gain[1] = 128;
		s_has_ym = 1;
		break;
	case Z_WBBC97: case Z_KARATBLZBL: case Z_KICKBALL:
		yc.type = RBX_YM3812;
		yc.clock = s_mc->zmap == Z_WBBC97 ? 3579545 : 4000000;
		yc.gain[0] = 256;
		s_has_ym = 1;
		break;
	}
	if (s_has_ym && (rbx_ym_init(0, &yc) || (s_has_ym == 1 && rbx_ym_set_async(0, 1)))) {
		rbx_fail("%s: out of memory", g->set);
		s_has_ym = 0;
		ps_exit();
		return -1;
	}
	if (s_m == M_PSPIKESB || s_m == M_PSPIKESC || s_m == M_AERFBOOT || s_m == M_AERFBOO2 || s_m == M_KICKBALL ||
	    s_m == M_WBBC97) {
		if (!s_oki_rom) {
			rbx_fail("%s: ROM regions missing", g->set);
			ps_exit();
			return -1;
		}
		const uint32_t clk = s_m == M_KICKBALL ? 1000000 : 1056000;
		oki_init(&s_oki, clk, s_m != M_PSPIKESB && s_m != M_KICKBALL, s_oki_rom, s_oki_size, s_m == M_WBBC97 ? 64 : 128);   // (route 1.0: 128, as the other boards' OKIs)
		if (s_m == M_PSPIKESB || s_m == M_PSPIKESC || s_m == M_AERFBOOT) {
			s_oki.read = oki_read_bank;
			s_oki.user = NULL;
		}
		s_has_oki = 1;
	}
	s_out_rate = s_has_ym ? rbx_ym_rate(0) : s_oki.rate;
	if (s_m == M_KARATBLZBL) {
		if (!s_upd_rom) {
			rbx_fail("%s: ROM regions missing", g->set);
			ps_exit();
			return -1;
		}
		upd7759_init(&s_upd, s_upd_rom, s_upd_size, 640000, s_out_rate);
		s_has_upd = 1;
	}
	s_mix_cap = (int)(s_out_rate / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_tmp = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_snap = calloc(1, sizeof(*s_snap));
	const int w = s_mc->vx1 - s_mc->vx0 + 1, h = s_mc->vy1 - s_mc->vy0 + 1;
	const int pal = s_m == M_WBBC97 ? 2048 + 0x8000 : s_mc->pal_n;
	if (!s_mix_l || !s_mix_r || !s_tmp || !s_snap || rbx_audio_setup_stereo(s_out_rate) || rbx_video_setup(w, h, pal)) {
		rbx_fail("%s: out of memory", g->set);
		ps_exit();
		return -1;
	}
	s_pri = (uint8_t *)calloc((size_t)rbx_video.pitch * (size_t)h, 1);
	if (s_m == M_WBBC97) {   // the bitmap's colours: GRB 555
		s_bmpens = (uint32_t *)malloc(sizeof(uint32_t) * 0x8000);
		if (s_bmpens)
			for (int c = 0; c < 0x8000; c++)
				s_bmpens[c] = (uint32_t)(pal5((c & 0x3e0) >> 5) << 16 | pal5((c & 0x7c00) >> 10) << 8 | pal5(c & 0x1f));
	}
	if (!s_pri || (s_m == M_WBBC97 && !s_bmpens)) {
		rbx_fail("%s: out of memory", g->set);
		ps_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times: the games' frame timing as in MAME)
	ps_reset();
	return 0;
}

static int ps_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (load)
		rbx_video_wait_idle();
	else
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "vsys", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	if (s_mc->z80_clock) {
		rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
		rc |= rbx_state_io(ctx, "zirq", &s_zirq, sizeof(s_zirq));
	}
	if (s_has_ym)
		rc |= rbx_ym_state(0, ctx, load);
	if (s_has_oki)
		rc |= rbx_state_io(ctx, "oki", &s_oki, (uint32_t)offsetof(okim6295_t, rom));
	if (s_has_upd)
		rc |= upd7759_state(&s_upd, ctx, load, "upd");
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		m68k_bus_update_irq();
		if (s_mc->z80_clock) {
			z.read_byte = z_read;
			z.write_byte = z_write;
			z.port_in = z_in;
			z.port_out = z_out;
			z.userdata = NULL;
			z.read_map = s_zmap;
			s_z80 = z;
			z_bank();
		}
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int ps_save(void *ctx) { return ps_state(ctx, 0); }
static int ps_load(void *ctx) { return ps_state(ctx, 1); }

const rbx_driver rbx_drv_pspikes = { "Video System Power Spikes", ps_init, ps_exit, ps_reset, ps_frame, ps_save, ps_load,
                                     ps_supported, ps_render, ps_help };

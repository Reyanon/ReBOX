/*  Taito B System, ported from MAME src/mame/taito/taito_b.cpp and taito_b_v.cpp
    (GPL-2.0+, copyright-holders: Jarek Burczynski; with help from Nicola Salmoria, Brian A. Troha,
    stephh, Gerardo Oporto Jorrin, David Graves). Memory maps, machine configurations, I/O chips
    (src/mame/taito/taitoio.cpp, BSD-3-Clause) and interrupt wiring follow MAME.
    68000 = Musashi; the sound board (Z80, YM, TC0140SYT, OKI) is machine/taito_sound.c.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../machine/taito_sound.h"
#include "../../machine/eeprom93c46.h"
#include "../video/tc0180vcu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- machine configurations (MAME taitob_state::rastsag2 ...) ----
enum { IO_TC0220IOC, IO_TC0040IOC, IO_TC0640FIO, IO_TC0510NIO };
enum { PAL_RGBX_444, PAL_RGB5X };
enum { SND_YM2610, SND_YM2610B, SND_YM2203 };
enum { MAP_RASTSAG2, MAP_CRIMEC, MAP_TETRIST, MAP_TETRISTA, MAP_HITICE, MAP_RAMBO3, MAP_PBOBBLE, MAP_SPACEDX,
       MAP_SPACEDXO, MAP_QZSHOWBY, MAP_VIOFIGHT, MAP_MASTERW, MAP_SILENTD, MAP_SELFEENA, MAP_SBM };

typedef struct {
	const char *name;
	int map;
	uint32_t cpu_clock, z80_clock;
	int io, snd, oki_clock, pal;
	int fb_base, bg_base, fg_base, tx_base;
	int inth, intl;
	int eeprom;
} machine_t;

static const machine_t s_machines[] = {
	{ "rastsag2", MAP_RASTSAG2, 12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGBX_444, 0x40, 0xc0, 0x80, 0x00, 4, 2, 0 },
	{ "masterw",  MAP_MASTERW,  12000000, 6000000, IO_TC0040IOC, SND_YM2203,  0,       PAL_RGBX_444, 0x10, 0x30, 0x20, 0x00, 5, 4, 0 },
	{ "tetrist",  MAP_TETRIST,  12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGB5X,    0x40, 0xc0, 0x80, 0x00, 4, 2, 0 },
	{ "tetrista", MAP_TETRISTA, 12000000, 6000000, IO_TC0040IOC, SND_YM2203,  0,       PAL_RGB5X,    0x10, 0x30, 0x20, 0x00, 5, 4, 0 },
	{ "ashura",   MAP_RASTSAG2, 12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGBX_444, 0x40, 0xc0, 0x80, 0x00, 4, 2, 0 },
	{ "crimec",   MAP_CRIMEC,   12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGBX_444, 0x80, 0x00, 0x40, 0xc0, 5, 3, 0 },
	{ "hitice",   MAP_HITICE,   12000000, 6000000, IO_TC0220IOC, SND_YM2203,  1056000, PAL_RGBX_444, 0x40, 0xc0, 0x80, 0x00, 4, 6, 0 },
	{ "rambo3p",  MAP_RAMBO3,   12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGBX_444, 0x40, 0xc0, 0x80, 0x00, 6, 1, 0 },
	{ "rambo3",   MAP_RAMBO3,   12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGBX_444, 0x10, 0x30, 0x20, 0x00, 6, 1, 0 },
	{ "pbobble",  MAP_PBOBBLE,  12000000, 8000000, IO_TC0640FIO, SND_YM2610B, 0,       PAL_RGB5X,    0x80, 0x00, 0x40, 0xc0, 3, 5, 1 },
	{ "spacedx",  MAP_SPACEDX,  12000000, 4000000, IO_TC0640FIO, SND_YM2610,  0,       PAL_RGB5X,    0x80, 0x00, 0x40, 0xc0, 3, 5, 1 },
	{ "spacedxo", MAP_SPACEDXO, 12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGB5X,    0x10, 0x30, 0x20, 0x00, 6, 4, 0 },
	{ "qzshowby", MAP_QZSHOWBY, 16000000, 4000000, IO_TC0640FIO, SND_YM2610B, 0,       PAL_RGB5X,    0x80, 0x00, 0x40, 0xc0, 3, 5, 1 },
	{ "viofight", MAP_VIOFIGHT, 12000000, 6000000, IO_TC0220IOC, SND_YM2203,  1056000, PAL_RGBX_444, 0x10, 0x30, 0x20, 0x00, 4, 1, 0 },
	{ "silentd",  MAP_SILENTD,  12000000, 4000000, IO_TC0220IOC, SND_YM2610B, 0,       PAL_RGB5X,    0x10, 0x30, 0x20, 0x00, 6, 4, 0 },
	{ "selfeena", MAP_SELFEENA, 12000000, 4000000, IO_TC0220IOC, SND_YM2610,  0,       PAL_RGB5X,    0x10, 0x30, 0x20, 0x00, 6, 4, 0 },
	{ "sbm",      MAP_SBM,      12000000, 4000000, IO_TC0510NIO, SND_YM2610B, 0,       PAL_RGBX_444, 0x40, 0xc0, 0x80, 0x00, 4, 5, 0 },
};

// ---- main CPU map: direct ranges and device ranges ----
enum { D_NONE, D_IO, D_IO040, D_SYT, D_EEP, D_COIN34, D_PORT, D_VCU_CTRL, D_VCU_FB, D_PIXRAM, D_PIXSCROLL, D_MB87078,
       D_TRACK, D_IO_SPACEDXO };
enum { LANE_HI = 1, LANE_LO = 2, LANE_WORD = 3 };
typedef struct { uint32_t start, end; int dev, lane; const char *tag; } range_t;
typedef struct { uint32_t rom_end, ram, ram_size, pal, vcu, io, syt; range_t r[12]; } map_t;

#define R(s, e, d, l, t) { s, e, d, l, t }
static const map_t s_maps[] = {
	[MAP_RASTSAG2] = { 0x7ffff, 0x600000, 0x8000, 0x200000, 0x400000, 0, 0, {
		R(0x800000, 0x800003, D_SYT, LANE_HI, 0), R(0xa00000, 0xa0000f, D_IO, LANE_HI, 0) } },
	[MAP_CRIMEC] = { 0x7ffff, 0xa00000, 0x10000, 0x800000, 0x400000, 0, 0, {
		R(0x200000, 0x20000f, D_IO, LANE_HI, 0), R(0x600000, 0x600003, D_SYT, LANE_HI, 0) } },
	[MAP_TETRIST] = { 0x7ffff, 0x800000, 0x8000, 0xa00000, 0x400000, 0, 0, {
		R(0x200000, 0x200003, D_SYT, LANE_HI, 0), R(0x600000, 0x60000f, D_IO, LANE_HI, 0) } },
	[MAP_TETRISTA] = { 0x7ffff, 0x800000, 0x4000, 0x200000, 0x400000, 0, 0, {
		R(0x600000, 0x600003, D_IO040, LANE_HI, 0), R(0xa00000, 0xa00003, D_SYT, LANE_HI, 0) } },
	[MAP_HITICE] = { 0x7ffff, 0x800000, 0x4000, 0xa00000, 0x400000, 0, 0, {
		R(0x600000, 0x60000f, D_IO, LANE_HI, 0), R(0x610000, 0x610001, D_PORT, LANE_WORD, "P3_P4"),
		R(0x700000, 0x700003, D_SYT, LANE_HI, 0), R(0xb00000, 0xb7ffff, D_PIXRAM, LANE_WORD, 0),
		R(0xbffff0, 0xbffff5, D_PIXSCROLL, LANE_WORD, 0) } },
	[MAP_RAMBO3] = { 0x7ffff, 0x800000, 0x4000, 0xa00000, 0x400000, 0, 0, {
		R(0x200000, 0x200003, D_SYT, LANE_HI, 0), R(0x600000, 0x60000f, D_IO, LANE_HI, 0),
		R(0x600010, 0x60001f, D_TRACK, LANE_WORD, 0) } },
	[MAP_PBOBBLE] = { 0x7ffff, 0x900000, 0x10000, 0x800000, 0x400000, 0, 0, {
		R(0x500000, 0x50000f, D_IO, LANE_HI, 0), R(0x500024, 0x500025, D_PORT, LANE_WORD, "P3_P4_A"),
		R(0x500026, 0x500027, D_EEP, LANE_WORD, 0), R(0x500028, 0x500029, D_COIN34, LANE_WORD, 0),
		R(0x50002e, 0x50002f, D_PORT, LANE_WORD, "P3_P4_B"), R(0x600000, 0x600003, D_MB87078, LANE_HI, 0),
		R(0x700000, 0x700003, D_SYT, LANE_HI, 0) } },
	[MAP_SPACEDX] = { 0x7ffff, 0x900000, 0x10000, 0x800000, 0x400000, 0, 0, {
		R(0x500000, 0x50000f, D_IO, LANE_HI, 0), R(0x500024, 0x500025, D_PORT, LANE_WORD, "P3_P4_A"),
		R(0x500026, 0x500027, D_EEP, LANE_WORD, 0), R(0x500028, 0x500029, D_COIN34, LANE_WORD, 0),
		R(0x50002e, 0x50002f, D_PORT, LANE_WORD, "P3_P4_B"), R(0x600000, 0x600003, D_MB87078, LANE_HI, 0),
		R(0x700000, 0x700003, D_SYT, LANE_HI, 0) } },
	[MAP_SPACEDXO] = { 0x7ffff, 0x400000, 0x10000, 0x300000, 0x500000, 0, 0, {
		R(0x100000, 0x100003, D_SYT, LANE_HI, 0), R(0x200000, 0x20000f, D_IO_SPACEDXO, LANE_LO, 0),
		R(0x210000, 0x210001, D_PORT, LANE_WORD, "IN3"), R(0x220000, 0x220001, D_PORT, LANE_WORD, "IN4"),
		R(0x230000, 0x230001, D_PORT, LANE_WORD, "IN5") } },
	[MAP_QZSHOWBY] = { 0xfffff, 0x900000, 0x10000, 0x800000, 0x400000, 0, 0, {
		R(0x200000, 0x20000f, D_IO, LANE_HI, 0), R(0x200024, 0x200025, D_PORT, LANE_WORD, "P3_P4_A"),
		R(0x200026, 0x200027, D_EEP, LANE_WORD, 0), R(0x200028, 0x200029, D_COIN34, LANE_WORD, 0),
		R(0x20002e, 0x20002f, D_PORT, LANE_WORD, "P3_P4_B"), R(0x600000, 0x600003, D_SYT, LANE_HI, 0),
		R(0x700000, 0x700003, D_MB87078, LANE_HI, 0) } },
	[MAP_VIOFIGHT] = { 0x7ffff, 0xa00000, 0x4000, 0x600000, 0x400000, 0, 0, {
		R(0x200000, 0x200003, D_SYT, LANE_HI, 0), R(0x800000, 0x80000f, D_IO, LANE_HI, 0) } },
	[MAP_MASTERW] = { 0x7ffff, 0x200000, 0x4000, 0x600000, 0x400000, 0, 0, {
		R(0x800000, 0x800003, D_IO040, LANE_HI, 0), R(0xa00000, 0xa00003, D_SYT, LANE_HI, 0) } },
	[MAP_SILENTD] = { 0x7ffff, 0x400000, 0x4000, 0x300000, 0x500000, 0, 0, {
		R(0x100000, 0x100003, D_SYT, LANE_HI, 0), R(0x200000, 0x20000f, D_IO, LANE_LO, 0),
		R(0x210000, 0x210001, D_PORT, LANE_WORD, "IN3"), R(0x220000, 0x220001, D_PORT, LANE_WORD, "IN4"),
		R(0x230000, 0x230001, D_PORT, LANE_WORD, "IN5") } },
	[MAP_SELFEENA] = { 0x7ffff, 0x100000, 0x4000, 0x300000, 0x200000, 0, 0, {
		R(0x400000, 0x40000f, D_IO, LANE_HI, 0), R(0x410000, 0x41000f, D_IO, LANE_HI, 0),
		R(0x500000, 0x500003, D_SYT, LANE_HI, 0) } },
	[MAP_SBM] = { 0x7ffff, 0x100000, 0x10000, 0x200000, 0x900000, 0, 0, {
		R(0x300000, 0x30000f, D_IO, LANE_LO, 0), R(0x320000, 0x320003, D_SYT, LANE_HI, 0) } },
};

// ---- board state ----
static const machine_t *s_m;
static const map_t *s_map;
static uint8_t *s_rom;              // 68000 program, host-order words
static uint32_t s_rom_size;
static uint8_t s_ram[0x10000];
static uint8_t s_pal[0x2000];
static tc0180vcu_t *s_vcu;
// drawing snapshot taken at vblank (render thread or inline, rbx_video_frame_done)
static tc0180vcu_t *s_snap;
static uint8_t s_snap_pal[0x2000];
static int s_snap_draw_page, s_snap_clear;
static uint8_t s_io_regs[8], s_io_port;
static uint16_t s_eep_latch, s_coin_word;
static eeprom93c46_t s_eeprom;
static char s_nv_path[256];
static int s_port_ids[8];          // I/O chip inputs 0, 1, 2, 3, 7
static int s_port_coin;            // EEPROM data bit lives in this port (FIO "COIN")
static int s_range_port[12];
// hitice pixel layer
static uint16_t *s_pixram;          // 0x40000 words
static uint16_t s_pixel_scroll[3];

// ---- frame timing ----
#define LINES 256
#define VBLANK_LINE 240
static uint64_t s_cpu_frac;   // main CPU cycle fraction (tsnd_line_cycles)

// ---- I/O chips ----
static uint8_t io_read(int offset)
{
	switch (offset) {
	case 0: return (uint8_t)rbx_port_read(s_port_ids[0]);
	case 1: return (uint8_t)rbx_port_read(s_port_ids[1]);
	case 2: return (uint8_t)rbx_port_read(s_port_ids[2]);
	case 3: return (uint8_t)rbx_port_read(s_port_ids[3]);
	case 4: return s_io_regs[4];
	case 7: return (uint8_t)rbx_port_read(s_port_ids[7]);
	}
	return 0xff;
}

static void io_write(int offset, uint8_t data)
{
	s_io_regs[offset & 7] = data;   // 0 = watchdog, 4 = coin lockout / counters
}

// ---- main CPU devices ----
static const range_t *find_range(uint32_t a, int *idx)
{
	for (int i = 0; i < 12 && s_map->r[i].dev; i++)
		if (a >= s_map->r[i].start && a <= s_map->r[i].end) {
			*idx = i;
			return &s_map->r[i];
		}
	return NULL;
}

static void eeprom_w(uint16_t data, uint16_t mask)
{
	s_eep_latch = (uint16_t)((s_eep_latch & ~mask) | (data & mask));
	if (mask & 0xff00) {
		uint8_t d = (uint8_t)(data >> 8);
		eeprom_write_lines(&s_eeprom, d & 0x10, d & 0x08, d & 0x04);
		rbx_port_custom(s_port_coin, 0x01, eeprom_do(&s_eeprom) ? 0x01 : 0);
	}
}

static uint8_t dev8_read(const range_t *r, int idx, uint32_t off)
{
	switch (r->dev) {
	case D_IO: case D_IO_SPACEDXO: return io_read((int)off);
	case D_IO040: return (off & 1) ? 0 : io_read(s_io_port);
	case D_SYT: return (off & 1) ? tc0140syt_master_comm_r(tsnd_syt()) : 0;
	}
	(void)idx;
	return 0xff;
}

static void dev8_write(const range_t *r, uint32_t off, uint8_t v)
{
	switch (r->dev) {
	case D_IO: case D_IO_SPACEDXO: io_write((int)off, v); break;
	case D_IO040:
		if (off & 1) s_io_port = v;
		else if (s_io_port < 8) s_io_regs[s_io_port] = v;
		break;
	case D_SYT:
		if (off & 1) tc0140syt_master_comm_w(tsnd_syt(), v);
		else tc0140syt_master_port_w(tsnd_syt(), v);
		break;
	case D_MB87078: break;   // electronic volume (TODO: MB87078 gain on the YM outputs)
	}
}

static uint16_t dev16_read(const range_t *r, int idx, uint32_t a)
{
	switch (r->dev) {
	case D_PORT: return (uint16_t)rbx_port_read(s_range_port[idx]);
	case D_EEP: return s_eep_latch;
	case D_COIN34: return s_coin_word;
	case D_PIXRAM: return s_pixram[(a - r->start) >> 1];
	case D_TRACK: {
		// rambo3: 0x600010.. P1 tracky lo/hi, trackx lo/hi, then P2 (MAME rambo3_state)
		int w = (int)((a - r->start) >> 1);
		int pl = w >> 2;
		int axis = (w & 2) ? 0 : 1;   // tracky first (reads the X port in MAME: names are swapped there)
		int v = rbx_analog(pl + 1, axis ? RBX_IPT_TRACKBALL_Y : RBX_IPT_TRACKBALL_X);
		return (w & 1) ? (uint16_t)v : (uint16_t)((v & 0xff) << 8);
	}
	}
	return 0;
}

static void dev16_write(const range_t *r, uint32_t a, uint16_t v, uint16_t mask)
{
	switch (r->dev) {
	case D_EEP: eeprom_w(v, mask); break;
	case D_COIN34: s_coin_word = (uint16_t)((s_coin_word & ~mask) | (v & mask)); break;
	case D_PIXRAM: {
		uint32_t o = (a - r->start) >> 1;
		s_pixram[o] = (uint16_t)((s_pixram[o] & ~mask) | (v & mask));
		break;
	}
	case D_PIXSCROLL: {
		int o = (int)((a - r->start) >> 1);
		s_pixel_scroll[o] = (uint16_t)((s_pixel_scroll[o] & ~mask) | (v & mask));
		break;
	}
	}
}

static int vcu_dev(uint32_t a, uint32_t *off)
{
	uint32_t base = s_map->vcu;
	if (a >= base + 0x18000 && a < base + 0x18020) { *off = (a - base - 0x18000) >> 1; return 1; }
	if (a >= base + 0x40000 && a < base + 0x80000) { *off = (a - base - 0x40000) >> 1; return 2; }
	return 0;
}

static uint16_t main_read16(uint32_t a)
{
	uint32_t off;
	switch (vcu_dev(a, &off)) {
	case 1: return vcu_ctrl_r(s_vcu, (int)off);
	case 2: rbx_video_wait_idle(); return vcu_fb_r(s_vcu, off);
	}
	int idx;
	const range_t *r = find_range(a, &idx);
	if (!r)
		return 0;
	if (r->lane == LANE_WORD)
		return dev16_read(r, idx, a);
	uint8_t v = dev8_read(r, idx, (a - r->start) >> 1);
	return r->lane == LANE_HI ? (uint16_t)(v << 8) : v;
}

static uint8_t main_read8(uint32_t a)
{
	uint32_t off;
	if (vcu_dev(a, &off))
		return (uint8_t)(main_read16(a & ~1u) >> ((a & 1) ? 0 : 8));
	int idx;
	const range_t *r = find_range(a, &idx);
	if (!r)
		return 0;
	if (r->lane == LANE_WORD)
		return (uint8_t)(dev16_read(r, idx, a & ~1u) >> ((a & 1) ? 0 : 8));
	if ((r->lane == LANE_HI) != !(a & 1))
		return 0;
	return dev8_read(r, idx, (a - r->start) >> 1);
}

static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	uint32_t off;
	switch (vcu_dev(a, &off)) {
	case 1: vcu_ctrl_w(s_vcu, (int)off, v, mask); return;
	case 2: rbx_video_wait_idle(); vcu_fb_w(s_vcu, off, v, mask); return;
	}
	int idx;
	const range_t *r = find_range(a, &idx);
	if (!r)
		return;
	if (r->lane == LANE_WORD) {
		dev16_write(r, a, v, mask);
		return;
	}
	uint32_t o = (a - r->start) >> 1;
	if (r->dev == D_IO_SPACEDXO) {          // spacedxo also writes the high byte here
		io_write((int)o, (mask & 0x00ff) ? (uint8_t)v : (uint8_t)(v >> 8));
		return;
	}
	if (r->lane == LANE_HI && (mask & 0xff00))
		dev8_write(r, o, (uint8_t)(v >> 8));
	else if (r->lane == LANE_LO && (mask & 0x00ff))
		dev8_write(r, o, (uint8_t)v);
}

static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_write16m(a & ~1u, v, 0x00ff);
	else
		main_write16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- video ----
static void palette_update(void)
{
	const uint16_t *p = (const uint16_t *)s_snap_pal;
	for (int i = 0; i < 4096; i++) {
		uint16_t d = p[i];
		int r, g, b;
		if (s_m->pal == PAL_RGBX_444) {
			r = (d >> 12) & 15; g = (d >> 8) & 15; b = (d >> 4) & 15;
			r |= r << 4; g |= g << 4; b |= b << 4;
		} else {
			r = ((d >> 11) & 0x1e) | ((d >> 3) & 1);
			g = ((d >> 7) & 0x1e) | ((d >> 2) & 1);
			b = ((d >> 3) & 0x1e) | ((d >> 1) & 1);
			r = (r << 3) | (r >> 2); g = (g << 3) | (g >> 2); b = (b << 3) | (b >> 2);
		}
		rbx_video.palette[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

static void draw_pixel_layer(uint16_t *bitmap, int pitch)
{
	if ((s_pixel_scroll[0] & 0x5800) != 0x5000)
		return;
	int scrollx = -2 * (int16_t)s_pixel_scroll[1];
	int scrolly = 16 - (int16_t)s_pixel_scroll[2];
	for (int r = 0; r < 224; r++) {
		int sy = ((r + 16 - scrolly) % 512 + 512) % 512;
		const uint16_t *row = s_pixram + sy * 512;
		uint16_t *d = bitmap + r * pitch;
		for (int x = 0; x < 320; x++) {
			int sx = ((x - scrollx) % 1024 + 1024) % 1024;
			uint8_t c = (uint8_t)row[sx >> 1];
			if (c)
				d[x] = (uint16_t)(0x800 + c);
		}
	}
}

static void screen_update(void)
{
	tc0180vcu_t *v = s_snap;
	if (!(v->video_control & 0x20)) {
		memset(rbx_video.bitmap, 0, (size_t)rbx_video.pitch * rbx_video.height * 2);
		return;
	}
	palette_update();
	uint16_t *bm = rbx_video.bitmap;
	int pitch = rbx_video.pitch;
	if (s_m->map == MAP_HITICE) {
		vcu_draw_bg(v, bm, pitch);
		vcu_draw_fb(v, bm, pitch, 1);
		vcu_draw_fg(v, bm, pitch);
		draw_pixel_layer(bm, pitch);
		vcu_draw_fb(v, bm, pitch, 0);
		vcu_draw_tx(v, bm, pitch);
	} else {
		vcu_render(v, bm, pitch);
	}
}

// drawing side: the screen of the snapshot, then MAME's vblank sprite pass into the frame buffer
static void tb_render(void)
{
	screen_update();
	vcu_draw_sprites(s_snap, s_snap_draw_page, s_snap_clear);
}

// main side at vblank: wait for the previous drawing, snapshot, flip the frame buffer page
static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap, s_vcu, sizeof(*s_snap));
	memcpy(s_snap_pal, s_pal, sizeof(s_pal));
	vcu_vblank_flip(s_vcu, &s_snap_draw_page, &s_snap_clear);
	rbx_video_frame_done();
}

// ---- init / reset ----
static const machine_t *find_machine(const char *name)
{
	for (size_t i = 0; i < sizeof(s_machines) / sizeof(s_machines[0]); i++)
		if (!strcmp(s_machines[i].name, name))
			return &s_machines[i];
	return NULL;
}

static void map_main(void)
{
	m68k_bus_reset_map();
	m68k_bus_map(0, s_map->rom_end, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(s_map->ram, s_map->ram + s_map->ram_size - 1, s_ram, s_map->ram_size, 1, 1);
	m68k_bus_map(s_map->pal, s_map->pal + 0x1fff, s_pal, 0x2000, 1, 1);
	m68k_bus_map(s_map->vcu, s_map->vcu + VCU_RAM_BYTES - 1, s_vcu->ram, VCU_RAM_BYTES, 1, 1);
	if (s_m->map == MAP_SPACEDXO)        // 0x302000-0x303fff readonly
		m68k_bus_map(0x302000, 0x303fff, s_pal, 0x2000, 1, 0);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
}

static void tb_reset(void)
{
	// (the palette / video RAM too: a game that has not written it yet showed the last game's, as at start it is 0)
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_io_regs, 0, sizeof(s_io_regs));
	s_io_port = 0;
	s_eep_latch = s_coin_word = 0;
	vcu_reset(s_vcu);
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	tsnd_reset();
	if (s_pixram)
		memset(s_pixram, 0, 0x40000 * 2);
	memset(s_pixel_scroll, 0, sizeof(s_pixel_scroll));
	s_cpu_frac = 0;
}

static void tb_exit(void);

static int tb_init(const rbx_game *g)
{
	s_m = find_machine(g->machine);
	if (!s_m) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_map = &s_maps[s_m->map];
	s_rom = rbx_region("maincpu", &s_rom_size);
	uint32_t gsize;
	uint8_t *gfx = rbx_region("tc0180vcu", &gsize);
	if (!s_rom || !gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	s_vcu = (tc0180vcu_t *)calloc(1, sizeof(tc0180vcu_t));
	s_snap = (tc0180vcu_t *)calloc(1, sizeof(tc0180vcu_t));
	if (!s_vcu || !s_snap || vcu_init(s_vcu, gfx, gsize, s_m->fb_base, s_m->bg_base, s_m->fg_base, s_m->tx_base)) {
		rbx_fail("%s: out of memory (graphics)", g->set);
		tb_exit();
		return -1;
	}
	if (s_m->map == MAP_HITICE) {
		s_pixram = (uint16_t *)calloc(0x40000, 2);
		if (!s_pixram) { tb_exit(); return -1; }
	}
	// I/O chip inputs
	static const char *const ioc_tags[8] = { "DSWA", "DSWB", "IN0", "IN1", 0, 0, 0, "IN2" };
	static const char *const fio_tags[8] = { "SERVICE", "COIN", "START", "P1_P2_A", 0, 0, 0, "P1_P2_B" };
	static const char *const nio_tags[8] = { "DSWA", "DSWB", "JOY", "START", 0, 0, 0, "PHOTOSENSOR" };
	const char *const *tags = s_m->io == IO_TC0640FIO ? fio_tags : s_m->io == IO_TC0510NIO ? nio_tags : ioc_tags;
	for (int i = 0; i < 8; i++)
		s_port_ids[i] = tags[i] ? rbx_port_find(tags[i]) : -1;
	s_port_coin = rbx_port_find("COIN");
	for (int i = 0; i < 12; i++)
		s_range_port[i] = s_map->r[i].tag ? rbx_port_find(s_map->r[i].tag) : -1;
	// sound (MAME routes: YM2610 SSG 0.75, FM 1.0; YM2203 SSG 0.25 each, FM 0.80; OKI 0.50)
	tsnd_config sc;
	memset(&sc, 0, sizeof(sc));
	sc.z80_clock = s_m->z80_clock;
	sc.frame_rate_mhz = 60000;
	sc.lines = LINES;
	if (s_m->snd == SND_YM2203) {
		sc.map = TSND_MAP_9000;
		sc.ym.type = RBX_YM2203;
		sc.ym.clock = 3000000;
		sc.ym.gain[0] = sc.ym.gain[1] = sc.ym.gain[2] = 64;
		sc.ym.gain[3] = 205;
		sc.oki_clock = (uint32_t)s_m->oki_clock;
		sc.oki_pin7_high = 1;
		sc.oki_gain = 128;
	} else {
		sc.map = TSND_MAP_E000;
		sc.ym.type = s_m->snd == SND_YM2610B ? RBX_YM2610B : RBX_YM2610;
		sc.ym.clock = 8000000;
		sc.ym.gain[0] = 192;
		sc.ym.gain[1] = sc.ym.gain[2] = 256;
	}
	if (tsnd_init(&sc)) {
		rbx_fail("%s: sound init failed", g->set);
		tb_exit();
		return -1;
	}
	// EEPROM
	s_nv_path[0] = 0;
	eeprom_init(&s_eeprom);
	if (s_m->eeprom) {
		const char *dir = NULL;
		extern const char *rbx_nvram_dir(void);
		dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			eeprom_load(&s_eeprom, s_nv_path);
		}
		rbx_port_custom(s_port_coin, 0x01, 0x01);
	}
	if (rbx_video_setup(320, 224, 4096)) {
		tb_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	map_main();
	tb_reset();
	return 0;
}

static void tb_exit(void)
{
	if (s_m && s_m->eeprom && s_nv_path[0] && s_eeprom.dirty)
		eeprom_save(&s_eeprom, s_nv_path);
	if (s_vcu)
		vcu_exit(s_vcu);
	free(s_vcu);
	s_vcu = NULL;
	free(s_snap);
	s_snap = NULL;
	free(s_pixram);
	s_pixram = NULL;
	tsnd_exit();
}

// ---- frame ----
static void tb_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == VBLANK_LINE) {
			vblank_capture();
			m68k_bus_hold_irq(s_m->inth);
		} else if (line == VBLANK_LINE + 8) {
			m68k_bus_clear_irq(s_m->inth);
			m68k_bus_hold_irq(s_m->intl);
		}
		m68k_execute(tsnd_line_cycles(&s_cpu_frac, s_m->cpu_clock));
		tsnd_line();
	}
	m68k_bus_clear_irq(s_m->intl);
	tsnd_frame_end();
}

// ---- save states ----
static int tb_state(void *ctx, int load)
{
	uint8_t cpu[1024];
	uint32_t csize = m68k_context_size();
	if (csize > sizeof(cpu))
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq = m68k_bus_irq_pending();
	int misc[4] = { (int)s_io_port, s_eep_latch, s_coin_word, 0 };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, 4);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "vcu", s_vcu->ram, VCU_RAM_BYTES);
	rc |= rbx_state_io(ctx, "vcuctrl", s_vcu->ctrl, sizeof(s_vcu->ctrl));
	rc |= rbx_state_io(ctx, "vcufb", s_vcu->fb[0], 2 * VCU_FB_W * VCU_FB_H * 2);
	int vs[8] = { s_vcu->fb_page, s_vcu->video_control, s_vcu->bg_bank[0], s_vcu->bg_bank[1], s_vcu->fg_bank[0],
	              s_vcu->fg_bank[1], s_vcu->tx_bank, 0 };
	rc |= rbx_state_io(ctx, "vcuregs", vs, sizeof(vs));
	rc |= rbx_state_io(ctx, "io", s_io_regs, sizeof(s_io_regs));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	rc |= rbx_state_io(ctx, "eeprom", &s_eeprom, sizeof(s_eeprom));
	if (s_pixram) {
		rc |= rbx_state_io(ctx, "pixram", s_pixram, 0x40000 * 2);
		rc |= rbx_state_io(ctx, "pixscr", s_pixel_scroll, sizeof(s_pixel_scroll));
	}
	uint64_t fr = s_cpu_frac;
	rc |= rbx_state_io(ctx, "frac", &fr, sizeof(fr));
	rc |= tsnd_state(ctx, load);
	if (rc)
		return -1;
	if (load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		s_io_port = (uint8_t)misc[0]; s_eep_latch = (uint16_t)misc[1]; s_coin_word = (uint16_t)misc[2];
		s_vcu->fb_page = vs[0]; s_vcu->video_control = (uint8_t)vs[1];
		s_vcu->bg_bank[0] = vs[2]; s_vcu->bg_bank[1] = vs[3]; s_vcu->fg_bank[0] = vs[4]; s_vcu->fg_bank[1] = vs[5];
		s_vcu->tx_bank = vs[6];
		s_cpu_frac = fr;
		if (s_m->eeprom)
			rbx_port_custom(s_port_coin, 0x01, eeprom_do(&s_eeprom) ? 0x01 : 0);
	}
	return 0;
}

static int tb_save(void *ctx) { return tb_state(ctx, 0); }
static int tb_load(void *ctx) { return tb_state(ctx, 1); }

static int tb_supported(const char *machine) { return find_machine(machine) != NULL; }

const rbx_driver rbx_drv_taito_b = { "Taito B System", tb_init, tb_exit, tb_reset, tb_frame, tb_save, tb_load, tb_supported, tb_render };

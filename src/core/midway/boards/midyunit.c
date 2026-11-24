/*  Williams / Midway Y-unit (Smash T.V., Trog, High Impact, Super High Impact, Strike Force, Mortal Kombat, Total
    Carnage), ported to C from MAME src/mame/williams/midyunit.cpp, midyunit_m.cpp, midyunit_v.cpp, midyunit.h
    (BSD-3-Clause, copyright-holders: Alex Pasadyn, Zsolt Vasvari, Ernesto Corvi, Aaron Giles; thanks-to: Kurt Mahan).
    TMS34010 (src/core/cpu/tms34010), the DMA blitter, CMOS, the protection sequences and the CVSD / ADPCM sound boards
    (machine/williams_snd.c).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/tms34010/tms34010.h"
#include "../machine/williams_snd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { DMA_COMMAND = 0, DMA_ROWBYTES, DMA_OFFSETLO, DMA_OFFSETHI, DMA_XSTART, DMA_YSTART, DMA_WIDTH, DMA_HEIGHT,
       DMA_PALETTE, DMA_COLOR };

typedef struct { uint16_t reset_sequence[3]; uint16_t data_sequence[100]; } protection_data;

static const protection_data trog_prot = { { 0x0f00, 0x0f00, 0x0f00 },
	{ 0x3000, 0x1000, 0x2000, 0x0000, 0x2000, 0x3000, 0x3000, 0x1000, 0x0000, 0x0000, 0x2000, 0x3000, 0x1000, 0x1000, 0x2000 } };
static const protection_data hiimpact_prot = { { 0x0b00, 0x0b00, 0x0b00 },
	{ 0x2000, 0x4000, 0x4000, 0x0000, 0x6000, 0x6000, 0x2000, 0x4000, 0x2000, 0x4000, 0x2000, 0x0000, 0x4000, 0x6000, 0x2000 } };
static const protection_data shimpact_prot = { { 0x0f00, 0x0e00, 0x0d00 },
	{ 0x0000, 0x4000, 0x2000, 0x5000, 0x2000, 0x1000, 0x4000, 0x6000, 0x3000, 0x0000, 0x2000, 0x5000, 0x5000, 0x5000, 0x2000 } };
static const protection_data strkforc_prot = { { 0x1234 }, { 0 } };
static const protection_data mk_prot = { { 0x0d00, 0x0c00, 0x0900 },
	{ 0x4600, 0xf600, 0xa600, 0x0600, 0x2600, 0x9600, 0xc600, 0xe600, 0x8600, 0x7600, 0x8600, 0x8600, 0x9600, 0xd600,
	  0x6600, 0xb600, 0xd600, 0xe600, 0xf600, 0x7600, 0xb600, 0xa600, 0x3600 } };
static const protection_data totcarn_prot = { { 0x0f00, 0x0f00, 0x0f00 },
	{ 0x4a00, 0x6a00, 0xda00, 0x6a00, 0x9a00, 0x4a00, 0x2a00, 0x9a00, 0x1a00, 0x8a00, 0xaa00 } };

#define GFX_BYTES 0x800000

// the machine configuration
static uint32_t s_clock;
static int s_bpp, s_sound;
static uint32_t s_palette_mask;
static int s_npens;
static const protection_data *s_prot_data;
static int s_mkturbo;
static const rbx_game *s_game;

// memory (heap, ADR-040)
static uint16_t *s_mainram;    // 0x10000 words
static uint16_t *s_cmos;       // 0x4000 words
static uint16_t *s_palram;     // 0x2000 words
static uint16_t *s_vram;       // 0x40000 words (m_local_videoram)
static uint8_t *s_gfx;         // GFX_BYTES (m_gfx_rom)
static uint16_t *s_pen_map;    // 65536
static uint16_t *s_maindata;
static uint32_t *s_pal;        // the palette in RGB
static uint16_t *s_fb;         // the screen's visible lines (512 x 512)
static char s_nv_path[512];
// (the CMOS written to its file a moment after the game changes it, not only at the game's end: the app closed from
// the system menu skips the end, and the game said "CMOS invalid -- errors detected" at every start)
static uint32_t s_nv_hash, s_nv_tick;
static uint32_t nv_hash(const uint16_t *p, uint32_t n)
{
	uint32_t h = 2166136261u;
	for (uint32_t i = 0; i < n; i++)
		h = (h ^ p[i]) * 16777619u;
	return h;
}

// state (saved)
static struct {
	uint32_t cmos_page;
	uint16_t prot_result, prot_sequence[3];
	uint8_t prot_index, cmos_w_enable, autoerase_enable, videobank_select;
	uint16_t dma_register[16];
	struct { uint32_t offset; int32_t rowbytes, xpos, ypos, width, height; uint16_t palette, color; } dma;
	uint8_t dma_pending, ae_pending, pad_[2];
	int32_t ae_param;
	uint64_t dma_at;        // TMS cycle of the DMA's end
	uint64_t ticks;         // 8 MHz ticks since the start
	uint64_t cyc_frac;      // (the TMS cycles of the ticks: remainder)
	uint64_t cyc_target;
	uint64_t line_tick0, line_cyc0;   // the current line's start: ticks, TMS cycles
} S;

static int s_port[6];

// the sound board brought up to the TMS34010's time (MAME: its timeslices; perfect_quantum after a command)
static void sound_sync(void)
{
	uint64_t c = tms34010_total_cycles();
	uint64_t t = S.line_tick0 + (c > S.line_cyc0 ? (c - S.line_cyc0) * 64000000u / s_clock : 0);
	if (t > S.ticks)
		t = S.ticks;
	wms_snd_run(t);
}

// ---- the DMA blitter (midyunit_v.cpp) ----
static void dma_draw(uint16_t command)
{
	int const dx = (command & 0x10) ? -1 : 1;
	int const height = S.dma.height;
	int const width = S.dma.width;
	uint8_t const *const base = s_gfx;
	uint32_t offset = S.dma.offset >> 3;
	uint16_t const pal = S.dma.palette;
	uint16_t const color = pal | S.dma.color;
	command &= 0x0f;
	for (int y = 0; y < height; y++) {
		int tx = S.dma.xpos;
		int ty = S.dma.ypos;
		uint32_t o = offset;
		ty = (ty + y) & 0x1ff;
		offset += (uint32_t)S.dma.rowbytes;
		uint16_t *dest = &s_vram[ty * 512];
		if (o >= 0x06000000 && command < 0x0c)
			continue;
		// the fast path (the same pixels): the row's source inside the ROM and its destination without the wrap
		if (width > 0 && o < GFX_BYTES && (uint32_t)width <= GFX_BYTES - o &&
			(dx > 0 ? tx >= 0 && tx + width <= 512 : tx <= 511 && tx - width + 1 >= 0)) {
			uint8_t const *s = base + o;
			uint16_t *d = dest + tx;
#define ROW(BODY) do { for (int x = 0; x < width; x++, d += dx) { uint8_t const p = s[x]; BODY; } } while (0)
			switch (command) {
			case 0x00: break;
			case 0x01: ROW(if (p == 0) *d = pal); break;
			case 0x02: ROW(if (p != 0) *d = pal | p); break;
			case 0x03: ROW(*d = pal | p); break;
			case 0x04: case 0x05: ROW(if (p == 0) *d = color); break;
			case 0x06: case 0x07: ROW(*d = (p == 0) ? color : (pal | p)); break;
			case 0x08: case 0x0a: ROW(if (p != 0) *d = color); break;
			case 0x09: case 0x0b: ROW(*d = (p != 0) ? color : (pal | p)); break;
			default: ROW((void)p; *d = color); break;
			}
#undef ROW
			continue;
		}
		// (the source past the 8 MB: MAME reads beyond its buffer; here the pixels read 0)
#define PIX() (o < GFX_BYTES ? base[o] : 0)
		switch (command) {
		case 0x00:
			break;
		case 0x01:
			for (int x = 0; x < width; x++, tx += dx) { if (PIX() == 0) dest[tx & 0x1ff] = pal; o++; }
			break;
		case 0x02:
			for (int x = 0; x < width; x++, tx += dx) { uint8_t p = PIX(); o++; if (p != 0) dest[tx & 0x1ff] = pal | p; }
			break;
		case 0x03:
			for (int x = 0; x < width; x++, tx += dx) { dest[tx & 0x1ff] = pal | PIX(); o++; }
			break;
		case 0x04:
		case 0x05:
			for (int x = 0; x < width; x++, tx += dx) { if (PIX() == 0) dest[tx & 0x1ff] = color; o++; }
			break;
		case 0x06:
		case 0x07:
			for (int x = 0; x < width; x++, tx += dx) { uint8_t p = PIX(); o++; dest[tx & 0x1ff] = (p == 0) ? color : (pal | p); }
			break;
		case 0x08:
		case 0x0a:
			for (int x = 0; x < width; x++, tx += dx) { if (PIX() != 0) dest[tx & 0x1ff] = color; o++; }
			break;
		case 0x09:
		case 0x0b:
			for (int x = 0; x < width; x++, tx += dx) { uint8_t p = PIX(); o++; dest[tx & 0x1ff] = (p != 0) ? color : (pal | p); }
			break;
		default:
			for (int x = 0; x < width; x++, tx += dx) dest[tx & 0x1ff] = color;
			break;
		}
#undef PIX
	}
}

static void dma_w(uint32_t offset, uint16_t data)
{
	S.dma_register[offset] = data;
	if (offset != DMA_COMMAND)
		return;
	int const command = S.dma_register[DMA_COMMAND];
	tms34010_set_irq(0, 0);
	if (!(command & 0x8000))
		return;
	S.dma.rowbytes = (int16_t)S.dma_register[DMA_ROWBYTES];
	S.dma.xpos = (int16_t)S.dma_register[DMA_XSTART];
	S.dma.ypos = (int16_t)S.dma_register[DMA_YSTART];
	S.dma.width = S.dma_register[DMA_WIDTH];
	S.dma.height = S.dma_register[DMA_HEIGHT];
	S.dma.palette = (uint16_t)(S.dma_register[DMA_PALETTE] << 8);
	S.dma.color = S.dma_register[DMA_COLOR] & 0xff;
	uint32_t gfxoffset = S.dma_register[DMA_OFFSETLO] | ((uint32_t)S.dma_register[DMA_OFFSETHI] << 16);
	if (command & 0x10) {
		gfxoffset -= (uint32_t)(S.dma.width - 1) * 8;
		S.dma.rowbytes = (S.dma.rowbytes - S.dma.width + 3) & ~3;
		S.dma.xpos += S.dma.width - 1;
	} else
		S.dma.rowbytes = (S.dma.rowbytes + S.dma.width + 3) & ~3;
	// (MAME's clipping; its offset changes are overwritten below, as there)
	if (S.dma.ypos < 0) {
		S.dma.height -= -S.dma.ypos;
		S.dma.offset += (uint32_t)((-S.dma.ypos * S.dma.rowbytes) << 3);
		S.dma.ypos = 0;
	}
	if (S.dma.ypos + S.dma.height > 512)
		S.dma.height = 512 - S.dma.ypos;
	if (!(command & 0x10)) {
		if (S.dma.xpos < 0) {
			S.dma.width -= -S.dma.xpos;
			S.dma.offset += (uint32_t)(-S.dma.xpos << 3);
			S.dma.xpos = 0;
		}
		if (S.dma.xpos + S.dma.width > 512)
			S.dma.width = 512 - S.dma.xpos;
	} else {
		if (S.dma.xpos >= 512) {
			S.dma.width -= S.dma.xpos - 511;
			S.dma.offset += (uint32_t)((S.dma.xpos - 511) << 3);
			S.dma.xpos = 511;
		}
		if (S.dma.xpos - S.dma.width < 0)
			S.dma.width = S.dma.xpos;
	}
	if (gfxoffset < 0x02000000)
		gfxoffset += 0x02000000;
	S.dma.offset = gfxoffset - 0x02000000;
	dma_draw((uint16_t)command);
	// m_dma_timer->adjust(41 ns * width * height)
	int64_t px = (int64_t)S.dma.width * S.dma.height;
	if (px < 0) px = 0;
	S.dma_at = tms34010_total_cycles() + (uint64_t)(((uint64_t)px * 41u * (s_clock / 8) + 999999999u) / 1000000000u);
	S.dma_pending = 1;
	tms34010_abort();
}

static void dma_done(void)
{
	S.dma_pending = 0;
	S.dma_register[DMA_COMMAND] &= ~0x8000;
	tms34010_set_irq(0, 1);
}

// ---- the bus ----
static void vram_w(uint32_t offset, uint16_t data)
{
	offset *= 2;
	if (S.videobank_select) {
		s_vram[offset] = (uint16_t)((data & 0x00ff) | (S.dma_register[DMA_PALETTE] << 8));
		s_vram[offset + 1] = (uint16_t)((data >> 8) | (S.dma_register[DMA_PALETTE] & 0xff00));
	} else {
		s_vram[offset] = (uint16_t)((s_vram[offset] & 0x00ff) | (data << 8));
		s_vram[offset + 1] = (uint16_t)((s_vram[offset + 1] & 0x00ff) | (data & 0xff00));
	}
}
static uint16_t vram_r(uint32_t offset)
{
	offset *= 2;
	if (S.videobank_select)
		return (uint16_t)((s_vram[offset] & 0x00ff) | (s_vram[offset + 1] << 8));
	return (uint16_t)((s_vram[offset] >> 8) | (s_vram[offset + 1] & 0xff00));
}

static inline uint8_t pal5bit(uint16_t v) { v &= 0x1f; return (uint8_t)((v << 3) | (v >> 2)); }
static void paletteram_w(uint32_t offset, uint16_t data)
{
	s_palram[offset] = data;
	s_pal[offset & s_palette_mask] = (uint32_t)pal5bit(data >> 10) << 16 | (uint32_t)pal5bit(data >> 5) << 8 | pal5bit(data);
}

static uint16_t port_read(int i)
{
	return s_port[i] >= 0 ? (uint16_t)rbx_port_read(s_port[i]) : 0xffff;
}

static void cmos_enable_w(uint16_t data)
{
	S.cmos_w_enable = (uint8_t)((~data >> 9) & 1);
	if (!s_prot_data)
		return;
	data &= 0x0f00;
	S.prot_sequence[0] = S.prot_sequence[1];
	S.prot_sequence[1] = S.prot_sequence[2];
	S.prot_sequence[2] = data;
	if (s_prot_data->reset_sequence[0] == 0x1234) {
		if (data == 0x500)
			S.prot_result = (uint16_t)(s_mainram[(0x10a4390 - 0x1000000) >> 4] << 4);
	} else {
		if (S.prot_sequence[0] == s_prot_data->reset_sequence[0] && S.prot_sequence[1] == s_prot_data->reset_sequence[1] &&
			S.prot_sequence[2] == s_prot_data->reset_sequence[2])
			S.prot_index = 0;
		if ((S.prot_sequence[1] & 0x0800) != 0 && (S.prot_sequence[2] & 0x0800) == 0)
			S.prot_result = s_prot_data->data_sequence[S.prot_index++ % 100];
	}
}

static uint16_t bus_read(uint32_t a)
{
	if (a < 0x00200000)
		return vram_r(a >> 4);
	if (a >= 0x01400000 && a < 0x01410000)
		return s_cmos[((a - 0x01400000) >> 4) + S.cmos_page];
	if (a >= 0x01800000 && a < 0x01820000)
		return s_palram[(a - 0x01800000) >> 4];
	if ((a & ~0x00080000u) >= 0x01a00000 && (a & ~0x00080000u) < 0x01a000a0)
		return S.dma_register[((a & ~0x00080000u) - 0x01a00000) >> 4];
	if (a >= 0x01c00000 && a < 0x01c00060) {
		if (s_sound == WMS_SND_ADPCM && s_port[1] >= 0 && a == 0x01c00010) {
			// (the state known to be 0: no sync; the board then runs in longer pieces, the same result)
			if (wms_snd_irq_known0())
				rbx_port_custom(s_port[1], 0x4000, 0);
			else {
				sound_sync();
				rbx_port_custom(s_port[1], 0x4000, wms_snd_irq_read() ? 0x4000 : 0);
			}
		}
		return port_read((int)((a - 0x01c00000) >> 4));
	}
	if (a >= 0x01c00060 && a < 0x01c00080)
		return S.prot_result;
	if (a >= 0x02000000 && a < 0x06000000) {   // gfxrom_r (4 bpp; 6 bpp is a direct page)
		uint32_t o = ((a - 0x02000000) >> 4) * 2;
		if (o + 1 >= GFX_BYTES)
			return 0;
		if (s_palette_mask == 0x00ff)
			return (uint16_t)(s_gfx[o] | (s_gfx[o] << 4) | (s_gfx[o + 1] << 8) | (s_gfx[o + 1] << 12));
		return (uint16_t)(s_gfx[o] | (s_gfx[o + 1] << 8));
	}
	if (a >= 0xff800000) {
		if (s_mkturbo && a >= 0xfffff400 && a < 0xfffff410)
			return (uint16_t)rand();   // (mkturbo_prot_r: machine().rand())
		return s_maindata[(a - 0xff800000) >> 4];
	}
	return 0;
}

static void bus_write(uint32_t a, uint16_t v)
{
	if (a < 0x00200000) { vram_w(a >> 4, v); return; }
	if (a >= 0x01400000 && a < 0x01410000) { s_cmos[((a - 0x01400000) >> 4) + S.cmos_page] = v; return; }
	if (a >= 0x01800000 && a < 0x01820000) { paletteram_w((a - 0x01800000) >> 4, v); return; }
	if ((a & ~0x00080000u) >= 0x01a00000 && (a & ~0x00080000u) < 0x01a000a0) {
		dma_w(((a & ~0x00080000u) - 0x01a00000) >> 4, v);
		return;
	}
	if (a >= 0x01c00060 && a < 0x01c00080) { cmos_enable_w(v); return; }
	if (a >= 0x01e00000 && a < 0x01e00020) {   // cvsd_sound_w / adpcm_sound_w
		if (a != 0x01e00000)
			return;
		sound_sync();
		wms_snd_reset_write((~v >> 8) & 1);
		wms_snd_write(s_sound == WMS_SND_CVSD ? (uint16_t)((v & 0xff) | ((v & 0x200) >> 1)) : v);
		return;
	}
	if (a >= 0x01f00000 && a < 0x01f00020) {   // control_w
		S.cmos_page = ((v >> 6) & 3) * 0x1000;
		S.videobank_select = (v >> 5) & 1;
		S.autoerase_enable = (~v >> 4) & 1;
		return;
	}
	tms34010_unmapped();
}

// tms34010_config.pure_read: the reads of bus_read without an effect (mkturbo's random number is not; the ADPCM board's
// interrupt is while it is known to be 0: only the TMS34010's command write sets it, and a loop being skipped writes nothing)
static int yu_pure_read(uint32_t a)
{
	if (s_sound == WMS_SND_ADPCM && a == 0x01c00010)
		return wms_snd_irq_known0();
	return !(s_mkturbo && a >= 0xfffff400 && a < 0xfffff410);
}

static void to_shiftreg(uint32_t address, uint16_t *shiftreg)
{
	uint32_t i = (address >> 3) & 0x3ffff;
	if (i > 0x40000 - 1024) i = 0x40000 - 1024;
	memcpy(shiftreg, &s_vram[i], 2 * 512 * sizeof(uint16_t));
}
static void from_shiftreg(uint32_t address, uint16_t *shiftreg)
{
	uint32_t i = (address >> 3) & 0x3ffff;
	if (i > 0x40000 - 1024) i = 0x40000 - 1024;
	memcpy(&s_vram[i], shiftreg, 2 * 512 * sizeof(uint16_t));
}

static void autoerase_line(int scanline)
{
	if (S.autoerase_enable && scanline >= 0 && scanline < 510)
		memcpy(&s_vram[512 * scanline], &s_vram[512 * (510 + (scanline & 1))], 512 * sizeof(uint16_t));
}

static void scanline_update(int scanline, const tms34010_display_params *params, uint16_t *dest)
{
	uint16_t const *const src = &s_vram[(params->rowaddr << 9) & 0x3fe00];
	int coladdr = params->coladdr << 1;
	int hs = params->hsblnk > 2048 ? 2048 : params->hsblnk;
	// (the runs up to the row's end: coladdr wraps at 512)
	for (int x = params->heblnk; x < hs;) {
		const int c = coladdr & 0x1ff;
		const int n = 512 - c < hs - x ? 512 - c : hs - x;
		const uint16_t *s = src + c;
		uint16_t *d = dest + x;
		for (int i = 0; i < n; i++)
			d[i] = s_pen_map[s[i]];
		x += n;
		coladdr += n;
	}
	autoerase_line(params->rowaddr - 1);
	if (scanline == tms34010_screen.max_y) {
		S.ae_pending = 1;
		S.ae_param = params->rowaddr;
	}
}

// ---- machine ----
static void init_gfxrom(int bpp, const uint8_t *base, uint32_t size)
{
	const uint32_t chunk = GFX_BYTES / 4;
#define B(k, i) ((uint32_t)((k) * chunk + (i)) < size ? base[(k) * chunk + (i)] : 0)
	switch (bpp) {
	case 4:
		for (uint32_t i = 0; i < GFX_BYTES; i += 2) {
			uint8_t d1 = (B(0, (i + 0) / 4) >> (2 * ((i + 0) % 4))) & 3;
			uint8_t d2 = (B(1, (i + 0) / 4) >> (2 * ((i + 0) % 4))) & 3;
			uint8_t d3 = (B(0, (i + 1) / 4) >> (2 * ((i + 1) % 4))) & 3;
			uint8_t d4 = (B(1, (i + 1) / 4) >> (2 * ((i + 1) % 4))) & 3;
			s_gfx[i + 0] = (uint8_t)(d1 | (d2 << 2));
			s_gfx[i + 1] = (uint8_t)(d3 | (d4 << 2));
		}
		break;
	case 6:
		for (uint32_t i = 0; i < GFX_BYTES; i += 2) {
			uint8_t d1 = (B(0, (i + 0) / 4) >> (2 * ((i + 0) % 4))) & 3;
			uint8_t d2 = (B(1, (i + 0) / 4) >> (2 * ((i + 0) % 4))) & 3;
			uint8_t d3 = (B(2, (i + 0) / 4) >> (2 * ((i + 0) % 4))) & 3;
			uint8_t d4 = (B(0, (i + 1) / 4) >> (2 * ((i + 1) % 4))) & 3;
			uint8_t d5 = (B(1, (i + 1) / 4) >> (2 * ((i + 1) % 4))) & 3;
			uint8_t d6 = (B(2, (i + 1) / 4) >> (2 * ((i + 1) % 4))) & 3;
			s_gfx[i + 0] = (uint8_t)(d1 | (d2 << 2) | (d3 << 4));
			s_gfx[i + 1] = (uint8_t)(d4 | (d5 << 2) | (d6 << 4));
		}
		break;
	default:
		for (uint32_t i = 0; i < GFX_BYTES; i += 4) {
			s_gfx[i + 0] = B(0, i / 4);
			s_gfx[i + 1] = B(1, i / 4);
			s_gfx[i + 2] = B(2, i / 4);
			s_gfx[i + 3] = B(3, i / 4);
		}
		break;
	}
#undef B
}

static int yu_supported(const char *m)
{
	return !strncmp(m, "yunit_cvsd_", 11) || !strncmp(m, "yunit_adpcm_", 12);
}

static void yu_exit(void);
static void yu_reset(void);

static void nv_save(void)
{
	if (!s_nv_path[0] || !s_cmos)
		return;
	FILE *f = fopen(s_nv_path, "wb");
	if (f) {
		fwrite(s_cmos, 2, 0x4000, f);
		fclose(f);
	}
	s_nv_hash = nv_hash(s_cmos, 0x4000);
}

static int yu_init(const rbx_game *g)
{
	rbx_cheat_prefer("tms34010");   // (the cheats' main CPU: another CPU is set up first)
	const char *m = g->machine;
	const char *init = strchr(m, ':');
	init = init ? init + 1 : "";
	s_game = g;
	memset(&S, 0, sizeof(S));
	s_sound = !strncmp(m, "yunit_adpcm_", 12) ? WMS_SND_ADPCM : WMS_SND_CVSD;
	s_clock = strstr(m, "_slow") ? 40000000 : strstr(m, "_faster") ? 50000000 : 48000000;
	s_bpp = strstr(m, "4bit") ? 4 : 6;
	s_palette_mask = s_bpp == 4 ? 0x00ff : 0x0fff;
	s_npens = s_bpp == 4 ? 256 : 4096;
	s_prot_data = NULL;
	s_mkturbo = 0;
	s_nv_path[0] = 0;

	s_mainram = (uint16_t *)calloc(0x10000, 2);
	s_cmos = (uint16_t *)calloc(0x4000, 2);
	s_palram = (uint16_t *)calloc(0x2000, 2);
	s_vram = (uint16_t *)calloc(0x40000, 2);
	s_gfx = (uint8_t *)calloc(GFX_BYTES, 1);
	s_pen_map = (uint16_t *)calloc(65536, 2);
	s_pal = (uint32_t *)calloc((size_t)s_npens + 1, 4);
	s_fb = (uint16_t *)calloc(512 * 512, 2);
	if (!s_mainram || !s_cmos || !s_palram || !s_vram || !s_gfx || !s_pen_map || !s_pal || !s_fb) {
		rbx_fail("%s: out of memory", g->set);
		yu_exit();
		return -1;
	}
	uint32_t msz, gsz, csz, osz = 0;
	s_maindata = (uint16_t *)rbx_region("maindata", &msz);
	uint8_t *gfx = rbx_region("gfx", &gsz);
	uint8_t *snd = rbx_region(s_sound == WMS_SND_ADPCM ? "adpcm:cpu" : "cvsd:cpu", &csz);
	uint8_t *oki = s_sound == WMS_SND_ADPCM ? rbx_region("adpcm:oki", &osz) : NULL;
	if (!s_maindata || msz < 0x100000 || !gfx || !snd || (s_sound == WMS_SND_ADPCM && !oki)) {
		yu_exit();
		return -1;
	}
	// game inits (midyunit_m.cpp)
	int prot_start = 0, prot_end = 0, small = 0;
	if (!strcmp(init, "trog")) { s_prot_data = &trog_prot; small = 1; prot_start = 0x9eaf; prot_end = 0x9ed9; }
	else if (!strcmp(init, "smashtv")) { small = 1; prot_start = 0x9cf6; prot_end = 0x9d21; }
	else if (!strcmp(init, "hiimpact")) { s_prot_data = &hiimpact_prot; prot_start = 0x9b79; prot_end = 0x9ba3; }
	else if (!strcmp(init, "shimpact")) { s_prot_data = &shimpact_prot; prot_start = 0x9c06; prot_end = 0x9c15; }
	else if (!strcmp(init, "strkforc")) { s_prot_data = &strkforc_prot; small = 1; prot_start = 0x9f7d; prot_end = 0x9fa7; }
	else if (!strcmp(init, "totcarn")) { s_prot_data = &totcarn_prot; prot_start = 0xfc04; prot_end = 0xfc2e; }
	else if (!strncmp(init, "mk", 2)) {
		s_prot_data = &mk_prot;
		prot_start = 0xfb9c;
		prot_end = 0xfbc6;
		if (!strcmp(init, "mkyturbo"))
			s_mkturbo = 1;
		if (!strcmp(init, "mkrep")) {
			uint16_t *rom = s_maindata;
			rom[0x94234 / 2] = 0x05a0; rom[0x94236 / 2] = 0xe660; rom[0x94238 / 2] = 0x0104;
			rom[0x9637c / 2] = 0x0d3f; rom[0x9637e / 2] = 0xf343; rom[0x96380 / 2] = 0x4c00;
			rom[0x9c476 / 2] = 0xb5a0; rom[0x9c478 / 2] = 0x0030; rom[0x9c47a / 2] = 0x1420;
			rom[0x9c47c / 2] = 0xb00d; rom[0x9c47e / 2] = 0x0030;
		}
		if (!strcmp(init, "mkla3bl") && gsz >= 0x400000) {
			uint8_t *buffer = (uint8_t *)malloc(0x400000);
			if (!buffer) { rbx_fail("%s: out of memory", g->set); yu_exit(); return -1; }
			memcpy(buffer, gfx, 0x400000);
			for (int i = 0x000000; i < 0x100000; i++) gfx[i] = buffer[0x200000 | ((i & 0xfffff) * 2 + 1)];
			for (int i = 0x100000; i < 0x200000; i++) gfx[i] = buffer[(i & 0xfffff) * 2 + 1];
			for (int i = 0x200000; i < 0x300000; i++) gfx[i] = buffer[0x200000 | ((i & 0xfffff) * 2)];
			for (int i = 0x300000; i < 0x400000; i++) gfx[i] = buffer[(i & 0xfffff) * 2];
			free(buffer);
		}
	}
	init_gfxrom(s_bpp, gfx, gsz);
	for (int i = 0; i < 65536; i++)
		s_pen_map[i] = (uint16_t)(s_bpp == 4 ? (((i & 0xf000) >> 8) | (i & 0x000f)) : (((i & 0xc000) >> 8) | (i & 0x0f3f)));

	if (wms_snd_init(s_sound, snd, csz, oki, osz)) {
		yu_exit();
		return -1;
	}
	if (small)
		wms_snd_rom_protection((uint16_t)prot_start, (uint16_t)prot_end);
	else
		wms_snd_hidden_ram((uint16_t)prot_start, (uint16_t)prot_end);

	static const char *const tags[6] = { "IN0", "IN1", "IN2", "DSW", "UNK0", "UNK1" };
	for (int i = 0; i < 6; i++)
		s_port[i] = rbx_port_find(tags[i]);

	tms34010_config tc;
	memset(&tc, 0, sizeof(tc));
	tc.clock = s_clock;
	tc.pixclock = 24000000 / 6;
	tc.pixperclock = 2;
	tc.htotal = 506; tc.hbend = 90; tc.hbstart = 500; tc.vtotal = 289; tc.vbend = 20; tc.vbstart = 276;
	tc.read = bus_read;
	tc.write = bus_write;
	tc.pure_read = yu_pure_read;
	tc.to_shiftreg = to_shiftreg;
	tc.from_shiftreg = from_shiftreg;
	tc.scanline = scanline_update;
	tc.black_pen = (uint16_t)s_npens;
	tc.bitmap = s_fb;
	tc.pitch = 512;
	tms34010_init(&tc);
	tms34010_map(0x01000000, 0x010fffff, s_mainram, s_mainram);
	if (s_bpp != 4)
		tms34010_map(0x02000000, 0x05ffffff, (uint16_t *)s_gfx, NULL);
	tms34010_map(0xff800000, s_mkturbo ? 0xffefffff : 0xffffffff, s_maindata, NULL);
	tms34010_code_rom(0xff800000, s_mkturbo ? 0xffefffff : 0xffffffff);

	if (rbx_audio_setup(wms_snd_rate()) || rbx_video_setup(512, 400, s_npens + 1)) {
		yu_exit();
		return -1;
	}
	{
		extern const char *rbx_nvram_dir(void);
		const char *dir = rbx_nvram_dir();
		if (dir) {
			snprintf(s_nv_path, sizeof(s_nv_path), "%s%s.nv", dir, g->set);
			FILE *f = fopen(s_nv_path, "rb");
			if (f) {
				if (fread(s_cmos, 2, 0x4000, f) != 0x4000)
					memset(s_cmos, 0, 0x4000 * 2);
				fclose(f);
			}
			s_nv_hash = nv_hash(s_cmos, 0x4000);
			s_nv_tick = 0;
		}
	}
	yu_reset();
	return 0;
}

static void yu_exit(void)
{
	nv_save();
	wms_snd_exit();
	free(s_mainram); s_mainram = NULL;
	free(s_cmos); s_cmos = NULL;
	free(s_palram); s_palram = NULL;
	free(s_vram); s_vram = NULL;
	free(s_gfx); s_gfx = NULL;
	free(s_pen_map); s_pen_map = NULL;
	free(s_pal); s_pal = NULL;
	free(s_fb); s_fb = NULL;
	tms34010_release();   // (the JIT's block table, ADR-200)
	s_nv_path[0] = 0;
}

static void yu_reset(void)
{
	uint64_t ticks = S.ticks;
	S.dma_pending = 0;
	S.ae_pending = 0;
	S.ticks = 0;
	S.cyc_frac = 0;
	S.cyc_target = 0;
	(void)ticks;
	tms34010_reset();
	wms_snd_reset();
	wms_snd_reset_write(1);
	wms_snd_reset_write(0);
	memset(s_fb, 0, 512 * 512 * 2);
}

// run the TMS34010 to its cycle target, with the DMA's end on its cycle
static void run_main(uint64_t target)
{
	for (;;) {
		uint64_t now = tms34010_total_cycles();
		if (S.dma_pending && S.dma_at <= now) {
			dma_done();
			continue;
		}
		if (now >= target)
			break;
		uint64_t lim = target;
		if (S.dma_pending && S.dma_at < lim)
			lim = S.dma_at;
		tms34010_execute((int)(lim - now));
	}
}

static void yu_frame(void)
{
	if (s_nv_path[0] && ++s_nv_tick >= 120) {   // (every 2 s: the CMOS changed since its file was written)
		s_nv_tick = 0;
		if (nv_hash(s_cmos, 0x4000) != s_nv_hash)
			nv_save();
	}
	tms34010_screen_t *scr = &tms34010_screen;
	do {
		if (S.ae_pending) {   // (m_autoerase_line_timer: the line after the last visible one)
			S.ae_pending = 0;
			autoerase_line(S.ae_param);
		}
		int len = tms34010_line_start();
		S.line_tick0 = S.ticks;
		S.line_cyc0 = S.cyc_target;
		S.ticks += (uint64_t)len;
		S.cyc_frac += (uint64_t)len * s_clock;
		uint64_t n = S.cyc_frac / 64000000u;
		S.cyc_frac -= n * 64000000u;
		S.cyc_target += n;
		run_main(S.cyc_target);
		wms_snd_run(S.ticks);
	} while (!tms34010_line_end());
	rbx_audio.frame_mhz = (uint32_t)(8000000ull * 1000u / ((uint64_t)scr->width * (uint64_t)scr->height));

	rbx_video_wait_idle();
	int w = scr->max_x - scr->min_x + 1, h = scr->max_y - scr->min_y + 1;
	if (w > 512) w = 512;
	if (h > 400) h = 400;
	rbx_video.width = w;
	rbx_video.height = h;
	for (int y = 0; y < h; y++)
		memcpy(rbx_video.bitmap + (size_t)y * rbx_video.pitch, s_fb + (size_t)y * 512, (size_t)w * 2);
	memcpy(rbx_video.palette, s_pal, (size_t)(s_npens + 1) * 4);
	rbx_video.palette_used = s_npens + 1;
	rbx_video_frame_done();
	wms_snd_frame_end();   // (last: the sound worker's lag overlaps the render wait and the copy, see midtunit.c)
}

static int yu_state(void *ctx, int load)
{
	if (rbx_state_io(ctx, "yuS", &S, sizeof(S)) || rbx_state_io(ctx, "yuram", s_mainram, 0x10000 * 2) ||
		rbx_state_io(ctx, "yucmos", s_cmos, 0x4000 * 2) || rbx_state_io(ctx, "yupal", s_palram, 0x2000 * 2) ||
		rbx_state_io(ctx, "yuvram", s_vram, 0x40000 * 2) || tms34010_state(ctx, load) || wms_snd_state(ctx, load))
		return -1;
	if (load)
		for (uint32_t i = 0; i < 0x2000; i++)
			paletteram_w(i, s_palram[i]);
	return 0;
}
static int yu_save(void *ctx) { return yu_state(ctx, 0); }
static int yu_load(void *ctx) { return yu_state(ctx, 1); }

const rbx_driver rbx_drv_midyunit = { "Midway Y-Unit", yu_init, yu_exit, yu_reset, yu_frame, yu_save, yu_load,
                                      yu_supported, NULL, NULL };

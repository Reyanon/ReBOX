/*  Data East Battle Rangers / Bloody Wolf. Ported from MAME src/mame/dataeast/battlera.cpp (BSD-3-Clause,
    copyright-holders: Bryan McPhail) with the PC Engine video chips it uses: src/devices/video/huc6260.cpp (VCE)
    and huc6270.cpp (VDC) (BSD-3-Clause, copyright-holders: Wilbert Pol).
    Two HuC6280 at 7.16 MHz. The main one drives a HuC6270 (64K words of VRAM: one background map of 8x8 tiles,
    64 sprites of 16x16 to 32x64 read into a line buffer, raster / vblank / DMA IRQs) whose pixel stream the VCE
    (the battlera variant: its 512 colours also mapped at 1E1000) turns into colours, master clock by master clock as
    MAME's update_events (pixels made when the CPUs reach the hsync / vsync points and three more points of a line).
    The sound one plays its internal PSG (music, both sides to mono), a YM2203 and an MSM5205 (its VCK on IRQ2).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/h6280/h6280.h"
#include "../../sound/c6280.h"
#include "../../sound/msm5205u.h"
#include "../../sound/rbx_ym.h"

#include <stdlib.h>
#include <string.h>

#define MASTER 21477272
#define WPF 1365
#define LPF 263
#define FRAME_MHZ 59826          // set_raw(21.477 MHz, 1365, 64, 1152, 263, 18, 260)
#define VIS_X0 64
#define VIS_W 272                // (1088 master clocks at 4 per pixel: the 5.37 MHz dot clock)
#define VIS_Y0 18
#define VIS_H 242
#define HSYNC_START (WPF - 237)

// ---------------------------------------------------------------- HuC6270
enum { MAWR, MARR, VxR, CR = 5, RCR, BXR, BYR, MWR, HSR, HDR, VPR, VDW, VCR, DCR, SOUR, DESR, LENR, DVSSR };
enum { V_VSW, V_VDS, V_VDW, V_VCR };
enum { H_HDS, H_HDW, H_HDE, H_HSW };
#define ST_VD 0x20
#define ST_DV 0x10
#define ST_DS 0x08
#define ST_RR 0x04
#define ST_OR 0x02
#define ST_CR 0x01
#define VRAM_MASK 0xffff

static struct {
	uint16_t mawr, marr, vrr, vwr, cr, rcr, bxr, byr, mwr, hsr, hdr, vpr, vdw, vcr, dcr, sour, desr, lenr, dvssr;
	uint8_t register_index, status, hsync, vsync, vert_state, horz_state, vd_triggered, dvssr_written, dma_enabled;
	uint8_t satb_countdown, irq;
	int32_t vert_to_go, horz_to_go, horz_steps, raster_count, sprites_this_line, sprite_row_index;
	uint16_t byr_latched, bxr_latched, bat_address, bat_address_mask, bat_row, bat_column;
	uint16_t bat_tile_row[8], sat[4 * 64], sprite_row[1024];
	// the VCE
	uint16_t palette[512], address, greyscales, pixels_per_clock, pixel_data, pixel_clock, last_h, last_v, height;
	uint8_t blur, pad;
} s_v;
static uint16_t s_vram[0x10000];
static const uint8_t vram_increments[4] = { 1, 32, 64, 128 };

static h6280_t s_cpu, s_snd;

static void vdc_irq(int state)
{
	s_v.irq = (uint8_t)state;
	h6280_set_irq(&s_cpu, H6280_IRQ1, state);
}

static void fetch_bat_tile_row(void)
{
	const uint16_t bat_data = s_vram[s_v.bat_address & VRAM_MASK];
	const uint16_t tile_palette = (bat_data >> 8) & 0xf0;
	const uint32_t tile_address = ((uint32_t)(bat_data & 0x0fff) << 4) + s_v.bat_row;
	uint16_t data1 = s_vram[tile_address & VRAM_MASK], data2 = (data1 >> 7) & 0x1fe;
	uint16_t data3 = s_vram[(tile_address + 8) & VRAM_MASK], data4 = (data3 >> 5) & 0x7f8;
	data3 = (uint16_t)(data3 << 2);
	for (int i = 7; i >= 0; i--) {
		uint16_t c = (data1 & 1) | (data2 & 2) | (data3 & 4) | (data4 & 8);
		if (c)   // (colour 0 of the background always from palette 0)
			c |= tile_palette;
		s_v.bat_tile_row[i] = c;
		data1 >>= 1;
		data2 >>= 1;
		data3 >>= 1;
		data4 >>= 1;
	}
}

static void add_sprite(int index, int x, int pattern, int line, int flip_x, int palette, int priority, int sat_lsb)
{
	if (s_v.sprites_this_line >= 16)
		return;
	uint32_t b0, b1, b2, b3;
	if (flip_x)
		flip_x = 0x0f;
	pattern += (line >> 4) << 1;
	const uint32_t pa = ((uint32_t)pattern << 6) + (uint32_t)(line & 0x0f);
	if ((s_v.mwr & 0x0c) == 0x04) {
		b0 = s_vram[(pa + (sat_lsb ? 0x20 : 0x00)) & VRAM_MASK];
		b1 = (uint32_t)s_vram[(pa + (sat_lsb ? 0x30 : 0x10)) & VRAM_MASK] << 1;
		b2 = b3 = 0;
	} else {
		b0 = s_vram[pa & VRAM_MASK];
		b1 = (uint32_t)s_vram[(pa + 0x10) & VRAM_MASK] << 1;
		b2 = (uint32_t)s_vram[(pa + 0x20) & VRAM_MASK] << 2;
		b3 = (uint32_t)s_vram[(pa + 0x30) & VRAM_MASK] << 3;
	}
	for (int j = 15; j >= 0; j--) {
		uint8_t data = (uint8_t)((b3 & 8) | (b2 & 4) | (b1 & 2) | (b0 & 1));
		if (data) {
			data |= (uint8_t)(palette << 4);
			const int px = x + (j ^ flip_x);
			if (px < 1024) {
				if (!s_v.sprite_row[px])
					s_v.sprite_row[px] = (uint16_t)((priority ? 0x4000 : 0) | (index << 8) | data);
				else if (!(s_v.sprite_row[px] & 0xff00))   // (sprite 0 collision)
					s_v.sprite_row[px] |= 0x8000;
			}
		}
		b0 >>= 1;
		b1 >>= 1;
		b2 >>= 1;
		b3 >>= 1;
	}
	s_v.sprites_this_line++;
}

static void select_sprites(void)
{
	static const int cgy_table[4] = { 16, 32, 64, 64 };
	s_v.sprites_this_line = 0;
	memset(s_v.sprite_row, 0, sizeof(s_v.sprite_row));
	s_v.sprite_row_index = 0x20;
	for (int i = 0; i < 4 * 64; i += 4) {
		const uint16_t *s = &s_v.sat[i];
		const int cgy = (s[3] >> 12) & 3, height = cgy_table[cgy];
		int line = s_v.raster_count - 1 - s[0];
		if (line < 0 || line >= height)
			continue;
		const int index = i >> 2, xpos = s[1], palette = s[3] & 0x0f, priority = (s[3] >> 7) & 1;
		const int sat_lsb = s[2] & 1, cgx = (s[3] >> 8) & 1;
		int pattern = s[2] >> 1;
		if (cgy & 1) pattern &= ~0x0002;
		if (cgy & 2) pattern &= ~0x0006;
		if ((s[3] >> 15) & 1)
			line = (height - 1) - line;
		if (cgx) {
			pattern &= ~0x0001;
			if ((s[3] >> 11) & 1) {
				add_sprite(index, xpos, pattern + 1, line, 1, palette, priority, sat_lsb);
				add_sprite(index, xpos + 16, pattern, line, 1, palette, priority, sat_lsb);
			} else {
				add_sprite(index, xpos, pattern, line, 0, palette, priority, sat_lsb);
				add_sprite(index, xpos + 16, pattern + 1, line, 0, palette, priority, sat_lsb);
			}
		} else
			add_sprite(index, xpos, pattern, line, (s[3] >> 11) & 1, palette, priority, sat_lsb);
	}
	if (s_v.sprites_this_line >= 16 && ((s_v.cr >> 1) & 1)) {
		s_v.status |= ST_OR;
		vdc_irq(1);
	}
}

static void handle_vblank(void)
{
	if (s_v.vd_triggered)
		return;
	if ((s_v.cr >> 3) & 1) {
		s_v.status |= ST_VD;
		vdc_irq(1);
	}
	if (s_v.dvssr_written || ((s_v.dcr >> 4) & 1)) {   // the VRAM -> SATB DMA
		for (int i = 0; i < 4 * 64; i += 4) {
			s_v.sat[i + 0] = s_vram[(s_v.dvssr + i + 0) & VRAM_MASK] & 0x03ff;
			s_v.sat[i + 1] = s_vram[(s_v.dvssr + i + 1) & VRAM_MASK] & 0x03ff;
			s_v.sat[i + 2] = s_vram[(s_v.dvssr + i + 2) & VRAM_MASK] & 0x07ff;
			s_v.sat[i + 3] = s_vram[(s_v.dvssr + i + 3) & VRAM_MASK];
		}
		s_v.dvssr_written = 0;
		if (s_v.dcr & 1)
			s_v.satb_countdown = 4;
	}
	s_v.vd_triggered = 1;
}

static void next_vert_state(void)
{
	switch (s_v.vert_state) {
	case V_VSW:
		s_v.vert_state = V_VDS;
		s_v.vert_to_go = ((s_v.vpr >> 8) & 0xff) + 2;
		break;
	case V_VDS:
		s_v.vert_state = V_VDW;
		s_v.vert_to_go = (s_v.vdw & 0x1ff) + 1;
		s_v.byr_latched = s_v.byr;
		s_v.vd_triggered = 0;
		break;
	case V_VDW:
		s_v.vert_state = V_VCR;
		s_v.vert_to_go = s_v.vcr & 0xff;
		handle_vblank();
		break;
	case V_VCR:
		s_v.vert_state = V_VSW;
		s_v.vert_to_go = (s_v.vpr & 0x1f) + 1;
		break;
	}
}

static void next_horz_state(void)
{
	switch (s_v.horz_state) {
	case H_HDS: {
		static const int width_shift[4] = { 5, 6, 7, 7 };
		s_v.bxr_latched = s_v.bxr;
		s_v.horz_state = H_HDW;
		s_v.horz_to_go = (s_v.hdr & 0x7f) + 1;
		const int width = width_shift[(s_v.mwr >> 4) & 3];
		const uint16_t v = s_v.byr_latched & (((s_v.mwr >> 6) & 1) ? 0x1ff : 0xff);
		s_v.bat_row = v & 7;
		s_v.bat_address_mask = (uint16_t)((1 << width) - 1);
		s_v.bat_address = (uint16_t)(((v >> 3) << width) | ((s_v.bxr_latched >> 3) & s_v.bat_address_mask));
		s_v.bat_column = s_v.bxr & 7;
		fetch_bat_tile_row();
		break;
	}
	case H_HDW:
		s_v.horz_state = H_HDE;
		s_v.horz_to_go = ((s_v.hdr >> 8) & 0x7f) + 1;
		break;
	case H_HDE:
		s_v.horz_state = H_HSW;
		s_v.horz_to_go = (s_v.hsr & 0x1f) + 1;
		break;
	case H_HSW: {
		s_v.horz_state = H_HDS;
		const int t = (s_v.hsr >> 8) & 0x7f;
		s_v.horz_to_go = (t > 2 ? t : 2) + 1;
		while (s_v.vert_to_go == 0)
			next_vert_state();
		select_sprites();
		break;
	}
	}
	s_v.horz_steps = 0;
}

static uint16_t next_pixel(void)
{
	uint16_t data = 0x100;   // (HUC6270_SPRITE)
	if (s_v.vert_state == V_VDW && s_v.horz_state == H_HDW) {
		const uint16_t sr = s_v.sprite_row[s_v.sprite_row_index & 1023];
		const uint8_t sprite_data = sr & 0xff;
		if ((s_v.cr >> 7) & 1) {
			data = s_v.bat_tile_row[s_v.bat_column];
			if (sprite_data && ((s_v.cr >> 6) & 1)) {
				if ((sr >> 14) & 1)
					data = 0x100 | sprite_data;
				else if (data == 0)
					data = 0x100 | sprite_data;
			}
		} else if ((s_v.cr >> 6) & 1)
			data = 0x100 | sprite_data;
		s_v.sprite_row_index++;
		if (++s_v.bat_column >= 8) {
			s_v.bat_address = (uint16_t)((s_v.bat_address & ~s_v.bat_address_mask) |
			                             ((s_v.bat_address + 1) & s_v.bat_address_mask));
			s_v.bat_column = 0;
			fetch_bat_tile_row();
		}
		if ((sr >> 15) && (s_v.cr & 1)) {
			s_v.status |= ST_CR;
			vdc_irq(1);
		}
	}
	if (++s_v.horz_steps == 8) {
		s_v.horz_to_go--;
		s_v.horz_steps = 0;
		while (s_v.horz_to_go == 0)
			next_horz_state();
	}
	return data;
}

static void vsync_changed(int state)
{
	if (s_v.vsync != state) {
		if (!state) {
			s_v.vert_state = V_VCR;
			s_v.vert_to_go = 0;
			while (s_v.vert_to_go == 0)
				next_vert_state();
		} else
			handle_vblank();
	}
	s_v.vsync = (uint8_t)state;
}

static void handle_dma(void)
{
	if (!s_v.dma_enabled)
		return;
	const int desr_inc = (s_v.dcr & 8) ? -1 : 1, sour_inc = (s_v.dcr & 4) ? -1 : 1;
	do {   // (the VRAM -> VRAM DMA at once)
		const uint16_t data = s_vram[s_v.sour];
		s_vram[s_v.desr] = data;
		s_v.sour = (uint16_t)(s_v.sour + sour_inc);
		s_v.desr = (uint16_t)(s_v.desr + desr_inc);
		s_v.lenr--;
	} while (s_v.lenr != 0xffff);
	if ((s_v.dcr >> 1) & 1) {
		s_v.status |= ST_DV;
		vdc_irq(1);
	}
	s_v.dma_enabled = 0;
}

static void hsync_changed(int state)
{
	if (s_v.hsync != state) {
		if (state) {
			if (s_v.satb_countdown && --s_v.satb_countdown == 0) {
				s_v.status |= ST_DS;
				vdc_irq(1);
			}
			s_v.horz_state = H_HSW;
			s_v.horz_to_go = 0;
			s_v.horz_steps = 0;
			s_v.byr_latched++;
			s_v.raster_count++;
			if (s_v.vert_to_go == 2 && s_v.vert_state == V_VDS)
				s_v.raster_count = 0x40;
			s_v.vert_to_go--;
			while (s_v.horz_to_go == 0)
				next_horz_state();
			handle_dma();
		} else if (s_v.raster_count == s_v.rcr && ((s_v.cr >> 2) & 1)) {
			s_v.status |= ST_RR;
			vdc_irq(1);
		}
	}
	s_v.hsync = (uint8_t)state;
}

static uint8_t vdc_read(int offset)
{
	switch (offset & 3) {
	case 0: {
		const uint8_t d = s_v.status;
		s_v.status &= (uint8_t)~(ST_VD | ST_DV | ST_RR | ST_CR | ST_OR | ST_DS);
		vdc_irq(0);
		return d;
	}
	case 2: return (uint8_t)s_v.vrr;
	case 3: {
		const uint8_t d = (uint8_t)(s_v.vrr >> 8);
		if (s_v.register_index == VxR) {
			s_v.marr = (uint16_t)(s_v.marr + vram_increments[(s_v.cr >> 11) & 3]);
			s_v.vrr = s_vram[s_v.marr];
		}
		return d;
	}
	}
	return 0;
}

static void regs_w(int reg, uint16_t data, uint16_t mask)
{
#define COMBINE(r) (r) = (uint16_t)(((r) & ~mask) | (data & mask))
	switch (reg) {
	case MAWR: COMBINE(s_v.mawr); break;
	case MARR: COMBINE(s_v.marr); s_v.vrr = s_vram[s_v.marr]; break;
	case VxR:
		COMBINE(s_v.vwr);
		if (mask & 0xff00) {
			s_vram[s_v.mawr] = s_v.vwr;
			s_v.mawr = (uint16_t)(s_v.mawr + vram_increments[(s_v.cr >> 11) & 3]);
		}
		break;
	case CR: COMBINE(s_v.cr); break;
	case RCR: COMBINE(s_v.rcr); s_v.rcr &= 0x3ff; break;
	case BXR: COMBINE(s_v.bxr); s_v.bxr &= 0x3ff; break;
	case BYR: COMBINE(s_v.byr); s_v.byr &= 0x1ff; s_v.byr_latched = s_v.byr; break;
	case MWR: COMBINE(s_v.mwr); break;
	case HSR: COMBINE(s_v.hsr); break;
	case HDR: COMBINE(s_v.hdr); break;
	case VPR: COMBINE(s_v.vpr); break;
	case VDW: COMBINE(s_v.vdw); break;
	case VCR: COMBINE(s_v.vcr); break;
	case DCR: COMBINE(s_v.dcr); break;
	case SOUR: COMBINE(s_v.sour); break;
	case DESR: COMBINE(s_v.desr); break;
	case LENR: COMBINE(s_v.lenr); if (mask & 0xff00) s_v.dma_enabled = 1; break;
	case DVSSR: COMBINE(s_v.dvssr); s_v.dvssr_written = 1; break;
	}
#undef COMBINE
}

static void vdc_write(int offset, uint8_t data)
{
	switch (offset & 3) {
	case 0: s_v.register_index = data & 0x1f; break;
	case 2: regs_w(s_v.register_index, data, 0x00ff); break;
	case 3: regs_w(s_v.register_index, (uint16_t)(data << 8), 0xff00); break;
	}
}

// ---------------------------------------------------------------- the VCE
static uint16_t s_bmp[VIS_H][VIS_W];

static uint8_t vce_read(int offset)
{
	switch (offset & 7) {
	case 4: return (uint8_t)s_v.palette[s_v.address];
	case 5: {
		const uint8_t d = (uint8_t)(0xfe | (s_v.palette[s_v.address] >> 8));
		s_v.address = (s_v.address + 1) & 0x1ff;
		return d;
	}
	}
	return 0xff;
}
static void vce_write(int offset, uint8_t data)
{
	switch (offset & 7) {
	case 0:
		s_v.greyscales = (uint16_t)((data & 0x80) << 2);
		s_v.blur = (data >> 2) & 1;
		s_v.pixels_per_clock = (data & 2) ? 2 : (data & 1) ? 3 : 4;
		break;
	case 2: s_v.address = (uint16_t)((s_v.address & 0x100) | data); break;
	case 3: s_v.address = (uint16_t)((s_v.address & 0x0ff) | ((data & 1) << 8)); break;
	case 4: s_v.palette[s_v.address] = (uint16_t)((s_v.palette[s_v.address] & 0x100) | data); break;
	case 5:
		s_v.palette[s_v.address] = (uint16_t)((s_v.palette[s_v.address] & 0x0ff) | ((data & 1) << 8));
		s_v.address = (s_v.address + 1) & 0x1ff;
		break;
	}
}

// a pixel's pen at master clock h of the row (the clocks VIS_X0 + 4k sampled)
static inline void put_pixel(uint16_t *row, int h, int ppc, uint16_t d)
{
	if (row && h + ppc > VIS_X0 && h < VIS_X0 + VIS_W * 4) {
		const uint16_t pen = (uint16_t)((s_v.palette[d & 0x1ff] & 0x1ff) | s_v.greyscales);
		int x = h < VIS_X0 ? VIS_X0 : h;
		x = (x + 3) & ~3;
		for (; x < h + ppc && x < VIS_X0 + VIS_W * 4; x += 4)
			row[(x - VIS_X0) >> 2] = pen;
	}
}

// n whole pixels from master clock h (no event among them): next_pixel's work a horizontal state's run at a time
// (the same VDC steps and pixels); returns the clock after them
static int pixels(int h, int n, int ppc, uint16_t *row)
{
	while (n > 0) {
		const int left = (s_v.horz_to_go - 1) * 8 + (8 - s_v.horz_steps);   // (pixels to the state's end)
		if (left <= 0 || left > 8 * 0x10000) {   // (not a count from next_horz_state)
			put_pixel(row, h, ppc, next_pixel());
			h += ppc;
			n--;
			continue;
		}
		const int c = n < left ? n : left;
		if (s_v.vert_state == V_VDW && s_v.horz_state == H_HDW) {
			const int bg = (s_v.cr >> 7) & 1, spr = (s_v.cr >> 6) & 1, ccr = s_v.cr & 1;
			// (4 clocks a pixel from a sampled clock, all on the row: a pixel a bitmap pen)
			uint16_t *out = row && ppc == 4 && !(h & 3) && h >= VIS_X0 && h + c * 4 <= VIS_X0 + VIS_W * 4
			              ? row + ((h - VIS_X0) >> 2) : NULL;
			for (int i = 0; i < c; i++, h += ppc) {
				const uint16_t sr = s_v.sprite_row[s_v.sprite_row_index & 1023];
				const uint8_t sprite_data = sr & 0xff;
				uint16_t data = 0x100;
				if (bg) {
					data = s_v.bat_tile_row[s_v.bat_column];
					if (sprite_data && spr) {
						if ((sr >> 14) & 1)
							data = 0x100 | sprite_data;
						else if (data == 0)
							data = 0x100 | sprite_data;
					}
				} else if (spr)
					data = 0x100 | sprite_data;
				s_v.sprite_row_index++;
				if (++s_v.bat_column >= 8) {
					s_v.bat_address = (uint16_t)((s_v.bat_address & ~s_v.bat_address_mask) |
					                             ((s_v.bat_address + 1) & s_v.bat_address_mask));
					s_v.bat_column = 0;
					fetch_bat_tile_row();
				}
				if ((sr >> 15) && ccr) {
					s_v.status |= ST_CR;
					vdc_irq(1);
				}
				if (out)
					out[i] = (uint16_t)((s_v.palette[data & 0x1ff] & 0x1ff) | s_v.greyscales);
				else
					put_pixel(row, h, ppc, data);
			}
		} else {
			if (row && h + c * ppc > VIS_X0 && h < VIS_X0 + VIS_W * 4) {   // (the sprite colour 0)
				const uint16_t pen = (uint16_t)((s_v.palette[0x100] & 0x1ff) | s_v.greyscales);
				int x = h < VIS_X0 ? VIS_X0 : h;
				x = (x + 3) & ~3;
				for (; x < h + c * ppc && x < VIS_X0 + VIS_W * 4; x += 4)
					row[(x - VIS_X0) >> 2] = pen;
			}
			h += c * ppc;
		}
		const int t = s_v.horz_steps + c;
		s_v.horz_to_go -= t >> 3;
		s_v.horz_steps = t & 7;
		while (s_v.horz_to_go == 0)
			next_horz_state();
		n -= c;
	}
	return h;
}

// the VCE's master clocks up to h_end (exclusive) of the current line: pixels, hsync / vsync (stepped a pixel clock
// or up to the next event at a time; the same pixels as MAME's clock by clock)
static void vce_run(int h_end)
{
	int h = s_v.last_h, v = s_v.last_v;
	const int ppc = s_v.pixels_per_clock;
	while (h < h_end) {
		const int ev = h < HSYNC_START ? HSYNC_START : h < HSYNC_START + 30 ? HSYNC_START + 30 : WPF;
		if (s_v.pixel_clock == 0) {
			// whole pixels short of the next event / h_end at once (the last one below: the event after it)
			const int lim = h_end < ev ? h_end : ev;
			if (h + ppc < lim)
				h = pixels(h, (lim - h - 1) / ppc, ppc, v >= VIS_Y0 && v < VIS_Y0 + VIS_H ? s_bmp[v - VIS_Y0] : NULL);
			s_v.pixel_data = next_pixel();
		}
		int run = ppc - s_v.pixel_clock;
		if (h + run > ev) run = ev - h;
		if (h + run > h_end && h_end < WPF) run = h_end - h;
		if (v >= VIS_Y0 && v < VIS_Y0 + VIS_H && h + run > VIS_X0 && h < VIS_X0 + VIS_W * 4) {
			const uint16_t pen = (uint16_t)((s_v.palette[s_v.pixel_data & 0x1ff] & 0x1ff) | s_v.greyscales);
			int x = h < VIS_X0 ? VIS_X0 : h;
			x = (x + 3) & ~3;   // (the sampled clocks: VIS_X0 + 4k; VIS_X0 is a multiple of 4)
			for (; x < h + run && x < VIS_X0 + VIS_W * 4; x += 4)
				s_bmp[v - VIS_Y0][(x - VIS_X0) >> 2] = pen;
		}
		{
			const int pc2 = s_v.pixel_clock + run;   // (== ppc but after a change of the clock rate)
			s_v.pixel_clock = (uint16_t)(pc2 == ppc ? 0 : pc2 >= 0 && pc2 < ppc ? pc2 : pc2 % ppc);
		}
		h += run;
		if (h == HSYNC_START)
			hsync_changed(0);
		else if (h == HSYNC_START + 30) {
			if (v >= s_v.height - 4)
				vsync_changed((v >= s_v.height - 4 && v < s_v.height - 1) ? 0 : 1);
		} else if (h == WPF) {   // (the end of hsync: the next line)
			h = 0;
			hsync_changed(1);
			s_v.pixel_clock = 0;
			v = (v + 1) % s_v.height;
			if (h_end == WPF)
				break;
		}
	}
	s_v.last_h = (uint16_t)h;
	s_v.last_v = (uint16_t)v;
}

// ---------------------------------------------------------------- machine
static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static uint8_t s_ram[0x2000], s_sram[0x2000];
static uint8_t s_control_port_select, s_msm5205next, s_toggle, s_latch, s_latch_pending;
static int s_port[5];
static uint32_t s_frame;
static uint64_t s_mclk;
static uint64_t s_ym_frac;
static c6280_t s_psg;
static msm5205u_t s_msm;
static int s_mix_cap;
static uint16_t (*s_snap)[VIS_W];
// The sound board only hears the main CPU's latch, so it runs a frame behind on a worker (rbx_aux): the frame's latch
// writes queued with their line and segment, the job runs the sound CPU through that frame applying them before the
// segment they were written in (as when it ran after the main CPU's segment), its samples go out with the next frame
// (the same samples with or without the worker thread)
#define EV_MAX 2048
typedef struct { uint16_t line; uint8_t seg, val; } br_ev_t;
static br_ev_t s_ev[2][EV_MAX];
static int s_ev_n[2], s_ev_cur, s_line, s_seg;
static int32_t *s_out;
static int s_out_n;
static uint64_t s_smclk;   // (the sound side's master clock)
static volatile int s_job_frame;   // a frame queued (the job does the work queued: a sync runs it again)

static uint8_t main_r(void *ctx, uint32_t pa)
{
	(void)ctx;
	if (pa >= 0x1e1000 && pa < 0x1e1400) {   // the VCE's palette, direct
		const uint32_t o = pa - 0x1e1000;
		return (uint8_t)(s_v.palette[o >> 1] >> ((o & 1) << 3));
	}
	if (pa >= 0x1fe000 && pa < 0x1fe400) return vdc_read((int)(pa & 3));
	if (pa >= 0x1fe400 && pa < 0x1fe800) return vce_read((int)(pa & 7));
	return 0xff;
}
static void main_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if (pa == 0x1e0800) {   // the sound latch (its pending data: the sound IRQ1): to the sound job
		const int c = s_ev_cur;
		if (s_ev_n[c] < EV_MAX) {
			br_ev_t *e = &s_ev[c][s_ev_n[c]++];
			e->line = (uint16_t)s_line;
			e->seg = (uint8_t)s_seg;
			e->val = v;
		}
	} else if (pa >= 0x1e1000 && pa < 0x1e1400) {
		const uint32_t o = pa - 0x1e1000;
		const int shift = (int)(o & 1) << 3;
		s_v.palette[o >> 1] = (uint16_t)((s_v.palette[o >> 1] & ~(0xff << shift)) | (v << shift));
	} else if (pa >= 0x1fe000 && pa < 0x1fe400)
		vdc_write((int)(pa & 3), v);
	else if (pa >= 0x1fe400 && pa < 0x1fe800)
		vce_write((int)(pa & 7), v);
}
static void main_st(void *ctx, int port, uint8_t v) { (void)ctx; vdc_write(port, v); }
static uint8_t main_port_in(void *ctx)
{
	(void)ctx;
	uint8_t d = 0xff;
	for (int i = 0; i < 5; i++)
		if (!((s_control_port_select >> i) & 1))
			d &= (uint8_t)rbx_port_read(s_port[i]);
	return d;
}
static void main_port_out(void *ctx, uint8_t v) { (void)ctx; s_control_port_select = v; }

static void snd_w(void *ctx, uint32_t pa, uint8_t v)
{
	(void)ctx;
	if (pa == 0x040000 || pa == 0x040001) rbx_ym_write(0, pa & 1, v);
	else if (pa == 0x080000 || pa == 0x080001) s_msm5205next = v;
}
static uint8_t snd_port_in(void *ctx)
{
	(void)ctx;
	s_latch_pending = 0;   // (the generic latch's read: its pending data cleared)
	h6280_set_irq(&s_snd, H6280_IRQ1, 0);
	return s_latch;
}
static void snd_port_out(void *ctx, uint8_t v) { (void)ctx; (void)v; msm5205u_reset_w(&s_msm, 0); }
static void snd_psg(void *ctx, int offset, uint8_t v) { (void)ctx; c6280_w(&s_psg, offset, v); }
static void snd_irq_ack(void *ctx, int line)
{
	(void)ctx;
	if (line == H6280_IRQ2)   // (adpcm_int's HOLD_LINE)
		h6280_set_irq(&s_snd, H6280_IRQ2, 0);
}
static void adpcm_int(msm5205u_t *m, void *user)
{
	(void)user;
	msm5205u_data_w(m, s_msm5205next >> 4);
	s_msm5205next = (uint8_t)(s_msm5205next << 4);
	s_toggle ^= 1;
	if (s_toggle)
		h6280_set_irq(&s_snd, H6280_IRQ2, 1);
}

static void br_render(void)
{
	for (int i = 0; i < 512; i++) {   // GRB 3:3:3, then the greyscales
		const int r = (i >> 3) & 7, g = (i >> 6) & 7, b = i & 7;
		const int r8 = (r << 5) | (r << 2) | (r >> 1), g8 = (g << 5) | (g << 2) | (g >> 1), b8 = (b << 5) | (b << 2) | (b >> 1);
		const int y = ((66 * r8 + 129 * g8 + 25 * b8 + 128) >> 8) + 16;
		rbx_video.palette[i] = (uint32_t)(r8 << 16 | g8 << 8 | b8);
		rbx_video.palette[512 + i] = (uint32_t)(y << 16 | y << 8 | y);
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++)
		memcpy(bm + y * pitch, s_snap[y], VIS_W * 2);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap, s_bmp, sizeof(s_bmp));
	rbx_video_frame_done();
}

static void br_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_sram, 0, sizeof(s_sram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_bmp, 0, sizeof(s_bmp));
	memset(&s_v, 0, sizeof(s_v));
	// huc6270 device_reset
	s_v.hsr = 0x0202;
	s_v.hdr = 0x041f;
	s_v.vpr = 0x0f02;
	s_v.vdw = 0x00ef;
	s_v.vcr = 0x0004;
	s_v.raster_count = 0x4000;
	s_v.vert_state = V_VSW;
	s_v.horz_state = H_HDS;
	// huc6260 device_reset (the screen at time 0: the vblank start line)
	s_v.pixels_per_clock = 4;
	s_v.height = LPF;
	s_v.last_v = VIS_Y0 + VIS_H;
	s_v.last_h = 0;
	s_control_port_select = 0xff;
	rbx_aux_sync();
	s_ev_n[0] = s_ev_n[1] = 0;
	s_ev_cur = 0;
	s_out_n = 0;
	s_smclk = 0;
	s_job_frame = 0;
	s_msm5205next = s_toggle = 0;
	s_latch = s_latch_pending = 0;
	h6280_set_irq(&s_cpu, H6280_IRQ1, 0);
	h6280_reset(&s_cpu);
	h6280_set_irq(&s_snd, H6280_IRQ1, 0);
	h6280_set_irq(&s_snd, H6280_IRQ2, 0);
	h6280_reset(&s_snd);
	c6280_reset(&s_psg);
	rbx_ym_reset(0);
	msm5205u_reset(&s_msm);
	s_frame = 0;
	s_mclk = 0;
	s_ym_frac = 0;
}

static const int s_segs[6] = { 341, 682, 1023, HSYNC_START, HSYNC_START + 30, WPF };

static void snd_job(void)   // (the frame queued last: the other buffer than the current one)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_ev_cur ^ 1;
	const br_ev_t *e = s_ev[b];
	int k = 0, n = 0;
	memset(s_out, 0, sizeof(int32_t) * (size_t)s_mix_cap);   // (the renders add)
	for (int line = 0; line < LPF; line++) {
		const uint64_t base = s_smclk;
		const int64_t q = (int64_t)(base / 3);
		const int r = (int)(base - (uint64_t)q * 3);
		for (int sg = 0; sg < 6; sg++) {
			for (; k < s_ev_n[b] && e[k].line == line && e[k].seg == sg; k++) {
				s_latch = e[k].val;
				s_latch_pending = 1;
				h6280_set_irq(&s_snd, H6280_IRQ1, 1);
			}
			h6280_run_until(&s_snd, q + (r + s_segs[sg]) / 3);
		}
		s_smclk = base + WPF;
		const int m = rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, 12000000 / 8, FRAME_MHZ, LPF), s_out + n,
		                             s_mix_cap - n);
		c6280_render(&s_psg, s_out + n, m, rbx_ym_rate(0), 154);   // 0.60
		msm5205u_render(&s_msm, s_out + n, m, rbx_ym_rate(0), 218);   // 0.85
		n += m;
	}
	s_ev_n[b] = 0;
	s_out_n = n;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void br_exit(void);

static int br_init(const rbx_game *g)
{
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	if (!s_rom || s_rom_size < 0x100000 || !s_srom || s_srom_size < 0x10000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	h6280_init(&s_cpu);
	s_cpu.read = main_r;
	s_cpu.write = main_w;
	s_cpu.st_write = main_st;
	s_cpu.port_in = main_port_in;
	s_cpu.port_out = main_port_out;
	for (int p = 0; p < 0x80; p++)
		s_cpu.rpage[p] = s_rom + p * 0x2000;
	s_cpu.rpage[0xf8] = s_cpu.wpage[0xf8] = s_ram;
	h6280_init(&s_snd);
	s_snd.write = snd_w;
	s_snd.port_in = snd_port_in;
	s_snd.port_out = snd_port_out;
	s_snd.psg_write = snd_psg;
	s_snd.irq_ack = snd_irq_ack;
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_srom + p * 0x2000;
	s_snd.rpage[0xf8] = s_snd.wpage[0xf8] = s_sram;
	c6280_init(&s_psg, MASTER / 3);
	msm5205u_init(&s_msm, 384000, MSM5205_S48_4B, adpcm_int, NULL);
	// YM2203 1.5 MHz 0.40, the PSG 0.60 (both sides), the MSM5205 0.85
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = 12000000 / 8;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 102;
	s_snap = (uint16_t(*)[VIS_W])calloc(VIS_H, sizeof(*s_snap));
	int rc = !s_snap || rbx_ym_init(0, &yc);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_out = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_out || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		br_exit();
		return -1;
	}
	static const char *const tags[5] = { "IN0", "IN1", "IN2", "DSW2", "DSW1" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(tags[i]);
	rbx_aux_set_job(snd_job);
	br_reset();
	return 0;
}

static void br_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	free(s_out);
	s_out = NULL;
	free(s_snap);
	s_snap = NULL;
}

// the main CPU to its clock c, then the VCE to h_end of the line
static void run_to(int64_t c, int h_end)
{
	h6280_run_until(&s_cpu, c);
	vce_run(h_end);
}

static void br_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LPF; line++) {   // (from the VCE's line 260)
		const uint64_t base = s_mclk;
		const int64_t q = (int64_t)(base / 3);   // (the CPU clock: the master clock / 3)
		const int r = (int)(base - (uint64_t)q * 3);
		s_line = line;
		for (int k = 0; k < 6; k++) {
			s_seg = k;
			run_to(q + (r + s_segs[k]) / 3, s_segs[k]);
		}
		s_mclk = base + WPF;
	}
	rbx_aux_sync();   // the previous frame's samples out, this frame's latch writes to the sound job
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	s_ev_cur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
	vblank_capture();
	s_frame++;
}

static int br_state(void *ctx, int load)
{
	uint8_t cpu[H6280_STATE_SIZE], snd[H6280_STATE_SIZE], psg[C6280_STATE_SIZE], msm[MSM5205_STATE_SIZE];
	rbx_aux_sync();   // (the sound job done: the sound board at the frame's end, its samples waiting)
	if (!load) {
		memcpy(cpu, &s_cpu, H6280_STATE_SIZE);
		memcpy(snd, &s_snd, H6280_STATE_SIZE);
		memcpy(psg, &s_psg, C6280_STATE_SIZE);
		memcpy(msm, &s_msm, MSM5205_STATE_SIZE);
	}
	uint64_t t[3] = { s_mclk, s_ym_frac, s_frame };
	uint8_t m[8] = { s_control_port_select, s_msm5205next, s_toggle, s_latch, s_latch_pending };
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", cpu, sizeof(cpu));
	rc |= rbx_state_io(ctx, "snd", snd, sizeof(snd));
	rc |= rbx_state_io(ctx, "psg", psg, sizeof(psg));
	rc |= rbx_state_io(ctx, "msm", msm, sizeof(msm));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "vdc", &s_v, sizeof(s_v));
	rc |= rbx_state_io(ctx, "bmp", s_bmp, sizeof(s_bmp));
	rc |= rbx_state_io(ctx, "t", t, sizeof(t));
	rc |= rbx_state_io(ctx, "m", m, sizeof(m));
	uint64_t sq[2] = { s_smclk, (uint64_t)s_out_n };
	rc |= rbx_state_io(ctx, "sndq", sq, sizeof(sq));
	rc |= rbx_state_io(ctx, "sndout", s_out, sizeof(int32_t) * (size_t)s_mix_cap);
	if (!rc && load) {
		memcpy(&s_cpu, cpu, H6280_STATE_SIZE);
		memcpy(&s_snd, snd, H6280_STATE_SIZE);
		memcpy(&s_psg, psg, C6280_STATE_SIZE);
		memcpy(&s_msm, msm, MSM5205_STATE_SIZE);
		s_mclk = t[0];
		s_ym_frac = t[1];
		s_frame = (uint32_t)t[2];
		s_control_port_select = m[0];
		s_msm5205next = m[1];
		s_toggle = m[2];
		s_latch = m[3];
		s_latch_pending = m[4];
		s_smclk = sq[0];
		s_out_n = sq[1] < (uint64_t)s_mix_cap ? (int)sq[1] : 0;
	}
	return rc ? -1 : 0;
}

static int br_save(void *ctx) { return br_state(ctx, 0); }
static int br_load(void *ctx) { return br_state(ctx, 1); }
static int br_supported(const char *machine) { return !strcmp(machine, "battlera"); }

const rbx_driver rbx_drv_battlera = { "DECO Battle Rangers", br_init, br_exit, br_reset, br_frame, br_save, br_load,
                                      br_supported, br_render, NULL };

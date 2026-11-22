/*  The Seibu COP bootleg (see seicopbl.h).
    Ported from MAME 0.289 src/mame/seibu/seicopbl.cpp (LGPL-2.1-or-later, used here under its GPL option;
    copyright-holders: Olivier Galibert, Angelo Salese, David Haywood, Tomasz Slanina).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "seicopbl.h"
#include "../../rbx.h"
#include "../../cpu/m68k/m68k.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define PI 3.14159265358979323846

static struct {
	uint16_t ram[0x100];          // the latches (the space's plain RAM areas), by word
	uint32_t reg[8];
	uint16_t status, dist, angle, prng_max, scale;
	int16_t pad;
	uint32_t d104_move_offset;
	int32_t dx, dy;
	uint8_t state_end;
} s;

// the host's big-endian accesses (MAME's m_host_space)
static uint16_t rw(uint32_t a) { return (uint16_t)m68k_read_memory_16(a & 0xffffff); }
static uint32_t rd(uint32_t a) { return m68k_read_memory_32(a & 0xffffff); }
static uint8_t rb(uint32_t a) { return (uint8_t)m68k_read_memory_8(a & 0xffffff); }
static void ww(uint32_t a, uint16_t v) { m68k_write_memory_16(a & 0xffffff, v); }
static void wd(uint32_t a, uint32_t v) { m68k_write_memory_32(a & 0xffffff, v); }
static void wb(uint32_t a, uint8_t v) { m68k_write_memory_8(a & 0xffffff, v); }

static uint64_t (*s_host_cycles)(void);

void seicopbl_init(uint64_t (*host_cycles)(void)) { s_host_cycles = host_cycles; }
void seicopbl_reset(void) { memset(&s, 0, offsetof(__typeof__(s), state_end)); }

static void cmd_trigger_w(uint32_t offset, uint16_t data)
{
	const uint32_t offs = (offset & 3) * 4;
	switch (data) {
	default: break;
	case 0xdde5: {
		int div = rw(s.reg[4] + offs);
		const int16_t dir_offset = (int16_t)rw(s.reg[4] + offs + 8);
		if (div == 0)
			div = 1;
		ww(s.reg[6] + offs + 4, (uint16_t)((rw(s.reg[5] + offs + 4) + dir_offset) / div));
		break;
	}
	case 0x0204:
	case 0x0205: {
		const int ppos = (int)rd(s.reg[0] + 4 + offs);
		const int npos = (int)((uint32_t)ppos + rd(s.reg[0] + 0x10 + offs));
		const int delta = (npos >> 16) - (ppos >> 16);
		wd(s.reg[0] + 4 + offs, (uint32_t)npos);
		ww(s.reg[0] + 0x1c + offs, (uint16_t)(rw(s.reg[0] + 0x1c + offs) + delta));
		break;
	}
	case 0x0905: {
		const uint32_t val = rd(s.reg[0] + 16 + offs) + rd(s.reg[0] + 0x28 + offs);
		wd(s.reg[0] + 16 + offs, val);
		break;
	}
	case 0x118e: case 0x130e: case 0x138e: case 0x330e: case 0xe30e: case 0xe18e: {
		const int target_reg = ((data & 0xf000) == 0xe000) ? 2 : 1;
		const int sy = (int)rd(s.reg[0] + 4) >> 16;
		const int sx = (int)rd(s.reg[0] + 8) >> 16;
		int dy = (int)rd(s.reg[target_reg] + 4) >> 16;
		int dx = (int)rd(s.reg[target_reg] + 8) >> 16;
		dy -= sy;
		dx -= sx;
		if (!dx) {
			s.status = 0x8000;
			s.angle = 0;
		} else {
			s.status = 0;
			s.angle = (uint16_t)(int)(atan((double)dy / (double)dx) * 128.0 / PI);
			if (dx < 0)
				s.angle = (uint16_t)(s.angle + 0x80);
		}
		s.dy = dy;
		s.dx = dx;
		if (data == 0x118e || data == 0xe18e)
			return;
		if (data & 0x80)
			wb(s.reg[0] + 0x37, (uint8_t)s.angle);
		break;
	}
	case 0x3bb0:
		s.dist = (uint16_t)(int)sqrt((double)(s.dx * s.dx + s.dy * s.dy));
		ww(s.reg[0] + 0x38, s.dist);
		break;
	case 0x42c2: {
		const int div = rw(s.reg[0] + 0x34);
		if (!div) {
			s.status |= 0x8000;
			ww(s.reg[0] + 0x38, 0);
			break;
		}
		ww(s.reg[0] + 0x38, (uint16_t)((s.dist << (5 - s.scale)) / div));
		break;
	}
	case 0x5105: {
		const uint32_t val = rd(s.reg[0]) + rw(s.reg[0] + 8);
		wd(s.reg[0] + 4, val);
		break;
	}
	case 0x5905: {
		const int val = (int)rw(s.reg[2] + 0x10 + offs) - (int)rd(s.reg[0] + 8 + offs);
		wd(s.reg[1] + 4 + offs, (uint32_t)val);
		break;
	}
	case 0x8100: {
		(void)rb(s.reg[0] + 0x35);
		const int raw_angle = rw(s.reg[0] + (0x34 ^ 2)) & 0xff;
		const double angle = raw_angle * PI / 128;
		double amp = (65536 >> 5) * (rw(s.reg[0] + (0x36 ^ 2)) & 0xff);
		if (raw_angle == 0xc0)
			amp *= 2;
		const int res = (int)(amp * sin(angle)) << s.scale;
		wd(s.reg[0] + 0x10, (uint32_t)res);
		break;
	}
	case 0x8900: {
		const int raw_angle = rw(s.reg[0] + (0x34 ^ 2)) & 0xff;
		const double angle = raw_angle * PI / 128;
		double amp = (65536 >> 5) * (rw(s.reg[0] + (0x36 ^ 2)) & 0xff);
		if (raw_angle == 0x80)
			amp *= 2;
		const int res = (int)(amp * cos(angle)) << s.scale;
		wd(s.reg[0] + 20, (uint32_t)res);
		break;
	}
	case 0xd104: {
		const uint32_t val = rd(s.reg[2]) + rw(s.reg[3] + 8);
		wd(s.reg[1] + 4, val);
		break;
	}
	}
}

uint16_t seicopbl_read(uint32_t off)
{
	off &= 0x1fe;
	if (off == 0x02c) return s.prng_max;
	if (off == 0x044) return s.scale;
	if (off == 0x046 || off == 0x048) return (uint16_t)(s.d104_move_offset >> ((off - 0x046) / 2 * 16));
	if (off >= 0x0a0 && off < 0x0b0) return (uint16_t)(s.reg[(off - 0x0a0) >> 1] >> 16);
	if (off >= 0x0c0 && off < 0x0d0) return (uint16_t)s.reg[(off - 0x0c0) >> 1];
	if (off >= 0x1a0 && off < 0x1a8)   // prng_r: the host's cycles
		return (uint16_t)((s_host_cycles ? s_host_cycles() : 0) % ((uint64_t)s.prng_max + 1));
	if (off == 0x1b0) return s.status;
	if (off == 0x1b2) return s.dist;
	if (off == 0x1b4) return s.angle;
	return s.ram[off >> 1];
}

void seicopbl_write(uint32_t off, uint16_t data, uint16_t mem_mask)
{
	off &= 0x1fe;
	if (off == 0x02c) { s.prng_max = (uint16_t)((s.prng_max & ~mem_mask) | (data & mem_mask)); return; }
	if (off == 0x044) { s.scale = (uint16_t)(((s.scale & ~mem_mask) | (data & mem_mask)) & 3); return; }
	if (off == 0x046 || off == 0x048) {   // d104_move_w (offset 1: the low word)
		if (off == 0x048)
			s.d104_move_offset = (s.d104_move_offset & 0xffff0000u) | data;
		else
			s.d104_move_offset = (s.d104_move_offset & 0xffff) | (uint32_t)data << 16;
		return;
	}
	if (off >= 0x0a0 && off < 0x0b0) {
		uint32_t *r = &s.reg[(off - 0x0a0) >> 1];
		*r = (*r & 0xffff) | (uint32_t)data << 16;
		return;
	}
	if (off >= 0x0c0 && off < 0x0d0) {
		uint32_t *r = &s.reg[(off - 0x0c0) >> 1];
		*r = (*r & 0xffff0000u) | data;
		return;
	}
	if (off >= 0x100 && off < 0x106) {
		cmd_trigger_w((off - 0x100) >> 1, data);
		return;
	}
	s.ram[off >> 1] = (uint16_t)((s.ram[off >> 1] & ~mem_mask) | (data & mem_mask));
}

int seicopbl_state(void *ctx, int load)
{
	(void)load;
	return rbx_state_io(ctx, "seicopbl", &s, (uint32_t)offsetof(__typeof__(s), state_end));
}

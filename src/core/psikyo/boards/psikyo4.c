/*  Psikyo PS4 (SH-2, two screens: Taisen Hot Gimmick 1-4 / Integral, Lode Runner - The Dig Fight, Quiz de Idol!
    Hot Debut). Ported from MAME 0.289 src/mame/psikyo/psikyo4.cpp, psikyo4.h, psikyo4_v.cpp (BSD-3-Clause,
    copyright-holders: David Haywood, Paul Priest).
    An SH-2 (SH7604) at 28.6 MHz, the PS6807 sprite chip (one list for both screens: 8 bpp 16x16 tiles, no zoom), a
    palette of 2048 RGB 888 entries with a clear colour and a brightness for each screen, a YMF278B whose wave ROM is
    banked in four 1 MB windows, a 93C56 EEPROM. Both 320x224 screens (Lode Runner, Hot Debut: 320x240) side by side,
    60 Hz; the YMF278B's outputs to one speaker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/sh2/sh2.h"
#include "../../cpu/sh2/sh2idle.h"
#include "../../sound/rbx_ym.h"
#include "../../machine/eepromser.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK (57272700 / 2)
#define LINES 256
#define SCR_W 320
#define PENS 0x900                 // pens of a screen: the palette, its clear colour (0x800), black past it

// ---- ROMs, configuration ----
static uint8_t *s_rom, *s_gfx, *s_ymf;
static uint32_t s_rom_size, s_gfx_size, s_ymf_size, s_n8, s_gfx_banks, s_ymf_banks;
static int s_vis_h;
static int s_port_p12, s_port_p34, s_port_jp4, s_port_sys, s_port_key[2][4];
static sh2_t s_cpu;
static sh2_idle_t s_idle;   // (cpu/sh2/sh2idle.c: the vblank waits)
static int s_noidle;
static eepser_t s_eeprom;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct p4_state {   // (on the heap while the game runs, ADR-040)
	uint32_t ram[0x100000 / 4];
	uint32_t spr[0x4000 / 4];      // sprite RAM (0x3800 bytes) and the registers' page
	uint32_t pal[0x2000 / 4];
	uint32_t vidregs[3];
	uint32_t bgpen[2];
	int32_t brt[2];                // MAME's contrast as (0x7f - brightness), -1 = 1.0 (not written)
	uint32_t ymf_bank[4];          // the windows' offsets in the wave ROM
	int32_t gfxbank;
	uint8_t io_select;
	uint64_t ym_frac, cpu_frac;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

static struct {
	uint32_t spr[0x3800 / 4];
	uint32_t vidregs[3];
	uint32_t pens[2 * PENS];
} *s_snap;

// ---- memory ----
static void gfxbank_map(void)
{
	const uint32_t bank = (uint32_t)b.gfxbank < s_gfx_banks ? (uint32_t)b.gfxbank : 0;
	sh2_map(&s_cpu, 0x03006000, 0x03007fff, s_gfx + bank * 0x2000, 0x2000, 1, 0);
}

static uint8_t mahjong_ctrl(int p)
{
	int ret = 0x3f;
	for (int i = 0; i < 4; i++)
		if ((b.io_select >> i) & 1 && s_port_key[p][i] >= 0)
			ret &= (int)rbx_port_read(s_port_key[p][i]);
	return (uint8_t)(ret | 0xc0);
}

static uint32_t players_r(int p)
{
	const int port = p ? s_port_p34 : s_port_p12;
	if (s_port_sys < 0)   // (Lode Runner, Hot Debut: plain ports)
		return port >= 0 ? rbx_port_read(port) : 0xffffffff;
	// (the Hot Gimmick games: the system inputs, the mahjong keys of the selected rows)
	return (rbx_port_read(s_port_sys) & 0xff) | 0x00ffff00 | (uint32_t)mahjong_ctrl(p) << 24;
}

static uint32_t rd32(uint32_t a)
{
	if (a - 0x03003000 < 0x800)
		return b.spr[(a & 0x3fff) >> 2];
	if (a == 0x03003fe0) {
		if (s_port_jp4 < 0)
			return 0;
		rbx_port_custom(s_port_jp4, 0x00100000, eepser_do_read_93cxx(&s_eeprom, sh2_total_cycles(&s_cpu)) ? 0x00100000 : 0);
		return rbx_port_read(s_port_jp4);
	}
	if (a == 0x03003fe8 || a == 0x03003fec)
		return b.vidregs[(a - 0x03003fe4) >> 2];
	if (a == 0x05800000 || a == 0x05800004)
		return players_r((int)((a >> 2) & 1));
	return 0;
}

static uint8_t r8(void *u, uint32_t a)
{
	(void)u;
	if (a - 0x05000000 < 8)
		return rbx_ym_read(0, a & 7);
	return (uint8_t)(rd32(a & ~3u) >> (24 - 8 * (a & 3)));
}
static uint16_t r16(void *u, uint32_t a)
{
	if (a - 0x05000000 < 8)
		return (uint16_t)(r8(u, a) << 8 | r8(u, a + 1));
	return (uint16_t)(rd32(a & ~3u) >> (16 - 8 * (a & 2)));
}
static uint32_t r32(void *u, uint32_t a)
{
	if (a - 0x05000000 < 8)
		return (uint32_t)r16(u, a) << 16 | r16(u, a + 2);
	return rd32(a);
}

static void wr32(uint32_t a, uint32_t data, uint32_t mask)
{
	a &= ~3u;
	if (a - 0x03003000 < 0x800) {
		uint32_t *w = &b.spr[(a & 0x3fff) >> 2];
		*w = (*w & ~mask) | (data & mask);
		return;
	}
	if (a - 0x05000000 < 8) {
		for (int i = 0; i < 4; i++)
			if (mask & (0xffu << (24 - 8 * i)))
				rbx_ym_write(0, (a + (uint32_t)i) & 7, (uint8_t)(data >> (24 - 8 * i)));
		return;
	}
	switch (a) {
	case 0x03003fe0:   // (the EEPROM: byte 0x03003fe1)
		if (mask & 0x00ff0000) {
			const uint8_t d = (uint8_t)(data >> 16);
			const int64_t now = sh2_total_cycles(&s_cpu);
			eepser_di_write(&s_eeprom, (d >> 5) & 1);
			eepser_cs_write(&s_eeprom, (d >> 7) & 1, now);
			eepser_clk_write(&s_eeprom, (d >> 6) & 1, now);
		}
		break;
	case 0x03003fe4: case 0x03003fe8: case 0x03003fec: {
		const int off = (int)((a - 0x03003fe4) >> 2);
		const uint32_t old = b.vidregs[off];
		b.vidregs[off] = (old & ~mask) | (data & mask);
		if (off == 2 && ((old ^ b.vidregs[off]) & 0x1fff) && (b.vidregs[off] & 0x1fff) < s_gfx_banks) {
			b.gfxbank = (int32_t)(b.vidregs[off] & 0x1fff);
			gfxbank_map();
		}
		break;
	}
	case 0x03003ff0: case 0x03003ff8:   // (the brightness: bytes 0x03003ff3, 0x03003ffb)
		if (mask & 0xff) {
			int brt = (int)(data & 0xff);
			if (brt > 0x7f)
				brt = 0x7f;
			b.brt[a == 0x03003ff8] = 0x7f - brt;
		}
		break;
	case 0x03003ff4: case 0x03003ffc: {
		uint32_t *w = &b.bgpen[a == 0x03003ffc];
		*w = (*w & ~mask) | (data & mask);
		break;
	}
	case 0x05800008:
		if (mask & 0xffff0000) {   // ymf_bank_w (a 16-bit write: the nibbles of its mask)
			uint32_t bankdata = data >> 16, bankmask = mask >> 16;
			for (int i = 0; i < 4; i++) {
				if (bankmask & 0x0f) {
					const uint32_t n = bankdata & 0x0f;
					if (n < s_ymf_banks)
						b.ymf_bank[i] = n * 0x100000;
				}
				bankdata >>= 4;
				bankmask >>= 4;
			}
		}
		if (mask & 0x0000ff00)
			b.io_select = (uint8_t)(data >> 8);
		break;
	}
}
static void w8(void *u, uint32_t a, uint8_t d)
{
	(void)u;
	const int sh = 24 - 8 * (int)(a & 3);
	wr32(a, (uint32_t)d << sh, 0xffu << sh);
}
static void w16(void *u, uint32_t a, uint16_t d)
{
	(void)u;
	const int sh = 16 - 8 * (int)(a & 2);
	wr32(a, (uint32_t)d << sh, 0xffffu << sh);
}
static void w32(void *u, uint32_t a, uint32_t d)
{
	(void)u;
	wr32(a, d, 0xffffffffu);
}

static void map_mem(void)
{
	sh2_map(&s_cpu, 0x00000000, 0x000fffff, s_rom, s_rom_size < 0x100000 ? s_rom_size : 0x100000, 1, 0);
	if (s_rom_size > 0x100000)
		sh2_map(&s_cpu, 0x02000000, 0x021fffff, s_rom + 0x100000, s_rom_size - 0x100000, 1, 0);
	sh2_map(&s_cpu, 0x03000000, 0x03002fff, b.spr, 0x3000, 1, 1);
	sh2_map(&s_cpu, 0x03004000, 0x03005fff, b.pal, sizeof(b.pal), 1, 1);
	sh2_map(&s_cpu, 0x06000000, 0x060fffff, b.ram, sizeof(b.ram), 1, 1);
	gfxbank_map();
}

static void ym_irq(int state) { sh2_set_irq_line(&s_cpu, 12, state ? SH2_ASSERT : SH2_CLEAR); }

// ---- video ----
static void draw_screen(int scr, uint16_t *dest, int pitch)
{
	const uint16_t base = (uint16_t)(scr * PENS);
	for (int y = 0; y < s_vis_h; y++)
		for (int x = 0; x < SCR_W; x++)
			dest[y * pitch + x] = (uint16_t)(base + 0x800);
	const uint32_t *src = s_snap->spr;
	const int flipscreen = (s_snap->vidregs[1] >> (scr ? 23 : 31)) & 1;
	for (int n = 0; n < 0xc00 / 2 - 2; n++) {
		const uint32_t w = src[(0x2c04 / 4) + n / 2];
		const uint16_t listdat = (uint16_t)((n & 1) ? w : w >> 16);
		const uint32_t sprnum = (uint32_t)(listdat & 0x3ff) * 2;
		if (!(listdat & 0x8000) && ((listdat >> 13) & 1) == scr) {
			const uint32_t s0 = src[sprnum], s1 = src[sprnum + 1];
			int ypos = (int)((s0 >> 16) & 0x3ff), xpos = (int)(s0 & 0x3ff);
			const int high = (int)(s0 >> 28) + 1, wide = (int)((s0 >> 12) & 15) + 1;
			const uint32_t tnum = s1 & 0x7ffff;
			const uint32_t colr = (s1 >> 24) & 0x3f;
			int flipx = (s1 >> 30) & 1, flipy = (int)(s1 >> 31);
			if (ypos & 0x200) ypos -= 0x400;
			if (xpos & 0x200) xpos -= 0x400;
			if (flipscreen) {
				ypos = s_vis_h - ypos - high * 16;
				xpos = 40 * 8 - xpos - wide * 16;
				flipx = !flipx;
				flipy = !flipy;
			}
			int loopnum = 0;
			for (int j = flipy ? high - 1 : 0; flipy ? j >= 0 : j < high; j += flipy ? -1 : 1)
				for (int i = flipx ? wide - 1 : 0; flipx ? i >= 0 : i < wide; i += flipx ? -1 : 1) {
					const uint8_t *t = s_gfx + rbx_wrap(tnum + (uint32_t)loopnum++, s_n8) * 256;
					const int sx = xpos + 16 * i, sy = ypos + 16 * j;
					for (int py = 0; py < 16; py++) {
						const int yy = sy + py;
						if (yy < 0 || yy >= s_vis_h)
							continue;
						const uint8_t *row = t + (flipy ? 15 - py : py) * 16;
						uint16_t *d = dest + yy * pitch;
						for (int px = 0; px < 16; px++) {
							const int xx = sx + px;
							const uint8_t v = row[flipx ? 15 - px : px];
							if (v && xx >= 0 && xx < SCR_W)
								d[xx] = (uint16_t)(base + colr * 32 + v);
						}
					}
				}
		}
		if (listdat & 0x4000)
			break;
	}
}

static void p4_render(void)
{
	memcpy(rbx_video.palette, s_snap->pens, sizeof(s_snap->pens));
	for (int scr = 0; scr < 2; scr++)
		draw_screen(scr, rbx_video.bitmap + scr * SCR_W, rbx_video.pitch);
}

static uint32_t pen(uint32_t v, int brt)
{
	int r = (int)(v >> 24), g = (int)((v >> 16) & 0xff), bl = (int)((v >> 8) & 0xff);
	if (brt >= 0) {   // (MAME set_pen_contrast: the colour times the contrast, truncated)
		const float c = (float)((double)brt / 127.0);
		r = (int)((float)r * c);
		g = (int)((float)g * c);
		bl = (int)((float)bl * c);
	}
	return (uint32_t)(r << 16 | g << 8 | bl);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, b.spr, sizeof(s_snap->spr));
	memcpy(s_snap->vidregs, b.vidregs, sizeof(b.vidregs));
	for (int s = 0; s < 2; s++) {
		for (int i = 0; i < 0x800; i++)
			s_snap->pens[s * PENS + i] = pen(b.pal[i], b.brt[s]);
		s_snap->pens[s * PENS + 0x800] = pen(b.bgpen[s], -1);
	}
	rbx_video_frame_done();
}

// ---- frame ----
static void p4_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		if (line == s_vis_h) {
			vblank_capture();
			sh2_set_irq_line(&s_cpu, 4, SH2_HOLD);   // (the vblank interrupt)
		}
		sh2_execute(&s_cpu, rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, 60000, LINES));
		if (!(line & 15))
			sh2_idle_learn(&s_idle, &s_cpu);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, CPU_CLOCK, 60000, LINES);
		s_mix_n += rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void p4_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, sizeof(b));
	b.brt[0] = b.brt[1] = -1;
	for (int i = 0; i < 4; i++)
		b.ymf_bank[i] = (uint32_t)i < s_ymf_banks ? (uint32_t)i * 0x100000 : 0;
	map_mem();
	sh2_reset(&s_cpu);
	sh2_idle_reset(&s_idle, &s_cpu, s_noidle);
	rbx_ym_reset(0);
	s_mix_n = 0;
}

static int p4_supported(const char *machine) { return !strcmp(machine, "ps4big") || !strcmp(machine, "ps4small"); }

static void p4_exit(void)
{
	rbx_video_wait_idle();
	eepser_nv_save(&s_eeprom);
	rbx_ym_exit(0);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
}

static int p4_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = (struct p4_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!p4_supported(g->machine)) {
		rbx_fail("%s: machine %s not emulated", g->set, g->machine);
		return -1;
	}
	s_vis_h = !strcmp(g->machine, "ps4small") ? 240 : 224;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_gfx = rbx_region("gfx1", &s_gfx_size);
	s_ymf = rbx_region("ymf", &s_ymf_size);
	if (!s_rom || !s_gfx || !s_ymf) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	sh2_swap32(s_rom, s_rom_size & ~3u);   // (MAME's region of a big-endian CPU in host-order words)
	s_n8 = s_gfx_size / 256;
	s_gfx_banks = s_gfx_size / 0x2000;
	s_ymf_banks = s_ymf_size / 0x100000;
	s_noidle = getenv("RBX_PSH_NOIDLE") != NULL;
	s_port_p12 = rbx_port_find("P1_P2");
	s_port_p34 = rbx_port_find("P3_P4");
	s_port_jp4 = rbx_port_find("JP4");
	s_port_sys = rbx_port_find("SYSTEM");
	for (int p = 0; p < 2; p++)
		for (int i = 0; i < 4; i++) {
			char tag[8] = "KEY0";
			tag[3] = (char)('0' + p * 5 + i);
			s_port_key[p][i] = rbx_port_find(tag);
		}
	eepser_init(&s_eeprom, EEPSER_93CXX, 256, 8, CPU_CLOCK);
	s_eeprom.cmd_address_bits = 9;   // (93C56 8-bit: MAME's 9 address bits of a command)
	for (int i = 0; i < 256; i++)
		s_eeprom.data[i] = 0;        // (default_value(0))
	uint32_t esz = 0;
	const uint8_t *e = rbx_region("eeprom", &esz);
	if (e)
		eepser_default(&s_eeprom, e, esz);
	eepser_nv_load(&s_eeprom, g->set);
	sh2_init(&s_cpu);
	s_cpu.read8 = r8;
	s_cpu.read16 = r16;
	s_cpu.read32 = r32;
	s_cpu.write8 = w8;
	s_cpu.write16 = w16;
	s_cpu.write32 = w32;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YMF278B;
	yc.clock = CPU_CLOCK;
	yc.adpcma = s_ymf;
	yc.adpcma_size = s_ymf_size;
	yc.pcm_bank = b.ymf_bank;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_snap || rbx_ym_init(0, &yc)) {
		rbx_fail("%s: out of memory", g->set);
		p4_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(2 * SCR_W, s_vis_h, 2 * PENS)) {
		rbx_fail("%s: out of memory", g->set);
		p4_exit();
		return -1;
	}
	p4_reset();
	return 0;
}

static int p4_state(void *ctx, int load)
{
	if (load)
		rbx_video_wait_idle();
	int rc = 0;
	rc |= rbx_state_io(ctx, "sh2", &s_cpu, SH2_STATE_SIZE);
	rc |= rbx_state_io(ctx, "psikyo4", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= eepser_state(&s_eeprom, ctx, load, "eeprom");
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		sh2_post_load(&s_cpu);
		gfxbank_map();
	}
	return rc ? -1 : 0;
}

static int p4_save(void *ctx) { return p4_state(ctx, 0); }
static int p4_load(void *ctx) { return p4_state(ctx, 1); }

const rbx_driver rbx_drv_psikyo4 = { "Psikyo PS4", p4_init, p4_exit, p4_reset, p4_frame, p4_save, p4_load, p4_supported,
                                     p4_render, NULL };

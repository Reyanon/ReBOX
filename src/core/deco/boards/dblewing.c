/*  Mitchell Double Wings. Ported from MAME src/mame/dataeast/dblewing.cpp (BSD-3-Clause, copyright-holders:
    David Haywood, Bryan McPhail).
    DE102 (encrypted 68000) at 14 MHz (IRQ 6 held at vblank), the tilemap chip 141 (graphics encrypted as the 56's),
    sprite chip 52 (always on top), the 104 (inputs and the sound latch). The sound: Z80 at 3.58 MHz with YM2151 and an
    OKI, its INT from the latch or the YM; its I/O space reads the ROM.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../machine/deco102.h"
#include "../machine/deco146.h"
#include "../machine/decocrpt.h"
#include "../video/deco16ic.h"
#include "../video/decospr.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 14000000       // 28 MHz / 2
#define Z80_CLOCK 3580000        // 32.22 MHz / 9 (the YM2151 too)
#define YM_CLOCK (32220000 / 9)
#define OKI_CLOCK 1000000        // 28 MHz / 28, pin 7 high
#define FRAME_MHZ 58443          // set_refresh_hz(58.443)
#define LINES 256                // set_size(64*8, 32*8)
#define VIS_Y0 8                 // set_visarea(0, 319, 8, 247)
#define VIS_H 240
#define VIS_W 320
#define VBL_LINES 37             // vblank 2500 us (into the next frame's first lines)

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint16_t *s_opcodes;
static uint16_t s_ram[0x2000];             // 0xff0000 .. 0xff3fff (mirrored to 0xffffff)
static uint16_t s_misc[0x10];              // 0x284000, 0x288000, 0x28c000 .. 0x28c00f
static uint16_t s_spriteram[0x400];
static uint16_t s_pal[0x400];              // xBGR_444
static uint16_t s_rowscroll[2][0x800];
static deco16ic_t s_tile;
static decospr_t s_spr;
static deco146_t s_prot;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static int s_port_in, s_port_sys, s_port_dsw;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_z80_frac, s_ym_frac;
static uint8_t s_pri_buf[VIS_W * VIS_H];
static struct {
	deco16ic_t tile;
	uint16_t rowscroll[2][0x800], spriteram[0x400], pal[0x400];
	uint32_t frame;
} *s_snap;

// ---- sound: Z80, YM2151, OKI ----
static z80 s_z80;
static uint8_t s_zram[0x800];              // 0x8000 .. 0x87ff
static okim6295_t s_oki;
static int s_ym_irq, s_latch_irq, s_irq;   // (soundirq: any of the two)
static uint32_t s_oki_acc;
static int32_t s_oki_hold;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static const uint8_t *s_zmap[256];

static void ym_irq(int state) { s_ym_irq = state; s_irq = s_ym_irq | s_latch_irq; }
static void latch_irq(void *u, int state) { (void)u; s_latch_irq = state; s_irq = s_ym_irq | s_latch_irq; }

static uint8_t z80_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800) return s_zram[a - 0x8000];
	if (a == 0xa000 || a == 0xa001) return rbx_ym_read(0, 1);   // status_r
	if (a == 0xb000 || a == 0xf000) return oki_read(&s_oki);
	if (a == 0xc000) return deco146_soundlatch_r(&s_prot);
	if (a == 0xd000) return s_prot.soundlatch_pending ? 0 : 1;   // irq_latch_r: bit 0 the IRQ's kind (0: latch, 1: YM)
	return 0;
}
static void z80_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) s_zram[a - 0x8000] = v;
	else if (a == 0xa000 || a == 0xa001) rbx_ym_write(0, a & 1, v);
	else if (a == 0xb000 || a == 0xf000) oki_write(&s_oki, v);
}
static uint8_t z80_in(z80 *z, uint8_t port)   // sound_io: the ROM at the 16-bit port address
{
	(void)port;
	return z->mem_ptr < s_zrom_size ? s_zrom[z->mem_ptr] : 0;
}
static void z80_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

static void z80_start(void)
{
	z80_init(&s_z80);
	s_z80.read_byte = z80_read;
	s_z80.write_byte = z80_write;
	s_z80.port_in = z80_in;
	s_z80.port_out = z80_out;
	s_z80.read_map = s_zmap;
}

// the Z80 and the chips through one line: the YM's samples (on the worker) and the OKI held at the YM's rate
static void sound_line(void)
{
	const int cycles = rbx_line_cycles(&s_z80_frac, Z80_CLOCK, FRAME_MHZ, LINES);
	z80_run(&s_z80, s_z80.cyc + (unsigned long)cycles, &s_irq, 0);   // (INT: level)
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		s_oki_acc += s_oki.rate;
		while (s_oki_acc >= yr) {
			s_oki_acc -= yr;
			s_oki_hold = oki_sample(&s_oki);
		}
		s_mix[s_mix_n + i] += s_oki_hold;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- the 104's ports ----
static int vblank(void) { return ((s_line - (VIS_Y0 + VIS_H) + LINES) % LINES) < VBL_LINES; }
static uint16_t port_a(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_in); }
static uint16_t port_b(void *u)
{
	(void)u;
	rbx_port_custom(s_port_sys, 0x0008, vblank() ? 0x0008 : 0);
	return (uint16_t)rbx_port_read(s_port_sys);
}
static uint16_t port_c(void *u) { (void)u; return (uint16_t)rbx_port_read(s_port_dsw); }

// ---- bus ----
static uint16_t main_read16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x280000 && a < 0x284000) return deco146_read(&s_prot, (uint16_t)(a & 0x7fe), NULL);   // ioprot_r
	if (a >= 0x284000 && a < 0x284002) return s_misc[0];
	if (a >= 0x288000 && a < 0x288002) return s_misc[1];
	if (a >= 0x28c000 && a < 0x28c010) return s_misc[8 + ((a >> 1) & 7)];
	return 0;
}
static uint8_t main_read8(uint32_t a)
{
	const uint16_t v = main_read16(a & ~1u);
	return (uint8_t)(a & 1 ? v : v >> 8);
}
static void main_write16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x280000 && a < 0x284000)
		deco146_write(&s_prot, (uint16_t)(a & 0x7fe), v, mask, NULL);
	else if (a >= 0x284000 && a < 0x284002)
		s_misc[0] = (uint16_t)((s_misc[0] & ~mask) | (v & mask));
	else if (a >= 0x288000 && a < 0x288002)
		s_misc[1] = (uint16_t)((s_misc[1] & ~mask) | (v & mask));
	else if (a >= 0x28c000 && a < 0x28c010) {
		uint16_t *w = &s_misc[8 + ((a >> 1) & 7)];
		*w = (uint16_t)((*w & ~mask) | (v & mask));
		deco16ic_control_w(&s_tile, (int)((a >> 1) & 7), v, mask);
	}
}
static void main_write16(uint32_t a, uint16_t v) { main_write16m(a, v, 0xffff); }
static void main_write8(uint32_t a, uint8_t v)
{
	const int sh = a & 1 ? 0 : 8;
	main_write16m(a & ~1u, (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}

// ---- video ----
static int bank_cb(int bank) { return (bank & 0x70) << 8; }
static uint32_t pri_cb(uint16_t x, int extpri) { (void)x; (void)extpri; return 0; }   // (sprites always on top)

static void dw_render(void)
{
	for (int i = 0; i < 0x400; i++) {   // xBGR_444
		const uint16_t p = s_snap->pal[i];
		rbx_video.palette[i] = (uint32_t)(((p & 15) * 17) << 16 | (((p >> 4) & 15) * 17) << 8 | (((p >> 8) & 15) * 17));
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	deco16ic_t *t = &s_snap->tile;
	deco16ic_update(t);
	decospr_t spr = s_spr;
	spr.flip_screen = (deco16ic_control_r(t, 0) >> 7) & 1;
	for (int y = 0; y < VIS_H; y++)   // (bitmap.fill(0): "not confirmed")
		memset(bm + y * pitch, 0, VIS_W * 2);
	memset(s_pri_buf, 0, sizeof(s_pri_buf));
	deco16ic_draw(t, 1, s_snap->rowscroll[1], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 2, 0xff);
	deco16ic_draw(t, 0, s_snap->rowscroll[0], bm, s_pri_buf, pitch, VIS_W, VIS_Y0, 0, VIS_H, 0, 4, 0xff);
	decospr_draw(&spr, s_snap->spriteram, 0x400, bm, s_pri_buf, pitch, VIS_W, VIS_H, VIS_Y0, 1, s_snap->frame);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->rowscroll, s_rowscroll, sizeof(s_rowscroll));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	memcpy(s_snap->pal, s_pal, sizeof(s_pal));
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void dw_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_misc, 0, sizeof(s_misc));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_pal, 0, sizeof(s_pal));
	memset(s_rowscroll, 0, sizeof(s_rowscroll));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	deco16ic_reset(&s_tile);
	s_ym_irq = s_latch_irq = s_irq = 0;
	deco146_reset(&s_prot);
	memset(s_zram, 0, sizeof(s_zram));
	z80_start();
	rbx_ym_reset(0);
	oki_reset(&s_oki);
	s_oki_acc = 0;
	s_oki_hold = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_z80_frac = s_ym_frac = 0;
}

static void dw_exit(void);

static int dw_init(const rbx_game *g)
{
	uint32_t tsize, ssize, osize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize), *oki = rbx_region("oki", &osize);
	if (!s_rom || s_rom_size < 0x80000 || !s_zrom || !tiles || !spr || !oki) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	if (!(s_opcodes = (uint16_t *)malloc(0x80000)) || deco56_decrypt_gfx(tiles, tsize) ||
	    deco102_decrypt_cpu((uint16_t *)s_rom, s_opcodes, 0x80000, 0x399d, 0x25, 0x3d)) {
		rbx_fail("%s: out of memory", g->set);
		dw_exit();
		return -1;
	}
	{	// tile_8x8_layout / tile_16x16_layout (RGN_FRAC(1,2))
		static uint32_t x8[8], y8[8], x16[16], y16[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 16);
			x16[i] = (uint32_t)(256 + i);
			x16[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 16);
		const uint32_t th = tsize * 4, sh = ssize * 4;
		const uint32_t pt[4] = { th + 8, th, 8, 0 }, ps[4] = { sh + 8, sh, 8, 0 };
		const rbx_gfx_layout l8 = { 8, 8, 4, pt, x8, y8, 16 * 8 };
		const rbx_gfx_layout l16 = { 16, 16, 4, pt, x16, y16, 32 * 16 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, x16, y16, 32 * 16 };
		if (rbx_gfx_decode(&s_gfx8, &l8, tiles, tsize / 2) || rbx_gfx_decode(&s_gfx16, &l16, tiles, tsize / 2) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			dw_exit();
			return -1;
		}
	}
	memset(&s_tile, 0, sizeof(s_tile));
	for (int i = 0; i < 2; i++) {
		s_tile.tm[i].size = DECO_64x32;
		s_tile.tm[i].col_bank = i ? 0x10 : 0x00;
		s_tile.tm[i].col_mask = 0x0f;
		s_tile.tm[i].bank_cb = bank_cb;
	}
	s_tile.gfx8 = &s_gfx8;
	s_tile.gfx16 = &s_gfx16;
	memset(&s_spr, 0, sizeof(s_spr));
	s_spr.gfx = &s_gfxspr;
	s_spr.color_base = 0x200;
	s_spr.pri_cb = pri_cb;
	deco146_init(&s_prot, 1);
	static const uint8_t interleave[10] = { 4, 5, 3, 6, 2, 7, 1, 8, 0, 9 };   // set_interface_scramble_interleave
	deco146_set_scramble(&s_prot, interleave);
	s_prot.magic_read_xor_on = 1;
	s_prot.port_a = port_a;
	s_prot.port_b = port_b;
	s_prot.port_c = port_c;
	s_prot.soundlatch_irq = latch_irq;
	s_snap = calloc(1, sizeof(*s_snap));
	// sound: YM2151 3.58 MHz 1.0 (both outputs), OKI 1 MHz 1.0, one speaker
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 256;
	yc.irq = ym_irq;
	if (!s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		dw_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 0x400)) {
		rbx_fail("%s: out of memory", g->set);
		dw_exit();
		return -1;
	}
	oki_init(&s_oki, OKI_CLOCK, 1, oki, osize, 256);
	memset(s_zmap, 0, sizeof(s_zmap));
	for (uint32_t p = 0; p < 0x80 && p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	s_port_in = rbx_port_find("INPUTS");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_bus_reset_map();
	m68k_bus_map(0, 0x7ffff, s_rom, 0x80000, 1, 0);
	m68k_bus.op = (const uint8_t *)s_opcodes;
	m68k_bus.op_size = 0x80000;
	m68k_bus_map(0x100000, 0x100fff, s_tile.vram[0], 0x1000, 1, 1);
	m68k_bus_map(0x102000, 0x102fff, s_tile.vram[1], 0x1000, 1, 1);
	m68k_bus_map(0x104000, 0x104fff, s_rowscroll[0], sizeof(s_rowscroll[0]), 1, 1);
	m68k_bus_map(0x106000, 0x106fff, s_rowscroll[1], sizeof(s_rowscroll[1]), 1, 1);
	m68k_bus_map(0x300000, 0x3007ff, s_spriteram, sizeof(s_spriteram), 1, 1);
	m68k_bus_map(0x320000, 0x3207ff, s_pal, sizeof(s_pal), 1, 1);
	for (uint32_t m = 0; m < 4; m++)   // (mirror 0xc000)
		m68k_bus_map(0xff0000 + m * 0x4000, 0xff3fff + m * 0x4000, s_ram, sizeof(s_ram), 1, 1);
	m68k_bus.read8 = main_read8;
	m68k_bus.read16 = main_read16;
	m68k_bus.write8 = main_write8;
	m68k_bus.write16 = main_write16;
	dw_reset();
	return 0;
}

static void dw_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	m68k_bus.op = NULL;
	m68k_bus.op_size = 0;
	free(s_opcodes);
	s_opcodes = NULL;
}

static void dw_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // irq6_line_hold at vblank
			vblank_capture();
			m68k_bus_hold_irq(6);
		}
		if (line >= VIS_Y0 && line < VIS_Y0 + VIS_H)
			deco16ic_latch_line(&s_tile, line);
		m68k_execute(rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	sound_frame_end();
	s_frame++;
}

static int dw_state(void *ctx, int load)
{
	const uint32_t csize = m68k_context_size();
	uint8_t *cpu = (uint8_t *)malloc(csize);
	if (!cpu)
		return -1;
	if (!load)
		m68k_get_context(cpu);
	uint32_t irq[2] = { m68k_bus_irq_pending(), m68k_bus_cur->irq_asserted };
	uint64_t fr[3] = { s_cpu_frac, s_z80_frac, s_ym_frac };
	int32_t misc[6] = { (int32_t)s_frame, s_ym_irq, s_latch_irq, (int32_t)s_oki_acc, s_oki_hold, 0 };
	z80 z = s_z80;
	int rc = 0;
	rc |= rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "mram", s_misc, sizeof(s_misc));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "pal", s_pal, sizeof(s_pal));
	rc |= rbx_state_io(ctx, "rows", s_rowscroll, sizeof(s_rowscroll));
	rc |= rbx_state_io(ctx, "vram", s_tile.vram, sizeof(s_tile.vram));
	rc |= rbx_state_io(ctx, "tctl", s_tile.control, sizeof(s_tile.control));
	rc |= rbx_state_io(ctx, "prot", &s_prot, DECO146_STATE_SIZE);
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zram", s_zram, sizeof(s_zram));
	rc |= rbx_state_io(ctx, "oki", s_oki.v, sizeof(s_oki.v) + sizeof(int));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq[0]);
		m68k_bus_cur->irq_asserted = irq[1];
		m68k_bus_update_irq();
		z.read_byte = z80_read;
		z.write_byte = z80_write;
		z.port_in = z80_in;
		z.port_out = z80_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		s_cpu_frac = fr[0];
		s_z80_frac = fr[1];
		s_ym_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		s_ym_irq = misc[1];
		s_latch_irq = misc[2];
		s_irq = s_ym_irq | s_latch_irq;
		s_oki_acc = (uint32_t)misc[3];
		s_oki_hold = misc[4];
		for (int i = 0; i < 8; i++)
			deco16ic_control_w(&s_tile, i, s_tile.control[i], 0xffff);
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int dw_save(void *ctx) { return dw_state(ctx, 0); }
static int dw_load(void *ctx) { return dw_state(ctx, 1); }
static int dw_supported(const char *machine) { return !strcmp(machine, "dblewing"); }

const rbx_driver rbx_drv_dblewing = { "DECO 16 Double Wings", dw_init, dw_exit, dw_reset, dw_frame, dw_save, dw_load,
                                      dw_supported, dw_render, NULL };

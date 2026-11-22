/*  Video System Aero Fighters (newer hardware). Ported from MAME 0.289 src/mame/vsystem/aerofgt.cpp, vsystem_spr.cpp
    (CG10103 / VS9108 sprites), vs9209.cpp (VS9209 I/O) (BSD-3-Clause, copyright-holders: Nicola Salmoria,
    David Haywood, AJR).
    A 68000 at 10 MHz; two 8x8 tile layers of 512x512, the CG10103 sprites (16x16 tiles through a look-up RAM, zoomed,
    the sprite and look-up RAMs buffered at the vblank); a palette of 1024 xRGB 555; a Z80 at 5 MHz with a YM2610.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/decoband.h"
#include "../video/vsystem_gfx.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 10000000
#define Z80_CLOCK 5000000
#define FPS_MHZ 61310
#define LINES 256
#define VIS_W 320
#define VIS_H 224
#define PAL_N 1024
#define BANDS 8

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static vsg_t s_tiles[2], s_sprg;
static int s_ports[8];   // the VS9209's ports A..H (P1, P2, SYSTEM, DSW1, DSW2, -, -, JP1)
static z80 s_z80;
static int s_zirq;
static const uint8_t *s_zmap[256];
static int32_t *s_mix_l, *s_mix_r;
static int s_mix_n, s_mix_cap;

// ---- the state (saved) ----
static struct aerofgt_state {   // (on the heap while the game runs, ADR-040)
	uint16_t ram[0x8000];          // work RAM (0xfef000-0xffefff)
	uint16_t pal[0x400];
	uint16_t raster[0x800];        // 0x1b0000-0x1b0fff (the scroll registers and the RAM above them)
	uint16_t vram[2][0x1000];
	uint16_t look[0x8000];         // 0x1c0000-0x1c7fff (the upper half: never written)
	uint16_t spr[0x1000];
	uint16_t look_buf[0x8000];     // BUFFERED_SPRITERAM16: the copies made at the vblank
	uint16_t spr_buf[0x1000];
	uint16_t bank[4], scrolly[2];
	uint8_t gfxbank[8];
	uint8_t io_latch[8], io_dir[8];
	uint8_t latch, pending, zbank;
	uint8_t zram[0x800];
	uint64_t cpu_frac, z80_frac, z80_target, ym_frac;
	uint8_t state_end;
} *s_bp;
#define b (*s_bp)

typedef struct {
	int16_t sx, sy;
	uint8_t flags;
	uint8_t color;
	uint32_t code, zx, zy, pmask;
} op_t;
#define MAX_OPS 40000
static struct {
	uint32_t pens[PAL_N];
	uint16_t vram[2][0x1000];
	int32_t rowscroll[2];
	vsg_layer lay[2];
	int nops;
	op_t ops[MAX_OPS];
} *s_snap;
static uint16_t s_pal_conv[PAL_N];
static int s_pal_valid;
static uint8_t *s_pri;

static inline int pal5(int v) { return (v << 3) | (v >> 2); }

// ---- sound latch: the Z80 kept up with the 68000 (MAME's retry_access on the pending read) ----
static void z80_catchup(void)
{
	const uint64_t now = b.z80_target + (uint64_t)rbx_muldiv((uint32_t)m68k_cycles_run(), Z80_CLOCK / 1000, CPU_CLOCK / 1000);
	if ((long)(s_z80.cyc - (unsigned long)now) < 0)
		z80_run(&s_z80, (unsigned long)now, &s_zirq, 0);
}
static void latch_w(uint8_t v)
{
	z80_catchup();
	b.latch = v;
	if (!b.pending)
		z80_gen_nmi(&s_z80);
	b.pending = 1;
}

// ---- the VS9209 ----
static uint8_t io_read(int off)
{
	const int port = off & 7;
	if (off & 8)
		return 0;   // (write-only registers: unmapped)
	uint8_t mask = (uint8_t)~b.io_dir[port];
	if (port == 7)
		mask &= 0x0f;
	uint8_t in = 0;
	if (mask) {
		if (port == 6) {
			z80_catchup();
			in = b.pending;
		} else if (s_ports[port] >= 0)
			in = (uint8_t)rbx_port_read(s_ports[port]);
		in &= mask;
	}
	return (uint8_t)(in | (b.io_latch[port] & b.io_dir[port]));
}
static void io_write(int off, uint8_t v)
{
	const int port = off & 7;
	if (port == 7)
		v &= 0x0f;
	if (off & 8)
		b.io_dir[port] = v;
	else
		b.io_latch[port] = v;   // (port G's bit 7: the MB3773 watchdog, not emulated)
}

// ---- the 68000 ----
static inline void combine(uint16_t *p, uint16_t d, uint16_t m) { *p = (uint16_t)((*p & ~m) | (d & m)); }

static uint16_t main_r16(uint32_t a)
{
	a &= 0xfffffe;
	if (a >= 0xffffa0 && a < 0xffffc0)
		return io_read((int)((a - 0xffffa0) >> 1));
	return 0;
}
static uint8_t main_r8(uint32_t a)
{
	const uint16_t w = main_r16(a);
	return (uint8_t)((a & 1) ? w : w >> 8);
}
static void main_w(uint32_t a, uint16_t d, uint16_t m)
{
	a &= 0xfffffe;
	if (a >= 0xffff80 && a < 0xffff88) {   // gfxbank_w
		const int off = (int)((a - 0xffff80) >> 1);
		combine(&b.bank[off], d, m);
		b.gfxbank[2 * off + 0] = (uint8_t)(b.bank[off] >> 8);
		b.gfxbank[2 * off + 1] = (uint8_t)b.bank[off];
	} else if (a == 0xffff88)
		combine(&b.scrolly[0], d, m);
	else if (a == 0xffff90)
		combine(&b.scrolly[1], d, m);
	else if (a >= 0xffffa0 && a < 0xffffc0) {
		if (m & 0x00ff)
			io_write((int)((a - 0xffffa0) >> 1), (uint8_t)d);
	} else if (a == 0xffffc0 && (m & 0x00ff))
		latch_w((uint8_t)d);
}
static void main_w16(uint32_t a, uint16_t v) { main_w(a, v, 0xffff); }
static void main_w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		main_w(a, v, 0x00ff);
	else
		main_w(a, (uint16_t)(v << 8), 0xff00);
}

static void map_mem(void)
{
	m68k_bus_reset_map();
	const uint32_t n = s_rom_size < 0x80000 ? s_rom_size : 0x80000;
	m68k_bus_map(0x000000, n - 1, s_rom, n, 1, 0);
	m68k_bus_map(0x1a0000, 0x1a07ff, b.pal, 0x800, 1, 1);
	m68k_bus_map(0x1b0000, 0x1b0fff, b.raster, 0x1000, 1, 1);
	m68k_bus_map(0x1b2000, 0x1b3fff, b.vram[0], 0x2000, 1, 1);
	m68k_bus_map(0x1b4000, 0x1b5fff, b.vram[1], 0x2000, 1, 1);
	m68k_bus_map(0x1c0000, 0x1c7fff, b.look, 0x8000, 1, 1);
	m68k_bus_map(0x1d0000, 0x1d1fff, b.spr, 0x2000, 1, 1);
	m68k_bus_map(0xfef000, 0xffefff, b.ram, 0x10000, 1, 1);
	m68k_bus.read8 = main_r8;
	m68k_bus.read16 = main_r16;
	m68k_bus.write8 = main_w8;
	m68k_bus.write16 = main_w16;
}

// ---- the Z80 ----
static void z_bank(void)
{
	for (int p = 0x80; p < 0x100; p++) {
		const uint32_t o = (uint32_t)b.zbank * 0x8000 + (uint32_t)(p - 0x80) * 256;
		s_zmap[p] = o + 256 <= s_zrom_size ? s_zrom + o : NULL;
	}
}
static uint8_t z_read(void *u, uint16_t a)
{
	(void)u;
	const uint8_t *p = s_zmap[a >> 8];
	if (p)
		return p[a & 0xff];
	if (a >= 0x7800 && a < 0x8000)
		return b.zram[a - 0x7800];
	return 0;
}
static void z_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x7800 && a < 0x8000)
		b.zram[a - 0x7800] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p <= 0x03) return rbx_ym_read(0, p & 3);
	if (p == 0x0c) return b.latch;
	return 0;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p <= 0x03) rbx_ym_write(0, p & 3, v);
	else if (p == 0x04) { b.zbank = v & 3; z_bank(); }
	else if (p == 0x08) b.pending = 0;
}
static void ym_irq(int state) { s_zirq = state; }
static void z_start(void)
{
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x78 && (uint32_t)p * 256 + 256 <= s_zrom_size; p++)
		s_zmap[p] = s_zrom + p * 256;
	z_bank();
	z80_init(&s_z80);
	s_z80.read_byte = z_read;
	s_z80.write_byte = z_write;
	s_z80.port_in = z_in;
	s_z80.port_out = z_out;
	s_z80.read_map = s_zmap;
}

// ---- sprites (vsystem_spr_device::draw_sprites, from the buffered RAMs) ----
static void op_add(uint32_t code, uint32_t color, int fx, int fy, int sx, int sy, uint32_t zx, uint32_t zy, uint32_t pmask)
{
	const int dw = (int)((zx * 16 + 0x8000) >> 16), dh = (int)((zy * 16 + 0x8000) >> 16);
	if (dw < 1 || dh < 1 || sx > VIS_W - 1 || sx + dw - 1 < 0 || sy > VIS_H - 1 || sy + dh - 1 < 0 || s_snap->nops >= MAX_OPS)
		return;
	op_t *o = &s_snap->ops[s_snap->nops++];
	o->sx = (int16_t)sx;
	o->sy = (int16_t)sy;
	o->flags = (uint8_t)((fx ? 1 : 0) | (fy ? 2 : 0));
	o->color = (uint8_t)color;
	o->code = code;
	o->zx = zx;
	o->zy = zy;
	o->pmask = pmask;
}

static void build_sprites(void)
{
	const uint16_t *s = b.spr_buf;
	int end;
	for (end = 0; end < 0x2000 / 16; end++)
		if (s[end] & 0x4000)
			break;
	for (int offs = end - 1; offs >= 0; offs--) {   // (the priority callback set: drawn from the end)
		if (s[offs] & 0x8000)
			continue;
		const uint16_t *r = s + 4 * (s[offs] & 0x03ff);
		const int oy = r[0] & 0x01ff, ysize = (r[0] & 0x0e00) >> 9, zoomy = 32 - ((r[0] & 0xf000) >> 12);
		const int ox = r[1] & 0x01ff, xsize = (r[1] & 0x0e00) >> 9, zoomx = 32 - ((r[1] & 0xf000) >> 12);
		const int flipx = (r[2] >> 14) & 1, flipy = (r[2] >> 15) & 1;
		uint32_t color = (r[2] & 0x3f00) >> 8;
		uint32_t map = (uint32_t)(r[2] & 1) << 16 | r[3];
		const uint32_t pmask = (color & 0x20) ? 0 : 0xcccc;   // pri_callback: GFX_PMASK_2
		color &= 0x3f;
		const uint32_t zx = (uint32_t)zoomx << 11, zy = (uint32_t)zoomy << 11;
		const int ystart = flipy ? ysize : 0, yend = flipy ? -1 : ysize + 1, yinc = flipy ? -1 : 1;
		const int xstart = flipx ? xsize : 0, xend = flipx ? -1 : xsize + 1, xinc = flipx ? -1 : 1;
		for (int yc = ystart; yc != yend; yc += yinc) {
			const int yo = oy + yc * zoomy / 2;
			for (int xc = xstart; xc != xend; xc += xinc) {
				const uint32_t code = b.look_buf[map++ & 0x7fff];
				const int xo = ox + xc * zoomx / 2;
				op_add(code, color, flipx, flipy, xo, yo, zx, zy, pmask);
				op_add(code, color, flipx, flipy, -0x200 + xo, yo, zx, zy, pmask);
				op_add(code, color, flipx, flipy, xo, -0x200 + yo, zx, zy, pmask);
				op_add(code, color, flipx, flipy, -0x200 + xo, -0x200 + yo, zx, zy, pmask);
			}
		}
	}
}

static void vblank(void)
{
	rbx_video_wait_idle();
	for (int i = 0; i < PAL_N; i++) {
		const uint16_t v = b.pal[i];
		if (s_pal_valid && s_pal_conv[i] == v)
			continue;
		s_pal_conv[i] = v;
		s_snap->pens[i] = (uint32_t)(pal5((v >> 10) & 31) << 16 | pal5((v >> 5) & 31) << 8 | pal5(v & 31));
	}
	s_pal_valid = 1;
	memcpy(s_snap->vram, b.vram, sizeof(b.vram));
	s_snap->rowscroll[0] = b.raster[0x0000] - 18;
	s_snap->rowscroll[1] = b.raster[0x0200] - 20;
	for (int l = 0; l < 2; l++) {
		vsg_layer *y = &s_snap->lay[l];
		memset(y, 0, sizeof(*y));
		y->vram = s_snap->vram[l];
		y->gfx = &s_tiles[l];
		y->cols = y->rows = 64;
		y->code_mask = 0x07ff;
		y->bank_shift = 11;
		y->bank_mask = 3;
		for (int i = 0; i < 4; i++)
			y->bank[i] = (uint32_t)b.gfxbank[l * 4 + i] << 11;
		y->color_shift = 13;
		y->opaque = l == 0;
		y->priority = (uint8_t)(l + 1);
		y->scrollrows = 1;
		y->rowscroll = &s_snap->rowscroll[l];
		y->colscroll = b.scrolly[l];
		y->xextent = VIS_W;
		y->yextent = VIS_H;
	}
	s_snap->nops = 0;
	build_sprites();
	rbx_video_frame_done();
	// (the screen's vblank: the RAMs buffered after the frame was drawn)
	memcpy(b.spr_buf, b.spr, sizeof(b.spr));
	memcpy(b.look_buf, b.look, sizeof(b.look));
}

static void band(int j)
{
	const int y0 = VIS_H * j / BANDS, y1 = VIS_H * (j + 1) / BANDS - 1;
	const int pitch = rbx_video.pitch;
	for (int y = y0; y <= y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * pitch;
		uint8_t *p = s_pri + y * pitch;
		memset(p, 0, VIS_W);
		vsg_layer_line(&s_snap->lay[0], y, 0, VIS_W - 1, d, p);
		vsg_layer_line(&s_snap->lay[1], y, 0, VIS_W - 1, d, p);
	}
	const vsg_target t = { rbx_video.bitmap, s_pri, pitch, 0, 0, 0, VIS_W - 1, y0, y1 };
	const op_t *o = s_snap->ops;
	for (int i = 0; i < s_snap->nops; i++, o++) {
		if (o->sy > y1 || o->sy + 32 < y0)
			continue;   // (no tile taller than 16 lines)
		vsg_zoom(&s_sprg, &t, o->code, o->color, o->flags & 1, (o->flags >> 1) & 1, o->sx, o->sy, o->zx, o->zy, 1, o->pmask);
	}
}
static decoband_t s_bands = { 0, 0, 0, 0, BANDS, band };
static int af_help(int helper) { return decoband_help(&s_bands, helper); }

static void af_render(void)
{
	memcpy(rbx_video.palette, s_snap->pens, sizeof(s_snap->pens));
	decoband_run(&s_bands);
}

static void af_frame(void)
{
	rbx_audio.frame_mhz = FPS_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {
			vblank();
			m68k_bus_hold_irq(1);
		}
		m68k_execute(rbx_line_cycles(&b.cpu_frac, CPU_CLOCK, FPS_MHZ, LINES));
		b.z80_target += (uint64_t)rbx_line_cycles(&b.z80_frac, Z80_CLOCK, FPS_MHZ, LINES);
		z80_run(&s_z80, (unsigned long)b.z80_target, &s_zirq, 0);
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&b.ym_frac, 8000000, FPS_MHZ, LINES);
		s_mix_n += rbx_ym_advance_lr(0, clocks, s_mix_l + s_mix_n, s_mix_r + s_mix_n, s_mix_cap - s_mix_n);
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put_stereo(s_mix_l[i], s_mix_r[i]);
	memset(s_mix_l, 0, sizeof(int32_t) * (size_t)s_mix_n);
	memset(s_mix_r, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static void af_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	map_mem();
	z_start();
	s_zirq = 0;
	rbx_ym_reset(0);
	s_pal_valid = 0;
	s_mix_n = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
}

static void af_exit(void)
{
	rbx_video_wait_idle();
	m68k_set_mame_timing(0);
	rbx_ym_exit(0);
	vsg_free(&s_tiles[0]);
	memset(&s_tiles[1], 0, sizeof(s_tiles[1]));   // (shares the first's pixels)
	vsg_free(&s_sprg);
	free(s_snap);
	s_snap = NULL;
	free(s_pri);
	s_pri = NULL;
	free(s_mix_l);
	free(s_mix_r);
	s_mix_l = s_mix_r = NULL;
	free(s_bp);   // (last: the render thread and the saves are done with it)
	s_bp = NULL;
}

static int af_supported(const char *machine) { return !strcmp(machine, "aerofgt") || !strncmp(machine, "aerofgt:", 8); }

static int af_init(const rbx_game *g)
{
	if (!s_bp && !(s_bp = (struct aerofgt_state *)calloc(1, sizeof(*s_bp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t tsz = 0, ssz = 0, asz = 0, bsz = 0;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	uint8_t *tiles = rbx_region("tiles", &tsz), *spr = rbx_region("sprites", &ssz);
	uint8_t *pa = rbx_region("ymsnd:adpcma", &asz), *pb = rbx_region("ymsnd:adpcmb", &bsz);
	if (!s_rom || !s_zrom || !tiles || !spr) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	m68k_bus_swap16(s_rom, s_rom_size);
	memset(s_tiles, 0, sizeof(s_tiles));
	memset(&s_sprg, 0, sizeof(s_sprg));
	if (vsg_packed(&s_tiles[0], tiles, tsz, 8, 1, 0, 16) || vsg_packed(&s_sprg, spr, ssz, 16, 1, 512, 32)) {
		rbx_fail("%s: out of memory", g->set);
		vsg_free(&s_tiles[0]);
		vsg_free(&s_sprg);
		return -1;
	}
	s_tiles[1] = s_tiles[0];
	s_tiles[1].colorbase = 256;
	static const char *const tags[8] = { "P1", "P2", "SYSTEM", "DSW1", "DSW2", "", "", "JP1" };
	for (int i = 0; i < 8; i++)
		s_ports[i] = tags[i][0] ? rbx_port_find(tags[i]) : -1;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2610;
	yc.clock = 8000000;
	yc.adpcma = pa;
	yc.adpcma_size = asz;
	yc.adpcmb = pb ? pb : pa;
	yc.adpcmb_size = pb ? bsz : asz;
	yc.gain[0] = 192;
	yc.gain[1] = yc.gain[2] = 256;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: out of memory", g->set);
		vsg_free(&s_tiles[0]);
		vsg_free(&s_sprg);
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix_l = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_mix_r = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_snap = calloc(1, sizeof(*s_snap));
	if (!s_mix_l || !s_mix_r || !s_snap || rbx_audio_setup_stereo(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, PAL_N) ||
	    !(s_pri = (uint8_t *)calloc((size_t)rbx_video.pitch * VIS_H, 1))) {
		rbx_fail("%s: out of memory", g->set);
		af_exit();
		return -1;
	}
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);   // (MAME's DIVx / MULx / ADDQ.W An times)
	af_reset();
	return 0;
}

static int af_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "aerofgt", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "zirq", &s_zirq, sizeof(s_zirq));
	rc |= rbx_ym_state(0, ctx, load);
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		m68k_bus_update_irq();
		z.read_byte = z_read;
		z.write_byte = z_write;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_z80 = z;
		z_bank();
		s_pal_valid = 0;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int af_save(void *ctx) { return af_state(ctx, 0); }
static int af_load(void *ctx) { return af_state(ctx, 1); }

const rbx_driver rbx_drv_aerofgt = { "Video System Aero Fighters", af_init, af_exit, af_reset, af_frame, af_save, af_load,
                                     af_supported, af_render, af_help };

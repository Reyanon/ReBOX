/*  Seibu Cup Soccer :Selection: bootlegs: a 68000 with the COP bootleg (an FPGA clone), three 16x16 tilemaps and an
    8x8 text layer with priorities, 16x16 sprites; a Z80 sound CPU with an OKI MSM6295 (banked); 2048 colors
    (xBGR_555 RAM).
    Ported from MAME 0.289 src/mame/seibu/seicupbl.cpp (BSD-3-Clause, copyright-holders: Angelo Salese).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time. MAME marks the sets not working (the protection not emulated).
*/
#include "../../rbx.h"
#include "../../cpu/m68k_bus.h"
#include "../../cpu/m68k/m68k.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/okim6295.h"
#include "../../video/gfxlayout.h"
#include "../machine/seicopbl.h"
#include "../machine/smix.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 288
#define VIS_W 320
#define VIS_H 240
#define CPU_CLOCK 15000000
#define SND_CLOCK 4000000
#define NPENS 2049
#define PEN_BLACK 2048

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_oki_size;
static const uint8_t *s_okirom;
static rbx_gfx s_gchar, s_gmbk, s_gspr, s_gbk3;
static int s_port[5];
static z80 s_snd;
static okim6295_t s_oki;
static smix_t *s_mix;
static uint8_t *s_pri;
static uint64_t s_cyc_total;
static int s_in_exec;

typedef struct {
	uint8_t ram[0x20000];        // 0x100000-0x11ffff (the I/O at 0x100400-0x1007ff apart), host-order words
	uint16_t layer_disable;
	uint8_t zram[0x800];
	uint8_t latch, latch_pending, oki_bank;
	uint64_t main_frac, snd_frac, oki_frac;
	uint8_t state_end;
} board_t;
static board_t *s_bp;
#define b (*s_bp)

typedef struct {
	uint16_t sc0[0x400], sc1[0x400], sc2[0x400], sc3[0x800], spr[0x400], vregs[8], layer_disable;
	uint32_t pal[NPENS];
} snap_t;
static snap_t *s_snapp;
#define s_snap (*s_snapp)

#define RAM16(off) ((uint16_t *)(b.ram + (off)))

static uint64_t host_cycles(void) { return s_cyc_total + (s_in_exec ? (uint64_t)m68k_cycles_run() : 0); }

// ---- the sound CPU ----
static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_zrom[a];
	if (a < 0x8800) return b.zram[a - 0x8000];
	if (a == 0x9800) return oki_read(&s_oki);
	if (a == 0xa000) {
		b.latch_pending = 0;
		return b.latch;
	}
	return 0xff;
}

static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800) b.zram[a - 0x8000] = v;
	else if (a == 0x9000) {   // okim_rombank_w
		b.oki_bank = v & 7;
		s_oki.bank = (uint32_t)b.oki_bank * 0x40000;
	} else if (a == 0x9800) oki_write(&s_oki, v);
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the main CPU's devices (0x100000-0x100fff) ----
static uint16_t r16(uint32_t a)
{
	a &= 0xffffff;
	if (a >= 0x100400 && a < 0x100600)
		return seicopbl_read(a - 0x100400);
	switch (a) {
	case 0x100700: return (uint16_t)rbx_port_read(s_port[0]);
	case 0x100704: return (uint16_t)rbx_port_read(s_port[1]);
	case 0x100708: return (uint16_t)rbx_port_read(s_port[2]);
	case 0x10070c: return (uint16_t)rbx_port_read(s_port[3]);
	case 0x10071c: return (uint16_t)rbx_port_read(s_port[4]);
	default: break;
	}
	if (a >= 0x100000 && a < 0x120000)
		return RAM16(0)[(a - 0x100000) >> 1];
	return 0xffff;
}

static uint8_t r8(uint32_t a)
{
	const uint16_t v = r16(a & ~1u);
	return (uint8_t)((a & 1) ? v : v >> 8);
}

static void w16m(uint32_t a, uint16_t v, uint16_t mask)
{
	a &= 0xffffff;
	if (a >= 0x100400 && a < 0x100600) {
		seicopbl_write(a - 0x100400, v, mask);
		return;
	}
	if (a == 0x10065c)
		b.layer_disable = (uint16_t)((b.layer_disable & ~mask) | (v & mask));
	else if (a == 0x100740 && (mask & 0x00ff)) {   // the sound latch (0x100741): the Z80's NMI on its data
		b.latch = (uint8_t)v;
		if (!b.latch_pending)
			z80_gen_nmi(&s_snd);
		b.latch_pending = 1;
		return;
	}
	if (a >= 0x100000 && a < 0x120000) {
		uint16_t *p = &RAM16(0)[(a - 0x100000) >> 1];
		*p = (uint16_t)((*p & ~mask) | (v & mask));
	}
}

static void w16(uint32_t a, uint16_t v) { w16m(a, v, 0xffff); }
static void w8(uint32_t a, uint8_t v)
{
	if (a & 1)
		w16m(a & ~1u, v, 0x00ff);
	else
		w16m(a, (uint16_t)(v << 8), 0xff00);
}

// ---- the drawing ----
static void layer(const uint16_t *ram, int cols, int tsz, const rbx_gfx *g, uint32_t bank, uint16_t cbase, int sx, int sy,
                  uint8_t pv)
{
	const int mw = cols * tsz, mh = 32 * tsz, tshift = tsz == 16 ? 4 : 3;   // (both powers of two)
	for (int r = 0; r < VIS_H; r++)
		sd_tile_row(rbx_video.bitmap + r * rbx_video.pitch, s_pri + r * VIS_W, pv, ram, cols, tshift, mw - 1,
		            (r + sy) & (mh - 1), sx, g, bank, 0, cbase, 15, VIS_W);
}

static void draw_sprites(void)
{
	const sd_target tg = { rbx_video.bitmap, rbx_video.pitch, 0, 0, VIS_W, VIS_H, s_pri };
	const uint16_t *spr = s_snap.spr;
	for (int offs = 0; offs < 0x400; offs += 4) {
		const uint16_t data = spr[offs];
		if (!(data & 0x8000))
			continue;
		int cur_pri = (spr[offs + 1] & 0xc000) >> 14;
		if (data & 0x0040)
			cur_pri |= 4;
		uint32_t pri_mask;
		switch (cur_pri) {
		case 0: pri_mask = (uint32_t)-256; break;
		case 1: pri_mask = 0xfff0; break;
		case 2: pri_mask = (uint32_t)-4; break;
		case 3: pri_mask = 0; break;
		case 4: pri_mask = (uint32_t)-32; break;
		default: pri_mask = 0; break;
		}
		uint32_t sprite = spr[offs + 1] & 0x3fff;
		int x = spr[offs + 2] & 0xfff, y = spr[offs + 3] & 0xfff;
		if (x & 0x800) x -= 0x1000;
		if (y & 0x800) y -= 0x1000;
		const uint32_t color = (uint32_t)((data & 0x3f) + 0x40) * 16;
		const int fx = (data >> 14) & 1, fy = (data >> 13) & 1;
		const int dy = ((data & 0x0380) >> 7) + 1, dx = ((data & 0x1c00) >> 10) + 1;
		for (int ax = 0; ax < dx; ax++)
			for (int ay = 0; ay < dy; ay++)
				sd_prio_transpen(&tg, &s_gspr, sprite++, color, fx, fy, x + (fx ? dx - ax - 1 : ax) * 16,
				                 y + (fy ? dy - ay - 1 : ay) * 16, pri_mask, 15);
	}
}

static void sc_render(void)
{
	memcpy(rbx_video.palette, s_snap.pal, sizeof(s_snap.pal));
	memset(s_pri, 0, VIS_W * VIS_H);
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PEN_BLACK;
	}
	const uint16_t *v = s_snap.vregs, dis = s_snap.layer_disable;
	if (!(dis & 1)) layer(s_snap.sc0, 32, 16, &s_gmbk, 0, 0, (int16_t)(v[0] - 0x1f0), v[1], 1);
	if (!(dis & 2)) layer(s_snap.sc1, 32, 16, &s_gmbk, 0x1000, 0x10 * 16, (int16_t)(v[2] - 0x1f0), v[3], 2);
	if (!(dis & 4)) layer(s_snap.sc2, 32, 16, &s_gbk3, 0, 32 * 16, (int16_t)(v[4] - 0x1f0), v[5], 4);
	if (!(dis & 8)) layer(s_snap.sc3, 64, 8, &s_gchar, 0, 48 * 16, (int16_t)(v[6] - 0x1f0), v[7], 8);
	if (!(dis & 0x10)) draw_sprites();
}

static void capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap.sc0, b.ram + 0x0800, sizeof(s_snap.sc0));
	memcpy(s_snap.sc2, b.ram + 0x1000, sizeof(s_snap.sc2));
	memcpy(s_snap.sc1, b.ram + 0x1800, sizeof(s_snap.sc1));
	memcpy(s_snap.sc3, b.ram + 0x2000, sizeof(s_snap.sc3));
	memcpy(s_snap.spr, b.ram + 0x7000, sizeof(s_snap.spr));
	memcpy(s_snap.vregs, b.ram + 0x660, sizeof(s_snap.vregs));
	s_snap.layer_disable = b.layer_disable;
	const uint16_t *pal = RAM16(0x3000);
	for (int i = 0; i < 2048; i++) {   // xBGR_555
		const uint16_t w = pal[i];
		const uint32_t r = w & 31, g = (w >> 5) & 31, bl = (w >> 10) & 31;
		s_snap.pal[i] = (r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (bl << 3 | bl >> 2);
	}
	s_snap.pal[PEN_BLACK] = 0;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sc_reset(void)
{
	rbx_video_wait_idle();
	memset(&b, 0, offsetof(__typeof__(b), state_end));
	s_cyc_total = 0;
	m68k_bus_irq_reset();
	m68k_pulse_reset();
	seicopbl_reset();
	z80_init(&s_snd);
	s_snd.read_byte = snd_read;
	s_snd.write_byte = snd_write;
	s_snd.port_in = z_in;
	s_snd.port_out = z_out;
	oki_reset(&s_oki);
	s_oki.bank = 0;
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void sc_exit(void)
{
	rbx_video_wait_idle();
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gmbk);
	rbx_gfx_free(&s_gspr);
	rbx_gfx_free(&s_gbk3);
	free(s_pri);
	s_pri = NULL;
	free(s_bp);
	s_bp = NULL;
	free(s_snapp);
	s_snapp = NULL;
	free(s_mix);
	s_mix = NULL;
}

static void invert(uint8_t *p, uint32_t n)
{
	for (uint32_t i = 0; i < n; i++)
		p[i] = (uint8_t)~p[i];
}

static int sc_init(const rbx_game *g)
{
	uint32_t zs, sps, chs, mbs, b3s;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &zs);
	uint8_t *sp = rbx_region("sprite", &sps), *ch = rbx_region("char", &chs), *mb = rbx_region("mbk_tile", &mbs),
	        *b3 = rbx_region("bk3_tile", &b3s);
	s_okirom = rbx_region("oki", &s_oki_size);
	if (!s_rom || s_rom_size < 0x100000 || !s_zrom || zs < 0x8000 || !sp || !ch || !mb || !b3 || !s_okirom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_bp = calloc(1, sizeof(board_t));
	s_snapp = calloc(1, sizeof(snap_t));
	s_mix = calloc(1, sizeof(*s_mix));
	s_pri = calloc(1, VIS_W * VIS_H);
	if (!s_bp || !s_snapp || !s_mix || !s_pri) {
		sc_exit();
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// ROMREGION_INVERT (the tables leave it to the driver): sprite, char, mbk_tile, bk3_tile; lbk_tile (a copy of the
	// inverted bk3_tile inverted again) is not drawn
	invert(sp, sps);
	invert(ch, chs);
	invert(mb, mbs);
	invert(b3, b3s);
	m68k_bus_swap16(s_rom, s_rom_size);
	{   // cupsocsb_8x8_tilelayout, cupsocsb_tilelayout, gfx_8x8x4_col_2x2_group_packed_lsb
		static const uint32_t tp[4] = { 8, 12, 0, 4 };
		static const uint32_t cx[8] = { 0, 3, 2, 1, 16, 19, 18, 17 };
		static const uint32_t tx[16] = { 0, 3, 2, 1, 16, 19, 18, 17, 512, 515, 514, 513, 528, 531, 530, 529 };
		static const uint32_t sp4[4] = { 0, 1, 2, 3 };
		static const uint32_t sx[16] = { 4, 0, 12, 8, 20, 16, 28, 24, 516, 512, 524, 520, 532, 528, 540, 536 };
		static uint32_t ty[16];
		for (int i = 0; i < 16; i++)
			ty[i] = (uint32_t)i * 32;
		const rbx_gfx_layout lc = { 8, 8, 4, tp, cx, ty, 256 }, lt = { 16, 16, 4, tp, tx, ty, 1024 };
		const rbx_gfx_layout ls = { 16, 16, 4, sp4, sx, ty, 1024 };
		if (rbx_gfx_decode(&s_gchar, &lc, ch, chs) || rbx_gfx_decode(&s_gmbk, &lt, mb, mbs) ||
		    rbx_gfx_decode(&s_gbk3, &lt, b3, b3s) || rbx_gfx_decode(&s_gspr, &ls, sp, sps) ||
		    rbx_video_setup(VIS_W, VIS_H, NPENS) || rbx_audio_setup(48000)) {
			sc_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	oki_init(&s_oki, 1000000, 1, s_okirom, s_oki_size, 256);
	seicopbl_init(host_cycles);
	static const char *const names[5] = { "DSW1", "PLAYERS12", "PLAYERS34", "SYSTEM", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	m68k_bus_reset_map();
	m68k_bus.read8 = r8;
	m68k_bus.read16 = r16;
	m68k_bus.write8 = w8;
	m68k_bus.write16 = w16;
	m68k_bus_map(0x000000, 0x0fffff, s_rom, s_rom_size, 1, 0);
	m68k_bus_map(0x101000, 0x11ffff, b.ram + 0x1000, sizeof(b.ram) - 0x1000, 1, 1);
	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_mame_timing(1);
	sc_reset();
	return 0;
}

static void sc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == VIS_H) {   // vblank: IRQ 4 (HOLD_LINE)
			capture();
			m68k_bus_hold_irq(4);
		}
		const int c = rbx_line_cycles(&b.main_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		s_in_exec = 1;
		s_cyc_total += (uint64_t)m68k_execute(c);
		s_in_exec = 0;
		int irq = 0;
		z80_run(&s_snd, s_snd.cyc + (unsigned long)rbx_line_cycles(&b.snd_frac, SND_CLOCK, FRAME_MHZ, LINES), &irq, 0);
		smix_begin(s_mix);
		int32_t *tmp = smix_buf(s_mix);   // (all n assigned below: no clearing)
		int n = rbx_line_cycles(&b.oki_frac, s_oki.rate, FRAME_MHZ, LINES);
		if (n > SMIX_TMP)
			n = SMIX_TMP;
		for (int i = 0; i < n; i++)
			tmp[i] = oki_sample(&s_oki);
		smix_add(s_mix, tmp, n);
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int sc_state(void *ctx, int load)
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
	int rc = rbx_state_io(ctx, "m68k", cpu, csize);
	rc |= rbx_state_io(ctx, "irq", &irq, sizeof(irq));
	rc |= rbx_state_io(ctx, "seicupbl", &b, (uint32_t)offsetof(__typeof__(b), state_end));
	rc |= seicopbl_state(ctx, load);
	z80 c = s_snd;
	rc |= rbx_state_io(ctx, "seicupbl_snd", &c, sizeof(c));
	rc |= rbx_state_io(ctx, "seicupbl_oki", s_oki.v, sizeof(s_oki.v));
	if (!rc && load) {
		m68k_set_context(cpu);
		m68k_bus_set_irq_pending(irq);
		c.read_byte = snd_read;
		c.write_byte = snd_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = NULL;
		s_snd = c;
		s_oki.bank = (uint32_t)b.oki_bank * 0x40000;
	}
	free(cpu);
	return rc ? -1 : 0;
}

static int sc_save(void *ctx) { return sc_state(ctx, 0); }
static int sc_load(void *ctx) { return sc_state(ctx, 1); }
static int sc_supported(const char *m) { return !strcmp(m, "cupsocbl"); }

const rbx_driver rbx_drv_seicupbl = { "Seibu Cup Soccer Bootleg", sc_init, sc_exit, sc_reset, sc_frame, sc_save, sc_load,
                                      sc_supported, sc_render, NULL };

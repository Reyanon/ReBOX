/*  Seibu Kaihatsu Metal Freezer: a Z80 (opcodes and data encrypted apart, code run from work RAM too), an 8x8 tile
    layer (two ROM banks, a scrolled part below a fixed band) and 16x16 sprites, the T5182 sound MCU with a YM2151;
    512 colors (xBGR_444 RAM).
    Ported from MAME 0.289 src/mame/seibu/metlfrzr.cpp (BSD-3-Clause, copyright-holders: Angelo Salese, David Haywood).
    This file is part of ReBOX. GPL-2.0-or-later.

    The CPUs interleaved a scanline at a time. Not done: the flip screen, the coin lockouts.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../video/gfxlayout.h"
#include "../machine/smix.h"
#include "../machine/t5182.h"
#include "../video/seibu_draw.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 60000
#define LINES 256
#define VIS_Y0 16
#define VIS_W 256
#define VIS_H 224
#define CPU_CLOCK 6000000
#define NPENS 0x201
#define PEN_BLACK 0x200

static uint8_t *s_rom, *s_op;   // s_op: the opcode space (0xf000-0xffff: the work RAM itself)
static rbx_gfx s_gtile[2], s_gspr[2];
static int s_port[5];
static z80 s_main;
static smix_t *s_mix;

static struct {
	uint8_t vram[0x1000], pal[0x400], vregs[0x20], ram[0x1800];
	uint8_t bank, tilebank, rowscroll;
	int main_irq;
	uint64_t main_frac, snd_frac, ym_frac;
	uint8_t state_end;
} t;

static struct {
	uint8_t vram[0x1000], vregs[0x20], spr[0x200];
	uint8_t tilebank, rowscroll;
	uint32_t pal[NPENS];
} *s_snap;

#define WRAM (s_op + 0xf000)

static void set_bank(void)
{
	memcpy(s_op + 0x8000, s_rom + 0x10000 + (uint32_t)t.bank * 0x4000, 0x4000);
}

// ---- the main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return s_rom[a];
	if (a < 0xc000) return s_rom[0x10000 + (uint32_t)t.bank * 0x4000 + (a - 0x8000)];
	if (a < 0xd000) return t.vram[a - 0xc000];
	if (a < 0xd400) return t.pal[a - 0xd000];
	if (a < 0xd480) return t5182_sharedram_r(a - 0xd400);
	if (a >= 0xd600 && a <= 0xd604) return (uint8_t)rbx_port_read(s_port[a - 0xd600]);
	if (a == 0xd711) return t5182_semaphore_snd_r();
	if (a >= 0xd800 && a < 0xf000) return t.ram[a - 0xd800];
	if (a >= 0xf000) return WRAM[a - 0xf000];
	return 0xff;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0xc000 && a < 0xd000) t.vram[a - 0xc000] = v;
	else if (a >= 0xd000 && a < 0xd400) t.pal[a - 0xd000] = v;
	else if (a >= 0xd400 && a < 0xd480) t5182_sharedram_w(a - 0xd400, v);
	else if (a >= 0xd600 && a < 0xd620) t.vregs[a - 0xd600] = v;
	else if (a == 0xd700) {   // output_w
		t.tilebank = (v >> 4) & 1;
		t.bank = (v >> 2) & 3;
		t.rowscroll = (v >> 1) & 1;
		set_bank();
	} else if (a == 0xd710) t5182_sound_irq_w();
	else if (a == 0xd712) t5182_semaphore_main_release_w();
	else if (a == 0xd713) t5182_semaphore_main_acquire_w();
	else if (a >= 0xd800 && a < 0xf000) t.ram[a - 0xd800] = v;
	else if (a >= 0xf000) WRAM[a - 0xf000] = v;
}

static uint8_t z_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void z_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- the drawing ----
static void mf_render(void)
{
	memcpy(rbx_video.palette, s_snap->pal, sizeof(s_snap->pal));
	for (int r = 0; r < VIS_H; r++) {
		uint16_t *d = rbx_video.bitmap + r * rbx_video.pitch;
		for (int x = 0; x < VIS_W; x++)
			d[x] = PEN_BLACK;
	}
	const sd_target tg = { rbx_video.bitmap, rbx_video.pitch, 0, VIS_Y0, VIS_W, VIS_H, NULL };
	const rbx_gfx *g = &s_gtile[s_snap->tilebank];
	const uint8_t *vr = s_snap->vregs;
	const int xs = vr[0x17] + ((vr[0x06] & 1) << 8);
	const int xbase = (xs >> 3) * 32;
	for (int count = 0; count < 32 * 33; count++) {   // legacy_bg_draw
		int tb = count, shift = 0;
		const int y = count & 0x1f, x = count >> 5;
		if (y > 7 || !s_snap->rowscroll) {
			tb += xbase;
			shift = xs & 7;
		}
		tb &= 0x7ff;
		const uint32_t tile = s_snap->vram[tb * 2] + (uint32_t)((s_snap->vram[tb * 2 + 1] & 0xf0) << 4);
		sd_transpen(&tg, g, tile, 0x100 + (uint32_t)(s_snap->vram[tb * 2 + 1] & 0xf) * 16, 0, 0, x * 8 - shift, y * 8, 0xf);
	}
	for (int count = 0x200 - 4; count > -1; count -= 4) {   // legacy_obj_draw
		const uint8_t *s = &s_snap->spr[count];
		int x = s[3];
		if (s[1] & 0x80)
			x -= 256;
		sd_transpen(&tg, &s_gspr[(s[1] >> 6) & 1], (uint32_t)(s[0] | ((s[1] & 0x30) >> 4) << 8), (uint32_t)(s[1] & 0xf) * 16,
		            0, 0, x, s[2], 0xf);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, t.vram, sizeof(t.vram));
	memcpy(s_snap->vregs, t.vregs, sizeof(t.vregs));
	memcpy(s_snap->spr, WRAM + 0xe00, sizeof(s_snap->spr));
	s_snap->tilebank = t.tilebank;
	s_snap->rowscroll = t.rowscroll;
	for (int i = 0; i < 0x200; i++) {   // xBGR_444
		const uint16_t w = (uint16_t)(t.pal[i] | t.pal[0x200 + i] << 8);
		s_snap->pal[i] = (uint32_t)(w & 0xf) * 0x110000u | (uint32_t)((w >> 4) & 0xf) * 0x1100u | (uint32_t)((w >> 8) & 0xf) * 0x11u;
	}
	s_snap->pal[PEN_BLACK] = 0;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void mf_reset(void)
{
	rbx_video_wait_idle();
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	memset(WRAM, 0, 0x1000);
	set_bank();
	z80_init(&s_main);
	s_main.read_byte = main_read;
	s_main.write_byte = main_write;
	s_main.port_in = z_in;
	s_main.port_out = z_out;
	s_main.op_rom = s_op;
	t5182_reset();
	s_mix->n = 0;
	s_mix->frac = 0;
}

static void mf_exit(void)
{
	rbx_video_wait_idle();
	t5182_exit();
	for (int i = 0; i < 2; i++) {
		rbx_gfx_free(&s_gtile[i]);
		rbx_gfx_free(&s_gspr[i]);
	}
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_op);
	s_op = NULL;
}

static int mf_init(const rbx_game *g)
{
	uint32_t rs, s[4], cs, es;
	s_rom = rbx_region("maincpu", &rs);
	static const char *const gn[4] = { "tiles1", "tiles2", "sprites1", "sprites2" };
	const uint8_t *gr[4];
	for (int i = 0; i < 4; i++)
		gr[i] = rbx_region(gn[i], &s[i]);
	const uint8_t *t5cpu = rbx_region("t5182:cpu", &cs), *t5ext = rbx_region("t5182:external", &es);
	if (!s_rom || rs < 0x20000 || !gr[0] || !gr[1] || !gr[2] || !gr[3] || !t5cpu || cs < 0x2000 || !t5ext) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_op = calloc(1, 0x10000);
	s_snap = calloc(1, sizeof(*s_snap));
	s_mix = calloc(1, sizeof(*s_mix));
	if (!s_op || !s_snap || !s_mix) {
		rbx_fail("%s: out of memory", g->set);
		mf_exit();
		return -1;
	}
	{   // init_metlfrzr: opcodes and data decrypted apart (as Air Raid)
		static const int b8[8] = { 7, 6, 1, 4, 3, 2, 5, 0 };
		for (uint32_t a = 0; a < 0x8000; a++) {
			uint8_t o = s_rom[a], d = s_rom[a];
			if ((a & 0x20) && !(a & 0x08)) o ^= 0x40;
			if ((a & 0x400) && !(a & 0x200) && (a & 0x08)) o ^= 0x20;
			if ((((a >> 10) ^ (a >> 9)) & 1) && (a & 0x02)) o ^= 0x02;
			if ((a & 0x200) || !(a & 0x20) || (a & 0x08)) o = (uint8_t)sd_bitswap(o, 8, b8);
			if (a & 0x20) d ^= 0x40;
			if ((a & 0x200) || !(a & 0x20)) d = (uint8_t)sd_bitswap(d, 8, b8);
			s_op[a] = o;
			s_rom[a] = d;
		}
	}
	{   // tile_layout, sprite_layout
		static const uint32_t pl[4] = { 0, 4, 8, 12 };
		static const uint32_t tx[8] = { 19, 18, 17, 16, 3, 2, 1, 0 };
		static const uint32_t ty[8] = { 0, 32, 64, 96, 128, 160, 192, 224 };
		static const uint32_t sx[16] = { 0, 1, 2, 3, 16, 17, 18, 19, 512, 513, 514, 515, 528, 529, 530, 531 };
		static uint32_t sy[16];
		for (int i = 0; i < 16; i++)
			sy[i] = (uint32_t)i * 32;
		const rbx_gfx_layout lt = { 8, 8, 4, pl, tx, ty, 256 }, ls = { 16, 16, 4, pl, sx, sy, 1024 };
		if (rbx_gfx_decode(&s_gtile[0], &lt, gr[0], s[0]) || rbx_gfx_decode(&s_gtile[1], &lt, gr[1], s[1]) ||
		    rbx_gfx_decode(&s_gspr[0], &ls, gr[2], s[2]) || rbx_gfx_decode(&s_gspr[1], &ls, gr[3], s[3])) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			mf_exit();
			return -1;
		}
	}
	if (t5182_init(t5cpu, t5ext, es, rbx_port_find("T5182_COIN"), 128) || rbx_audio_setup(48000) ||
	    rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		mf_exit();
		return -1;
	}
	smix_init(s_mix, 48000, FRAME_MHZ, LINES);
	static const char *const names[5] = { "P1", "P2", "START", "DSW1", "DSW2" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	mf_reset();
	return 0;
}

static void mf_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		if (line == 240) {   // vblank-out: RST 10h
			vblank_capture();
			s_main.int_data = 0xd7;
			t.main_irq = 1;
		} else if (line == 0) {   // RST 08h
			s_main.int_data = 0xcf;
			t.main_irq = 1;
		}
		z80_run(&s_main, s_main.cyc + (unsigned long)rbx_line_cycles(&t.main_frac, CPU_CLOCK, FRAME_MHZ, LINES), &t.main_irq, 1);
		t5182_run(rbx_line_cycles(&t.snd_frac, T5182_CLOCK, FRAME_MHZ, LINES));
		smix_begin(s_mix);
		int32_t *tmp = smix_tmp(s_mix);
		const int n = t5182_sound((uint32_t)rbx_line_cycles(&t.ym_frac, T5182_CLOCK, FRAME_MHZ, LINES), tmp, SMIX_TMP);
		smix_add(s_mix, tmp, n);
		smix_end(s_mix);
	}
	smix_flush(s_mix);
}

static int mf_state(void *ctx, int load)
{
	int rc = rbx_state_io(ctx, "metlfrzr", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "metlfrzr_wram", WRAM, 0x1000);
	z80 c = s_main;
	rc |= rbx_state_io(ctx, "metlfrzr_main", &c, sizeof(c));
	if (!rc && load) {
		c.read_byte = main_read;
		c.write_byte = main_write;
		c.port_in = z_in;
		c.port_out = z_out;
		c.userdata = NULL;
		c.read_map = NULL;
		c.op_rom = s_op;
		s_main = c;
		set_bank();
	}
	rc |= t5182_state(ctx, load);
	return rc ? -1 : 0;
}

static int mf_save(void *ctx) { return mf_state(ctx, 0); }
static int mf_load(void *ctx) { return mf_state(ctx, 1); }
static int mf_supported(const char *m) { return !strcmp(m, "metlfrzr"); }

const rbx_driver rbx_drv_metlfrzr = { "Seibu Metal Freezer", mf_init, mf_exit, mf_reset, mf_frame, mf_save, mf_load,
                                      mf_supported, mf_render, NULL };

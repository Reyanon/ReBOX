/*  Wood Place The Deep / Run Deep, from MAME src/mame/dataeast/thedeep.cpp (BSD-3-Clause, copyright-holders:
    Luca Elia).
    A Z80 at 6 MHz (its NMI at the frame's first line when enabled; its IRQ from the MCU, cleared by the protection
    read), the i8751 (the coins on P0; P1: the ROM bank and the flip; P2 / P3: the command and reply latches, the coin
    flag; INT1 for a line at mid-frame; MAME: perfect sync, here sixteenths of a line), a BAC06 playfield (8-bit,
    little-endian, column scroll; MAME's "thedeep kludge"), the MXC06 sprites, an 8x8 text layer, the PROM palette
    (512). The sound: R65C02 at 1.5 MHz with a YM2203 (on its IRQ), the latch on its NMI. MAME's screen: 60 Hz,
    256x248 without a vblank.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/mcs51/mcs51.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"
#include "../video/bac06.h"
#include "../video/decmxc06.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 6000000        // 12 MHz / 2
#define SND_CLOCK 1500000
#define MCU_CLOCK (8000000 / 12)
#define YM_CLOCK 3000000         // YM2203 (12 MHz / 4)
#define FRAME_MHZ 60000
#define LINES 248
#define VIS_H 248
#define VIS_W 256
#define SLICES 16

static uint8_t *s_rom, *s_mcu_rom;
static uint32_t s_rom_size, s_mcu_size;
static uint8_t s_ram[0x2000], s_spr[0x400], s_text[0x800], s_ram2[0x800];   // (c000, e400, e800, f800 .. ffff)
static int s_bank, s_flip, s_nmi_enable, s_nmi, s_irq;
static z80 s_cpu;
static const uint8_t *s_zmap[256];
static bac06_t s_tile;
static decmxc06_t s_spr_gen;
static rbx_gfx s_gfx8, s_gfx16, s_gfxspr;
static uint32_t s_palette[512];
static int s_port_in[2], s_port_dsw[2], s_port_coins;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym_frac;
static struct {
	bac06_t tile;
	uint8_t text[0x800];
	uint16_t spr[0x200];
	int flip;
	uint32_t frame;
} *s_snap;

// ---- the i8751 ----
static mcs51_t s_mcu;
static uint8_t s_main_to_mcu, s_mcu_to_main, s_mcu_p2, s_mcu_p3, s_coin_result;

static void set_bank(int b)
{
	s_bank = b & 3;
	for (int p = 0; p < 0x40; p++)
		s_zmap[0x80 + p] = s_rom + 0x10000 + (uint32_t)s_bank * 0x4000 + (uint32_t)p * 256;
}
static uint8_t mcu_in(void *u, int port)
{
	(void)u;
	if (port == 0) {   // mcu_p0_r: the coins (bits 1-3) and "none in" (bit 0)
		const uint8_t c = (uint8_t)rbx_port_read(s_port_coins);
		return (uint8_t)((c & 0x0e) | ((c & 0x0e) == 0x0e));
	}
	if (port == 2) return s_main_to_mcu;
	return 0xff;
}
static void mcu_out(void *u, int port, uint8_t data)
{
	(void)u;
	if (port == 1) {   // mcu_p1_w: the bank (bits 1-2), the flip (bit 0 low)
		set_bank((data >> 1) & 3);
		s_flip = !(data & 1);
	} else if (port == 2)
		s_mcu_p2 = data;
	else if (port == 3) {   // mcu_p3_w: falling edges
		const uint8_t fall = (uint8_t)(s_mcu_p3 & ~data);
		if (fall & 0x01) s_irq = 1;
		if (fall & 0x02) mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 0);
		if (fall & 0x10) s_coin_result = 1;
		if (fall & 0x40) s_mcu_to_main = s_mcu_p2;
		if (fall & 0x80) s_mcu.forced_inputs[2] = s_main_to_mcu;
		s_mcu_p3 = data;
	}
}

// ---- sound: R65C02, YM2203 ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static void ym_irq(int state) { m6502_set_irq(&s_snd, state); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x3000) {   // (reading the latch clears its NMI)
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x0800 || a == 0x0801) rbx_ym_write(0, a & 1, v);
}

// ---- bus ----
// the BAC06's 8-bit accessors (MAME ctrlreg8_w, scrollreg8_w<true>, vram8_r / w<true>, colscroll8_r / w<true>)
static void ctrl8_w(uint32_t o, uint8_t v) { bac06_ctrl_w(&s_tile, (int)(o >> 1), v, 0x00ff); }
static void scroll8_w(uint32_t o, uint8_t v)
{
	const uint32_t sh = o < 4 ? (o & 1) << 3 : 0;
	bac06_scroll_w(&s_tile, (int)(o >> 1), (uint16_t)(v << sh), (uint16_t)(0xff << sh));
}
static uint8_t vram8_r(uint32_t o) { return (uint8_t)(bac06_vram_r(&s_tile, (int)(o >> 1)) >> ((o & 1) << 3)); }
static void vram8_w(uint32_t o, uint8_t v)
{
	bac06_vram_w(&s_tile, (int)(o >> 1), (uint16_t)(v << ((o & 1) << 3)), (uint16_t)(0xff << ((o & 1) << 3)));
}
static uint8_t col8_r(uint32_t o) { return (uint8_t)(s_tile.colscroll[(o >> 1) & 0xfff] >> ((o & 1) << 3)); }
static void col8_w(uint32_t o, uint8_t v)
{
	uint16_t *p = &s_tile.colscroll[(o >> 1) & 0xfff];
	const uint16_t m = (uint16_t)(0xff << ((o & 1) << 3));
	*p = (uint16_t)((*p & ~m) | ((uint16_t)(v << ((o & 1) << 3)) & m));
}
static uint8_t cpu_r(void *u, uint16_t a)
{
	(void)u;
	if (a < 0xc000) return s_zmap[a >> 8][a & 0xff];
	if (a < 0xe000) return s_ram[a - 0xc000];
	if (a >= 0xe400 && a < 0xe800) return s_spr[a - 0xe400];
	if (a >= 0xe800 && a < 0xf000) return s_text[a - 0xe800];
	if (a >= 0xf000 && a < 0xf800) return vram8_r(a - 0xf000u);
	if (a >= 0xf800 && a < 0xf840) return col8_r(a - 0xf800u);
	if (a >= 0xf840) return s_ram2[a - 0xf800];
	switch (a) {
	case 0xe000:   // protection_r: the reply, the IRQ cleared
		s_irq = 0;
		return s_mcu_to_main;
	case 0xe004: {   // e004_r: the coin flag (read once)
		const uint8_t d = s_coin_result;
		s_coin_result = 0;
		return d;
	}
	case 0xe008: return (uint8_t)rbx_port_read(s_port_in[0]);
	case 0xe009: return (uint8_t)rbx_port_read(s_port_in[1]);
	case 0xe00a: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0xe00b: return (uint8_t)rbx_port_read(s_port_dsw[1]);
	}
	return 0;
}
static void cpu_w(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0xc000) return;
	if (a < 0xe000) { s_ram[a - 0xc000] = v; return; }
	if (a >= 0xe400 && a < 0xe800) { s_spr[a - 0xe400] = v; return; }
	if (a >= 0xe800 && a < 0xf000) { s_text[a - 0xe800] = v; return; }
	if (a >= 0xf000 && a < 0xf800) { vram8_w(a - 0xf000u, v); return; }
	if (a >= 0xf800 && a < 0xf840) { col8_w(a - 0xf800u, v); return; }
	if (a >= 0xf840) { s_ram2[a - 0xf800] = v; return; }
	if (a >= 0xe200 && a < 0xe208) { ctrl8_w(a & 7u, v); return; }
	if (a >= 0xe210 && a < 0xe218) { scroll8_w(a & 7u, v); return; }
	switch (a) {
	case 0xe000:   // protection_w: the command, INT0
		s_main_to_mcu = v;
		mcs51_set_input(&s_mcu, MCS51_INT0_LINE, 1);
		break;
	case 0xe004:   // nmi_w
		s_nmi_enable = v;
		if (!v)
			s_nmi = 0;
		break;
	case 0xe00c: s_latch = v; m6502_set_nmi(&s_snd, 1); break;
	}
}
static uint8_t cpu_in(z80 *z, uint8_t port) { (void)z; (void)port; return 0xff; }
static void cpu_out(z80 *z, uint8_t port, uint8_t v) { (void)z; (void)port; (void)v; }

// ---- video ----
static void td_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	bac06_t *t = &s_snap->tile;
	t->flip_screen = flip;
	bac06_draw(t, bm, NULL, pitch, VIS_W, 0, 0, VIS_H, BAC06_OPAQUE, 0, 0xff);
	decmxc06_t spr = s_spr_gen;
	spr.flip_screen = flip;
	decmxc06_draw(&spr, s_snap->spr, 0x200, bm, NULL, pitch, VIS_W, VIS_H, 0, s_snap->frame);
	for (int y = 0; y < VIS_H; y++) {   // the text: 32x32 of 8x8, 2 bpp, pen 0 clear (MAME's tilemap flip: 256x248)
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y;
			if (flip) {
				px = 255 - px;
				py = 255 - ((y - (248 - 256)) & 0xff);
			}
			const int o = ((py >> 3) * 32 + (px >> 3)) * 2;
			const int colour = s_snap->text[o + 1];
			const uint32_t code = s_gfx8.count ? rbx_wrap((uint32_t)(s_snap->text[o] + (colour << 8)), s_gfx8.count) : 0;
			const uint8_t p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p)
				d[x] = (uint16_t)(((colour & 0xf0) >> 4) * 4 + p);
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	s_snap->tile = s_tile;
	memcpy(s_snap->text, s_text, sizeof(s_text));
	for (int i = 0; i < 0x200; i++)   // (the sprite RAM as little-endian words)
		s_snap->spr[i] = (uint16_t)(s_spr[2 * i] | (s_spr[2 * i + 1] << 8));
	s_snap->flip = s_flip;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void td_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_text, 0, sizeof(s_text));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_tile.vram, 0, sizeof(s_tile.vram));
	memset(s_tile.rowscroll, 0, sizeof(s_tile.rowscroll));
	memset(s_tile.colscroll, 0, sizeof(s_tile.colscroll));
	bac06_reset(&s_tile);
	s_flip = s_nmi_enable = s_nmi = s_irq = 0;
	set_bank(0);
	z80_init(&s_cpu);
	s_cpu.read_byte = cpu_r;
	s_cpu.write_byte = cpu_w;
	s_cpu.port_in = cpu_in;
	s_cpu.port_out = cpu_out;
	s_cpu.read_map = s_zmap;
	s_cpu.int_data = 0xff;
	s_main_to_mcu = s_mcu_to_main = s_mcu_p2 = s_coin_result = 0;
	s_mcu_p3 = 0xff;
	mcs51_reset(&s_mcu);
	memset(s_sram, 0, sizeof(s_sram));
	s_latch = 0;
	m6502_reset(&s_snd);
	m6502_set_irq(&s_snd, 0);
	m6502_set_nmi(&s_snd, 0);
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_mcu_frac = s_ym_frac = 0;
}

static void td_exit(void);

static int td_init(const rbx_game *g)
{
	rbx_cheat_prefer("z80");   // (the cheats' main CPU: another CPU is set up first)
	uint32_t bsize, tsize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_mcu_rom = rbx_region("mcu", &s_mcu_size);
	uint8_t *bg = rbx_region("bg_gfx", &bsize), *text = rbx_region("text", &tsize), *spr = rbx_region("sprites", &ssize),
	        *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom || s_zrom_size < 0x10000 || !s_mcu_rom || s_mcu_size < 0x1000 || !bg ||
	    !text || !spr || !prom || psize < 0x600) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// layout_16x16x4 (the quarters as planes; the right half 16 bytes on), layout_8x8x2 (2 bpp in the
		// nibbles, the left half the region's second half)
		static uint32_t x16[16], y16[16], x8[8], y8[8];
		for (int i = 0; i < 8; i++) {
			x16[i] = (uint32_t)(8 * 8 * 2 + i);
			x16[8 + i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		for (int i = 0; i < 4; i++) {
			x8[i] = (uint32_t)(tsize / 2 * 8 + i);
			x8[4 + i] = (uint32_t)i;
		}
		const uint32_t qb = bsize / 4 * 8, qs = ssize / 4 * 8;
		const uint32_t pb[4] = { 0, qb, 2 * qb, 3 * qb }, ps[4] = { 0, qs, 2 * qs, 3 * qs }, pc[2] = { 0, 4 };
		const rbx_gfx_layout lb = { 16, 16, 4, pb, x16, y16, 16 * 16 }, ls = { 16, 16, 4, ps, x16, y16, 16 * 16 };
		const rbx_gfx_layout l8 = { 8, 8, 2, pc, x8, y8, 8 * 8 };
		if (rbx_gfx_decode(&s_gfx16, &lb, bg, bsize / 4) || rbx_gfx_decode(&s_gfxspr, &ls, spr, ssize / 4) ||
		    rbx_gfx_decode(&s_gfx8, &l8, text, tsize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			td_exit();
			return -1;
		}
	}
	for (int i = 0; i < 512; i++)   // palette: red / green the third PROM's nibbles, blue the second's low nibble
		s_palette[i] = (uint32_t)(((prom[0x400 + i] & 15) * 17) << 16 | ((prom[0x400 + i] >> 4) * 17) << 8 |
		                          (prom[0x200 + i] & 15) * 17);
	memset(&s_tile, 0, sizeof(s_tile));   // set_gfx_region_wide(0, 0, 0): from 0x100; set_thedeep_kludge
	s_tile.gfx16 = &s_gfx16;
	s_tile.base16 = 0x100;
	s_tile.thedeep_kludge = 1;
	memset(&s_spr_gen, 0, sizeof(s_spr_gen));
	s_spr_gen.gfx = &s_gfxspr;
	s_spr_gen.color_base = 0x80;
	mcs51_init(&s_mcu, s_mcu_rom, 0x1000);
	s_mcu.port_in = mcu_in;
	s_mcu.port_out = mcu_out;
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // (YM2203 3 MHz, all outputs 1.0)
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 256;
	yc.irq = ym_irq;
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 512);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		td_exit();
		return -1;
	}
	m6502_init(&s_snd, M6502_R65C02);
	for (int p = 0; p < 8; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_zrom + p * 256;
	s_snd.read = snd_r;
	s_snd.write = snd_w;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = s_rom + p * 256;
	for (int p = 0; p < 0x20; p++)
		s_zmap[0xc0 + p] = s_ram + p * 256;
	s_port_in[0] = rbx_port_find("e008");
	s_port_in[1] = rbx_port_find("e009");
	s_port_dsw[0] = rbx_port_find("e00a");
	s_port_dsw[1] = rbx_port_find("e00b");
	s_port_coins = rbx_port_find("COINS");
	td_reset();
	return 0;
}

static void td_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfx16);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
}

static void run_main(unsigned long end)
{
	if (s_nmi == 1) {   // (the NMI line held: an edge taken once)
		z80_gen_nmi(&s_cpu);
		s_nmi = 2;
	}
	z80_run(&s_cpu, end, &s_irq, 0);
}

static void td_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == 0 && s_nmi_enable && !s_nmi)   // the scanline timer: line 0 the NMI, 127 / 128 the MCU's INT1
			s_nmi = 1;
		if (line == 127)
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 1);
		else if (line == 128)
			mcs51_set_input(&s_mcu, MCS51_INT1_LINE, 0);
		const int cyc = rbx_line_cycles(&s_cpu_frac, CPU_CLOCK, FRAME_MHZ, LINES);
		const int mc = rbx_line_cycles(&s_mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES);
		const unsigned long t0 = s_cpu.cyc;
		for (int k = 0; k < SLICES; k++) {   // (MAME: perfect sync with the MCU)
			run_main(t0 + (unsigned long)(cyc * (k + 1) / SLICES));
			mcs51_execute(&s_mcu, mc * (k + 1) / SLICES - mc * k / SLICES);
		}
		const int sc = rbx_line_cycles(&s_snd_frac, SND_CLOCK, FRAME_MHZ, LINES);
		m6502_run_until(&s_snd, s_snd.cycles + sc);
		s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                          s_mix_cap - s_mix_n);
	}
	vblank_capture();   // (no vblank: the frame's end)
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_frame++;
}

static int td_state(void *ctx, int load)
{
	uint8_t snd[M6502_STATE_SIZE];
	if (!load)
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	z80 z = s_cpu;
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_mcu_frac, s_ym_frac };
	int32_t misc[12] = { (int32_t)s_frame, s_bank, s_flip, s_nmi_enable, s_nmi, s_irq, s_main_to_mcu, s_mcu_to_main,
	                     s_mcu_p2, s_mcu_p3, s_coin_result, s_latch };
	int rc = 0;
	rc |= rbx_state_io(ctx, "z80", &z, sizeof(z));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "text", s_text, sizeof(s_text));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "bac0", &s_tile, offsetof(bac06_t, gfx8));
	rc |= rbx_state_io(ctx, "mcu", &s_mcu.st, sizeof(s_mcu.st));
	rc |= rbx_state_io(ctx, "mcuf", s_mcu.forced_inputs, sizeof(s_mcu.forced_inputs));
	rc |= rbx_state_io(ctx, "m6502", snd, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		z.read_byte = cpu_r;
		z.write_byte = cpu_w;
		z.port_in = cpu_in;
		z.port_out = cpu_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		s_cpu = z;
		memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_mcu_frac = fr[2];
		s_ym_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		set_bank(misc[1]);
		s_flip = misc[2];
		s_nmi_enable = misc[3];
		s_nmi = misc[4];
		s_irq = misc[5];
		s_main_to_mcu = (uint8_t)misc[6];
		s_mcu_to_main = (uint8_t)misc[7];
		s_mcu_p2 = (uint8_t)misc[8];
		s_mcu_p3 = (uint8_t)misc[9];
		s_coin_result = (uint8_t)misc[10];
		s_latch = (uint8_t)misc[11];
	}
	return rc ? -1 : 0;
}

static int td_save(void *ctx) { return td_state(ctx, 0); }
static int td_load(void *ctx) { return td_state(ctx, 1); }
static int td_supported(const char *machine) { return !strcmp(machine, "thedeep"); }

const rbx_driver rbx_drv_thedeep = { "DECO The Deep", td_init, td_exit, td_reset, td_frame, td_save, td_load,
                                     td_supported, td_render, NULL };

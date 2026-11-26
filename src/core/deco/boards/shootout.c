/*  Data East Shoot Out, from MAME src/mame/dataeast/shootout.cpp (BSD-3-Clause, copyright-holders: Ernesto Corvi,
    Phil Stroffolino, Bryan McPhail).
    Shoot Out (World): a DECO 222 at 2 MHz (its NMI held by a coin), a 6502 at 1.5 MHz with the YM2203 (on its IRQ;
    the latch on its NMI; the main CPU waits 200 us after a command). Japan / the Korean bootleg: one 6502 (Korean:
    DECO 222) with the YM2203 (its IRQ the CPU's; its ports: the bank and the flip). A 2 bpp 8x8 background, a 2 bpp
    text layer, 16x16 / 16x32 sprites drawn from the list's end (flickering by a bit on odd frames, behind the text
    by another), the PROM palette.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 2000000        // 12 MHz / 6
#define SND_CLOCK 1500000
#define YM_CLOCK 1500000
#define FRAME_MHZ 59637          // set_raw(12 MHz / 2, 384, 0, 256, 262, 8, 248)
#define LINES 262
#define VIS_Y0 8
#define VIS_H 240
#define VIS_W 256

enum { V_WORLD, V_JAPAN, V_KOREA };
static int s_var;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static uint8_t (*s_ram_hp)[0x4000];              // 0x0000 .. 0x3fff (RAM, sprites, text, background: by the variant's map)
#define s_ram (*s_ram_hp)
static int s_spr, s_txt, s_bg;             // (their s_ram offsets)
static int s_bank, s_flip, s_coin_nmi;
static m6502_t s_cpu;
static rbx_gfx s_gfx8, s_gfxt, s_gfxspr;
static uint32_t s_palette[256];
static uint8_t (*s_pri_buf_hp)[VIS_W * VIS_H];
#define s_pri_buf (*s_pri_buf_hp)
static int s_port_p[2], s_port_dsw[2];
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ym_frac;
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static struct {
	uint8_t spr[0x200], txt[0x800], bg[0x800];
	int flip;
	uint32_t frame;
} *s_snap;

// ---- the World version's sound: 6502, YM2203 ----
static m6502_t s_snd;
static uint8_t s_sram[0x800];
static uint8_t *s_zrom;
static uint32_t s_zrom_size;
static uint8_t s_latch;

static void ym_irq(int state) { m6502_set_irq(s_var == V_WORLD ? &s_snd : &s_cpu, state); }
static uint8_t snd_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x4000 || a == 0x4001) return rbx_ym_read(0, a & 1);
	if (a == 0xa000) {   // sound_cpu_command_r (its NMI cleared)
		m6502_set_nmi(&s_snd, 0);
		return s_latch;
	}
	return 0;
}
static void snd_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x4000 || a == 0x4001) rbx_ym_write(0, a & 1, v);
}

// ---- bus ----
static int vblank(void) { return s_line < VIS_Y0 || s_line >= VIS_Y0 + VIS_H; }
static void set_bank(int b)   // (16 KB banks from 0x8000 of the CPU's region)
{
	s_bank = b & 0x0f;
	const uint32_t o = 0x8000 + (uint32_t)s_bank * 0x4000;
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = o + 0x4000 <= s_rom_size ? s_rom + o + (uint32_t)p * 256 : NULL;
}
static void ym_porta_w(uint8_t v) { set_bank(v); }   // (Japan / Korea)
static void ym_portb_w(uint8_t v) { s_flip = ~v & 1; }
static uint8_t in_r(uint16_t a)   // (0x1000 .. 0x1003)
{
	switch (a) {
	case 0x1000: return (uint8_t)rbx_port_read(s_port_dsw[0]);
	case 0x1001: return (uint8_t)rbx_port_read(s_port_p[0]);
	case 0x1002: return (uint8_t)rbx_port_read(s_port_p[1]);
	case 0x1003:
		rbx_port_custom(s_port_dsw[1], 0x80, vblank() ? 0x80 : 0);
		return (uint8_t)rbx_port_read(s_port_dsw[1]);
	}
	return 0;
}
static uint8_t cpu_r(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x1004 && a < 0x1100) return s_ram[a];   // (the RAM after the inputs on their page)
	if (s_var != V_WORLD && (a == 0x2800 || a == 0x2801)) return rbx_ym_read(0, a & 1);
	return in_r(a);
}
static void cpu_w(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x1004 && a < 0x1100) {
		s_ram[a] = v;
		return;
	}
	if (s_var != V_WORLD) {
		if (a == 0x2800 || a == 0x2801)
			rbx_ym_write(0, a & 1, v);
		return;
	}
	switch (a) {
	case 0x1000: set_bank(v); break;
	case 0x1001: s_flip = ~v & 1; break;
	case 0x1003:   // sound_cpu_command_w: the latch, its NMI; the main CPU spins 200 us
		s_latch = v;
		m6502_set_nmi(&s_snd, 1);
		s_cpu.cycles += CPU_CLOCK / 5000;
		break;
	}
}
static void coin_update(void)   // (the coins, active high: the main CPU's NMI while one is in)
{
	const int c = (rbx_port_read(s_port_p[1]) & 0xc0) != 0;
	if (c != s_coin_nmi) {
		s_coin_nmi = c;
		m6502_set_nmi(&s_cpu, c);
	}
}

// ---- video ----
static void spr_blit(uint32_t code, int fx, int fy, int sx, int sy, uint32_t pmask, uint16_t *bm, int pitch)
{
	if (!s_gfxspr.count)
		return;
	code %= s_gfxspr.count;
	pmask |= 1u << 31;
	const uint8_t *src = s_gfxspr.pix + code * 256;
	for (int y = 0; y < 16; y++) {
		const int dy = sy + y - VIS_Y0;
		if (dy < 0 || dy >= VIS_H)
			continue;
		const uint8_t *row = src + (fy ? 15 - y : y) * 16;
		for (int x = 0; x < 16; x++) {
			const int dx = sx + x;
			if (dx < 0 || dx >= VIS_W)
				continue;
			const uint8_t p = row[fx ? 15 - x : x];
			if (!p)
				continue;
			uint8_t *pr = &s_pri_buf[dy * VIS_W + dx];
			if (!((1u << (*pr & 0x1f)) & pmask))
				bm[dy * pitch + dx] = (uint16_t)(64 + p);
			*pr = 31;
		}
	}
}

static void so_render(void)
{
	memcpy(rbx_video.palette, s_palette, sizeof(s_palette));
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch, flip = s_snap->flip;
	const uint8_t *bg = s_snap->bg, *txt = s_snap->txt;
	for (int y = 0; y < VIS_H; y++) {   // the background (opaque) and the text (pen 0 clear, priority 1)
		uint16_t *d = bm + y * pitch;
		uint8_t *pr = s_pri_buf + y * VIS_W;
		for (int x = 0; x < VIS_W; x++) {
			int px = x, py = y + VIS_Y0;
			if (flip) {
				px = 255 - px;
				py = 255 - py;
			}
			const int idx = (py >> 3) * 32 + (px >> 3);
			int attr = bg[0x400 + idx];
			uint32_t code = s_gfxt.count ? rbx_wrap((uint32_t)(bg[idx] + 256 * (attr & 7)), s_gfxt.count) : 0;
			uint8_t p = s_gfxt.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			d[x] = (uint16_t)((attr >> 4) * 4 + p);
			pr[x] = 0;
			attr = txt[0x400 + idx];
			code = s_gfx8.count ? rbx_wrap((uint32_t)(txt[idx] + 256 * (attr & 3)), s_gfx8.count) : 0;
			p = s_gfx8.pix[(code * 8 + (uint32_t)(py & 7)) * 8 + (uint32_t)(px & 7)];
			if (p) {
				d[x] = (uint16_t)(128 + (attr >> 4) * 4 + p);
				pr[x] = 1;
			}
		}
	}
	const int bank_bits = s_var == V_WORLD ? 3 : 2, flicker = (s_snap->frame & 1) != 0;
	for (int n = 127; n >= 0; n--) {   // draw_sprites (from the list's end)
		const uint8_t *s = s_snap->spr + n * 4;
		const int attr = s[1];
		if (!(attr & 1) || (!flicker && (attr & 2)))
			continue;
		const uint32_t pmask = (attr & 0x08) ? 0x2 : 0;   // (behind the text)
		int sx = (240 - s[2]) & 0xff, sy = (240 - s[0]) & 0xff;
		uint32_t number = (uint32_t)(s[3] | ((attr << bank_bits) & 0x700));
		const int fx = flip ? !(attr & 4) : (attr & 4) != 0, fy = flip;
		if (attr & 0x10) {   // double height
			number &= ~1u;
			sy -= 16;
			spr_blit(number, fx, fy, flip ? 240 - sx : sx, flip ? 240 - sy : sy, pmask, bm, pitch);
			number++;
			sy += 16;
		}
		spr_blit(number, fx, fy, flip ? 240 - sx : sx, flip ? 240 - sy : sy, pmask, bm, pitch);
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->spr, s_ram + s_spr, sizeof(s_snap->spr));
	memcpy(s_snap->txt, s_ram + s_txt, sizeof(s_snap->txt));
	memcpy(s_snap->bg, s_ram + s_bg, sizeof(s_snap->bg));
	s_snap->flip = s_flip;
	s_snap->frame = s_frame;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void so_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	s_flip = s_coin_nmi = 0;
	set_bank(0);
	m6502_reset(&s_cpu);
	m6502_set_irq(&s_cpu, 0);
	m6502_set_nmi(&s_cpu, 0);
	if (s_var == V_WORLD) {
		memset(s_sram, 0, sizeof(s_sram));
		s_latch = 0;
		m6502_reset(&s_snd);
		m6502_set_irq(&s_snd, 0);
		m6502_set_nmi(&s_snd, 0);
	}
	rbx_ym_reset(0);
	s_mix_n = 0;
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ym_frac = 0;
}

static void so_exit(void);

static int so_init(const rbx_game *g)
{
	if (!s_pri_buf_hp && !(s_pri_buf_hp = calloc(1, sizeof(*s_pri_buf_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_ram_hp && !(s_ram_hp = calloc(1, sizeof(*s_ram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_var = !strcmp(g->machine, "shootoutj") ? V_JAPAN : !strcmp(g->machine, "shootoutk") ? V_KOREA : V_WORLD;
	uint32_t csize, tsize, ssize, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = s_var == V_WORLD ? rbx_region("audiocpu", &s_zrom_size) : NULL;
	uint8_t *chars = rbx_region("chars", &csize), *tiles = rbx_region("tiles", &tsize), *spr = rbx_region("sprites", &ssize),
	        *prom = rbx_region("proms", &psize);
	if (!s_rom || s_rom_size < 0x14000 || (s_var == V_WORLD && (!s_zrom || s_zrom_size < 0x4000)) || !chars ||
	    csize < 0x4000 || !tiles || tsize < 0x8000 || !spr || ssize < 0x30000 || !prom || psize < 0x100) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	{	// char_layout (1024, 2 bpp in the nibbles, the left half 0x2000 on), tile_layout (2048, the left half 0x4000
		// on), sprite_layout (2048, 3 bpp: planes 0, 0x10000, 0x20000)
		static uint32_t xc[8], xt[8], y8[8], xs[16], y16[16];
		for (int i = 0; i < 4; i++) {
			xc[i] = (uint32_t)(0x2000 * 8 + i);
			xc[4 + i] = (uint32_t)i;
			xt[i] = (uint32_t)(0x4000 * 8 + i);
			xt[4 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 8; i++) {
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)(128 + i);
			xs[8 + i] = (uint32_t)i;
		}
		for (int i = 0; i < 16; i++)
			y16[i] = (uint32_t)(i * 8);
		static const uint32_t p2[2] = { 0, 4 }, ps[3] = { 0, 0x10000 * 8, 2 * 0x10000 * 8 };
		const rbx_gfx_layout lc = { 8, 8, 2, p2, xc, y8, 8 * 8 }, lt = { 8, 8, 2, p2, xt, y8, 8 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 3, ps, xs, y16, 32 * 8 };
		if (rbx_gfx_decode(&s_gfx8, &lc, chars, 1024 * 8) || rbx_gfx_decode(&s_gfxt, &lt, tiles, 2048 * 8) ||
		    rbx_gfx_decode(&s_gfxspr, &ls, spr, 2048 * 32)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			so_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++) {   // palette: 3-3-2 (red / green 0x21, 0x47, 0x97; blue 0x52, 0xad)
		const int c = prom[i];
		const int r = 0x21 * (c & 1) + 0x47 * ((c >> 1) & 1) + 0x97 * ((c >> 2) & 1);
		const int gg = 0x21 * ((c >> 3) & 1) + 0x47 * ((c >> 4) & 1) + 0x97 * ((c >> 5) & 1);
		const int b = 0x52 * ((c >> 6) & 1) + 0xad * ((c >> 7) & 1);
		s_palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_snap = calloc(1, sizeof(*s_snap));
	rbx_ym_config yc;   // (YM2203 1.5 MHz, all outputs 1.0)
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 256;
	yc.irq = ym_irq;
	if (s_var != V_WORLD) {
		yc.port_a_write = ym_porta_w;
		yc.port_b_write = ym_portb_w;
	}
	int rc = !s_snap || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1);
	if (!rc) {
		s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
		s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
		rc = !s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 256);
	}
	if (rc) {
		rbx_fail("%s: out of memory", g->set);
		so_exit();
		return -1;
	}
	if (s_var == V_WORLD) {
		m6502_init(&s_snd, M6502_NMOS);
		for (int p = 0; p < 8; p++)
			s_snd.rpage[p] = s_snd.wpage[p] = s_sram + p * 256;
		for (int p = 0xc0; p < 0x100; p++)   // (0xd000: also an unknown write)
			s_snd.rpage[p] = s_zrom + (p - 0xc0) * 256;
		s_snd.read = snd_r;
		s_snd.write = snd_w;
	}
	m6502_init(&s_cpu, M6502_NMOS);
	s_cpu.deco222 = s_var != V_JAPAN;
	// main_map (World): RAM 0x0000, I/O 0x1000, RAM, sprites 0x1800, text 0x2000, background 0x2800; Japan / Korea:
	// RAM, I/O, sprites 0x2000, the YM2203 0x2800, text 0x3000, background 0x3800
	s_spr = s_var == V_WORLD ? 0x1800 : 0x2000;
	s_txt = s_var == V_WORLD ? 0x2000 : 0x3000;
	s_bg = s_var == V_WORLD ? 0x2800 : 0x3800;
	for (int p = 0; p < 0x40; p++) {
		int ram = p < 0x10 || (p >= 0x11 && p < 0x18);
		if (s_var == V_WORLD)
			ram = ram || (p >= 0x18 && p < 0x1a) || (p >= 0x20 && p < 0x30);
		else
			ram = ram || (p >= 0x20 && p < 0x22) || p >= 0x30;
		if (ram)
			s_cpu.rpage[p] = s_cpu.wpage[p] = s_ram + p * 256;
	}
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + (p - 0x80) * 256;
	s_cpu.read = cpu_r;
	s_cpu.write = cpu_w;
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	so_reset();
	return 0;
}

static void so_exit(void)
{
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gfx8);
	rbx_gfx_free(&s_gfxt);
	rbx_gfx_free(&s_gfxspr);
	free(s_snap);
	s_snap = NULL;
	free(s_pri_buf_hp);
	s_pri_buf_hp = NULL;
	free(s_ram_hp);
	s_ram_hp = NULL;
}

static void so_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	coin_update();
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H)
			vblank_capture();
		s_cpu_frac += (uint64_t)CPU_CLOCK * 1000;
		const uint64_t den = (uint64_t)FRAME_MHZ * LINES;
		const int64_t cyc = (int64_t)(s_cpu_frac / den);
		s_cpu_frac -= (uint64_t)cyc * den;
		m6502_run_until(&s_cpu, s_cpu.cycles + cyc);
		if (s_var == V_WORLD) {
			s_snd_frac += (uint64_t)SND_CLOCK * 1000;
			const int64_t sc = (int64_t)(s_snd_frac / den);
			s_snd_frac -= (uint64_t)sc * den;
			m6502_run_until(&s_snd, s_snd.cycles + sc);
		}
		s_mix_n += rbx_ym_advance(0, (uint32_t)rbx_line_cycles(&s_ym_frac, YM_CLOCK, FRAME_MHZ, LINES), s_mix + s_mix_n,
		                          s_mix_cap - s_mix_n);
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
	s_frame++;
}

static int so_state(void *ctx, int load)
{
	uint8_t cpu[M6502_STATE_SIZE], snd[M6502_STATE_SIZE];
	if (!load) {
		memcpy(cpu, &s_cpu, M6502_STATE_SIZE);
		memcpy(snd, &s_snd, M6502_STATE_SIZE);
	}
	uint64_t fr[3] = { s_cpu_frac, s_snd_frac, s_ym_frac };
	int32_t misc[5] = { (int32_t)s_frame, s_bank, s_flip, s_coin_nmi, s_latch };
	int rc = 0;
	rc |= rbx_state_io(ctx, "m6502", cpu, M6502_STATE_SIZE);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	if (s_var == V_WORLD) {
		rc |= rbx_state_io(ctx, "m6502s", snd, M6502_STATE_SIZE);
		rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	}
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		memcpy(&s_cpu, cpu, M6502_STATE_SIZE);
		if (s_var == V_WORLD)
			memcpy(&s_snd, snd, M6502_STATE_SIZE);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ym_frac = fr[2];
		s_frame = (uint32_t)misc[0];
		set_bank(misc[1]);
		s_flip = misc[2];
		s_coin_nmi = misc[3];
		s_latch = (uint8_t)misc[4];
	}
	return rc ? -1 : 0;
}

static int so_save(void *ctx) { return so_state(ctx, 0); }
static int so_load(void *ctx) { return so_state(ctx, 1); }
static int so_supported(const char *machine)
{
	return !strcmp(machine, "shootout") || !strcmp(machine, "shootoutj") || !strcmp(machine, "shootoutk");
}

const rbx_driver rbx_drv_shootout = { "DECO Shoot Out", so_init, so_exit, so_reset, so_frame, so_save, so_load,
                                      so_supported, so_render, NULL };

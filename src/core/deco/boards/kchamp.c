/*  Data East Karate Champ (one player and Player Vs Player). Ported from MAME src/mame/dataeast/kchamp.cpp,
    kchamp_v.cpp (BSD-3-Clause, copyright-holders: Ernesto Corvi).
    A Z80 at 3 MHz (the VS version: its opcodes encrypted, a decrypted copy) with an LS259 (flip, the vblank NMI's
    enable, the VS sound CPU's reset), a 32x32 map of 8x8 tiles, 64 sprites from three banks, a 256-colour PROM. The
    sound: a Z80 (one player: 4 MHz, an NMI at ~122 Hz, two AY-3-8910 at 1 MHz and a DAC; VS: 3 MHz, two AY-3-8910 at
    1.5 MHz and an MSM5205 fed through an LS157 nibble selector, its NMI on every other sample).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/msm5205u.h"
#include "../../video/gfxlayout.h"

#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 59186          // set_raw(12 MHz / 2, 384, 0, 256, 264, 16, 240)
#define LINES 264
#define VIS_Y0 16
#define VIS_H 224

static int s_vs;
static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static uint8_t (*s_opcodes_hp)[0x10000];
#define s_opcodes (*s_opcodes_hp)
static uint8_t s_ram[0x2000], s_ram2[0x1500], s_vram[0x400], s_cram[0x400], s_spriteram[0x100];
static uint8_t (*s_sram_hp)[0xa000];             // (VS: 0x6000-0xffff; one player: 0xe000-0xe2ff)
#define s_sram (*s_sram_hp)
static uint8_t s_latchbits;                // the LS259's outputs
static int s_nmi_line, s_snd_nmi_enable, s_snd_nmi_line, s_snd_reset, s_lo_nibble;
static uint8_t s_slatch, s_adpcm;
static int s_slatch_pending;
static int32_t s_dac;
static int s_port_p1, s_port_p2, s_port_sys, s_port_dsw;
static int s_line;
static uint32_t s_frame;
static uint64_t s_cpu_frac, s_snd_frac, s_ay_frac, s_nmi_frac;
static z80 s_cpu, s_snd;
static ay8910_t s_ay[2];
static msm5205u_t s_msm;
static int32_t *s_mix;
static int s_mix_cap;
static uint32_t s_ay_clock, s_snd_clock, s_rate;
static uint32_t s_rgb[256];
static rbx_gfx s_gfxtile, s_gfxspr[3];
static const uint8_t *s_map[256], *s_smap[256];
static struct {
	uint8_t vram[0x400], cram[0x400], spriteram[0x100], flip;
} *s_snap;

// ---- sound ----
static void msm_data(void) { msm5205u_data_w(&s_msm, s_lo_nibble ? s_adpcm >> 4 : s_adpcm & 15); }   // (LS157)
static void msm_vck(msm5205u_t *m, void *u)   // msmint: the nibble select toggled, the NMI on every other sample
{
	(void)m;
	(void)u;
	s_lo_nibble = !s_lo_nibble;
	msm_data();
	if (s_lo_nibble && s_snd_nmi_enable && !s_snd_nmi_line) {
		s_snd_nmi_line = 1;
		z80_gen_nmi(&s_snd);
	}
}
static void sound_control_w(uint8_t v)   // (VS)
{
	msm5205u_reset_w(&s_msm, !(v & 1));
	s_snd_nmi_enable = (v >> 1) & 1;
	if (!s_snd_nmi_enable)
		s_snd_nmi_line = 0;
}

static uint8_t snd_rd(void *u, uint16_t a)
{
	(void)u;
	if (s_vs)
		return a < 0x6000 ? s_srom[a] : s_sram[a - 0x6000];
	if (a < 0xe000) return s_srom[a];
	return a < 0xe300 ? s_sram[a - 0xe000] : 0;
}
static void snd_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_vs) {
		if (a >= 0x6000) s_sram[a - 0x6000] = v;
	} else if (a >= 0xe000 && a < 0xe300)
		s_sram[a - 0xe000] = v;
}
static uint8_t snd_in(z80 *z, uint8_t p)
{
	(void)z;
	if ((s_vs && p == 0x01) || (!s_vs && p == 0x06)) {   // the latch (its pending flag: the IRQ)
		s_slatch_pending = 0;
		return s_slatch;
	}
	return 0xff;
}
static void snd_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 4)   // data_address_w: odd the address
		(p & 1) ? ay8910_address_w(&s_ay[p >> 1], v) : ay8910_data_w(&s_ay[p >> 1], v);
	else if (s_vs) {
		if (p == 4) { s_adpcm = v; msm_data(); }   // the LS157's inputs
		else if (p == 5) sound_control_w(v);
	} else if (p == 4)
		s_dac = ((int32_t)v - 128) * 256;
	else if (p == 5) {   // kc_sound_control_w (offset 0): the NMI's enable
		s_snd_nmi_enable = (v >> 7) & 1;
		if (!s_snd_nmi_enable)
			s_snd_nmi_line = 0;
	}
}
static void snd_hook(z80 *z)
{
	z->read_byte = snd_rd;
	z->write_byte = snd_wr;
	z->port_in = snd_in;
	z->port_out = snd_out;
	z->userdata = NULL;
	z->read_map = s_smap;
	z->int_data = 0xff;
}
static void snd_cpu_reset(void)
{
	z80_init(&s_snd);
	snd_hook(&s_snd);
}

// ---- main ----
static uint8_t main_rd(void *u, uint16_t a)
{
	(void)u;
	if (s_vs) {
		if (a < 0xc000 || a >= 0xe000) return s_rom[a];
		if (a < 0xd000) return s_ram[a - 0xc000];
		if (a < 0xd400) return s_vram[a - 0xd000];
		if (a < 0xd800) return s_cram[a - 0xd400];
		if (a < 0xd900) return s_spriteram[a - 0xd800];
		return s_ram2[a - 0xd900];
	}
	if (a < 0xc000) return s_rom[a];
	if (a < 0xe000) return s_ram[a - 0xc000];
	if (a < 0xe400) return s_vram[a - 0xe000];
	if (a < 0xe800) return s_cram[a - 0xe400];
	if (a >= 0xea00 && a < 0xeb00) return s_spriteram[a - 0xea00];
	if (a >= 0xeb00) return s_ram2[a - 0xeb00];
	return 0;
}
static void main_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (s_vs) {
		if (a >= 0xc000 && a < 0xd000) s_ram[a - 0xc000] = v;
		else if (a >= 0xd000 && a < 0xd400) s_vram[a - 0xd000] = v;
		else if (a >= 0xd400 && a < 0xd800) s_cram[a - 0xd400] = v;
		else if (a >= 0xd800 && a < 0xd900) s_spriteram[a - 0xd800] = v;
		else if (a >= 0xd900 && a < 0xe000) s_ram2[a - 0xd900] = v;
		return;
	}
	if (a >= 0xc000 && a < 0xe000) s_ram[a - 0xc000] = v;
	else if (a >= 0xe000 && a < 0xe400) s_vram[a - 0xe000] = v;
	else if (a >= 0xe400 && a < 0xe800) s_cram[a - 0xe400] = v;
	else if (a >= 0xea00 && a < 0xeb00) s_spriteram[a - 0xea00] = v;
	else if (a >= 0xeb00) s_ram2[a - 0xeb00] = v;
}

static void latch259_w(int bit, int state)   // the LS259 (write_d0)
{
	const int old = (s_latchbits >> bit) & 1;
	s_latchbits = (uint8_t)((s_latchbits & ~(1 << bit)) | (state << bit));
	if (bit == 1 && !state)   // nmi_enable_w: off clears the NMI
		s_nmi_line = 0;
	if (bit == 2 && s_vs && old != state) {   // sound_reset_w
		s_snd_reset = !state;
		if (!state) {
			ay8910_reset(&s_ay[0]);
			ay8910_reset(&s_ay[1]);
			sound_control_w(0);
		} else
			snd_cpu_reset();
	}
}

static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)z;
	if (s_vs) {
		switch (p) {
		case 0x00: return (uint8_t)rbx_port_read(s_port_p1);
		case 0x40: return (uint8_t)rbx_port_read(s_port_p2);
		case 0x80: return (uint8_t)rbx_port_read(s_port_sys);
		case 0xc0: return (uint8_t)rbx_port_read(s_port_dsw);
		}
		return 0xff;
	}
	switch (p) {
	case 0x80: return (uint8_t)rbx_port_read(s_port_dsw);
	case 0x90: return (uint8_t)rbx_port_read(s_port_p1);
	case 0x98: return (uint8_t)rbx_port_read(s_port_p2);
	case 0xa0: return (uint8_t)rbx_port_read(s_port_sys);
	case 0xa8: snd_cpu_reset(); return 0;   // sound_reset_r: a reset pulse
	}
	return 0xff;
}
static void main_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (s_vs) {
		if (p < 8) latch259_w(p, v & 1);
		else if (p == 0x40) { s_slatch = v; s_slatch_pending = 1; }
		return;
	}
	if (p >= 0x80 && p < 0x88) latch259_w(p - 0x80, v & 1);
	else if (p == 0xa8) { s_slatch = v; s_slatch_pending = 1; }
}
static void main_hook(z80 *z)
{
	z->read_byte = main_rd;
	z->write_byte = main_wr;
	z->port_in = main_in;
	z->port_out = main_out;
	z->userdata = NULL;
	z->read_map = s_map;
	z->op_rom = s_vs ? s_opcodes : NULL;
	z->int_data = 0xff;
}

// ---- video ----
static void kc_render(void)
{
	for (int i = 0; i < 256; i++)
		rbx_video.palette[i] = s_rgb[i];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int flip = s_snap->flip;
	for (int y = 0; y < VIS_H; y++) {   // the map (TILEMAP_SCAN_ROWS, opaque; flip: mirrored)
		const int sy = y + VIS_Y0, ty = flip ? 255 - sy : sy;
		uint16_t *d = bm + y * pitch;
		for (int x = 0; x < 256; x++) {
			const int tx = flip ? 255 - x : x;
			const int idx = (ty >> 3) * 32 + (tx >> 3);
			const int code = s_snap->vram[idx] + ((s_snap->cram[idx] & 7) << 8), color = (s_snap->cram[idx] >> 3) & 0x1f;
			d[x] = (uint16_t)(128 + color * 4 + s_gfxtile.pix[(size_t)rbx_wrap(code, s_gfxtile.count) * 64 + (ty & 7) * 8 + (tx & 7)]);
		}
	}
	const int dx = s_vs ? 0 : -8, dy = s_vs ? 0 : 7;
	const uint8_t *sp = s_snap->spriteram;
	for (int offs = 0; offs < 0x100; offs += 4) {   // draw_sprites
		const int attr = sp[offs + 2], bank = (attr & 0x60) >> 5;
		if (bank > 2)
			continue;
		const int code = sp[offs + 1] + ((attr & 0x10) << 4), color = attr & 0x0f;
		int fx = 0, fy = (attr & 0x80) != 0;
		int sx = sp[offs + 3] + dx, sy = 240 - sp[offs] + dy;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
		}
		const rbx_gfx *g = &s_gfxspr[bank];
		const uint8_t *src = g->pix + (size_t)(code % g->count) * 256;
		for (int yy = 0; yy < 16; yy++) {
			const int y = sy + yy - VIS_Y0;
			if (y < 0 || y >= VIS_H)
				continue;
			for (int xx = 0; xx < 16; xx++) {
				const int x = sx + xx;
				if (x < 0 || x > 255)
					continue;
				const uint8_t p = src[(fy ? 15 - yy : yy) * 16 + (fx ? 15 - xx : xx)];
				if (p)
					bm[y * pitch + x] = (uint16_t)(color * 4 + p);
			}
		}
	}
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->vram, s_vram, sizeof(s_vram));
	memcpy(s_snap->cram, s_cram, sizeof(s_cram));
	memcpy(s_snap->spriteram, s_spriteram, sizeof(s_spriteram));
	s_snap->flip = s_latchbits & 1;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void kc_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_ram2, 0, sizeof(s_ram2));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_spriteram, 0, sizeof(s_spriteram));
	memset(s_sram, 0, sizeof(s_sram));
	s_latchbits = 0;
	s_nmi_line = s_snd_nmi_enable = s_snd_nmi_line = 0;
	s_snd_reset = s_vs;   // (VS: the LS259 clear holds the sound CPU in reset)
	s_lo_nibble = 1;      // (init_kchampvs)
	s_slatch = s_adpcm = 0;
	s_slatch_pending = 0;
	s_dac = 0;
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	msm5205u_reset(&s_msm);
	if (s_vs)
		msm5205u_reset_w(&s_msm, 1);
	z80_init(&s_cpu);
	main_hook(&s_cpu);
	snd_cpu_reset();
	s_line = 0;
	s_frame = 0;
	s_cpu_frac = s_snd_frac = s_ay_frac = s_nmi_frac = 0;
}

static void kc_exit(void);

static int kc_init(const rbx_game *g)
{
	if (!s_opcodes_hp && !(s_opcodes_hp = calloc(1, sizeof(*s_opcodes_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_sram_hp && !(s_sram_hp = calloc(1, sizeof(*s_sram_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	s_vs = !strcmp(g->machine, "kchampvs");
	uint32_t g1s, g2s, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	uint8_t *gfx1 = rbx_region("gfx1", &g1s), *gfx2 = rbx_region("gfx2", &g2s);
	const uint8_t *prom = rbx_region("proms", &ps);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || s_srom_size < 0x10000 || !gfx1 || g1s < 0x8000 || !gfx2 ||
	    g2s < 0x18000 || !prom || ps < 0x300) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	if (s_vs) {   // decrypt_code (and init_kchampvs: the first opcodes left plain)
		for (int a = 0; a < 0x10000; a++)
			s_opcodes[a] = (uint8_t)((s_rom[a] & 0x55) | ((s_rom[a] & 0x88) >> 2) | ((s_rom[a] & 0x22) << 2));
		if (strcmp(g->set, "kchampvs2")) {
			uint8_t *rom = s_rom;
			s_opcodes[0] = rom[0];
			int a = rom[1] + 256 * rom[2];
			s_opcodes[a] = rom[a];
			rom[a + 1] ^= 0xee;
			a = rom[a + 1] + 256 * rom[a + 2];
			s_opcodes[a] = rom[a];
			a += 2;
			s_opcodes[a] = rom[a];
		}
	}
	{	// tilelayout (2048 tiles, planes 0x4000 / 0), spritelayout (512 a bank, planes 0xc000 / 0, halves 0x2000 apart)
		static uint32_t x8[8], y8[8], xs[16], ys[16];
		for (int i = 0; i < 8; i++) {
			x8[i] = (uint32_t)i;
			y8[i] = (uint32_t)(i * 8);
			xs[i] = (uint32_t)i;
			xs[8 + i] = (uint32_t)(0x2000 * 8 + i);
		}
		for (int i = 0; i < 16; i++)
			ys[i] = (uint32_t)(i * 8);
		const uint32_t pt[2] = { 0x4000 * 8, 0 }, psp[2] = { 0xc000 * 8, 0 };
		const rbx_gfx_layout lt = { 8, 8, 2, pt, x8, y8, 8 * 8 }, ls = { 16, 16, 2, psp, xs, ys, 16 * 8 };
		int rc = rbx_gfx_decode(&s_gfxtile, &lt, gfx1, 2048 * 8);
		static const uint32_t off[3] = { 0x8000, 0x4000, 0x0000 };   // (gfx banks 1, 2, 3)
		for (int b = 0; b < 3 && !rc; b++)
			rc = rbx_gfx_decode(&s_gfxspr[b], &ls, gfx2 + off[b], 512 * 16);
		if (rc) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			kc_exit();
			return -1;
		}
	}
	for (int i = 0; i < 256; i++)   // kchamp_palette: 4 bits each from three PROMs
		s_rgb[i] = (uint32_t)((prom[i] & 15) * 17) << 16 | (uint32_t)((prom[256 + i] & 15) * 17) << 8 |
		           (uint32_t)((prom[512 + i] & 15) * 17);
	s_snap = calloc(1, sizeof(*s_snap));
	s_ay_clock = s_vs ? 1500000 : (!strcmp(g->machine, "kchamp_arfyc") ? 8867238 / 8 : 1000000);
	s_snd_clock = s_vs ? 3000000 : (!strcmp(g->machine, "kchamp_arfyc") ? 8867238 / 2 : 4000000);
	for (int i = 0; i < 2; i++)   // (0.3 each, legacy output)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, s_ay_clock, AY8910_LEGACY_OUTPUT, 77, 1, 0);
	msm5205u_init(&s_msm, 375000, MSM5205_S96_4B, msm_vck, NULL);
	s_rate = ay8910_rate(&s_ay[0]);
	while (s_rate > 60000)
		s_rate /= 2;
	s_mix_cap = (int)(ay8910_rate(&s_ay[0]) / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_snap || !s_mix || rbx_audio_setup(s_rate) || rbx_video_setup(256, VIS_H, 256)) {
		rbx_fail("%s: out of memory", g->set);
		kc_exit();
		return -1;
	}
	for (int i = 0; i < 2; i++)   // (the output at a rate under 60 kHz)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, s_ay_clock, AY8910_LEGACY_OUTPUT, 77,
		            (int)(ay8910_rate(&s_ay[i]) / s_rate), 0);
	memset(s_map, 0, sizeof(s_map));
	memset(s_smap, 0, sizeof(s_smap));
	for (int p = 0; p < 256; p++) {
		if (p < 0xc0 || (s_vs && p >= 0xe0))
			s_map[p] = s_rom + p * 256;
		if (s_vs ? p < 0x60 : p < 0xe0)
			s_smap[p] = s_srom + p * 256;
	}
	s_port_p1 = rbx_port_find("P1");
	s_port_p2 = rbx_port_find("P2");
	s_port_sys = rbx_port_find("SYSTEM");
	s_port_dsw = rbx_port_find("DSW");
	kc_reset();
	return 0;
}

static void kc_exit(void)
{
	rbx_gfx_free(&s_gfxtile);
	for (int b = 0; b < 3; b++)
		rbx_gfx_free(&s_gfxspr[b]);
	free(s_snap);
	s_snap = NULL;
	free(s_mix);
	s_mix = NULL;
	free(s_opcodes_hp);
	s_opcodes_hp = NULL;
	free(s_sram_hp);
	s_sram_hp = NULL;
}

static void kc_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_Y0 + VIS_H) {   // vblank_irq: the NMI when enabled
			vblank_capture();
			if ((s_latchbits & 2) && !s_nmi_line) {
				s_nmi_line = 1;
				z80_gen_nmi(&s_cpu);
			}
		}
		const int cyc = rbx_line_cycles(&s_cpu_frac, 3000000, FRAME_MHZ, LINES);
		int noirq = 0;
		z80_run(&s_cpu, s_cpu.cyc + (unsigned long)cyc, &noirq, 0);
		const int scyc = rbx_line_cycles(&s_snd_frac, s_snd_clock, FRAME_MHZ, LINES);
		if (!s_vs) {   // sound_nmi every 0x10000 clocks of 8 MHz
			s_nmi_frac += (uint64_t)8000000 * 1000;
			const uint64_t per = (uint64_t)0x10000 * FRAME_MHZ * LINES;
			if (s_nmi_frac >= per) {
				s_nmi_frac -= per;
				if (s_snd_nmi_enable && !s_snd_nmi_line) {
					s_snd_nmi_line = 1;
					z80_gen_nmi(&s_snd);
				}
			}
		}
		if (!s_snd_reset)
			z80_run(&s_snd, s_snd.cyc + (unsigned long)scyc, &s_slatch_pending, 0);
		else
			s_snd.cyc += (unsigned long)scyc;
		const uint32_t clocks = (uint32_t)rbx_line_cycles(&s_ay_frac, s_ay_clock, FRAME_MHZ, LINES);
		const int n0 = ay8910_render(&s_ay[0], clocks, s_mix, s_mix_cap);
		const int n1 = ay8910_render(&s_ay[1], clocks, s_mix, s_mix_cap);
		const int n = n0 > n1 ? n0 : n1;
		if (s_vs)
			msm5205u_render(&s_msm, s_mix, n, s_rate, 512);   // (gain 1.0: full scale 16384 x 2)
		for (int k = 0; k < n; k++) {
			rbx_audio_put(s_mix[k] + (s_vs ? 0 : s_dac * 3 / 10));
			s_mix[k] = 0;
		}
	}
	s_frame++;
}

static int kc_state(void *ctx, int load)
{
	z80 c = s_cpu, sn = s_snd;
	uint64_t fr[4] = { s_cpu_frac, s_snd_frac, s_ay_frac, s_nmi_frac };
	int32_t misc[14] = { (int32_t)s_frame, s_latchbits, s_nmi_line, s_snd_nmi_enable, s_snd_nmi_line, s_snd_reset,
	                     s_lo_nibble, s_slatch, s_adpcm, s_slatch_pending, s_dac, 0, 0, 0 };
	uint8_t ay[2][256];
	const uint32_t ays = ay8910_state_size();
	if (!load)
		for (int i = 0; i < 2; i++)
			memcpy(ay[i], &s_ay[i], ays);
	int rc = 0;
	rc |= rbx_state_io(ctx, "cpu", &c, sizeof(c));
	rc |= rbx_state_io(ctx, "snd", &sn, sizeof(sn));
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "ram2", s_ram2, sizeof(s_ram2));
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "spr", s_spriteram, sizeof(s_spriteram));
	rc |= rbx_state_io(ctx, "sram", s_sram, sizeof(s_sram));
	rc |= rbx_state_io(ctx, "ay", ay, sizeof(ay));
	rc |= rbx_state_io(ctx, "msm", &s_msm, MSM5205_STATE_SIZE);
	rc |= rbx_state_io(ctx, "frac", fr, sizeof(fr));
	rc |= rbx_state_io(ctx, "misc", misc, sizeof(misc));
	if (!rc && load) {
		main_hook(&c);
		snd_hook(&sn);
		s_cpu = c;
		s_snd = sn;
		for (int i = 0; i < 2; i++)
			memcpy(&s_ay[i], ay[i], ays);
		s_cpu_frac = fr[0];
		s_snd_frac = fr[1];
		s_ay_frac = fr[2];
		s_nmi_frac = fr[3];
		s_frame = (uint32_t)misc[0];
		s_latchbits = (uint8_t)misc[1];
		s_nmi_line = misc[2];
		s_snd_nmi_enable = misc[3];
		s_snd_nmi_line = misc[4];
		s_snd_reset = misc[5];
		s_lo_nibble = misc[6];
		s_slatch = (uint8_t)misc[7];
		s_adpcm = (uint8_t)misc[8];
		s_slatch_pending = misc[9];
		s_dac = misc[10];
	}
	return rc ? -1 : 0;
}

static int kc_save(void *ctx) { return kc_state(ctx, 0); }
static int kc_load(void *ctx) { return kc_state(ctx, 1); }
static int kc_supported(const char *m)
{
	return !strcmp(m, "kchamp") || !strcmp(m, "kchamp_arfyc") || !strcmp(m, "kchampvs");
}

const rbx_driver rbx_drv_kchamp = { "DECO Karate Champ", kc_init, kc_exit, kc_reset, kc_frame, kc_save, kc_load,
                                    kc_supported, kc_render, NULL };

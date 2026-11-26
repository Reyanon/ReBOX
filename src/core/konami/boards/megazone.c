/*  Konami Mega Zone. Ported from MAME src/mame/konami/megazone.cpp (BSD-3-Clause, copyright-holders: Chris Hardy) and
    the palette's resistor networks of src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud). A
    KONAMI1 at 1.536 MHz, a Z80 at 3.072 MHz (the inputs, an AY-3-8910, 2 KB shared with the main CPU) and an I8039 at
    7.16 MHz on a DAC, all line by line on the main thread; a scrolled 32x32 tilemap, the sprites, the 6-column score
    overlay. The AY's RC filters (port B) not done; the flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/filter_rc.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12)
#define Z80_CLOCK 3072000u     // (18.432 MHz / 6)
#define MCS_CYCLES 477273u     // (the I8039 at 14.318 MHz / 2: 15 clocks a machine cycle)
#define AY_CLOCK 1789772u      // (14.318 MHz / 8)
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0, 288, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_W 288
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200

static uint8_t *s_rom, *s_zrom, *s_drom, *s_prom;
static uint32_t s_rom_size, s_zrom_size, s_drom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static mcs48 s_mcs;
static ay8910_t s_ay;
static int s_port_in[3], s_port_dsw[2];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;

static struct {
	uint8_t ram[0x1400];     // 2000-33ff (2000 / 2400 the video RAMs, 2800 / 2c00 the colour RAMs, 3000 the sprites)
	uint8_t shared[0x800];   // main 3800-3fff, Z80 e000-e7ff
	uint8_t scrollx, scrolly, flip, irq_mask;
	uint8_t soundlatch, dac, i8039_status;
	int32_t z80_irq, mcs_left;
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, mcs_frac, ay_frac;
	int32_t hp_x, hp_y;      // (the output's high-pass: the DAC idles at 0)
	filter_rc_t flt[3];      // (the AY channels' RC filters, port B)
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t ram[0x1400], scrollx, scrolly; } s_vs;

static void capture(void)
{
	memcpy(s_vs.ram, t.ram, sizeof(s_vs.ram));
	s_vs.scrollx = t.scrollx;
	s_vs.scrolly = t.scrolly;
}

static void mz_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the sprites' 0-15, 100-1ff the characters' 16-31)
		rbx_video.palette[i] = s_rgb[s_prom[0x20 + i] & 0x0f];
		rbx_video.palette[0x100 + i] = s_rgb[(s_prom[0x120 + i] & 0x0f) | 0x10];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const uint8_t *vram0 = s_vs.ram, *vram1 = s_vs.ram + 0x400, *cram0 = s_vs.ram + 0x800, *cram1 = s_vs.ram + 0xc00;
	// (the tilemap drawn into a 256x256 bitmap, copyscrollbitmap'd by (32 - scrolly, -scrollx))
	const int scrollx = -s_vs.scrolly + 4 * 8, scrolly = -s_vs.scrollx;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		const int ty = (y - scrolly) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const int tx = (x - scrollx) & 255;
			const int offs = (ty >> 3) * 32 + (tx >> 3);
			const int c = cram0[offs];
			const int code = vram0[offs] + ((c & 0x80) ? 256 : 0);
			const int fx = (c >> 6) & 1, fy = (c >> 5) & 1;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64;
			d[x] = (uint16_t)(0x100 + (c & 0x0f) * 16 + src[(fy ? 7 - (ty & 7) : (ty & 7)) * 8 + (fx ? 7 - (tx & 7) : (tx & 7))]);
		}
	}
	const uint8_t *sr = s_vs.ram + 0x1000;
	for (int offs = 0x400 - 4; offs >= 0; offs -= 4) {   // (the sprites: transmask of the pens mapping to colour 0)
		const int sx = sr[offs + 3] + 32, sy = 255 - ((sr[offs + 1] + 16) & 0xff);
		const int color = sr[offs] & 0x0f;
		const int flipx = !(sr[offs] & 0x40), flipy = (sr[offs] & 0x80) != 0;
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)sr[offs + 2], s_nspr)) * 256;
		const uint8_t *lut = s_prom + 0x20 + color * 16;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int i = 0; i < 16; i++) {
				const int xx = sx + i;
				if (xx < 0 || xx >= VIS_W)
					continue;
				const int p = s[flipx ? 15 - i : i];
				if (lut[p] & 0x0f)
					d[xx] = (uint16_t)(color * 16 + p);
			}
		}
	}
	for (int ty = VIS_Y0 / 8; ty < (VIS_Y0 + VIS_H) / 8; ty++)   // (the score overlay: 6 columns of the second RAM)
		for (int tx = 0; tx < 6; tx++) {
			const int offs = ty * 32 + tx;
			const int c = cram1[offs];
			const int code = vram1[offs] + ((c & 0x80) ? 256 : 0);
			const int fx = (c >> 6) & 1, fy = (c >> 5) & 1;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64;
			for (int j = 0; j < 8; j++) {
				uint16_t *d = bm + (ty * 8 + j - VIS_Y0) * pitch + tx * 8;
				for (int i = 0; i < 8; i++)
					d[i] = (uint16_t)(0x100 + (c & 0x0f) * 16 + src[(fy ? 7 - j : j) * 8 + (fx ? 7 - i : i)]);
			}
		}
}

// ---- the main CPU's bus (main_map) ----
static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x3400)
		return t.ram[a - 0x2000];
	if (a >= 0x3800 && a < 0x4000)
		return t.shared[a - 0x3800];
	if (a >= 0x4000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}
static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0008) {   // (the LS259: Q5 the flip screen, Q7 the IRQ mask)
		if (a == 5)
			t.flip = v & 1;
		else if (a == 7)
			t.irq_mask = v & 1;
	} else if (a == 0x1000)
		t.scrolly = v;
	else if (a == 0x1800)
		t.scrollx = v;
	else if (a >= 0x2000 && a < 0x3400)
		t.ram[a - 0x2000] = v;
	else if (a >= 0x3800 && a < 0x4000)
		t.shared[a - 0x3800] = v;
}

// ---- the Z80 (sound_map, sound_io_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x2000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	switch (a) {
	case 0x6000: return port(s_port_in[0]);
	case 0x6001: return port(s_port_in[1]);
	case 0x6002: return port(s_port_in[2]);
	case 0x8000: return port(s_port_dsw[1]);
	case 0x8001: return port(s_port_dsw[0]);
	}
	if (a >= 0xe000 && a < 0xe800)
		return t.shared[a - 0xe000];
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a == 0x2000)
		mcs48_set_irq(&s_mcs, 1);   // (i8039_irq_w: ASSERT_LINE)
	else if (a == 0x4000)
		t.soundlatch = v;
	else if (a >= 0xe000 && a < 0xe800)
		t.shared[a - 0xe000] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	return p <= 2 ? ay8910_data_r(&s_ay) : 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p == 0)
		ay8910_address_w(&s_ay, v);
	else if (p == 2)
		ay8910_data_w(&s_ay, v);
}
// port_b_w: each channel's capacitors, 2 bits a channel (0.01 uF and 0.22 uF)
static void ay_port_b(void *user, uint8_t v)
{
	(void)user;
	for (int i = 0; i < 3; i++, v >>= 2) {
		const double c = ((v & 1) ? 10000e-12 : 0) + ((v & 2) ? 220000e-12 : 0);
		filter_rc_lowpass_3r(&t.flt[i], 1000, 2200, 200, c, ay8910_rate(&s_ay));
	}
}

static uint8_t ay_port_a(void *user)   // (port_a_r: a timer from the Z80's clock, the I8039's status)
{
	(void)user;
	const uint64_t clock = (uint64_t)s_z.cyc * 7159 / 12288;
	return (uint8_t)((((clock / (1024 / 2)) & 0x0f) << 4) | t.i8039_status);
}

// ---- the I8039 (every I/O read the latch; P1 the DAC, P2 bit 7 low clears the IRQ, bits 4-6 the status) ----
static uint8_t mcs_ext_r(void *u, uint8_t a) { (void)u; (void)a; return t.soundlatch; }
static void mcs_port_w(void *u, int p, uint8_t v)
{
	(void)u;
	if (p == 1)
		t.dac = v;
	else if (p == 2) {
		if (!(v & 0x80))
			mcs48_set_irq(&s_mcs, 0);
		t.i8039_status = (v & 0x70) >> 4;
	}
}

// ---- the sound: a line's AY-3-8910 samples and the DAC at the AY's rate ----
static void sound_line(void)
{
	t.ay_frac += (uint64_t)AY_CLOCK * HTOTAL;
	const uint32_t ac = (uint32_t)(t.ay_frac / PIX_CLOCK);
	t.ay_frac -= (uint64_t)ac * PIX_CLOCK;
	if (s_mix_n + ay8910_out_max(&s_ay, ac) > s_mix_cap)
		return;
	static int16_t ch[3 * 512];
	const int max = s_mix_cap - s_mix_n < 512 ? s_mix_cap - s_mix_n : 512;
	const int n = ay8910_render_ch(&s_ay, ac, ch, max);
	const int32_t dac = ((int32_t)t.dac - 128) * 256 / 2;   // (the DAC's range -1 to 1, 0.25)
	for (int i = 0; i < n; i++) {
		// (each channel at 0.30 through its filter, as MAME's routes)
		const int32_t ay = (filter_rc_step(&t.flt[0], ch[3 * i]) + filter_rc_step(&t.flt[1], ch[3 * i + 1]) +
		                    filter_rc_step(&t.flt[2], ch[3 * i + 2])) * 77 >> 8;
		const int32_t x = ay + dac;
		t.hp_y = x - t.hp_x + t.hp_y - t.hp_y / 225;
		t.hp_x = x;
		s_mix[s_mix_n + i] = t.hp_y;
	}
	s_mix_n += n;
}

static void sound_frame_end(void)
{
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

// ---- init / frame ----
static void mz_exit(void);
static void mz_reset(void);

// MAME compute_resistor_weights(0, 255, -1.0, ...): each resistor's output alone, all the networks scaled so that
// the one with the greatest output gives 255
static double raw_weights(const int *res, int n, int pulldown, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = pulldown ? 1.0 / pulldown : 1.0 / 1e12, r1 = 1.0 / 1e12;
		for (int j = 0; j < n; j++) {
			if (j == k)
				r1 += 1.0 / res[j];
			else
				r0 += 1.0 / res[j];
		}
		r0 = 1.0 / r0;
		r1 = 1.0 / r1;
		w[k] = 255.0 * r0 / (r1 + r0);
		sum += w[k];
	}
	return sum;
}

static void mcs_setup(void)
{
	s_mcs.rom = s_drom;
	s_mcs.rom_mask = (uint16_t)(s_drom_size - 1);
	s_mcs.userdata = NULL;
	s_mcs.ext_r = mcs_ext_r;
	s_mcs.port_w = mcs_port_w;
}

static int mz_init(const rbx_game *g)
{
	uint32_t ss, ts, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_drom = rbx_region("daccpu", &s_drom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *sprites = rbx_region("sprites", &ss), *tiles = rbx_region("tiles", &ts);
	if (!s_rom || !s_zrom || !s_drom || !s_prom || ps < 0x220 || !sprites || !tiles) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb; spritelayout: 16x16, planes { half + 4, half + 0, 4, 0 }, x 4 pixels a byte column
	// (a column 8 bytes on), y a byte a row (the lower half 32 bytes on))
	s_nchr = ts / 32;
	s_nspr = ss / 2 / 64;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	s_mix_cap = (int)(AY_CLOCK / 8 / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_chr || !s_spr || !s_mix) {
		rbx_fail("%s: out of memory", g->set);
		mz_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = tiles[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = tiles[e * 32 + (uint32_t)i] & 15;
		}
	const uint32_t half = ss / 2;
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = e * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
				const int q = x & 3;
				const uint8_t b1 = sprites[o], b2 = sprites[half + o];
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = (uint8_t)(((b2 >> (3 - q)) & 1) << 3 | ((b2 >> (7 - q)) & 1) << 2 |
				                                                      ((b1 >> (3 - q)) & 1) << 1 | ((b1 >> (7 - q)) & 1));
			}
	static const int res_rg[3] = { 1000, 470, 220 }, res_b[2] = { 470, 220 };
	double wr[3], wb[2];
	const double srg = raw_weights(res_rg, 3, 1000, wr), sb = raw_weights(res_b, 2, 1000, wb);
	const double scale = 255.0 / (srg > sb ? srg : sb);
	for (int i = 0; i < 0x20; i++) {
		const uint8_t c = s_prom[i];
		const int r = (int)(scale * (wr[0] * (c & 1) + wr[1] * ((c >> 1) & 1) + wr[2] * ((c >> 2) & 1)) + 0.5);
		const int gg = (int)(scale * (wr[0] * ((c >> 3) & 1) + wr[1] * ((c >> 4) & 1) + wr[2] * ((c >> 5) & 1)) + 0.5);
		const int b = (int)(scale * (wb[0] * ((c >> 6) & 1) + wb[1] * ((c >> 7) & 1)) + 0.5);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	ay8910_init(&s_ay, AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 77, 1, 0);   // (0.30: each channel through its RC filter, sound_line)
	if (rbx_audio_setup(ay8910_rate(&s_ay)) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		mz_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	mz_reset();
	return 0;
}

static void mz_exit(void)
{
	free(s_chr);
	free(s_spr);
	free(s_mix);
	s_chr = s_spr = NULL;
	s_mix = NULL;
}

static void mz_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.dac = 0;
	s_mix_n = 0;
	if (s_mix)
		memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_cap);
	ay8910_reset(&s_ay);
	for (int i = 0; i < 3; i++)
		t.flt[i].k = 0x10000;
	s_ay.port_a_read = ay_port_a;
	s_ay.port_b_write = ay_port_b;
	s_ay.user = NULL;
	mcs48_init(&s_mcs, MCS48_I8039);
	mcs_setup();
	mcs48_reset(&s_mcs);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.konami1 = 1;
	for (int p = 0x20; p < 0x34; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x20) * 256;
	for (int p = 0x38; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.shared + (p - 0x38) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x20; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0xe0 + p] = t.shared + p * 256;
	s_z.read_map = s_zmap;
}

static void mz_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_mask)
				m6809_hold_irq(&s_cpu);
			t.z80_irq = 1;   // (HOLD_LINE)
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.z_frac += (uint64_t)Z80_CLOCK * HTOTAL;
		const unsigned long zc = (unsigned long)(t.z_frac / PIX_CLOCK);
		t.z_frac -= (uint64_t)zc * PIX_CLOCK;
		t.z_end += zc;
		int line_irq = t.z80_irq;
		z80_run(&s_z, (unsigned long)t.z_end, &line_irq, 1);
		t.z80_irq = line_irq;
		t.mcs_frac += (uint64_t)MCS_CYCLES * HTOTAL;
		const int mc = (int)(t.mcs_frac / PIX_CLOCK);
		t.mcs_frac -= (uint64_t)mc * PIX_CLOCK;
		t.mcs_left += mc;
		if (t.mcs_left > 0)
			t.mcs_left -= mcs48_execute(&s_mcs, t.mcs_left);
		sound_line();
	}
	sound_frame_end();
	t.frame++;
}

static int mz_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "mz_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "mz", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "mz_mcs", (uint8_t *)&s_mcs + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	rc |= rbx_state_io(ctx, "mz_ay", &s_ay, ay8910_state_size());
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "mz_z80", &z, sizeof(z));
	if (!rc && load) {
		s_mix_n = 0;
		memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_cap);
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
		mcs_setup();
	}
	return rc ? -1 : 0;
}

static int mz_save(void *ctx) { return mz_state(ctx, 0); }
static int mz_load(void *ctx) { return mz_state(ctx, 1); }
static int mz_supported(const char *machine) { return !strcmp(machine, "megazone"); }

const rbx_driver rbx_drv_megazone = { "Konami Mega Zone", mz_init, mz_exit, mz_reset, mz_frame, mz_save, mz_load,
                                      mz_supported, mz_render, NULL };

/*  Birdie King / Birdie King 2 / Birdie King 3, ported from MAME src/mame/taito/bking.cpp
    (BSD-3-Clause, copyright-holders: Mike Balfour, Zsolt Vasvari). Memory and I/O maps, the resistor palette, the
    playfield tilemap, ball and crow objects, the PC3259 collision detection (computed on the main side at the vblank
    rising edge as MAME screen_vblank does, game logic reads it), the trackballs, the Taito 68705 MCU and extra ROM of
    bking3 (machine/taito68705) and the sound board (Z80, 2 x AY-3-8910, 8-bit DAC on AY #2 port A, sound NMI through
    INPUT_MERGER_ALL_HIGH, 36.6 Hz periodic IRQ) follow MAME.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"
#include "../machine/taito68705.h"
#include "../../sound/ay8910.h"

#include <stdlib.h>
#include <string.h>

// MAME: 60 Hz, 256 x 256, visible 0-255 x 16-239, vblank time 0 (264 lines assumed for the CPU timing)
#define LINES 264
#define VIS_Y0 16
#define VIS_Y1 239
#define SCREEN_W 256
#define RATE_MHZ 60000
#define MAIN_CLOCK 3000000    // 12 MHz / 4
#define AUDIO_CLOCK 3000000   // 6 MHz / 2
#define MCU_CLOCK 750000      // 3 MHz xtal / 4
#define AY_CLOCK 1500000      // 6 MHz / 4
#define AUDIO_IRQ_PERIOD 81920   // 3 MHz / (6 MHz / (4*16*16*10*16))
#define SLICES 2              // bking3: MAME quantum 6000 Hz

typedef struct {
	z80 z;
	int irq;
} zcpu;

static int s_is3;
static uint8_t *s_main_rom, *s_audio_rom, *s_coll_prom, *s_extra;
static uint32_t s_main_size, s_audio_size, s_extra_size;
static zcpu s_main, s_audio;
static uint8_t s_ram[0x400], s_pf[0x800], s_audio_ram[0x400];
static uint8_t s_pc3259_output[4], s_pc3259_mask, s_xld[3], s_yld[3];
static uint8_t s_ball1_pic, s_ball2_pic, s_crow_pic, s_crow_flip, s_palette_bank, s_controller, s_hit, s_flip, s_mute;
static uint8_t s_addr_h, s_addr_l;
static uint8_t s_latch;
static int s_latch_pending, s_nmi_en, s_nmi_line;
static uint32_t s_audio_irq_count;
static taito68705_t s_mcu;
static uint8_t s_track[2][2];   // trackball counters [controller][x / y]
// sound
static ay8910_t s_ay[2];
static int32_t s_dac_out;
static int32_t *s_mix, *s_tmp;
static int s_mix_cap, s_mix_n;
static uint32_t s_snd_done;
static unsigned long s_frame_cyc;
static uint64_t s_frac_main, s_frac_audio, s_frac_mcu;
// video
static uint8_t s_chars[1024 * 64], s_crow[16 * 16 * 32], s_ball[2][8 * 8 * 16];
static uint32_t s_pal[64];
static uint8_t s_snap_pf[0x800], s_snap_xld[3], s_snap_yld[3], s_snap_ball1, s_snap_ball2, s_snap_crow, s_snap_crow_flip;
static uint8_t s_snap_bank, s_snap_flip;
// ports
static int s_p_in0, s_p_in1, s_p_dsw[3];

static uint8_t port8(int p) { return p >= 0 ? (uint8_t)rbx_port_read(p) : 0xff; }

// ---- sound ----
static void snd_run_to(uint32_t pos)
{
	if (pos <= s_snd_done)
		return;
	uint32_t clk = pos - s_snd_done;
	int room = s_mix_cap - s_mix_n;
	if (room <= 0) {
		s_snd_done = pos;
		return;
	}
	memset(s_tmp, 0, sizeof(int32_t) * (size_t)(ay8910_out_max(&s_ay[0], clk) < room ? ay8910_out_max(&s_ay[0], clk) : room));
	int n = ay8910_render(&s_ay[0], clk, s_tmp, room);
	ay8910_render(&s_ay[1], clk, s_tmp, room);
	int32_t *o = s_mix + s_mix_n;
	for (int k = 0; k < n; k++)
		o[k] = s_mute ? 0 : s_tmp[k] + s_dac_out;
	s_mix_n += n;
	s_snd_done = pos;
}

// AY clock = sound CPU clock / 2
static void snd_sync(void) { snd_run_to((uint32_t)((s_audio.z.cyc - s_frame_cyc) / 2)); }

static void update_nmi(void)
{
	int line = s_latch_pending && s_nmi_en;   // INPUT_MERGER_ALL_HIGH
	if (line && !s_nmi_line)
		s_audio.z.nmi_pending = 1;
	s_nmi_line = line;
}

static void ay2_pa_w(void *u, uint8_t v)   // DAC_8BIT_R2R, route 0.25
{
	(void)u;
	snd_sync();
	s_dac_out = (int32_t)(32767.0 * 0.25 * (2.0 * v / 255.0 - 1.0));
}

// ---- main CPU ----
static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000) return a < s_main_size ? s_main_rom[a] : 0xff;
	if (a < 0x8400) return s_ram[a & 0x3ff];
	if (a >= 0x9000 && a < 0x9800) return s_pf[a & 0x7ff];
	return 0;
}

static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8400) s_ram[a & 0x3ff] = v;
	else if (a >= 0x9000 && a < 0x9800) s_pf[a & 0x7ff] = v;
}

static uint8_t main_in(z80 *z, uint8_t p)
{
	(void)z;
	switch (p) {
	case 0x00: return port8(s_p_in0);
	case 0x01: return port8(s_p_in1);
	case 0x02: return port8(s_p_dsw[0]);
	case 0x03: return port8(s_p_dsw[1]);
	case 0x04: return port8(s_p_dsw[2]);
	case 0x05: return s_track[s_controller][0];
	case 0x06: return s_track[s_controller][1];
	}
	if (s_is3) {
		switch (p) {
		case 0x2f: return taito68705_data_r(&s_mcu);
		case 0x4f:   // mcu_status_r
			return (uint8_t)((!taito68705_host_semaphore_r(&s_mcu) ? 1 : 0) | (taito68705_mcu_semaphore_r(&s_mcu) ? 2 : 0));
		case 0x60: { uint32_t o = (uint32_t)s_addr_h * 256 + s_addr_l; return o < s_extra_size ? s_extra[o] : 0xff; }
		case 0x6f: return 0x31;   // ext_check_r: no "bad rom.", no "bad ext."
		}
	}
	if (p >= 0x07 && p <= 0x1f)   // pos_r
		return (uint8_t)(s_pc3259_output[(p - 7) / 8] << 4);
	return 0;
}

static void main_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	switch (p) {
	case 0x00: s_xld[0] = (uint8_t)-v; return;
	case 0x01: s_yld[0] = (uint8_t)-v; return;
	case 0x02: s_xld[1] = (uint8_t)-v; return;
	case 0x03: s_yld[1] = (uint8_t)-v; return;
	case 0x04: s_xld[2] = (uint8_t)-v; return;
	case 0x05: s_yld[2] = (uint8_t)-v; return;
	case 0x06: s_pc3259_mask++; return;   // msk_w
	case 0x07: return;                    // watchdog
	case 0x08:   // cont1_w
		s_flip = (v >> 2) & 1;
		s_controller = (v >> 1) & 1;
		s_crow_pic = (v >> 4) & 0x0f;
		return;
	case 0x09:   // cont2_w
		s_ball1_pic = v & 7;
		s_ball2_pic = (v >> 3) & 7;
		s_hit = v >> 6;
		return;
	case 0x0a:   // cont3_w
		s_crow_flip = (~v) & 1;
		s_palette_bank = (v >> 1) & 3;
		snd_sync();
		s_mute = (v >> 3) & 1;
		return;
	case 0x0b: {   // soundlatch_w: bitswap<8>(0..7)
		uint8_t r = 0;
		for (int i = 0; i < 8; i++)
			if (v & (1 << i)) r |= (uint8_t)(0x80 >> i);
		s_latch = r;
		s_latch_pending = 1;
		update_nmi();
		return;
	}
	case 0x0d:   // hitclr_w
		s_pc3259_mask = 0;
		memset(s_pc3259_output, 0, sizeof(s_pc3259_output));
		return;
	}
	if (s_is3) {
		switch (p) {
		case 0x2f: taito68705_data_w(&s_mcu, v); return;
		case 0x6f: s_addr_h = v; return;
		case 0x8f: s_addr_l = v; return;
		}
	}
}

// ---- sound CPU ----
static uint8_t audio_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x3000 || (a >= 0xe000 && a < 0xf000)) return a < s_audio_size ? s_audio_rom[a] : 0xff;
	if (a >= 0x4000 && a < 0x4400) return s_audio_ram[a & 0x3ff];
	switch (a) {
	case 0x4401: snd_sync(); return ay8910_data_r(&s_ay[0]);
	case 0x4403: snd_sync(); return ay8910_data_r(&s_ay[1]);
	case 0x4800: s_latch_pending = 0; update_nmi(); return s_latch;
	case 0x4802: s_nmi_en = 0; update_nmi(); return 0;   // sndnmi_disable_r
	}
	return 0;
}

static void audio_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000 && a < 0x4400) { s_audio_ram[a & 0x3ff] = v; return; }
	switch (a) {
	case 0x4400: case 0x4401: snd_sync(); ay8910_write(&s_ay[0], a & 1, v); return;
	case 0x4402: case 0x4403: snd_sync(); ay8910_write(&s_ay[1], a & 1, v); return;
	case 0x4802: s_nmi_en = 1; update_nmi(); return;   // sndnmi_enable_w
	}
}

static uint8_t no_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void no_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void zcpu_init(zcpu *c, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                      uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t))
{
	unsigned long cyc = c->z.cyc;
	z80_init(&c->z);
	c->z.read_byte = rd;
	c->z.write_byte = wr;
	c->z.port_in = in;
	c->z.port_out = out;
	c->z.int_data = 0xff;
	c->z.cyc = cyc;
	c->irq = 0;
}

static void zcpu_run(zcpu *c, int cycles)
{
	unsigned long end = c->z.cyc + (unsigned long)cycles;
	while ((long)(c->z.cyc - end) < 0) {
		c->z.int_pending = c->irq ? 1 : 0;
		z80_step(&c->z);
		if (c->irq && !c->z.int_pending)
			c->irq = 0;   // HOLD_LINE acknowledged
	}
}

// ---- graphics ----
// MAME gfx_layout decoding (bit offsets counted from the MSB of each byte, first plane = most significant bit)
static void decode(const uint8_t *src, uint32_t srcsize, int w, int h, int count, int nplanes, const uint32_t *planes,
                   const uint32_t *xo, const uint32_t *yo, uint32_t inc, uint8_t *out)
{
	for (int t = 0; t < count; t++)
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++) {
				int v = 0;
				for (int p = 0; p < nplanes; p++) {
					uint32_t b = (uint32_t)t * inc + planes[p] + xo[x] + yo[y];
					int bit = (b >> 3) < srcsize ? (src[b >> 3] >> (7 - (b & 7))) & 1 : 0;
					v = v << 1 | bit;
				}
				out[(t * h + y) * w + x] = (uint8_t)v;
			}
}

static int decode_gfx(void)
{
	uint32_t sz, xo[16], yo[32];
	uint8_t *pf = rbx_region("playfield", &sz);
	if (!pf) return -1;
	{
		const uint32_t planes[3] = { 0, 1024 * 64, 2 * 1024 * 64 };
		for (int i = 0; i < 8; i++) { xo[i] = (uint32_t)(7 - i); yo[i] = (uint32_t)i * 8; }
		decode(pf, sz, 8, 8, 1024, 3, planes, xo, yo, 64, s_chars);
	}
	uint8_t *crow = rbx_region("crow", &sz);
	if (!crow) return -1;
	{
		const uint32_t planes[2] = { 0, 4 };
		for (int i = 0; i < 16; i++)
			xo[i] = (uint32_t)((3 - i / 4) * 32 * 8 + 3 - (i & 3));
		for (int i = 0; i < 32; i++)
			yo[i] = (uint32_t)(31 - i) * 8;
		decode(crow, sz, 16, 32, 16, 2, planes, xo, yo, 128 * 8, s_crow);
	}
	for (int b = 0; b < 2; b++) {
		uint8_t *ball = rbx_region(b ? "ball2" : "ball1", &sz);
		if (!ball) return -1;
		const uint32_t planes[1] = { 0 };
		for (int i = 0; i < 8; i++) xo[i] = (uint32_t)(7 - i);
		for (int i = 0; i < 16; i++) yo[i] = (uint32_t)i * 8;
		decode(ball, sz, 8, 16, 8, 1, planes, xo, yo, 16 * 8, s_ball[b]);
	}
	return 0;
}

static void init_palette(const uint8_t *prom)
{
	// compute_resistor_weights(0, 255, -1.0, ...) without pull resistors: weight = 255 * G_i / sum(G)
	static const double rg[3] = { 220, 390, 820 }, bres[2] = { 220, 390 };
	double gw[3], sum = 0;
	for (int i = 0; i < 3; i++) sum += 1.0 / rg[i];
	for (int i = 0; i < 3; i++) gw[i] = 255.0 * (1.0 / rg[i]) / sum;
	double bsum = 1.0 / bres[0] + 1.0 / bres[1];
	(void)bsum;   // MAME combines blue with the green weights (bweights unused)
	for (int i = 0; i < 64; i++) {
		int pen;
		if (i < 0x20) pen = ((i << 4) & 0x180) | (i & 7);
		else if (i < 0x30) pen = (((i - 0x20) << 5) & 0x180) | (((i - 0x20) & 3) << 5);
		else if (i < 0x38) pen = (((i - 0x30) << 6) & 0x180) | (((i - 0x30) & 1) << 3);
		else pen = (((i - 0x38) << 6) & 0x180) | (((i - 0x38) & 1) << 4);
		uint8_t c = prom[pen];
		int r = (int)(gw[0] * (c & 1) + gw[1] * ((c >> 1) & 1) + gw[2] * ((c >> 2) & 1) + 0.5);
		int g = (int)(gw[0] * ((c >> 3) & 1) + gw[1] * ((c >> 4) & 1) + gw[2] * ((c >> 5) & 1) + 0.5);
		int b = (int)(gw[0] * ((c >> 6) & 1) + gw[1] * ((c >> 7) & 1) + 0.5);
		s_pal[i] = (uint32_t)(r << 16 | g << 8 | b);
	}
}

// playfield pixel (0-7) at tilemap pixel coordinates (unflipped)
static inline int pf_pixel(const uint8_t *pf, int x, int y)
{
	int ti = (y >> 3) * 32 + (x >> 3);
	uint8_t c0 = pf[ti * 2], c1 = pf[ti * 2 + 1];
	int px = x & 7, py = y & 7;
	if (c1 & 4) px = 7 - px;
	if (c1 & 8) py = 7 - py;
	return s_chars[((c0 + 256u * c1) & 1023) * 64 + py * 8 + px];
}

// ---- collision detection (main side, MAME screen_vblank rising edge) ----
static void check_collision(void)
{
	int xld, yld;
	uint32_t latch;
	const uint8_t *ball;
	if (s_pc3259_mask == 6) {   // player 1
		xld = s_xld[0];
		yld = s_yld[0];
		ball = s_ball[0] + s_ball1_pic * 128;
		latch = 0x0c00;
	} else if (s_pc3259_mask == 3) {   // player 2
		xld = s_xld[1];
		yld = s_yld[1];
		ball = s_ball[1] + s_ball2_pic * 128;
		latch = 0x0400;
	} else
		return;
	const uint8_t *colmask = s_coll_prom + 8 * s_hit;
	for (int y = 0; y <= 15; y++)
		for (int x = 0; x <= 7; x++) {
			// tilemap drawn with scroll (flip ? -xld : xld) into the 8 x 16 rectangle
			int sx, sy;
			if (!s_flip) {
				sx = (x + xld) & 0xff;
				sy = (y + yld) & 0xff;
			} else {
				sx = (255 - x - xld) & 0xff;
				sy = (255 - y - yld) & 0xff;
			}
			if (colmask[pf_pixel(s_pf, sx, sy) & 7] && (ball[y * 8 + x] & 1)) {
				int col = (xld + x) / 8 + 1;
				int row = (yld + y) / 8 + 0;
				latch |= (uint32_t)(s_flip ? 31 - col : col) << 0;
				latch |= (uint32_t)(s_flip ? 31 - row : row) << 5;
				s_pc3259_output[0] = (latch >> 0x0) & 0xf;
				s_pc3259_output[1] = (latch >> 0x4) & 0xf;
				s_pc3259_output[2] = (latch >> 0x8) & 0xf;
				s_pc3259_output[3] = (latch >> 0xc) & 0xf;
				return;
			}
		}
}

// ---- video ----
static void draw_obj(const uint8_t *g, int w, int h, uint16_t base, int flipx, int flipy, int sx, int sy)
{
	for (int ty = 0; ty < h; ty++) {
		int y = sy + ty;
		if (y < VIS_Y0 || y > VIS_Y1) continue;
		const uint8_t *r = g + (flipy ? h - 1 - ty : ty) * w;
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		for (int tx = 0; tx < w; tx++) {
			int x = sx + tx;
			if ((unsigned)x >= SCREEN_W) continue;
			uint8_t p = r[flipx ? w - 1 - tx : tx];
			if (p)
				d[x] = (uint16_t)(base + p);
		}
	}
}

static void bk_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	const int fl = s_snap_flip;
	const uint16_t cbase = (uint16_t)(s_snap_bank * 8);
	for (int y = VIS_Y0; y <= VIS_Y1; y++) {
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch;
		int ty = fl ? 255 - y : y;
		for (int x = 0; x < SCREEN_W; x++) {
			int tx = fl ? 255 - x : x;
			d[x] = (uint16_t)(cbase + pf_pixel(s_snap_pf, tx, ty));
		}
	}
	const int bank = s_snap_bank;
	draw_obj(s_ball[0] + s_snap_ball1 * 128, 8, 16, (uint16_t)(0x30 + bank * 2), 0, 0, s_snap_xld[0], s_snap_yld[0]);
	draw_obj(s_ball[1] + s_snap_ball2 * 128, 8, 16, (uint16_t)(0x38 + bank * 2), 0, 0, s_snap_xld[1], s_snap_yld[1]);
	int cf = s_snap_crow_flip;
	draw_obj(s_crow + s_snap_crow * 512, 16, 32, (uint16_t)(0x20 + bank * 4), cf, cf,
		cf ? s_snap_xld[2] - 16 : 256 - s_snap_xld[2], cf ? s_snap_yld[2] - 16 : 256 - s_snap_yld[2]);
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap_pf, s_pf, sizeof(s_pf));
	memcpy(s_snap_xld, s_xld, 3);
	memcpy(s_snap_yld, s_yld, 3);
	s_snap_ball1 = s_ball1_pic;
	s_snap_ball2 = s_ball2_pic;
	s_snap_crow = s_crow_pic;
	s_snap_crow_flip = s_crow_flip;
	s_snap_bank = s_palette_bank;
	s_snap_flip = s_flip;
	rbx_video_frame_done();
}

// ---- init / frame ----
static int kind_of(const char *m)
{
	if (!strcmp(m, "bking")) return 0;
	if (!strcmp(m, "bking3")) return 1;
	return -1;
}

static void bk_reset(void)
{
	// (first: the AY resets below call the port callbacks, whose sound sync reads them: zero as at start, not the last
	// game's)
	s_frame_cyc = 0;
	s_snd_done = 0;
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_pf, 0, sizeof(s_pf));
	memset(s_audio_ram, 0, sizeof(s_audio_ram));
	memset(&s_main, 0, sizeof(s_main));
	memset(&s_audio, 0, sizeof(s_audio));
	zcpu_init(&s_main, main_read, main_write, main_in, main_out);
	zcpu_init(&s_audio, audio_read, audio_write, no_in, no_out);
	// machine_reset
	memset(s_pc3259_output, 0, sizeof(s_pc3259_output));
	s_pc3259_mask = 0;
	memset(s_xld, 0, sizeof(s_xld));
	memset(s_yld, 0, sizeof(s_yld));
	s_ball1_pic = s_ball2_pic = s_crow_pic = s_crow_flip = s_palette_bank = s_controller = s_hit = 0;
	s_flip = s_mute = 0;
	s_addr_h = s_addr_l = 0;
	s_latch = 0;
	s_latch_pending = s_nmi_en = s_nmi_line = 0;
	s_audio_irq_count = 0;
	memset(s_track, 0, sizeof(s_track));
	if (s_is3)
		taito68705_reset(&s_mcu);
	ay8910_reset(&s_ay[0]);
	ay8910_reset(&s_ay[1]);
	s_dac_out = (int32_t)(32767.0 * 0.25 * -1.0);
	s_frac_main = s_frac_audio = s_frac_mcu = 0;
	s_snd_done = 0;
	s_frame_cyc = s_audio.z.cyc;
	s_mix_n = 0;
}

static void bk_exit(void)
{
	free(s_mix);
	free(s_tmp);
	s_mix = s_tmp = NULL;
}

static int bk_init(const rbx_game *g)
{
	int k = kind_of(g->machine);
	if (k < 0) {
		rbx_fail("%s: machine %s is not emulated", g->set, g->machine);
		return -1;
	}
	s_is3 = k;
	uint32_t psize, csize;
	s_main_rom = rbx_region("maincpu", &s_main_size);
	s_audio_rom = rbx_region("audiocpu", &s_audio_size);
	s_coll_prom = rbx_region("collision_detection", &csize);
	uint8_t *prom = rbx_region("proms", &psize);
	if (!s_main_rom || !s_audio_rom || !s_coll_prom || csize < 0x20 || !prom || psize < 0x200 || decode_gfx()) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_extra = NULL;
	s_extra_size = 0;
	if (s_is3) {
		uint32_t msize;
		uint8_t *mrom = rbx_region("bmcu:mcu", &msize);
		s_extra = rbx_region("extra", &s_extra_size);
		memset(&s_mcu, 0, sizeof(s_mcu));
		if (!s_extra || !mrom || msize < 0x800 || taito68705_init(&s_mcu, TAITO68705_STD, mrom, 0x800)) {
			rbx_fail("%s: MCU / extra ROM missing", g->set);
			return -1;
		}
	}
	init_palette(prom);
	s_p_in0 = rbx_port_find("IN0");
	s_p_in1 = rbx_port_find("IN1");
	s_p_dsw[0] = rbx_port_find("DSWA");
	s_p_dsw[1] = rbx_port_find("DSWB");
	s_p_dsw[2] = rbx_port_find("DSWC");
	for (int i = 0; i < 2; i++) {
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, AY8910_LEGACY_OUTPUT, 64, 4, 1);   // route 0.25
		s_ay[i].user = NULL;
	}
	s_ay[1].port_a_write = ay2_pa_w;
	uint32_t rate = ay8910_rate(&s_ay[0]);
	s_mix_cap = (int)(rate / 50) + 64;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	s_tmp = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || !s_tmp || rbx_audio_setup(rate) || rbx_video_setup(SCREEN_W, VIS_Y1 - VIS_Y0 + 1, 64)) {
		bk_exit();
		return -1;
	}
	bk_reset();
	return 0;
}

static void read_trackballs(void)
{
	// TRACK0 = player 1, TRACK1 = player 2 (cocktail); Y is PORT_REVERSE; sensitivity 25 %
	for (int p = 0; p < 2; p++) {
		int tx = rbx_analog(p + 1, RBX_IPT_TRACKBALL_X), ty = rbx_analog(p + 1, RBX_IPT_TRACKBALL_Y);
		if (!tx && !ty) {   // no device: the d-pad rolls the ball (4 counts per frame)
			uint32_t in = rbx_input(p + 1);
			tx = ((in & RBX_IN_RIGHT) ? 4 : 0) - ((in & RBX_IN_LEFT) ? 4 : 0);
			ty = ((in & RBX_IN_DOWN) ? 4 : 0) - ((in & RBX_IN_UP) ? 4 : 0);
		}
		s_track[p][0] = (uint8_t)(s_track[p][0] + tx / 4);
		s_track[p][1] = (uint8_t)(s_track[p][1] - ty / 4);
	}
}

static void bk_frame(void)
{
	s_frame_cyc = s_audio.z.cyc;
	s_snd_done = 0;
	for (int i = 0; i < LINES; i++) {
		int line = (VIS_Y1 + 1 + i) % LINES;
		if (line == VIS_Y1 + 1) {
			read_trackballs();
			vblank_capture();
			check_collision();
			s_main.irq = 1;   // irq0_line_hold
		}
		int mc = rbx_line_cycles(&s_frac_main, MAIN_CLOCK, RATE_MHZ, LINES * SLICES);
		int ac = rbx_line_cycles(&s_frac_audio, AUDIO_CLOCK, RATE_MHZ, LINES * SLICES);
		int uc = rbx_line_cycles(&s_frac_mcu, MCU_CLOCK, RATE_MHZ, LINES * SLICES);
		for (int s = 0; s < SLICES; s++) {
			zcpu_run(&s_main, mc);
			s_audio_irq_count += (uint32_t)ac;
			if (s_audio_irq_count >= AUDIO_IRQ_PERIOD) {
				s_audio_irq_count -= AUDIO_IRQ_PERIOD;
				s_audio.irq = 1;
			}
			zcpu_run(&s_audio, ac);
			if (s_is3)
				taito68705_execute(&s_mcu, uc);
		}
		snd_sync();
	}
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	s_mix_n = 0;
}

static int zcpu_state(void *ctx, const char *tag, zcpu *c, int load)
{
	zcpu t = *c;
	if (rbx_state_io(ctx, tag, &t, sizeof(t)))
		return -1;
	if (load) {
		t.z.read_byte = c->z.read_byte;
		t.z.write_byte = c->z.write_byte;
		t.z.port_in = c->z.port_in;
		t.z.port_out = c->z.port_out;
		t.z.userdata = c->z.userdata;
		*c = t;
	}
	return 0;
}

static int bk_state(void *ctx, int load)
{
	int rc = 0;
	rc |= zcpu_state(ctx, "main", &s_main, load);
	rc |= zcpu_state(ctx, "audio", &s_audio, load);
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "pf", s_pf, sizeof(s_pf));
	rc |= rbx_state_io(ctx, "aram", s_audio_ram, sizeof(s_audio_ram));
	if (s_is3) {
		rc |= rbx_state_io(ctx, "tmcu", &s_mcu.st, sizeof(s_mcu.st));
		rc |= rbx_state_io(ctx, "tmcu6805", &s_mcu.mcu.st, sizeof(s_mcu.mcu.st));
	}
	rc |= rbx_state_io(ctx, "ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "ay1", &s_ay[1], ay8910_state_size());
	uint8_t v[32] = { s_pc3259_output[0], s_pc3259_output[1], s_pc3259_output[2], s_pc3259_output[3], s_pc3259_mask,
		s_xld[0], s_xld[1], s_xld[2], s_yld[0], s_yld[1], s_yld[2], s_ball1_pic, s_ball2_pic, s_crow_pic, s_crow_flip,
		s_palette_bank, s_controller, s_hit, s_flip, s_mute, s_addr_h, s_addr_l, s_latch, (uint8_t)s_latch_pending,
		(uint8_t)s_nmi_en, (uint8_t)s_nmi_line, s_track[0][0], s_track[0][1], s_track[1][0], s_track[1][1], 0, 0 };
	int32_t w[2] = { s_dac_out, (int32_t)s_audio_irq_count };
	rc |= rbx_state_io(ctx, "regs", v, sizeof(v));
	rc |= rbx_state_io(ctx, "misc", w, sizeof(w));
	if (rc)
		return -1;
	if (load) {
		memcpy(s_pc3259_output, v, 4);
		s_pc3259_mask = v[4];
		memcpy(s_xld, v + 5, 3);
		memcpy(s_yld, v + 8, 3);
		s_ball1_pic = v[11]; s_ball2_pic = v[12]; s_crow_pic = v[13]; s_crow_flip = v[14]; s_palette_bank = v[15];
		s_controller = v[16]; s_hit = v[17]; s_flip = v[18]; s_mute = v[19]; s_addr_h = v[20]; s_addr_l = v[21];
		s_latch = v[22]; s_latch_pending = v[23]; s_nmi_en = v[24]; s_nmi_line = v[25];
		s_track[0][0] = v[26]; s_track[0][1] = v[27]; s_track[1][0] = v[28]; s_track[1][1] = v[29];
		s_dac_out = w[0];
		s_audio_irq_count = (uint32_t)w[1];
		s_frame_cyc = s_audio.z.cyc;
		s_snd_done = 0;
	}
	return 0;
}

static int bk_save(void *ctx) { return bk_state(ctx, 0); }
static int bk_load(void *ctx) { return bk_state(ctx, 1); }
static int bk_supported(const char *machine) { return kind_of(machine) >= 0; }

const rbx_driver rbx_drv_bking = { "Birdie King", bk_init, bk_exit, bk_reset, bk_frame, bk_save, bk_load, bk_supported, bk_render };

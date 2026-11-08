/*  Taito / Midway Super Speed Race, ported from MAME src/mame/taito/sspeedr.cpp (license BSD-3-Clause,
    copyright-holders: Stefan Jokisch); score panel after MAME src/mame/layout/sspeedr.lay (CC0-1.0).
    Z80 at 19.968 MHz / 8, 456 x 264 raster (376 x 248 visible), IRQ asserted at vblank and cleared by port 0x1f.
    Track, drones and driver car are drawn as MAME's screen_update does (the drone pictures toggle every vblank).

    Score / time: the cabinet shows them on LED digits outside the monitor (MAME layout). The bitmap here is
    widened by a panel of PANEL rows above the game picture (left side once rotated, ROT270) where the 5 high
    scores, the score and the time are drawn as 7-segment digits from the LS48 outputs, plus the GAME OVER and
    EXTENDED PLAY lamps as bars.

    Controls: the wheel (64-position encoder, wraps) and the gas pedal (30 positions) are IPT_POSITIONAL fields
    with MAME's remap table; they are kept here. Without an analog device the d-pad steers (left / right) and
    UP presses the pedal (it ramps up while held, springs back when released); BUTTON1 is the shifter (toggle).

    Sound: approximation. MAME runs a netlist of the LM3900 engine oscillators and the noise board. Here:
    the engine is a pulse oscillator whose pitch follows the 4-bit pedal latch with the gear (LO SHIFT higher,
    HI SHIFT lower) and a slow glide, muted by ENGINE SOUND OFF; NOISE CR 1 (tyre squeal) is band-limited noise
    from a 4 kHz LFSR, NOISE CR 2 a brighter hiss; BOOM is a low-passed noise burst that decays after the bit
    is cleared; SILENCE mutes the output.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../rbx_core.h"
#include "../../cpu/z80/z80.h"

#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK (19968000 / 8)
#define VTOTAL 264
#define GAME_W 376
#define GAME_H 248
#define PANEL 112
#define RATE_MHZ 59470   // 7.159 MHz / (456 * 264)
#define AUDIO_RATE 48000

typedef struct {
	z80 z;
	int irq;
} zcpu;

static zcpu s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_track;
static uint8_t *s_cars[2];   // driver / drone: 16 pictures 32x16, 1 byte per pixel
static uint8_t s_ram[0x200];
static uint64_t s_frac;
static int s_port_in2, s_port_dsw;

typedef struct {
	uint8_t toggle, driver_vert, driver_pic, drones_mask, track_ice, pad;
	uint16_t driver_horz, drones_horz, track_horz;
	uint8_t drones_vert[3], track_vert[2];
	uint8_t digits[26];
	uint8_t lamps;
} vid_t;
static vid_t s_v, s_snap;

static uint8_t s_sound1, s_sound2;
static int s_wheel, s_pedal, s_wheel_sub;

typedef struct {
	double eng_f, eng_ph, boom_env, lp_boom, lp_sq, lp_hiss, hp_s, hp_in;
	double noise_acc;
	uint32_t lfsr, line_frac;
} snd_t;
static snd_t s_snd;

static const uint8_t ctl_table[64] = {
	0x3f, 0x3e, 0x3c, 0x3d, 0x39, 0x38, 0x3a, 0x3b, 0x33, 0x32, 0x30, 0x31, 0x35, 0x34, 0x36, 0x37,
	0x27, 0x26, 0x24, 0x25, 0x21, 0x20, 0x22, 0x23, 0x2b, 0x2a, 0x28, 0x29, 0x2d, 0x2c, 0x2e, 0x2f,
	0x0f, 0x0e, 0x0c, 0x0d, 0x09, 0x08, 0x0a, 0x0b, 0x03, 0x02, 0x00, 0x01, 0x05, 0x04, 0x06, 0x07,
	0x17, 0x16, 0x14, 0x15, 0x11, 0x10, 0x12, 0x13, 0x1b, 0x1a, 0x18, 0x19, 0x1d, 0x1c, 0x1e, 0x1f
};
static const uint8_t ls48_map[16] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7c, 0x07, 0x7f, 0x67, 0x58, 0x4c, 0x62,
                                      0x69, 0x78, 0x00 };

// ---- memory / I/O ----
static uint8_t mem_r(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x1000) return a < s_rom_size ? s_rom[a] : 0xff;
	if (a >= 0x2000 && a < 0x2200) return s_ram[a - 0x2000];
	return 0xff;
}

static void mem_w(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	if (a >= 0x2000 && a < 0x2200) s_ram[a - 0x2000] = d;
	else if (a >= 0x7f00 && a <= 0x7f17) s_v.digits[a - 0x7f00] = ls48_map[~d & 0xf];
}

static uint8_t io_r(z80 *z, uint8_t p)
{
	(void)z;
	switch (p) {
	case 0: return ctl_table[s_wheel & 63];
	case 1: return ctl_table[2 + s_pedal];
	case 3: return (uint8_t)rbx_port_read(s_port_dsw);
	case 4: return (uint8_t)rbx_port_read(s_port_in2);
	}
	return 0xff;
}

static void io_w(z80 *z, uint8_t p, uint8_t d)
{
	(void)z;
	switch (p) {
	case 0x00: s_sound1 = d; break;
	case 0x01: s_sound2 = d; break;
	case 0x02: s_v.lamps = d & 3; break;
	case 0x04: case 0x05: s_v.digits[24 + (p - 4)] = ls48_map[d & 0xf]; break;
	case 0x10: s_v.driver_horz = (uint16_t)((s_v.driver_horz & 0x100) | d); break;
	case 0x11: s_v.driver_pic = d & 0x1f; break;
	case 0x12: s_v.driver_horz = (uint16_t)((s_v.driver_horz & 0xff) | ((d & 1) << 8)); break;
	case 0x13: s_v.drones_horz = (uint16_t)((s_v.drones_horz & 0x100) | d); break;
	case 0x14: s_v.drones_horz = (uint16_t)((s_v.drones_horz & 0xff) | ((d & 1) << 8)); break;
	case 0x15: s_v.drones_mask = d & 0x3f; break;
	case 0x16: s_v.driver_vert = d; break;
	case 0x17: case 0x18: s_v.track_vert[p - 0x17] = d & 0x7f; break;
	case 0x19: s_v.track_horz = (uint16_t)((s_v.track_horz & 0x100) | d); break;
	case 0x1a: s_v.track_horz = (uint16_t)((s_v.track_horz & 0xff) | ((d & 1) << 8)); break;
	case 0x1b: s_v.track_ice = d & 7; break;
	case 0x1c: case 0x1d: case 0x1e: s_v.drones_vert[p - 0x1c] = d; break;
	case 0x1f: s_cpu.irq = 0; break;   // int_ack_w
	}
}

// ---- video ----
static void draw_track(uint16_t *bm, int pitch, const vid_t *v)
{
	for (int x = 0; x < GAME_W; x++) {
		unsigned cx = (unsigned)x + v->track_horz + 0x50;
		int flag = 0;
		if (v->track_ice & 2)
			flag = 1;
		else if (v->track_ice & 4)
			flag = (v->track_ice & 1) ? (cx <= 0x1ff) : (cx >= 0x200);
		if (cx >= 0x200)
			cx -= 0x1c8;
		int xo = (int)(((cx & 0x1c) >> 2) | ((cx & 0xe0) << 3));
		int hi = (cx & 2) != 0;
		int y = 0;
		for (; y < v->track_vert[0] && y < GAME_H; y++) {
			unsigned cy = (unsigned)(y - v->track_vert[0]);
			uint8_t b = s_track[((cy & 0x1f) << 3) | xo];
			bm[y * pitch + x] = hi ? b >> 4 : b & 0xf;
		}
		for (; y < 128 + v->track_vert[1] && y < GAME_H; y++)
			bm[y * pitch + x] = flag ? 15 : 0;
		for (; y < GAME_H; y++) {
			unsigned cy = (unsigned)(y - v->track_vert[1]);
			uint8_t b = s_track[((cy & 0x1f) << 3) | xo];
			bm[y * pitch + x] = hi ? b >> 4 : b & 0xf;
		}
	}
}

static void draw_car(uint16_t *bm, int pitch, const uint8_t *gfx, int code, int sx, int sy)
{
	const uint8_t *src = gfx + (code & 15) * 512;
	for (int y = 0; y < 16; y++) {
		int py = sy + y;
		if (py < 0 || py >= GAME_H)
			continue;
		for (int x = 0; x < 32; x++) {
			int px = sx + x;
			uint8_t c = src[y * 32 + x];
			if (c && px >= 0 && px < GAME_W)
				bm[py * pitch + px] = c;
		}
	}
}

// 7-segment digit in the rotated view: rx / ry = top-left (rotated pixels), w x h
static void put_rot(int rx, int ry, uint16_t c)
{
	if (rx < 0 || rx >= PANEL || ry < 0 || ry >= GAME_W)
		return;
	rbx_video.bitmap[rx * rbx_video.pitch + (GAME_W - 1 - ry)] = c;
}

static void rect_rot(int x0, int y0, int w, int h, uint16_t c)
{
	for (int y = y0; y < y0 + h; y++)
		for (int x = x0; x < x0 + w; x++)
			put_rot(x, y, c);
}

static void draw_digit(int rx, int ry, uint8_t seg)
{
	const int w = 8, h = 13, m = h / 2;
	uint16_t on = 16, off = 17;
	rect_rot(rx + 1, ry, w - 2, 1, (seg & 0x01) ? on : off);          // a
	rect_rot(rx + w - 1, ry + 1, 1, m - 1, (seg & 0x02) ? on : off);  // b
	rect_rot(rx + w - 1, ry + m + 1, 1, m - 1, (seg & 0x04) ? on : off); // c
	rect_rot(rx + 1, ry + h - 1, w - 2, 1, (seg & 0x08) ? on : off);  // d
	rect_rot(rx, ry + m + 1, 1, m - 1, (seg & 0x10) ? on : off);      // e
	rect_rot(rx, ry + 1, 1, m - 1, (seg & 0x20) ? on : off);          // f
	rect_rot(rx + 1, ry + m, w - 2, 1, (seg & 0x40) ? on : off);      // g
}

static void draw_panel(const vid_t *v)
{
	for (int y = 0; y < PANEL; y++)
		for (int x = 0; x < GAME_W; x++)
			rbx_video.bitmap[y * rbx_video.pitch + x] = 18;
	// layout units (x -168.29 .. 0, y 0 .. 376) scaled by PANEL / 168.29
	const double s = PANEL / 168.29;
	for (int r = 0; r < 6; r++) {
		int ly = r < 5 ? 33 + 27 * r : 175;
		for (int i = 0; i < 4; i++) {
			double lx = -147.823 + 15.919 * i;
			draw_digit((int)((lx + 168.29) * s), ly, v->digits[(r < 5 ? r * 4 : 20) + i]);
		}
	}
	draw_digit((int)((-131.90 + 168.29) * s), 264, v->digits[24]);
	draw_digit((int)((-115.984 + 168.29) * s), 264, v->digits[25]);
	rect_rot((int)((-90 + 168.29) * s), 357, (int)(70 * s), 8, (v->lamps & 1) ? 19 : 17);   // GAME OVER
	rect_rot((int)((-155 + 168.29) * s), 357, (int)(70 * s), 8, (v->lamps & 2) ? 20 : 17);  // EXTENDED PLAY
}

static void ss_render(void)
{
	const vid_t *v = &s_snap;
	const int pitch = rbx_video.pitch;
	uint16_t *bm = rbx_video.bitmap + PANEL * pitch;
	draw_track(bm, pitch, v);
	static const uint8_t code[6] = { 0xf, 0x4, 0x3, 0x9, 0x7, 0xc };
	for (int i = 0; i < 6; i++) {
		if ((v->drones_mask >> i) & 1)
			continue;
		int x = (code[i] << 5) - v->drones_horz - 0x50;
		if (x <= -32)
			x += 0x1c8;
		draw_car(bm, pitch, s_cars[1], code[i] ^ v->toggle, x, 0xf0 - v->drones_vert[i >> 1]);
	}
	if (v->driver_pic & 0x10) {
		int x = 0x1e0 - v->driver_horz - 0x50;
		if (x <= -32)
			x += 0x1c8;
		draw_car(bm, pitch, s_cars[0], v->driver_pic, x, 0xf0 - v->driver_vert);
	}
	draw_panel(v);
}

// ---- sound (approximation) ----
static int32_t snd_sample(void)
{
	snd_t *n = &s_snd;
	const double dt = 1.0 / AUDIO_RATE;
	double out = 0;
	// noise: LFSR clocked at 4 kHz
	n->noise_acc += 4000.0 * dt;
	while (n->noise_acc >= 1) {
		n->noise_acc -= 1;
		n->lfsr = (n->lfsr << 1) | (((n->lfsr >> 17) ^ (n->lfsr >> 12)) & 1);
	}
	double noise = (n->lfsr & 1) ? 1.0 : -1.0;
	// engine
	int pedal = s_sound1 & 0x0f;
	double target = 25.0 + pedal * 9.0;
	if (s_sound1 & 0x20) target *= 1.6;        // LO SHIFT
	else if (s_sound1 & 0x10) target *= 0.8;   // HI SHIFT
	n->eng_f += (target - n->eng_f) * (dt / ((s_sound1 & 0x10) ? 0.6 : 0.3));
	n->eng_ph += n->eng_f * dt;
	if (n->eng_ph >= 1) n->eng_ph -= 1;
	if (!(s_sound2 & 0x02))
		out += (n->eng_ph < 0.3 ? 1.0 : -0.43) * (0.25 + pedal * 0.02);
	// tyre squeal / hiss
	n->lp_sq += (noise - n->lp_sq) * 0.25;
	if (s_sound2 & 0x04) out += n->lp_sq * 0.3;
	n->lp_hiss += (noise - n->lp_hiss) * 0.6;
	if (s_sound2 & 0x08) out += n->lp_hiss * 0.2;
	// boom
	if (s_sound2 & 0x01) n->boom_env = 1.0;
	else n->boom_env -= n->boom_env * dt / 0.35;
	n->lp_boom += (noise - n->lp_boom) * 0.02;
	out += n->lp_boom * n->boom_env * 1.5;
	if (s_sound2 & 0x10)
		out = 0;   // SILENCE
	double a = 0.02 / (0.02 + dt);
	double h = a * (n->hp_s + out - n->hp_in);
	n->hp_s = h;
	n->hp_in = out;
	return (int32_t)(h * 9000);
}

// ---- controls ----
static void update_controls(void)
{
	uint32_t in = rbx_input(1);
	int d = rbx_analog(1, RBX_IPT_POSITIONAL);
	if (!d) {
		if (in & RBX_IN_LEFT) d = -4;
		if (in & RBX_IN_RIGHT) d = 4;
	}
	s_wheel_sub += d;
	while (s_wheel_sub >= 8) { s_wheel_sub -= 8; s_wheel = (s_wheel + 1) & 63; }
	while (s_wheel_sub <= -8) { s_wheel_sub += 8; s_wheel = (s_wheel - 1) & 63; }
	if (in & RBX_IN_UP) { if ((s_pedal += 2) > 29) s_pedal = 29; }
	else if ((s_pedal -= 3) < 0) s_pedal = 0;
}

// ---- machine ----
static void cpu_init(void)
{
	unsigned long cyc = s_cpu.z.cyc;
	z80_init(&s_cpu.z);
	s_cpu.z.read_byte = mem_r;
	s_cpu.z.write_byte = mem_w;
	s_cpu.z.port_in = io_r;
	s_cpu.z.port_out = io_w;
	s_cpu.z.int_data = 0xff;
	s_cpu.z.cyc = cyc;
	s_cpu.irq = 0;
}

static void ss_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(&s_v, 0, sizeof(s_v));
	memset(&s_snd, 0, sizeof(s_snd));
	s_snd.lfsr = 1;
	for (int i = 0; i < 26; i++)
		s_v.digits[i] = 0;
	s_sound1 = s_sound2 = 0;
	s_wheel = s_pedal = s_wheel_sub = 0;
	s_frac = 0;
	memset(&s_cpu, 0, sizeof(s_cpu));
	cpu_init();
}

static void ss_exit(void)
{
	for (int i = 0; i < 2; i++) {
		free(s_cars[i]);
		s_cars[i] = NULL;
	}
}

static int ss_init(const rbx_game *g)
{
	uint32_t sz[3];
	const uint8_t *src[2];
	s_rom = rbx_region("maincpu", &s_rom_size);
	src[0] = rbx_region("driver", &sz[0]);
	src[1] = rbx_region("drone", &sz[1]);
	s_track = rbx_region("track", &sz[2]);
	if (!s_rom || !src[0] || !src[1] || !s_track || sz[0] < 0x800 || sz[1] < 0x800 || sz[2] < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_port_in2 = rbx_port_find("IN2");
	s_port_dsw = rbx_port_find("DSW");
	// car_layout: 32x16, 4 bpp packed nibbles, each nibble doubled horizontally (x offsets 4,4,0,0,12,12,8,8 ...)
	static const int xo[32] = { 0x04, 0x04, 0x00, 0x00, 0x0c, 0x0c, 0x08, 0x08, 0x14, 0x14, 0x10, 0x10, 0x1c, 0x1c,
		0x18, 0x18, 0x24, 0x24, 0x20, 0x20, 0x2c, 0x2c, 0x28, 0x28, 0x34, 0x34, 0x30, 0x30, 0x3c, 0x3c, 0x38, 0x38 };
	for (int k = 0; k < 2; k++) {
		s_cars[k] = (uint8_t *)malloc(16 * 512);
		if (!s_cars[k]) {
			ss_exit();
			rbx_fail("%s: out of memory", g->set);
			return -1;
		}
		for (int c = 0; c < 16; c++)
			for (int y = 0; y < 16; y++)
				for (int x = 0; x < 32; x++) {
					int bit = c * 0x400 + y * 0x40 + xo[x];
					uint8_t b = src[k][bit >> 3];
					s_cars[k][c * 512 + y * 32 + x] = (uint8_t)((bit & 4) ? b & 0xf : b >> 4);
				}
	}
	if (rbx_video_setup(GAME_W, GAME_H + PANEL, 32) || rbx_audio_setup(AUDIO_RATE)) {
		ss_exit();
		return -1;
	}
	for (int i = 0; i < 16; i++) {
		int r = (i & 1) ? 0xb0 : 0x20, gg = (i & 2) ? 0xb0 : 0x20, b = (i & 4) ? 0xb0 : 0x20;
		if (i & 8) { r += 0x4f; gg += 0x4f; b += 0x4f; }
		rbx_video.palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	rbx_video.palette[16] = 0xff2629;   // lit segment (layout colour 1.0 / 0.15 / 0.17)
	rbx_video.palette[17] = 0x1a0606;   // unlit segment / lamp
	rbx_video.palette[18] = 0x000000;   // panel
	rbx_video.palette[19] = 0xffffff;   // GAME OVER lamp
	rbx_video.palette[20] = 0xffff66;   // EXTENDED PLAY lamp
	ss_reset();
	return 0;
}

static void ss_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	update_controls();
	for (int line = 0; line < VTOTAL; line++) {
		if (line == GAME_H) {
			rbx_video_wait_idle();
			s_snap = s_v;
			rbx_video_frame_done();
			s_v.toggle ^= 1;
			s_cpu.irq = 1;   // irq0_line_assert
		}
		unsigned long end = s_cpu.z.cyc + (unsigned long)rbx_line_cycles(&s_frac, CPU_CLOCK, RATE_MHZ, VTOTAL);
		while ((long)(s_cpu.z.cyc - end) < 0) {
			s_cpu.z.int_pending = s_cpu.irq ? 1 : 0;
			z80_step(&s_cpu.z);
		}
		s_snd.line_frac += AUDIO_RATE * 1000u;
		while (s_snd.line_frac >= (uint32_t)RATE_MHZ * VTOTAL) {
			s_snd.line_frac -= (uint32_t)RATE_MHZ * VTOTAL;
			rbx_audio_put(snd_sample());
		}
	}
}

static int ss_state(void *ctx, int load)
{
	zcpu t = s_cpu;
	int rc = rbx_state_io(ctx, "cpu", &t, sizeof(t));
	if (!rc && load) {
		t.z.read_byte = mem_r;
		t.z.write_byte = mem_w;
		t.z.port_in = io_r;
		t.z.port_out = io_w;
		t.z.userdata = NULL;
		s_cpu = t;
	}
	rc |= rbx_state_io(ctx, "ram", s_ram, sizeof(s_ram));
	rc |= rbx_state_io(ctx, "vid", &s_v, sizeof(s_v));
	rc |= rbx_state_io(ctx, "snd", &s_snd, sizeof(s_snd));
	int32_t m[5] = { s_sound1, s_sound2, s_wheel, s_pedal, s_wheel_sub };
	rc |= rbx_state_io(ctx, "misc", m, sizeof(m));
	rc |= rbx_state_io(ctx, "frac", &s_frac, sizeof(s_frac));
	if (!rc && load) {
		s_sound1 = (uint8_t)m[0]; s_sound2 = (uint8_t)m[1]; s_wheel = m[2]; s_pedal = m[3]; s_wheel_sub = m[4];
	}
	return rc ? -1 : 0;
}

static int ss_save(void *ctx) { return ss_state(ctx, 0); }
static int ss_load(void *ctx) { return ss_state(ctx, 1); }
static int ss_supported(const char *machine) { return !strcmp(machine, "sspeedr"); }

const rbx_driver rbx_drv_sspeedr = { "Super Speed Race", ss_init, ss_exit, ss_reset, ss_frame, ss_save, ss_load,
                                     ss_supported, ss_render };

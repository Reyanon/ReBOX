/*  Taito Crazy Balloon hardware, ported from MAME src/mame/taito/crbaloon.cpp, crbaloon_v.cpp, crbaloon_a.cpp,
    crbaloon.h (license BSD-3-Clause, copyright-holders: Nicola Salmoria, Derrick Renaud).
    Z80 at 9.987 MHz / 4, 320 x 262 raster (256 x 224 visible), level IRQ at vblank while D0 of port 6 is set,
    watchdog of 256 vblanks. The PC3092 (start / coin gate, flip screen) and PC3259 (sprite / background collision
    address) custom chips follow MAME's approximations: the collision address is computed at vblank from the one
    32x32 sprite against the background pixels, as screen_update_crbaloon does.

    Sound: approximation, not a circuit simulation. MAME uses an SN76477 plus a discrete netlist; here
      - music (DISCRETE_NOTE on 16H / 2 with the port 5 divider, 7-step octave counter, 2-bit resistor DAC) is
        rebuilt exactly as a counter waveform, gated by the music enable and high-passed;
      - the laugh (555 VCO driven by a 6.9 Hz 555 square through an RC) is a square wave whose pitch glides
        between two values at the slow oscillator rate;
      - the SN76477 is reduced to its parts used here: noise (24.9 kHz clock, ~1 kHz RC filter), the VCO
        (fixed, ~2 kHz) selected by APPEAR (mixer B), the one-shot envelope with attack / decay times from the
        MAME component values, triggered when EXPLOSION (/enable) is released; the BREATH bit (SLF resistor)
        has no audible effect at the SLF's ultrasonic rate and is ignored;
      - SOUND STOP (port 6 D1 low) mutes everything (MAME system_mute).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"

#include <stdlib.h>
#include <string.h>

#define MASTER_XTAL 9987000
#define CPU_CLOCK (MASTER_XTAL / 4)
#define HTOTAL 320
#define VTOTAL 262
#define VIS_W 256
#define VIS_H 224
#define RATE_MHZ 59558   // 4.9935 MHz / (320 * 262) in milli-Hz
#define AUDIO_RATE 48000

typedef struct {
	z80 z;
	int irq;
} zcpu;

static zcpu s_cpu;
static uint8_t *s_rom;
static uint32_t s_rom_size;
static const uint8_t *s_gfx2;
static uint8_t s_ram[0x400], s_vram[0x400], s_cram[0x400];
static uint8_t s_spr[3], s_pc3092[5];
static uint8_t s_irq_mask, s_sound, s_music_freq, s_flip;
static uint16_t s_coll;
static uint8_t s_coll_clear;
static int s_wd;
static uint64_t s_frac;
static int s_port_dsw0, s_port_in0, s_port_dsw1, s_port_in1, s_port_pc3092;
static uint8_t *s_chars;   // 256 chars, 8x8, 1 byte per pixel (0 / 1)

// sound state (approximation, see the header)
typedef struct {
	uint32_t note_cnt, note_oct;      // DISCRETE_NOTE counters
	uint32_t note_frac;               // 16H / 2 clock phase (x AUDIO_RATE)
	double laugh_ph, laugh_slow_ph, laugh_cv;
	uint32_t lfsr;
	double noise_acc, noise_flt, vco_ph;
	double env, oneshot;              // attack / decay cap (0..1), one-shot time left (s)
	uint8_t prev_enable;
	double hp_music, hp_music_in, hp_out, hp_out_in;
	uint32_t line_frac;
} snd_t;
static snd_t s_snd;

// snapshot for the renderer
typedef struct {
	uint8_t vram[0x400], cram[0x400], spr[3], flip;
} snap_t;
static snap_t s_snap;

// ---- PC3092 / PC3259 ----
static uint8_t pc3092_r(void)
{
	// enable coin & start input? (MAME's guess)
	return (s_pc3092[1] & 0x02) ? (uint8_t)(rbx_port_read(s_port_pc3092) & 0x0f) : 0;
}

static uint16_t collision_address(void) { return s_coll_clear ? 0xffff : s_coll; }

static uint8_t pc3259_r(int reg)
{
	uint16_t ca = collision_address();
	int collided = ca != 0xffff;
	uint8_t ret;
	switch (reg) {
	case 0: ret = collided ? (uint8_t)(ca & 0x0f) : 0; break;
	case 1: ret = collided ? (uint8_t)((ca >> 4) & 0x0f) : 0; break;
	case 2: ret = collided ? (uint8_t)(ca >> 8) : 0; break;
	default: ret = collided ? 0x08 : 0x07; break;
	}
	return (uint8_t)(ret | (rbx_port_read(s_port_dsw1) & 0xf0));
}

static void port_sound_w(uint8_t d)
{
	s_irq_mask = d & 1;
	s_coll_clear = !(d & 1);   // active low
	if (!s_irq_mask)
		s_cpu.irq = 0;
	s_sound = d;
}

// ---- memory / I/O ----
static uint8_t mem_r(void *u, uint16_t a)
{
	(void)u;
	a &= 0x7fff;
	if (a < 0x4000)
		return a < s_rom_size ? s_rom[a] : 0xff;
	if (a < 0x4800) return s_ram[a & 0x3ff];
	if (a < 0x5000) return s_vram[a & 0x3ff];
	if (a < 0x5800) return s_cram[a & 0x3ff];
	return 0xff;
}

static void mem_w(void *u, uint16_t a, uint8_t d)
{
	(void)u;
	a &= 0x7fff;
	if (a < 0x4000) return;
	if (a < 0x4800) s_ram[a & 0x3ff] = d;
	else if (a < 0x5000) s_vram[a & 0x3ff] = d;
	else if (a < 0x5800) s_cram[a & 0x3ff] = d;
}

static uint8_t io_r(z80 *z, uint8_t p)
{
	(void)z;
	p &= 0x0f;
	switch (p & 3) {
	case 0: return (uint8_t)rbx_port_read(s_port_dsw0);
	case 1: return (uint8_t)rbx_port_read(s_port_in0);
	case 2: return pc3259_r(p >> 2);
	default: return (uint8_t)((rbx_port_read(s_port_in1) & 0x0f) | (pc3092_r() << 4));
	}
}

static void io_w(z80 *z, uint8_t p, uint8_t d)
{
	(void)z;
	p &= 0x0f;
	if (p == 1) s_wd = 0;
	else if (p >= 2 && p <= 4) s_spr[p - 2] = d;
	else if (p == 5) s_music_freq = d;
	else if (p == 6) port_sound_w(d);
	else if (p >= 7 && p <= 0x0b) {
		s_pc3092[p - 7] = d & 0x0f;
		s_flip = s_pc3092[1] & 1;
	}
}

// ---- video ----
// background pixel (0 / 1) at a screen position, through the TILEMAP_SCAN_ROWS_FLIP_XY layout and the flip screen
static inline int bg_pixel(const uint8_t *vram, int flip, int sx, int sy, uint8_t *code_out)
{
	int tx = flip ? 255 - sx : sx, ty = flip ? 255 - sy : sy;
	int idx = (31 - (ty >> 3)) * 32 + (31 - (tx >> 3));
	if (code_out)
		*code_out = (uint8_t)idx;
	return s_chars[vram[idx] * 64 + (ty & 7) * 8 + (tx & 7)];
}

// sprite drawing order of draw_sprite_and_check_collision; plot(sx, sy) for every lit pixel
#define SPRITE_LOOP(SPR, FLIP, BODY) do { \
	const uint8_t *gfx_ = s_gfx2 + (((SPR)[0] & 0x0f) << 7); \
	uint8_t sy = (uint8_t)((SPR)[2] - 32); \
	if (FLIP) sy++; \
	for (int y = 0x1f; y >= 0; y--) { \
		uint8_t data = 0; \
		uint8_t sx = (FLIP) ? (uint8_t)((SPR)[1] + 1) : (SPR)[1]; \
		for (int x = 0x1f; x >= 0; x--) { \
			if ((x & 7) == 7) \
				data = sy >= 0xe0 ? 0 : gfx_[((x >> 3) << 5) | y]; \
			if (data & 0x80) { BODY } \
			sx++; \
			data <<= 1; \
		} \
		sy++; \
	} \
} while (0)

static void compute_collision(void)
{
	s_coll = 0xffff;
	SPRITE_LOOP(s_spr, s_flip, {
		if (bg_pixel(s_vram, s_flip, sx, sy, NULL))
			s_coll = (uint16_t)(((((sy ^ 0xff) >> 3) << 5) | ((sx ^ 0xff) >> 3)) + 1);
	});
}

static void cb_render(void)
{
	const snap_t *s = &s_snap;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = 0; y < VIS_H; y++) {
		uint16_t *row = bm + y * pitch;
		for (int x = 0; x < VIS_W; x++) {
			uint8_t idx;
			int px = bg_pixel(s->vram, s->flip, x, y, &idx);
			row[x] = (uint16_t)(((s->cram[idx] & 0x0f) << 1) | px);
		}
	}
	uint16_t pen = (uint16_t)(((s->spr[0] >> 4) << 1) | 1);
	SPRITE_LOOP(s->spr, s->flip, {
		if (sy < VIS_H)
			bm[sy * pitch + sx] = pen;
	});
}

static void vblank(void)
{
	compute_collision();
	rbx_video_wait_idle();
	memcpy(s_snap.vram, s_vram, sizeof(s_vram));
	memcpy(s_snap.cram, s_cram, sizeof(s_cram));
	memcpy(s_snap.spr, s_spr, sizeof(s_spr));
	s_snap.flip = s_flip;
	rbx_video_frame_done();
	if (s_irq_mask)
		s_cpu.irq = 1;
}

// ---- sound (approximation) ----
static double hp(double *state, double *prev_in, double in, double rc)
{
	// one-pole high-pass (DISCRETE_CRFILTER)
	double a = rc / (rc + 1.0 / AUDIO_RATE);
	double out = a * (*state + in - *prev_in);
	*state = out;
	*prev_in = in;
	return out;
}

static int32_t snd_sample(void)
{
	snd_t *n = &s_snd;
	const double dt = 1.0 / AUDIO_RATE;
	double out = 0;
	// music: DISCRETE_NOTE clocked at 16H / 2 = MASTER / 128
	{
		n->note_frac += MASTER_XTAL / 128;
		while (n->note_frac >= AUDIO_RATE) {
			n->note_frac -= AUDIO_RATE;
			if (++n->note_cnt > 255) {
				n->note_cnt = s_music_freq;
				n->note_oct = (n->note_oct + 1) & 7;
			}
		}
		// DAC_R1: bit 1 through 470k, bit 2 through 120k, 470k bias to ground
		double g1 = (n->note_oct & 2) ? 1.0 / 470e3 : 0, g2 = (n->note_oct & 4) ? 1.0 / 120e3 : 0;
		double v = 3.4 * (g1 + g2) / (1.0 / 470e3 + 1.0 / 120e3 + 1.0 / 470e3);
		if (!(s_sound & 0x04))
			v = 0;
		out += hp(&n->hp_music, &n->hp_music_in, v, 50e3 * 1e-6) * 0.5;   // VR3 at 50 %
	}
	// laugh: 555 VCO (~320 Hz) modulated by a 6.9 Hz square through a 10 uF RC
	if (s_sound & 0x40) {
		n->laugh_slow_ph += 6.86 * dt;
		if (n->laugh_slow_ph >= 1) n->laugh_slow_ph -= 1;
		double target = n->laugh_slow_ph < 0.5 ? 1.0 : -1.0;
		n->laugh_cv += (target - n->laugh_cv) * (dt / 0.03);
		double f = 320.0 * (1.0 + 0.35 * n->laugh_cv);
		n->laugh_ph += f * dt;
		if (n->laugh_ph >= 1) n->laugh_ph -= 1;
		out += (n->laugh_ph < 0.5 ? 1.0 : -1.0) * 0.25 * 1.2;   // VR2 at 50 % of 0.5
	} else
		n->laugh_ph = n->laugh_slow_ph = 0;
	// SN76477: one-shot triggered when /ENABLE goes low (EXPLOSION bit low after high)
	uint8_t enable = (s_sound >> 3) & 1;   // 1 = inhibit
	if (n->prev_enable && !enable)
		n->oneshot = 0.8024 * 47e3 * 1e-6 + 0.002079;
	n->prev_enable = enable;
	if (n->oneshot > 0) {
		n->oneshot -= dt;
		n->env += (1.0 - n->env) * (dt / (4.7e3 * 1e-6));   // attack
		if (n->env > 1) n->env = 1;
	} else
		n->env -= n->env * (dt / (220e3 * 1e-6));          // decay
	if (n->env > 0.001 && !enable) {
		double sig;
		if (s_sound & 0x20) {
			// APPEAR: mixer B set -> SLF / VCO
			n->vco_ph += 2000.0 * dt;
			if (n->vco_ph >= 1) n->vco_ph -= 1;
			sig = n->vco_ph < 0.5 ? 1.0 : -1.0;
		} else {
			// noise (clock 24.9 kHz from the 47k resistor), RC filter 330k / 470p
			n->noise_acc += 24900.0 * dt;
			while (n->noise_acc >= 1) {
				n->noise_acc -= 1;
				n->lfsr = (n->lfsr >> 1) | ((((n->lfsr >> 0) ^ (n->lfsr >> 3)) & 1) << 16);
			}
			double target = (n->lfsr & 1) ? 1.0 : -1.0;
			n->noise_flt += (target - n->noise_flt) * (dt / (dt + 330e3 * 470e-12 * 1.1));
			sig = n->noise_flt;
		}
		out += sig * n->env * 0.5 * 2.0;   // route gain 2.0
	}
	if (!(s_sound & 0x02))
		out = 0;   // SOUND STOP
	out = hp(&n->hp_out, &n->hp_out_in, out, 100e3 * 1e-6);
	return (int32_t)(out * 9000);
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

static void machine_reset(void)
{
	cpu_init();
	port_sound_w(0);
	s_music_freq = 0;
	s_wd = 0;
}

static void cb_reset(void)
{
	memset(s_ram, 0, sizeof(s_ram));
	memset(s_vram, 0, sizeof(s_vram));
	memset(s_cram, 0, sizeof(s_cram));
	memset(s_spr, 0, sizeof(s_spr));
	memset(s_pc3092, 0, sizeof(s_pc3092));
	memset(&s_snd, 0, sizeof(s_snd));
	s_snd.lfsr = 0x1ffff;
	s_flip = 0;
	s_coll = 0xffff;
	s_frac = 0;
	memset(&s_cpu, 0, sizeof(s_cpu));
	machine_reset();
}

static void cb_exit(void)
{
	free(s_chars);
	s_chars = NULL;
}

static int cb_init(const rbx_game *g)
{
	uint32_t gsize, g2size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	const uint8_t *gfx1 = rbx_region("gfx1", &gsize);
	s_gfx2 = rbx_region("gfx2", &g2size);
	if (!s_rom || !gfx1 || gsize < 0x800 || !s_gfx2 || g2size < 0x800) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	s_port_dsw0 = rbx_port_find("DSW0");
	s_port_in0 = rbx_port_find("IN0");
	s_port_dsw1 = rbx_port_find("DSW1");
	s_port_in1 = rbx_port_find("IN1");
	s_port_pc3092 = rbx_port_find("PC3092");
	s_chars = (uint8_t *)malloc(256 * 64);
	if (!s_chars) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	// charlayout: 1 bpp, x = bits 7..0, y offsets 7*8 .. 0*8 (rows stored bottom up)
	for (int c = 0; c < 256; c++)
		for (int y = 0; y < 8; y++) {
			uint8_t b = gfx1[c * 8 + (7 - y)];
			for (int x = 0; x < 8; x++)
				s_chars[c * 64 + y * 8 + x] = (uint8_t)((b >> (7 - x)) & 1);
		}
	if (rbx_video_setup(VIS_W, VIS_H, 32) || rbx_audio_setup(AUDIO_RATE)) {
		cb_exit();
		return -1;
	}
	// crbaloon_palette: odd pens = colour code (inverted HALF / B / G / R bits), even pens black
	for (int i = 0; i < 32; i++) {
		int pen = (i & 1) ? (i >> 1) : 0x0f;
		int h = (~pen & 8) ? 0xff : 0x55;
		int r = h * (~pen & 1), gg = h * ((~pen >> 1) & 1), b = h * ((~pen >> 2) & 1);
		rbx_video.palette[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	cb_reset();
	return 0;
}

static void cb_frame(void)
{
	rbx_audio.frame_mhz = RATE_MHZ;   // (the board's frame rate: a frame's sound is played in a 60 Hz frame, as its picture)
	for (int line = 0; line < VTOTAL; line++) {
		if (line == VIS_H) {
			vblank();
			if (++s_wd >= 256) {
				rbx_log("crbaloon: watchdog reset");
				machine_reset();
			}
		}
		unsigned long end = s_cpu.z.cyc + (unsigned long)rbx_line_cycles(&s_frac, CPU_CLOCK, RATE_MHZ, VTOTAL);
		while ((long)(s_cpu.z.cyc - end) < 0) {
			s_cpu.z.int_pending = s_cpu.irq ? 1 : 0;   // level line, cleared only through port 6 D0
			z80_step(&s_cpu.z);
		}
		// AUDIO_RATE samples per second spread over the lines (line rate = RATE_MHZ * VTOTAL / 1000)
		s_snd.line_frac += AUDIO_RATE * 1000u;
		while (s_snd.line_frac >= (uint32_t)RATE_MHZ * VTOTAL) {
			s_snd.line_frac -= (uint32_t)RATE_MHZ * VTOTAL;
			rbx_audio_put(snd_sample());
		}
	}
}

static int cb_state(void *ctx, int load)
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
	rc |= rbx_state_io(ctx, "vram", s_vram, sizeof(s_vram));
	rc |= rbx_state_io(ctx, "cram", s_cram, sizeof(s_cram));
	rc |= rbx_state_io(ctx, "spr", s_spr, sizeof(s_spr));
	rc |= rbx_state_io(ctx, "pc3092", s_pc3092, sizeof(s_pc3092));
	uint8_t m[5] = { s_irq_mask, s_sound, s_music_freq, s_flip, s_coll_clear };
	rc |= rbx_state_io(ctx, "misc", m, sizeof(m));
	rc |= rbx_state_io(ctx, "coll", &s_coll, sizeof(s_coll));
	rc |= rbx_state_io(ctx, "wd", &s_wd, sizeof(s_wd));
	rc |= rbx_state_io(ctx, "frac", &s_frac, sizeof(s_frac));
	rc |= rbx_state_io(ctx, "snd", &s_snd, sizeof(s_snd));
	if (!rc && load) {
		s_irq_mask = m[0]; s_sound = m[1]; s_music_freq = m[2]; s_flip = m[3]; s_coll_clear = m[4];
	}
	return rc ? -1 : 0;
}

static int cb_save(void *ctx) { return cb_state(ctx, 0); }
static int cb_load(void *ctx) { return cb_state(ctx, 1); }
static int cb_supported(const char *machine) { return !strcmp(machine, "crbaloon"); }

const rbx_driver rbx_drv_crbaloon = { "Crazy Balloon", cb_init, cb_exit, cb_reset, cb_frame, cb_save, cb_load,
                                      cb_supported, cb_render };

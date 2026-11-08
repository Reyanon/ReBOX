/*  Konami Tutankham. Ported from MAME src/mame/konami/tutankhm.cpp and tutankhm_v.cpp (BSD-3-Clause,
    copyright-holders: Mirko Buffoni), the sound board of src/mame/shared/timeplt_a.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and the palette's resistor networks of src/emu/video/resnet.cpp (BSD-3-Clause,
    copyright-holders: Couriersud). A 6809E at 1.536 MHz with a 4 KB ROM bank, a 256x256 4-bit bitmap (the left 192
    columns scrolled), 16 colours in RAM, the starfield (both of MAME's: the Scramble one by default) on a 768-wide
    screen (3 pixels a bitmap column); the sound: a Z80 at 1.79 MHz and two AY-3-8910s, a frame behind on the worker
    (each channel through its RC filter).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/filter_rc.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12)
#define Z80_CLOCK 1789772u     // (14.318 MHz / 8; the AYs too)
#define SND_RATE 48000
#define PIX_CLOCK 18432000u    // (set_raw(GALAXIAN_PIXEL_CLOCK, 1152, 0, 768, 264, 16, 240))
#define HTOTAL 1152
#define LINES 264
#define VIS_W 768
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x80             // (0-15 the palette RAM, 16-79 the star colours)
#define LINE_NS 62500u         // (1152 / 18.432 MHz)
#define BLINK_NS 831600000u    // (PERIOD_OF_555_ASTABLE(100000, 10000, 10 uF))
#define STAR_RNG_PERIOD ((1 << 17) - 1)

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_stars[2];   // (the LFSR's stars: [0] the bootleg's, [1] Scramble's)
static uint32_t s_raw_rgb[256], s_star_rgb[64];
static m6809_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static ay8910_t s_ay[2];
static int s_port_dsw[2], s_port_in[3], s_port_stars;

static struct {
	uint8_t vram[0x8000];    // 0000-7fff
	uint8_t pal[16];         // 8000-800f (mirror 00f0)
	uint8_t scroll;          // 8100 (mirror 000f)
	uint8_t ram[0x800];      // 8800-8fff
	uint8_t latch, bank, irq_enable, irq_toggle, blink;
	uint8_t stars_line[LINES];   // (the stars' enable as the beam passed each line: stars_enable_w's update_partial)
	uint8_t zram[0x400], soundlatch, z80_irq;
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ay_frac, snd_frac, time_ns, blink_next;
	uint32_t frame;
	filter_rc_t flt[2][3];   // (each AY channel's RC filter: filter_w)
	int32_t div_acc[2], div_n[2];   // (the filtered native samples averaged four to one)
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x8000], pal[16], scroll, flipx, flipy, stars_on[LINES], blink, mode; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	s_vs.scroll = t.scroll;
	s_vs.flipx = (t.latch >> 6) & 1;
	s_vs.flipy = (t.latch >> 7) & 1;
	memcpy(s_vs.stars_on, t.stars_line, sizeof(s_vs.stars_on));
	s_vs.blink = t.blink;
	s_vs.mode = s_port_stars >= 0 ? (uint8_t)(rbx_port_read(s_port_stars) & 1) : 1;
}

static void tk_render(void)   // (screen_update: screen_update_scramble / screen_update_bootleg)
{
	for (int i = 0; i < 16; i++)
		rbx_video.palette[i] = s_raw_rgb[s_vs.pal[i]];
	for (int i = 0; i < 64; i++)
		rbx_video.palette[16 + i] = s_star_rgb[i];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	const int xorx = s_vs.flipx ? 255 : 0, xory = s_vs.flipy ? 255 : 0;
	const int blink = s_vs.blink & 3;
	const uint8_t *stars = s_stars[s_vs.mode];
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int i = 0; i < VIS_W; i++)   // (black: the fill, a star of colour 0 the same)
			d[i] = 16;
		if (s_vs.mode) {   // (scramble_draw_stars: stars_draw_row(y, y * 512) over 256 6 MHz pixels)
			if (s_vs.stars_on[y]) {
				const uint8_t flipxor = s_vs.flipx ? 0xc0 : 0x00;
				uint32_t offs = (uint32_t)y * 512;
				for (int x = 0; x < 256; x++) {
					const int h8q = (~x >> 3) & 1;
					int en = (y ^ h8q) & 1, enab = 0;
					switch (blink) {
					case 0: enab = 1; break;
					case 1: enab = y & 1; break;
					case 2: enab = (y >> 1) & 1; break;
					case 3: enab = h8q; break;
					}
					en &= enab && ((x & 0xc0) ^ flipxor) != 0xc0;
					const uint8_t s0 = stars[offs], s1 = stars[offs + 1];
					offs += 2;
					if (en && (s0 & 0x80))
						d[3 * x] = (uint16_t)(16 + (s0 & 0x3f));
					if (en && (s1 & 0x80))
						d[3 * x + 1] = d[3 * x + 2] = (uint16_t)(16 + (s1 & 0x3f));
				}
			}
			for (int x = 0; x < 256; x++) {
				const int effx = x ^ xorx;
				const uint8_t effy = (uint8_t)((y ^ xory) + (effx < 192 ? s_vs.scroll : 0));
				const uint8_t shifted = (uint8_t)(s_vs.vram[effy * 128 + effx / 2] >> (4 * (effx & 1)));
				const uint16_t c = shifted & 0x0f;
				uint16_t *p = d + 3 * x;
				for (int k = 0; k < 3; k++)
					if (shifted || p[k] == 16)
						p[k] = c;
			}
		} else {   // (screen_update_bootleg: the bootleg's 6 MHz stars, from column 64)
			for (int x = 0; x < 256; x++) {
				const int effx = x ^ xorx;
				const uint8_t effy = (uint8_t)((y ^ xory) + (effx < 192 ? s_vs.scroll : 0));
				const uint8_t shifted = (uint8_t)(s_vs.vram[effy * 128 + effx / 2] >> (4 * (effx & 1)));
				int enab = 0;
				switch (blink) {
				case 0: enab = 1; break;
				case 1: enab = y & 1; break;
				case 2: enab = (y >> 1) & 1; break;
				case 3: enab = (~x >> 3) & 1; break;
				}
				const uint8_t star = stars[(uint32_t)(y * 384 + x + 84) % STAR_RNG_PERIOD];
				uint16_t c;
				if (s_vs.stars_on[y] && enab && !(shifted & 2) && (star & 0x80) && x > 63)
					c = (uint16_t)(16 + (star & 0x3f));
				else
					c = shifted & 0x0f;
				d[3 * x] = d[3 * x + 1] = d[3 * x + 2] = c;
			}
		}
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch, the IRQ and the mute queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ, Q_MUTE };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t s_q[2][Q_MAX];
static int s_qn[2], s_qcur;
static int32_t s_out[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;
static uint8_t s_mute;   // (the worker's: mute_w, the LA4460's DC mute)

static void q_put(int kind, uint32_t v)
{
	const int c = s_qcur;
	if (s_qn[c] < Q_MAX) {
		s_q[c][s_qn[c]].kind = (uint16_t)kind;
		s_q[c][s_qn[c]].v = v;
		s_qn[c]++;
	}
}

// ---- the main CPU's bus (main_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static void set_bank(void)
{
	const uint32_t base = 0x10000 + (uint32_t)t.bank * 0x1000;
	for (int p = 0; p < 16; p++)
		s_cpu.rpage[0x90 + p] = s_rom + (base + (uint32_t)p * 256) % s_rom_size;
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x8000)
		return t.vram[a];
	if (a < 0x8100)
		return t.pal[a & 0x0f];
	if (a < 0x8200) {
		switch (a & 0xfff0) {
		case 0x8100: return t.scroll;
		case 0x8160: return port(s_port_dsw[1]);
		case 0x8180: return port(s_port_in[0]);
		case 0x81a0: return port(s_port_in[1]);
		case 0x81c0: return port(s_port_in[2]);
		case 0x81e0: return port(s_port_dsw[0]);
		}
		return 0;
	}
	if (a >= 0x8800 && a < 0x9000)
		return t.ram[a - 0x8800];
	if (a >= 0x9000 && a < 0xa000)
		return s_rom[(0x10000 + (uint32_t)t.bank * 0x1000 + (a & 0xfff)) % s_rom_size];
	if (a >= 0xa000)
		return s_rom[a % s_rom_size];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x8000)
		t.vram[a] = v;
	else if (a < 0x8100)
		t.pal[a & 0x0f] = v;
	else if (a < 0x8110)
		t.scroll = v;
	else if (a >= 0x8200 && a < 0x8300) {   // (the LS259: Q0 the IRQ enable, Q4 the stars, Q5 the mute, Q6 / Q7 the flips)
		const int bit = a & 7, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 0) {
			t.irq_enable = (uint8_t)now;
			if (!now)
				m6809_set_irq(&s_cpu, 0);
		} else if (bit == 5)
			q_put(Q_MUTE, (uint32_t)now);
	} else if (a >= 0x8300 && a < 0x8400) {
		t.bank = v & 0x0f;
		set_bank();
	} else if (a >= 0x8600 && a < 0x8700)   // (sound_on_w: sh_irqtrigger_w(0), sh_irqtrigger_w(1))
		q_put(Q_IRQ, 0);
	else if (a >= 0x8700 && a < 0x8800)
		q_put(Q_LATCH, v);
	else if (a >= 0x8800 && a < 0x9000)
		t.ram[a - 0x8800] = v;
}

// ---- the sound board (timeplt_sound_map) ----
static const int s_timer[10] = { 0x00, 0x10, 0x20, 0x30, 0x40, 0x90, 0xa0, 0xb0, 0xa0, 0xd0 };
static uint8_t ay_port_a(void *u) { (void)u; return t.soundlatch; }
static uint8_t ay_port_b(void *u) { (void)u; return (uint8_t)s_timer[(s_z.cyc / 512) % 10]; }   // (portB_r)

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x3000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x4000)
		return t.zram[a & 0x3ff];
	if (a < 0x5000)
		return ay8910_data_r(&s_ay[0]);
	if (a >= 0x6000 && a < 0x7000)
		return ay8910_data_r(&s_ay[1]);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x3000 && a < 0x4000)
		t.zram[a & 0x3ff] = v;
	else if (a >= 0x4000 && a < 0x5000)
		ay8910_data_w(&s_ay[0], v);
	else if (a >= 0x5000 && a < 0x6000)
		ay8910_address_w(&s_ay[0], v);
	else if (a >= 0x6000 && a < 0x7000)
		ay8910_data_w(&s_ay[1], v);
	else if (a >= 0x7000 && a < 0x8000)
		ay8910_address_w(&s_ay[1], v);
	else if (a >= 0x8000) {   // (filter_w: the address lines choose each channel's capacitors, 0.22 uF and 0.047 uF)
		const uint32_t o = a - 0x8000u;
		for (int i = 0; i < 6; i++) {
			const int d = (int)(o >> (2 * i)) & 3;
			const double c = ((d & 1) ? 220000e-12 : 0) + ((d & 2) ? 47000e-12 : 0);
			filter_rc_lowpass_3r(&t.flt[i < 3 ? 1 : 0][i % 3], 1000, 5100, 0, c, Z80_CLOCK / 8.0);
		}
	}
}

// an AY's channels at 0.60 through their filters at its own rate, four native samples to one (the old divider)
static int ay_filtered(int c, uint32_t clocks, int32_t *dst, int max)
{
	static int16_t ch[3 * 512];
	const int nn = ay8910_render_ch(&s_ay[c], clocks, ch, 512);
	int n = 0;
	for (int i = 0; i < nn; i++) {
		t.div_acc[c] += (filter_rc_step(&t.flt[c][0], ch[3 * i]) + filter_rc_step(&t.flt[c][1], ch[3 * i + 1]) +
		                 filter_rc_step(&t.flt[c][2], ch[3 * i + 2])) * 154 >> 8;
		if (++t.div_n[c] == 4) {
			if (n < max)
				dst[n++] = t.div_acc[c] / 4;
			t.div_acc[c] = 0;
			t.div_n[c] = 0;
		}
	}
	return n;
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, the two AYs' channels 0.60 each)
{
	if (n <= 0)
		return;
	t.z_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const unsigned long cyc = (unsigned long)(t.z_frac / SND_RATE);
	t.z_frac -= (uint64_t)cyc * SND_RATE;
	int line = t.z80_irq;
	t.z_end += cyc;
	z80_run(&s_z, (unsigned long)t.z_end, &line, 1);
	t.z80_irq = (uint8_t)line;
	if (n > 64)
		n = 64;
	int32_t a1[96], a2[96];
	memset(a1, 0, sizeof(a1));
	memset(a2, 0, sizeof(a2));
	t.ay_frac += (uint64_t)Z80_CLOCK * (uint64_t)n;
	const uint32_t ac = (uint32_t)(t.ay_frac / SND_RATE);
	t.ay_frac -= (uint64_t)ac * SND_RATE;
	const int n1 = ay_filtered(0, ac, a1, 96), n2 = ay_filtered(1, ac, a2, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++)
		s_out[s_out_pos++] = s_mute ? 0 : (n1 > 0 ? a1[i * n1 / n] : 0) + (n2 > 0 ? a2[i * n2 / n] : 0);
}

static void job(void)
{
	if (!__atomic_load_n(&s_job_frame, __ATOMIC_ACQUIRE))
		return;
	const int b = s_qcur ^ 1;
	s_out_pos = 0;
	for (int i = 0; i < s_qn[b]; i++) {
		const q_ev_t *e = &s_q[b][i];
		switch (e->kind) {
		case Q_ADV: sound_run((int)e->v); break;
		case Q_LATCH: t.soundlatch = (uint8_t)e->v; break;
		case Q_IRQ: t.z80_irq = 1; break;
		case Q_MUTE: s_mute = (uint8_t)e->v; break;
		}
	}
	s_qn[b] = 0;
	s_out_n = s_out_pos;
	__atomic_store_n(&s_job_frame, 0, __ATOMIC_RELEASE);
}

static void sound_frame(void)
{
	rbx_aux_sync();
	for (int i = 0; i < s_out_n; i++)
		rbx_audio_put(s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void tk_exit(void);
static void tk_reset(void);

// MAME compute_resistor_weights(0, 224, -1.0, ...): each resistor's output alone, all the networks scaled so that
// the one with the greatest output gives 224 (RGB_MAXIMUM)
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

static void stars_init(uint8_t *st, int scramble)   // (stars_init_bootleg / stars_init_scramble)
{
	uint32_t shiftreg = 0;
	for (int i = 0; i < STAR_RNG_PERIOD; i++) {
		const int newbit = (int)(((shiftreg >> 12) ^ ~shiftreg) & 1);
		const int enabled = scramble ? (shiftreg & 0x1fe01) == 0x1fe00 : ((shiftreg & 0x1fe00) == 0x1fe00) && newbit == 0;
		const int color = (int)((~shiftreg & 0x1f8) >> 3);
		st[i] = (uint8_t)(color | (enabled << 7));
		shiftreg = (shiftreg >> 1) | ((uint32_t)newbit << 16);
	}
}

static int tk_init(const rbx_game *g)
{
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("timeplt_audio:tpsound", &s_zrom_size);
	if (!s_rom || s_rom_size < 0x20000 || !s_zrom) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_stars[0] = (uint8_t *)malloc(STAR_RNG_PERIOD + 2);
	s_stars[1] = (uint8_t *)malloc(STAR_RNG_PERIOD + 2);
	if (!s_stars[0] || !s_stars[1]) {
		rbx_fail("%s: out of memory", g->set);
		tk_exit();
		return -1;
	}
	stars_init(s_stars[0], 0);
	stars_init(s_stars[1], 1);
	// (raw_to_rgb_func: the red and green 1k / 470 / 220, the blue 470 / 220, all on a 470 pull-down)
	static const int res_rg[3] = { 1000, 470, 220 }, res_b[2] = { 470, 220 };
	double wr[3], wb[2];
	const double srg = raw_weights(res_rg, 3, 470, wr), sb = raw_weights(res_b, 2, 470, wb);
	const double scale = 224.0 / (srg > sb ? srg : sb);
	for (int i = 0; i < 256; i++) {
		const int r = (int)(scale * (wr[0] * (i & 1) + wr[1] * ((i >> 1) & 1) + wr[2] * ((i >> 2) & 1)) + 0.5);
		const int gg = (int)(scale * (wr[0] * ((i >> 3) & 1) + wr[1] * ((i >> 4) & 1) + wr[2] * ((i >> 5) & 1)) + 0.5);
		const int b = (int)(scale * (wb[0] * ((i >> 6) & 1) + wb[1] * ((i >> 7) & 1)) + 0.5);
		s_raw_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	// (galaxian_palette: the stars' 150 / 100 ohm levels compressed into 194-255)
	const int minval = 224 * 130 / 150, midval = 224 * 130 / 100, maxval = 224 * 130 / 60;
	const int starmap[4] = { 0, minval, minval + (255 - minval) * (midval - minval) / (maxval - minval), 255 };
	for (int i = 0; i < 64; i++) {
		const int r = starmap[((i >> 4) & 1) << 1 | ((i >> 5) & 1)];
		const int gg = starmap[((i >> 2) & 1) << 1 | ((i >> 3) & 1)];
		const int b = starmap[(i & 1) << 1 | ((i >> 1) & 1)];
		s_star_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, Z80_CLOCK, AY8910_LEGACY_OUTPUT, 154, 4, 0);   // (0.60)
	s_port_dsw[0] = rbx_port_find("DSW1");
	s_port_dsw[1] = rbx_port_find("DSW2");
	s_port_in[0] = rbx_port_find("IN0");
	s_port_in[1] = rbx_port_find("IN1");
	s_port_in[2] = rbx_port_find("IN2");
	s_port_stars = rbx_port_find("STARS");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		tk_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	tk_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void tk_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_stars[0]);
	free(s_stars[1]);
	s_stars[0] = s_stars[1] = NULL;
}

static void tk_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	s_mute = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.blink = 1;   // (the periodic timer's first tick at time 0; our frame starts 24 lines after MAME's vblank phase)
	t.blink_next = BLINK_NS + 24 * LINE_NS;
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	for (int i = 0; i < 2; i++)
		for (int c = 0; c < 3; c++)
			t.flt[i][c].k = 0x10000;
	s_ay[0].port_a_read = ay_port_a;
	s_ay[0].port_b_read = ay_port_b;
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0; p < 0x80; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.vram + p * 256;
	for (int p = 0x88; p < 0x90; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x88) * 256;
	set_bank();
	for (int p = 0xa0; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x30; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0x30; p < 0x40; p++)
		s_zmap[p] = t.zram + (p & 3) * 256;
	s_z.read_map = s_zmap;
}

static void tk_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq every other frame)
			capture();
			rbx_video_frame_done();
			t.irq_toggle ^= 1;
			if (t.irq_toggle && t.irq_enable)
				m6809_set_irq(&s_cpu, 1);
		}
		t.stars_line[vpos] = (t.latch >> 4) & 1;   // (a write in this line leaves it drawn with the old value)
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.time_ns += LINE_NS;   // (scramble_stars_blink_timer)
		while (t.time_ns >= t.blink_next) {
			t.blink++;
			t.blink_next += BLINK_NS;
		}
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int tk_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "tk_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "tk", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "tk_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "tk_ay1", &s_ay[1], ay8910_state_size());
	int32_t qn = s_out_n, mute = s_mute;
	rc |= rbx_state_io(ctx, "tk_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "tk_mute", &mute, sizeof(mute));
	rc |= rbx_state_io(ctx, "tk_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "tk_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_mute = (uint8_t)(mute & 1);
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		set_bank();
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int tk_save(void *ctx) { return tk_state(ctx, 0); }
static int tk_load(void *ctx) { return tk_state(ctx, 1); }
static int tk_supported(const char *machine) { return !strcmp(machine, "tutankhm"); }

const rbx_driver rbx_drv_tutankhm = { "Konami Tutankham", tk_init, tk_exit, tk_reset, tk_frame, tk_save, tk_load,
                                      tk_supported, tk_render, NULL };

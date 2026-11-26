/*  Konami Pooyan. Ported from MAME src/mame/konami/pooyan.cpp (BSD-3-Clause, copyright-holders: Allard van der
    Bas, Mike Cuddy, Nicola Salmoria), the sound board of src/mame/shared/timeplt_a.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and the palette's resistor networks of src/emu/video/resnet.cpp (BSD-3-Clause,
    copyright-holders: Couriersud). A Z80 at 3.072 MHz (its NMI at vblank), a 32x32 tilemap of 4 bpp tiles, the
    sprites, the colours through the lookup PROMs; the sound: a Z80 at 1.79 MHz and two AY-3-8910s, a frame behind on
    the worker (each channel through its RC filter). The flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/ay8910.h"
#include "../../sound/filter_rc.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3072000u     // (18.432 MHz / 3 / 2)
#define Z80_CLOCK 1789772u     // (14.318 MHz / 8; the AYs too)
#define SND_RATE 48000
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200

static uint8_t *s_rom, *s_zrom, *s_prom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static z80 s_m, s_z;
static const uint8_t *s_mmap[256], *s_zmap[256];
static ay8910_t s_ay[2];
static int s_port_in[3], s_port_dsw[2];

static struct {
	uint8_t vram[0x800];     // 8000-87ff (8000 the colour RAM, 8400 the video RAM)
	uint8_t ram[0x800];      // 8800-8fff
	uint8_t spr[2][0x100];   // 9000-90ff, 9400-94ff (mirrored to 9fff)
	uint8_t latch, irq_enable, nmi_line;
	uint8_t zram[0x400], soundlatch, z80_irq, mute;
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ay_frac, snd_frac;
	uint32_t frame;
	filter_rc_t flt[2][3];   // (each AY channel's RC filter: filter_w)
	int32_t div_acc[2], div_n[2];   // (the filtered native samples averaged four to one)
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t vram[0x800], s0[0x40], s1[0x40]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(s_vs.s0, t.spr[0], sizeof(s_vs.s0));
	memcpy(s_vs.s1, t.spr[1], sizeof(s_vs.s1));
}

static void py_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the characters' 16-31, 100-1ff the sprites' 0-15)
		rbx_video.palette[i] = s_rgb[(s_prom[0x20 + i] & 0x0f) | 0x10];
		rbx_video.palette[0x100 + i] = s_rgb[s_prom[0x120 + i] & 0x0f];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			const int idx = (y >> 3) * 32 + col;
			const int attr = s_vs.vram[idx];
			const int code = s_vs.vram[0x400 + idx];
			const int pen = (attr & 0x0f) * 16;
			const int fx = (attr & 0x40) != 0, fy = (attr & 0x80) != 0;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(pen + src[fx ? 7 - i : i]);
		}
	}
	for (int offs = 0x10; offs < 0x40; offs += 2) {   // (draw_sprites: transmask of the pens mapping to colour 0)
		const int sx = s_vs.s0[offs], sy = 240 - s_vs.s1[offs + 1];
		const int color = s_vs.s1[offs] & 0x0f;
		const int flipx = !(s_vs.s1[offs] & 0x40), flipy = (s_vs.s1[offs] & 0x80) != 0;
		const uint8_t *src = s_spr + (rbx_wrap((uint32_t)s_vs.s0[offs + 1], s_nspr)) * 256;
		const uint8_t *lut = s_prom + 0x120 + color * 16;
		for (int j = 0; j < 16; j++) {
			const int yy = sy + j;
			if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
				continue;
			const uint8_t *s = src + (flipy ? 15 - j : j) * 16;
			uint16_t *d = bm + (yy - VIS_Y0) * pitch;
			for (int k = 0; k < 16; k++) {
				const int xx = sx + k;
				if (xx >= VIS_W)
					break;
				const int p = s[flipx ? 15 - k : k];
				if (lut[p] & 0x0f)
					d[xx] = (uint16_t)(0x100 + color * 16 + p);
			}
		}
	}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch, the IRQ and the mute queued ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ, Q_MUTE };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out[2048];
static int s_out_n, s_out_pos;
static volatile int s_job_frame;

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

static uint8_t m_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0;
	if (a < 0x8800)
		return t.vram[a - 0x8000];
	if (a < 0x9000)
		return t.ram[a - 0x8800];
	if (a < 0xa000)
		return t.spr[(a >> 10) & 1][a & 0xff];
	if ((a & 0xa180) == 0xa000)
		return port(s_port_dsw[1]);
	if ((a & 0xa180) == 0xa080) {
		switch (a & 0x60) {
		case 0x00: return port(s_port_in[0]);
		case 0x20: return port(s_port_in[1]);
		case 0x40: return port(s_port_in[2]);
		default: return port(s_port_dsw[0]);
		}
	}
	return 0;
}

static void latch_w(int bit, int now)   // (the LS259 B2)
{
	const int old = (t.latch >> bit) & 1;
	t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
	switch (bit) {
	case 0:   // (irq_enable_w)
		t.irq_enable = (uint8_t)now;
		if (!now)
			t.nmi_line = 0;
		break;
	case 1:   // (sh_irqtrigger_w: HOLD_LINE on the rising edge)
		if (now && !old)
			q_put(Q_IRQ, 0);
		break;
	case 2:   // (mute_w)
		if (now != old)
			q_put(Q_MUTE, (uint32_t)now);
		break;
	}
}

static void m_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000)
		return;
	if (a < 0x8800)
		t.vram[a - 0x8000] = v;
	else if (a < 0x9000)
		t.ram[a - 0x8800] = v;
	else if (a < 0xa000)
		t.spr[(a >> 10) & 1][a & 0xff] = v;
	else if ((a & 0xa180) == 0xa100)
		q_put(Q_LATCH, v);   // (sound_data_w)
	else if ((a & 0xa180) == 0xa180)
		latch_w(a & 7, v & 1);
}
static uint8_t m_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void m_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

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
		s_out[s_out_pos++] = t.mute ? 0 : (n1 > 0 ? a1[i * n1 / n] : 0) + (n2 > 0 ? a2[i * n2 / n] : 0);
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
		case Q_MUTE: t.mute = (uint8_t)e->v; break;
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
static void py_exit(void);
static void py_reset(void);

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

static int py_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("timeplt_audio:tpsound", &s_zrom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_zrom || !s_prom || ps < 0x220 || !tiles || ts < 0x2000 || !sprites || ss < 0x2000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (charlayout: 256 of 8x8, planes { half + 4, half + 0, 4, 0 }, the 4-pixel columns 8 bytes apart; spritelayout:
	// 64 of 16x16, the columns at bytes 0, 8, 16, 24, the lower half 32 on)
	s_nchr = ts / 2 / 16;
	s_nspr = ss / 2 / 64;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		py_exit();
		return -1;
	}
#define PIX(hi, lo, q) (uint8_t)((((hi) >> (3 - (q))) & 1) << 3 | (((hi) >> (7 - (q))) & 1) << 2 | \
                                 (((lo) >> (3 - (q))) & 1) << 1 | (((lo) >> (7 - (q))) & 1))
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t o = e * 16 + (uint32_t)((x >> 2) * 8 + y);
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = PIX(tiles[ts / 2 + o], tiles[o], x & 3);
			}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint32_t o = e * 64 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = PIX(sprites[ss / 2 + o], sprites[o], x & 3);
			}
#undef PIX
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
	for (int i = 0; i < 2; i++)
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, Z80_CLOCK, AY8910_LEGACY_OUTPUT, 154, 4, 0);   // (0.60)
	static const char *const in[3] = { "IN0", "IN1", "IN2" };
	for (int i = 0; i < 3; i++)
		s_port_in[i] = rbx_port_find(in[i]);
	s_port_dsw[0] = rbx_port_find("DSW0");
	s_port_dsw[1] = rbx_port_find("DSW1");
	if (rbx_audio_setup(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		py_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	py_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void py_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void py_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 2; i++)
		ay8910_reset(&s_ay[i]);
	for (int i = 0; i < 2; i++)
		for (int c = 0; c < 3; c++)
			t.flt[i][c].k = 0x10000;
	s_ay[0].port_a_read = ay_port_a;
	s_ay[0].port_b_read = ay_port_b;
	z80_init(&s_m);
	s_m.read_byte = m_rd;
	s_m.write_byte = m_wr;
	s_m.port_in = m_in;
	s_m.port_out = m_out;
	s_m.int_data = 0xff;
	memset(s_mmap, 0, sizeof(s_mmap));
	for (int p = 0; p < 0x80; p++)
		s_mmap[p] = (uint32_t)p * 256 + 256 <= s_rom_size ? s_rom + p * 256 : NULL;
	for (int p = 0; p < 8; p++) {
		s_mmap[0x80 + p] = t.vram + p * 256;
		s_mmap[0x88 + p] = t.ram + p * 256;
	}
	for (int p = 0x90; p < 0xa0; p++)
		s_mmap[p] = t.spr[(p >> 2) & 1];
	s_m.read_map = s_mmap;
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

static void py_frame(void)
{
	int irq = 0;
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.irq_enable && !t.nmi_line) {   // (ASSERT_LINE: an edge only after irq_enable_w(0))
				t.nmi_line = 1;
				z80_gen_nmi(&s_m);
			}
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		z80_run(&s_m, (unsigned long)t.cpu_end, &irq, 1);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static void z_fix(z80 *z, z80 *cur)
{
	z->read_byte = cur->read_byte;
	z->write_byte = cur->write_byte;
	z->port_in = cur->port_in;
	z->port_out = cur->port_out;
	z->userdata = NULL;
	z->read_map = cur->read_map;
	z->op_rom = NULL;
}

static int py_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "py", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "py_ay0", &s_ay[0], ay8910_state_size());
	rc |= rbx_state_io(ctx, "py_ay1", &s_ay[1], ay8910_state_size());
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "py_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "py_out", s_out, sizeof(s_out));
	z80 m = s_m, z = s_z;
	rc |= rbx_state_io(ctx, "py_cpu", &m, sizeof(m));
	rc |= rbx_state_io(ctx, "py_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z_fix(&m, &s_m);
		z_fix(&z, &s_z);
		s_m = m;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int py_save(void *ctx) { return py_state(ctx, 0); }
static int py_load(void *ctx) { return py_state(ctx, 1); }
static int py_supported(const char *machine) { return !strcmp(machine, "pooyan"); }

const rbx_driver rbx_drv_pooyan = { "Konami Pooyan", py_init, py_exit, py_reset, py_frame, py_save, py_load,
                                    py_supported, py_render, NULL };

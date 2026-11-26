/*  Konami Finalizer: Super Transformation. Ported from MAME src/mame/konami/finalizr.cpp (BSD-3-Clause,
    copyright-holders: Nicola Salmoria) and src/mame/konami/konami1.cpp (the opcode decryption). A KONAMI1 (an encrypted
    6809E) at 1.536 MHz (its IRQ at vblank, NMIs every 32 lines), two 32x32 tilemaps (the foreground only its left 40
    pixels), the sprites, the colours through a resistor network and the lookup PROMs; the sound: an SN76489A written by
    the main CPU, and an M58715 (MCS-48) at 6.144 MHz on a DAC, a frame behind on the worker. The tilemaps' flip screen
    not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../sound/sn76496.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 1536000u     // (18.432 MHz / 12; the SN76489A too)
#define MCS_CYCLES 409600u     // (the M58715 at 18.432 MHz / 3: 15 clocks a machine cycle)
#define SND_RATE 48000
#define PIX_CLOCK 6144000u     // (set_raw(18.432 MHz / 3, 384, 0, 288, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_W 288
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x200

static uint8_t *s_rom, *s_srom, *s_prom;
static uint32_t s_rom_size, s_srom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel: the 8x8 characters (the 8x8 sprites too), the 16x16 sprites)
static uint32_t s_nchr, s_nspr;
static uint32_t s_rgb[32];
static m6809_t s_cpu;
static mcs48 s_mcs;
static sn76496_t s_sn;
static int s_port_dsw[3], s_port_p[2], s_port_system;
static int s_vpos;   // (the line being run: SYSTEM bit 7 the screen's vblank)

static struct {
	uint8_t ram[0x2000];     // 2000-3fff (2000 / 2800 the colour RAMs, 2400 / 2c00 the video RAMs, 3000 / 3800 the sprites)
	uint8_t scroll, charbank, spriterambank, nmi_enable, irq_enable, flip;
	uint8_t soundlatch, dac;
	int32_t hp_x, hp_y;   // (the output's high-pass)
	int32_t mcs_left;        // (the M58715's machine cycles run past its target)
	int64_t cpu_end;         // (the CPU's line target: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, mcs_frac, sn_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t ram[0x1000], spr[0x200], scroll, charbank, flip; } s_vs;

static void capture(void)
{
	memcpy(s_vs.ram, t.ram, sizeof(s_vs.ram));
	memcpy(s_vs.spr, t.ram + (t.spriterambank ? 0x1800 : 0x1000), sizeof(s_vs.spr));
	s_vs.scroll = t.scroll;
	s_vs.charbank = t.charbank;
	s_vs.flip = t.flip;
}

static void draw_layer(int which, int x0, int x1, int scrollx, uint16_t *bm, int pitch)
{
	const uint8_t *cram = s_vs.ram + which * 0x800, *vram = cram + 0x400;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int x = x0; x <= x1;) {
			const int tx = (x + scrollx) & 255;
			int n = 8 - (tx & 7);
			if (n > x1 - x + 1)
				n = x1 - x + 1;
			const int idx = (y >> 3) * 32 + (tx >> 3);
			const int attr = cram[idx];
			const int code = vram[idx] + ((attr & 0xc0) << 2) + (which ? 0 : s_vs.charbank << 10);
			const int pen = (attr & 0x0f) * 16;
			const int fx = (attr & 0x10) != 0, fy = (attr & 0x20) != 0;
			const uint8_t *src = s_chr + (rbx_wrap((uint32_t)code, s_nchr)) * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < n; i++) {
				const int px = (tx + i) & 7;
				d[x + i] = (uint16_t)(pen + src[fx ? 7 - px : px]);
			}
			x += n;
		}
	}
}

static void draw_gfx(const uint8_t *src, int sz, int pen, int flipx, int flipy, int sx, int sy, uint16_t *bm, int pitch)
{
	for (int j = 0; j < sz; j++) {
		const int yy = sy + j;
		if (yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
			continue;
		const uint8_t *s = src + (flipy ? sz - 1 - j : j) * sz;
		uint16_t *d = bm + (yy - VIS_Y0) * pitch;
		for (int i = 0; i < sz; i++) {
			const int xx = sx + i;
			if (xx < 0 || xx >= VIS_W)
				continue;
			const int p = s[flipx ? sz - 1 - i : i];
			if (p)
				d[xx] = (uint16_t)(pen + p);
		}
	}
}

static void fz_render(void)   // (screen_update)
{
	for (int i = 0; i < 0x100; i++) {   // (palette: 0-ff the characters' 16-31, 100-1ff the sprites' 0-15)
		rbx_video.palette[i] = s_rgb[(s_prom[0x40 + i] & 0x0f) | 0x10];
		rbx_video.palette[0x100 + i] = s_rgb[s_prom[0x140 + i] & 0x0f];
	}
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_layer(0, 0, VIS_W - 1, s_vs.scroll - 40, bm, pitch);
	const uint8_t *sr = s_vs.spr;
	for (int offs = 0; offs <= 0x200 - 5; offs += 5) {   // (draw_sprites)
		int sx = 40 + 1 + sr[offs + 3] - ((sr[offs + 4] & 0x01) << 8), sy = sr[offs + 2];
		int flipx = (sr[offs + 4] & 0x20) != 0, flipy = (sr[offs + 4] & 0x40) != 0;
		int code = sr[offs] + ((sr[offs + 1] & 0x0f) << 8);
		const int pen = 0x100 + ((sr[offs + 1] & 0xf0) >> 4) * 16;
		const int size = sr[offs + 4] & 0x1c;
#define S16(c, x, y) draw_gfx(s_spr + (rbx_wrap((uint32_t)(c), s_nspr)) * 256, 16, pen, flipx, flipy, x, y, bm, pitch)
#define S8(c, x, y) draw_gfx(s_chr + (rbx_wrap((uint32_t)(c), s_nchr)) * 64, 8, pen, flipx, flipy, x, y, bm, pitch)
		if (size >= 0x10) {
			if (s_vs.flip) {
				sx = 256 - sx;
				sy = 224 - sy;
				flipx = !flipx;
				flipy = !flipy;
			}
			S16(code + 0, flipx ? sx + 16 : sx, flipy ? sy + 16 : sy);
			S16(code + 1, flipx ? sx : sx + 16, flipy ? sy + 16 : sy);
			S16(code + 2, flipx ? sx + 16 : sx, flipy ? sy : sy + 16);
			S16(code + 3, flipx ? sx : sx + 16, flipy ? sy : sy + 16);
		} else {
			if (s_vs.flip) {
				sx = ((size & 0x08) ? 280 : 272) - sx;
				sy = ((size & 0x04) ? 248 : 240) - sy;
				flipx = !flipx;
				flipy = !flipy;
			}
			if (size == 0x00)
				S16(code, sx, sy);
			else {
				code = ((code & 0x3ff) << 2) | ((code & 0xc00) >> 10);
				if (size == 0x04) {
					S8(code & ~1, flipx ? sx + 8 : sx, sy);
					S8(code | 1, flipx ? sx : sx + 8, sy);
				} else if (size == 0x08) {
					S8(code & ~2, sx, flipy ? sy + 8 : sy);
					S8(code | 2, sx, flipy ? sy : sy + 8);
				} else
					S8(code, sx, sy);
			}
		}
#undef S16
#undef S8
	}
	draw_layer(1, 0, 39, 0, bm, pitch);   // (the foreground: its left 40 pixels)
}

// ---- the sound a frame behind on the worker (rbx_aux): the latch, the M58715's IRQ, the SN76489A's writes ----
#define Q_MAX 4096
enum { Q_ADV, Q_LATCH, Q_IRQ, Q_SN };
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

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	switch (a) {
	case 0x0800: return port(s_port_dsw[2]);
	case 0x0808: return port(s_port_dsw[1]);
	case 0x0810:
		rbx_port_custom(s_port_system, 0x80, (s_vpos >= VIS_Y0 + VIS_H || s_vpos < VIS_Y0) ? 0x80 : 0);
		return port(s_port_system);
	case 0x0811: return port(s_port_p[0]);
	case 0x0812: return port(s_port_p[1]);
	case 0x0813: return port(s_port_dsw[0]);
	}
	if (a >= 0x2000 && a < 0x4000)
		return t.ram[a - 0x2000];
	if (a >= 0x4000)
		return s_rom[rbx_wrap(a, s_rom_size)];
	return 0;
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a >= 0x2000 && a < 0x4000) {
		t.ram[a - 0x2000] = v;
		return;
	}
	switch (a) {
	case 0x0001: t.scroll = v; break;
	case 0x0003:   // (videoctrl_w)
		t.charbank = v & 3;
		t.spriterambank = (v >> 3) & 1;
		break;
	case 0x0004:   // (flipscreen_w)
		t.nmi_enable = v & 1;
		t.irq_enable = (v >> 1) & 1;
		t.flip = (v >> 3) & 1;
		break;
	case 0x081a: q_put(Q_SN, v); break;
	case 0x081c: q_put(Q_IRQ, 0); break;   // (sound_irq_w: ASSERT_LINE)
	case 0x081d: q_put(Q_LATCH, v); break;
	}
}

// ---- the sound CPU (sound_io_map: every read the latch; P1 the DAC, P2 bit 7 low clears the IRQ) ----
static uint8_t mcs_ext_r(void *u, uint8_t a) { (void)u; (void)a; return t.soundlatch; }
static void mcs_port_w(void *u, int p, uint8_t v)
{
	(void)u;
	if (p == 1)
		t.dac = v;
	else if (p == 2 && !(v & 0x80))
		mcs48_set_irq(&s_mcs, 0);
}

static void sound_run(int n)   // (a line's samples: the M58715, the SN76489A 0.75, the DAC 0.325; mono)
{
	if (n <= 0)
		return;
	t.mcs_frac += (uint64_t)MCS_CYCLES * (uint64_t)n;
	const int cyc = (int)(t.mcs_frac / SND_RATE);
	t.mcs_frac -= (uint64_t)cyc * SND_RATE;
	t.mcs_left += cyc;
	if (t.mcs_left > 0)
		t.mcs_left -= mcs48_execute(&s_mcs, t.mcs_left);
	if (n > 64)
		n = 64;
	int32_t sn[64];
	memset(sn, 0, sizeof(sn));
	t.sn_frac += (uint64_t)CPU_CLOCK * (uint64_t)n;
	const uint32_t sc = (uint32_t)(t.sn_frac / SND_RATE);
	t.sn_frac -= (uint64_t)sc * SND_RATE;
	const int k = sn76496_render(&s_sn, sc, sn, 64);
	const int32_t dac = ((int32_t)t.dac - 128) * 256 * 975 / 1000;   // (the R2R DAC 0.325 x 3, centred)
	for (int i = 0; i < n && s_out_pos < 2048; i++) {   // (a high-pass of about 34 Hz: the DAC idles at 0)
		const int32_t x = (k > 0 ? sn[i * k / n] : 0) + dac;
		t.hp_y = x - t.hp_x + t.hp_y - t.hp_y / 225;
		t.hp_x = x;
		s_out[s_out_pos++] = t.hp_y;
	}
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
		case Q_IRQ: mcs48_set_irq(&s_mcs, 1); break;
		case Q_SN: sn76496_write(&s_sn, (uint8_t)e->v); break;
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
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void fz_exit(void);
static void fz_reset(void);

// MAME compute_resistor_weights(0, 255, -1.0, ...) for three like networks: each resistor's output alone, scaled so
// that all of them on gives 255
static void resistor_weights(const int *res, int n, int pulldown, double *w)
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
		double v = 255.0 * r0 / (r1 + r0);
		w[k] = v < 0 ? 0 : v > 255 ? 255 : v;
		sum += w[k];
	}
	for (int k = 0; k < n; k++)
		w[k] *= 255.0 / sum;
}

static int fz_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t gs, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *gfx = rbx_region("gfx1", &gs);
	if (!s_rom || !s_srom || !s_prom || ps < 0x240 || !gfx) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (gfx_8x8x4_packed_msb; gfx_8x8x4_row_2x2_group_packed_msb: 16x16, the right half 32 bytes on, the lower 64)
	s_nchr = gs / 32;
	s_nspr = gs / 128;
	s_chr = (uint8_t *)malloc((size_t)s_nchr * 64);
	s_spr = (uint8_t *)malloc((size_t)s_nspr * 256);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		fz_exit();
		return -1;
	}
	for (uint32_t e = 0; e < s_nchr; e++)
		for (int i = 0; i < 32; i++) {
			s_chr[e * 64 + (uint32_t)i * 2] = gfx[e * 32 + (uint32_t)i] >> 4;
			s_chr[e * 64 + (uint32_t)i * 2 + 1] = gfx[e * 32 + (uint32_t)i] & 15;
		}
	for (uint32_t e = 0; e < s_nspr; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 16; x++) {
				const uint8_t b = gfx[e * 128 + (uint32_t)((x >> 3) * 32 + (y >> 3) * 64 + (y & 7) * 4 + (x & 7) / 2)];
				s_spr[e * 256 + (uint32_t)(y * 16 + x)] = (x & 1) ? b & 15 : b >> 4;
			}
	static const int res[4] = { 2200, 1000, 470, 220 };
	double w[4];
	resistor_weights(res, 4, 470, w);
#define CW(b0, b1, b2, b3) (int)(w[0] * (b0) + w[1] * (b1) + w[2] * (b2) + w[3] * (b3) + 0.5)
	for (int i = 0; i < 0x20; i++) {   // (the green's bits 4, 6, 5, 7)
		const uint8_t c = s_prom[i], d = s_prom[i + 0x20];
		const int r = CW(c & 1, (c >> 1) & 1, (c >> 2) & 1, (c >> 3) & 1);
		const int gg = CW((c >> 4) & 1, (c >> 6) & 1, (c >> 5) & 1, (c >> 7) & 1);
		const int b = CW(d & 1, (d >> 1) & 1, (d >> 2) & 1, (d >> 3) & 1);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
#undef CW
	sn76496_init(&s_sn, CPU_CLOCK, SN76489A_CONFIG, 576, 32, 1);   // (0.75, x 3: the level measured against MAME)
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		fz_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (60.61 Hz)
	fz_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void fz_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void mcs_setup(void)
{
	s_mcs.rom = s_srom;
	s_mcs.rom_mask = (uint16_t)(s_srom_size - 1);
	s_mcs.userdata = NULL;
	s_mcs.ext_r = mcs_ext_r;
	s_mcs.port_w = mcs_port_w;
}

static void fz_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.dac = 0x80;
	sn76496_reset(&s_sn);
	mcs48_init(&s_mcs, MCS48_I8039);
	mcs_setup();
	mcs48_reset(&s_mcs);
	m6809_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.konami1 = 1;
	for (int p = 0x20; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x20) * 256;
	for (int p = 0x40; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + ((uint32_t)p * 256) % s_rom_size;
	m6809_reset(&s_cpu);
}

static void fz_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		s_vpos = vpos;
		if (vpos == VIS_Y0 + VIS_H) {
			capture();
			rbx_video_frame_done();
		}
		if (vpos == 240 && t.irq_enable)   // (scanline: the vblank IRQ, else the NMI every 32 lines)
			m6809_hold_irq(&s_cpu);
		else if (vpos % 32 == 0 && t.nmi_enable) {
			m6809_set_nmi(&s_cpu, 1);
			m6809_set_nmi(&s_cpu, 0);
		}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		t.cpu_end += cyc;
		m6809_run_until(&s_cpu, t.cpu_end);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int fz_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "fz_cpu", &s_cpu, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "fz", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "fz_mcs", (uint8_t *)&s_mcs + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	rc |= rbx_state_io(ctx, "fz_sn", &s_sn, sn76496_state_size());
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "fz_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "fz_out", s_out, sizeof(s_out));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		mcs_setup();
	}
	return rc ? -1 : 0;
}

static int fz_save(void *ctx) { return fz_state(ctx, 0); }
static int fz_load(void *ctx) { return fz_state(ctx, 1); }
static int fz_supported(const char *machine) { return !strcmp(machine, "finalizr"); }

const rbx_driver rbx_drv_finalizr = { "Konami Finalizer", fz_init, fz_exit, fz_reset, fz_frame, fz_save, fz_load,
                                      fz_supported, fz_render, NULL };

/*  Konami Gyruss. Ported from MAME src/mame/konami/gyruss.cpp (BSD-3-Clause, copyright-holders: Mirko Buffoni, Michael
    Cuddy) and the palette's resistor networks of src/emu/video/resnet.cpp (BSD-3-Clause, copyright-holders: Couriersud).
    A Z80 at 3.07 MHz and a KONAMI1 at 1.54 MHz sharing 2 KB, line by line; a 32x32 tilemap of 2 bpp tiles drawn under
    and, for a part of the tiles, over the sprites, which the sub CPU rewrites during the frame (MAME's update_partial:
    kept here as bands of lines with their sprite RAM); the sound, a frame behind on the worker: a Z80 at 3.58 MHz, five
    AY-3-8910s, an I8039 and a DAC, in stereo through the mixers' weights, the output high-passed (the switched RC
    filters of the first two AYs not done). The flip screen not done.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/z80/z80.h"
#include "../../cpu/mcs48/mcs48.h"
#include "../../sound/ay8910.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define PIX_CLOCK 6144000u     // (18.432 MHz / 3)
#define HTOTAL 396
#define LINES 256              // (58.18 Hz)
#define MAIN_LINE 198          // (3.072 MHz a line)
#define SUB_LINE 99            // (1.536 MHz a line)
#define Z80_CLOCK 3579545u     // (14.318 MHz / 4)
#define AY_CLOCK 1789772u      // (14.318 MHz / 8)
#define MCS_CYCLES 533333u     // (8 MHz / 15)
#define SND_RATE 48000
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 0x140
#define SPR_BYTES 0xc0
#define MAX_BANDS 64

static uint8_t *s_rom, *s_srom, *s_zrom, *s_drom, *s_prom;
static uint32_t s_rom_size, s_srom_size, s_zrom_size, s_drom_size;
static uint8_t *s_chr, *s_spr;   // (a byte a pixel)
static uint32_t s_rgb[32];
static z80 s_m, s_z;
static m6809_t s_sub;
static mcs48 s_mcs;
static const uint8_t *s_mmap[256], *s_zmap[256];
static ay8910_t s_ay[5];
static int s_port_dsw[3], s_port_p[2], s_port_system;
static int s_vpos;

static struct {
	uint8_t cram[0x400], vram[0x400], ram[0x1000], shared[0x800], subram[0x800];   // (subram + 0x40: the sprites)
	uint8_t latch, nmi_mask, nmi_line, sub_irq_mask;
	uint8_t zram[0x400], soundlatch, soundlatch2, z80_irq, dac;
	int32_t hpl_x, hpl_y, hpr_x, hpr_y;   // (the outputs' high-pass)
	int32_t mcs_left;
	int64_t main_end, sub_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t z_frac, z_end, ay_frac, mcs_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// the bands of the frame being run: the sprite RAM from each start line on (update_partial before each write)
static struct { int start; uint8_t spr[SPR_BYTES]; } s_bands[MAX_BANDS];
static int s_nbands;

// ---- the drawing's snapshot ----
static struct {
	uint8_t cram[0x400], vram[0x400];
	int nbands, start[MAX_BANDS];
	uint8_t spr[MAX_BANDS][SPR_BYTES];
} s_vs;

static void capture(void)
{
	memcpy(s_vs.cram, t.cram, sizeof(s_vs.cram));
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	s_vs.nbands = s_nbands;
	for (int i = 0; i < s_nbands; i++) {
		s_vs.start[i] = s_bands[i].start;
		memcpy(s_vs.spr[i], s_bands[i].spr, SPR_BYTES);
	}
}

static void draw_tiles(int over, uint16_t *bm, int pitch)   // (over: only the group 0 tiles, the rest see-through)
{
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		for (int col = 0; col < 32; col++) {
			const int idx = (y >> 3) * 32 + col;
			const int c = s_vs.cram[idx];
			if (over && !(c & 0x10))
				continue;
			const int code = ((c & 0x20) << 3) | s_vs.vram[idx];
			const int pen = 0x100 + (c & 0x0f) * 4;
			const int fx = (c & 0x40) != 0, fy = (c & 0x80) != 0;
			const uint8_t *src = s_chr + (uint32_t)code * 64 + (fy ? 7 - (y & 7) : (y & 7)) * 8;
			for (int i = 0; i < 8; i++)
				d[col * 8 + i] = (uint16_t)(pen + src[fx ? 7 - i : i]);
		}
	}
}

static void gy_render(void)   // (screen_update, band by band for the sprites)
{
	for (int i = 0; i < NPENS; i++)   // (palette: 0-ff the sprites, 100-13f the tiles, with the PROM's upper 16 colours)
		rbx_video.palette[i] = s_rgb[(s_prom[0x20 + i] & 0x0f) | (i >= 0x100 ? 0x10 : 0)];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	draw_tiles(0, bm, pitch);
	for (int b = 0; b < s_vs.nbands; b++) {
		const int y0 = s_vs.start[b], y1 = b + 1 < s_vs.nbands ? s_vs.start[b + 1] - 1 : VIS_Y0 + VIS_H - 1;
		const uint8_t *sr = s_vs.spr[b];
		for (int offs = 0xbc; offs >= 0; offs -= 4) {   // (draw_sprites)
			const int x = sr[offs], y = 241 - sr[offs + 3];
			const int bank = sr[offs + 1] & 1;
			const int code = ((sr[offs + 2] & 0x20) << 2) | (sr[offs + 1] >> 1);
			const int pen = (sr[offs + 2] & 0x0f) * 16;
			const int fx = !(sr[offs + 2] & 0x40), fy = (sr[offs + 2] & 0x80) != 0;
			const uint8_t *src = s_spr + (uint32_t)((bank << 8) | code) * 128;
			for (int j = 0; j < 16; j++) {
				const int yy = y + j;
				if (yy < y0 || yy > y1 || yy < VIS_Y0 || yy >= VIS_Y0 + VIS_H)
					continue;
				const uint8_t *s = src + (fy ? 15 - j : j) * 8;
				uint16_t *d = bm + (yy - VIS_Y0) * pitch;
				for (int k = 0; k < 8; k++) {
					const int xx = x + k;
					if (xx >= VIS_W)
						continue;
					const int p = s[fx ? 7 - k : k];
					if (p)
						d[xx] = (uint16_t)(pen + p);
				}
			}
		}
	}
	draw_tiles(1, bm, pitch);
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_outl[2048], s_outr[2048];
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

// ---- the main CPU's bus (main_cpu1_map) ----
static uint8_t port(int p) { return (uint8_t)rbx_port_read(p); }

static uint8_t main_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_rom_size ? s_rom[a] : 0;
	if (a < 0x8400)
		return t.cram[a - 0x8000];
	if (a < 0x8800)
		return t.vram[a - 0x8400];
	if (a >= 0x9000 && a < 0xa000)
		return t.ram[a - 0x9000];
	if (a >= 0xa000 && a < 0xa800)
		return t.shared[a - 0xa000];
	switch (a) {
	case 0xc000: return port(s_port_dsw[1]);
	case 0xc080: return port(s_port_system);
	case 0xc0a0: return port(s_port_p[0]);
	case 0xc0c0: return port(s_port_p[1]);
	case 0xc0e0: return port(s_port_dsw[0]);
	case 0xc100: return port(s_port_dsw[2]);
	}
	return 0;
}

static void main_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x8000)
		return;
	if (a < 0x8400)
		t.cram[a - 0x8000] = v;
	else if (a < 0x8800)
		t.vram[a - 0x8400] = v;
	else if (a >= 0x9000 && a < 0xa000)
		t.ram[a - 0x9000] = v;
	else if (a >= 0xa000 && a < 0xa800)
		t.shared[a - 0xa000] = v;
	else if (a == 0xc080)
		q_put(Q_IRQ, 0);   // (sh_irqtrigger_w: HOLD_LINE)
	else if (a == 0xc100)
		q_put(Q_LATCH, v);
	else if (a >= 0xc180 && a < 0xc188) {   // (the LS259: Q0 the NMI's mask)
		const int bit = a & 7, now = v & 1;
		t.latch = (uint8_t)((t.latch & ~(1 << bit)) | (now << bit));
		if (bit == 0) {
			t.nmi_mask = (uint8_t)now;
			if (!now)
				t.nmi_line = 0;
		}
	}
}
static uint8_t m_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void m_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// ---- the sub CPU (main_cpu2_map) ----
static uint8_t sub_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x0000)
		return (uint8_t)s_vpos;   // (scanline_r)
	if (a >= 0x4000 && a < 0x4800)
		return t.subram[a - 0x4000];
	if (a >= 0x6000 && a < 0x6800)
		return t.shared[a - 0x6000];
	if (a >= 0xe000)
		return s_srom[rbx_wrap(a, s_srom_size)];
	return 0;
}
static void sub_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x2000) {   // (slave_irq_mask_w)
		t.sub_irq_mask = v & 1;
		if (!t.sub_irq_mask)
			m6809_set_irq(&s_sub, 0);
	} else if (a >= 0x4040 && a < 0x4100) {   // (spriteram_w: update_partial(vpos) first)
		const int o = a - 0x4040;
		if (s_vpos >= VIS_Y0 && s_vpos < VIS_Y0 + VIS_H - 1 && s_nbands > 0 && s_bands[s_nbands - 1].start != s_vpos + 1 &&
		    s_nbands < MAX_BANDS) {
			s_bands[s_nbands].start = s_vpos + 1;
			memcpy(s_bands[s_nbands].spr, t.subram + 0x40, SPR_BYTES);
			s_nbands++;
		}
		t.subram[a - 0x4000] = v;
		if (s_nbands > 0 && s_bands[s_nbands - 1].start > s_vpos)
			s_bands[s_nbands - 1].spr[o] = v;
	} else if (a >= 0x4000 && a < 0x4800)
		t.subram[a - 0x4000] = v;
	else if (a >= 0x6000 && a < 0x6800)
		t.shared[a - 0x6000] = v;
}

// ---- the sound board (audio_cpu1_map, audio_cpu1_io_map) ----
static const uint8_t s_timer[10] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x09, 0x0a, 0x0b, 0x0a, 0x0d };
static uint8_t ay_port_a(void *u) { (void)u; return s_timer[(s_z.cyc / 1024) % 10]; }   // (porta_r)

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x6000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x6400)
		return t.zram[a - 0x6000];
	if (a == 0x8000)
		return t.soundlatch;
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x6000 && a < 0x6400)
		t.zram[a - 0x6000] = v;
}
static uint8_t z_in(z80 *z, uint8_t p)
{
	(void)z;
	if (p < 0x14 && (p & 3) == 1)
		return ay8910_data_r(&s_ay[p >> 2]);
	return 0xff;
}
static void z_out(z80 *z, uint8_t p, uint8_t v)
{
	(void)z;
	if (p < 0x14) {
		if ((p & 3) == 0)
			ay8910_address_w(&s_ay[p >> 2], v);
		else if ((p & 3) == 2)
			ay8910_data_w(&s_ay[p >> 2], v);
	} else if (p == 0x14)
		mcs48_set_irq(&s_mcs, 1);   // (i8039_irq_w: ASSERT_LINE)
	else if (p == 0x18)
		t.soundlatch2 = v;
}

// ---- the I8039 (every I/O read the second latch; P1 the DAC, a write to P2 clears the IRQ) ----
static uint8_t mcs_ext_r(void *u, uint8_t a) { (void)u; (void)a; return t.soundlatch2; }
static void mcs_port_w(void *u, int p, uint8_t v)
{
	(void)u;
	if (p == 1)
		t.dac = v;
	else if (p == 2)
		mcs48_set_irq(&s_mcs, 0);
}

static void sound_run(int n)   // (a line's samples: the Z80, the I8039, then the AYs and the DAC mixed to the two sides)
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
	t.mcs_frac += (uint64_t)MCS_CYCLES * (uint64_t)n;
	const int mc = (int)(t.mcs_frac / SND_RATE);
	t.mcs_frac -= (uint64_t)mc * SND_RATE;
	t.mcs_left += mc;
	if (t.mcs_left > 0)
		t.mcs_left -= mcs48_execute(&s_mcs, t.mcs_left);
	if (n > 16)
		n = 16;
	int32_t a[5][64];
	memset(a, 0, sizeof(a));
	t.ay_frac += (uint64_t)AY_CLOCK * (uint64_t)n;
	const uint32_t ac = (uint32_t)(t.ay_frac / SND_RATE);
	t.ay_frac -= (uint64_t)ac * SND_RATE;
	int k[5];
	for (int c = 0; c < 5; c++)
		k[c] = ay8910_render(&s_ay[c], ac, a[c], 64);
	const int32_t dac = (int32_t)t.dac * 70;   // (0 to 4 V, through 4.7k into the left mixer)
	for (int i = 0; i < n && s_out_pos < 2048; i++) {
#define S(c) (k[c] > 0 ? a[c][i * k[c] / n] : 0)
		const int32_t r = S(0) + S(2) + S(3), l = S(1) + S(4) + dac;
#undef S
		t.hpl_y = l - t.hpl_x + t.hpl_y - t.hpl_y / 225;
		t.hpl_x = l;
		t.hpr_y = r - t.hpr_x + t.hpr_y - t.hpr_y / 225;
		t.hpr_x = r;
		s_outl[s_out_pos] = t.hpl_y;
		s_outr[s_out_pos++] = t.hpr_y;
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
		case Q_IRQ: t.z80_irq = 1; break;
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
		rbx_audio_put_stereo(s_outl[i], s_outr[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void gy_exit(void);
static void gy_reset(void);

// MAME compute_resistor_weights(0, 255, -1.0, ...): each resistor's output alone, all the networks scaled so that
// the one with the greatest output gives 255
static double raw_weights(const int *res, int n, int pulldown, double *w)
{
	double sum = 0;
	for (int k = 0; k < n; k++) {
		double r0 = 1.0 / pulldown, r1 = 1.0 / 1e12;
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

static int gy_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t ts, ss, ps;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("sub", &s_srom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	s_drom = rbx_region("audio2", &s_drom_size);
	s_prom = rbx_region("proms", &ps);
	const uint8_t *tiles = rbx_region("tiles", &ts), *sprites = rbx_region("sprites", &ss);
	if (!s_rom || !s_srom || !s_zrom || !s_drom || !s_prom || ps < 0x220 || !tiles || ts < 0x2000 || !sprites || ss < 0x8000) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	// (charlayout: 512 of 8x8 2 bpp, planes { 4, 0 }, the right half 8 bytes on; spritelayout: 256 of 8x16 at bytes 0
	// and 0x10, planes { 0x4000 + 4, 0x4000, 4, 0 }, the right half 8 bytes on, the lower half 32 on)
	s_chr = (uint8_t *)malloc(512 * 64);
	s_spr = (uint8_t *)malloc(512 * 128);
	if (!s_chr || !s_spr) {
		rbx_fail("%s: out of memory", g->set);
		gy_exit();
		return -1;
	}
	for (uint32_t e = 0; e < 512; e++)
		for (int y = 0; y < 8; y++)
			for (int x = 0; x < 8; x++) {
				const uint8_t b = tiles[e * 16 + (uint32_t)((x >> 2) * 8 + y)];
				const int q = x & 3;
				s_chr[e * 64 + (uint32_t)(y * 8 + x)] = (uint8_t)(((b >> (3 - q)) & 1) << 1 | ((b >> (7 - q)) & 1));
			}
	for (uint32_t e = 0; e < 512; e++)
		for (int y = 0; y < 16; y++)
			for (int x = 0; x < 8; x++) {
				const uint32_t o = (e & 0xff) * 64 + (e >> 8) * 0x10 + (uint32_t)((x >> 2) * 8 + (y & 7) + (y >> 3) * 32);
				const uint8_t b1 = sprites[o & 0x3fff], b2 = sprites[0x4000 + (o & 0x3fff)];
				const int q = x & 3;
				s_spr[e * 128 + (uint32_t)(y * 8 + x)] = (uint8_t)(((b2 >> (3 - q)) & 1) << 3 | ((b2 >> (7 - q)) & 1) << 2 |
				                                                     ((b1 >> (3 - q)) & 1) << 1 | ((b1 >> (7 - q)) & 1));
			}
	static const int res_rg[3] = { 1000, 470, 220 }, res_b[2] = { 470, 220 };
	double wr[3], wb[2];
	const double srg = raw_weights(res_rg, 3, 470, wr), sb = raw_weights(res_b, 2, 470, wb);
	const double scale = 255.0 / (srg > sb ? srg : sb);
	for (int i = 0; i < 0x20; i++) {
		const uint8_t c = s_prom[i];
		const int r = (int)(scale * (wr[0] * (c & 1) + wr[1] * ((c >> 1) & 1) + wr[2] * ((c >> 2) & 1)) + 0.5);
		const int gg = (int)(scale * (wr[0] * ((c >> 3) & 1) + wr[1] * ((c >> 4) & 1) + wr[2] * ((c >> 5) & 1)) + 0.5);
		const int b = (int)(scale * (wb[0] * ((c >> 6) & 1) + wb[1] * ((c >> 7) & 1)) + 0.5);
		s_rgb[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	// (the mixers' weights x 11, 2.2k against 200 ohms and 3 x 0.33 through 3.3k / 3, levelled on MAME's WAV)
	static const int gain[5] = { 172, 188, 113, 113, 124 };
	for (int i = 0; i < 5; i++) {
		ay8910_init(&s_ay[i], AY8910_TYPE_AY8910, AY_CLOCK, 0, gain[i], 4, 0);
		ay8910_set_res_loads(&s_ay[i], 3300, 3300, 3300);
	}
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	s_port_system = rbx_port_find("SYSTEM");
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		gy_exit();
		return -1;
	}
	rbx_audio.frame_mhz = (uint32_t)((uint64_t)PIX_CLOCK * 1000 / ((uint64_t)HTOTAL * LINES));   // (58.18 Hz)
	gy_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void gy_exit(void)
{
	rbx_aux_set_job(NULL);
	free(s_chr);
	free(s_spr);
	s_chr = s_spr = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void gy_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	s_nbands = 0;
	for (int i = 0; i < 5; i++)
		ay8910_reset(&s_ay[i]);
	s_ay[2].port_a_read = ay_port_a;
	z80_init(&s_m);
	s_m.read_byte = main_rd;
	s_m.write_byte = main_wr;
	s_m.port_in = m_in;
	s_m.port_out = m_out;
	s_m.int_data = 0xff;
	memset(s_mmap, 0, sizeof(s_mmap));
	for (int p = 0; p < 0x80; p++)
		s_mmap[p] = (uint32_t)p * 256 + 256 <= s_rom_size ? s_rom + p * 256 : NULL;
	for (int p = 0; p < 4; p++) {
		s_mmap[0x80 + p] = t.cram + p * 256;
		s_mmap[0x84 + p] = t.vram + p * 256;
	}
	for (int p = 0; p < 0x10; p++)
		s_mmap[0x90 + p] = t.ram + p * 256;
	for (int p = 0; p < 8; p++)
		s_mmap[0xa0 + p] = t.shared + p * 256;
	s_m.read_map = s_mmap;
	m6809_init(&s_sub);
	s_sub.read = sub_rd;
	s_sub.write = sub_wr;
	s_sub.konami1 = 1;
	for (int p = 0x41; p < 0x48; p++)   // (4000-40ff through the handler: the sprites)
		s_sub.rpage[p] = s_sub.wpage[p] = t.subram + (p - 0x40) * 256;
	for (int p = 0x60; p < 0x68; p++)
		s_sub.rpage[p] = s_sub.wpage[p] = t.shared + (p - 0x60) * 256;
	for (int p = 0xe0; p < 0x100; p++)
		s_sub.rpage[p] = s_srom + ((uint32_t)p * 256) % s_srom_size;
	m6809_reset(&s_sub);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x60; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 4; p++)
		s_zmap[0x60 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
	mcs48_init(&s_mcs, MCS48_I8039);
	s_mcs.rom = s_drom;
	s_mcs.rom_mask = (uint16_t)(s_drom_size - 1);
	s_mcs.userdata = NULL;
	s_mcs.ext_r = mcs_ext_r;
	s_mcs.port_w = mcs_port_w;
	mcs48_reset(&s_mcs);
}

static void gy_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		s_vpos = vpos;
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the update, vblank_irq)
			capture();
			rbx_video_frame_done();
			if (t.nmi_mask && !t.nmi_line) {   // (ASSERT_LINE: an edge only after the mask's write of 0)
				t.nmi_line = 1;
				z80_gen_nmi(&s_m);
			}
			if (t.sub_irq_mask)
				m6809_set_irq(&s_sub, 1);
		}
		if (vpos == VIS_Y0) {   // (the first band: the sprite RAM at the top of the screen)
			s_nbands = 1;
			s_bands[0].start = VIS_Y0;
			memcpy(s_bands[0].spr, t.subram + 0x40, SPR_BYTES);
		}
		t.main_end += MAIN_LINE;
		int irq = 0;
		z80_run(&s_m, (unsigned long)t.main_end, &irq, 1);
		t.sub_end += SUB_LINE;
		m6809_run_until(&s_sub, t.sub_end);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static void z_fix(z80 *z, uint8_t (*rd)(void *, uint16_t), void (*wr)(void *, uint16_t, uint8_t),
                  uint8_t (*in)(z80 *, uint8_t), void (*out)(z80 *, uint8_t, uint8_t), const uint8_t **map)
{
	z->read_byte = rd;
	z->write_byte = wr;
	z->port_in = in;
	z->port_out = out;
	z->userdata = NULL;
	z->read_map = map;
	z->op_rom = NULL;
}

static int gy_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "gy_sub", &s_sub, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "gy", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "gy_nb", &s_nbands, sizeof(s_nbands));
	rc |= rbx_state_io(ctx, "gy_bands", s_bands, sizeof(s_bands));
	rc |= rbx_state_io(ctx, "gy_mcs", (uint8_t *)&s_mcs + MCS48_STATE_OFFSET, (uint32_t)MCS48_STATE_SIZE);
	for (int i = 0; i < 5; i++) {
		static const char *const tag[5] = { "gy_ay0", "gy_ay1", "gy_ay2", "gy_ay3", "gy_ay4" };
		rc |= rbx_state_io(ctx, tag[i], &s_ay[i], ay8910_state_size());
	}
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "gy_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "gy_outl", s_outl, sizeof(s_outl));
	rc |= rbx_state_io(ctx, "gy_outr", s_outr, sizeof(s_outr));
	z80 m = s_m, z = s_z;
	rc |= rbx_state_io(ctx, "gy_main", &m, sizeof(m));
	rc |= rbx_state_io(ctx, "gy_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z_fix(&m, main_rd, main_wr, m_in, m_out, s_mmap);
		z_fix(&z, z_rd, z_wr, z_in, z_out, s_zmap);
		s_m = m;
		s_z = z;
	}
	return rc ? -1 : 0;
}

static int gy_save(void *ctx) { return gy_state(ctx, 0); }
static int gy_load(void *ctx) { return gy_state(ctx, 1); }
static int gy_supported(const char *machine) { return !strcmp(machine, "gyruss"); }

const rbx_driver rbx_drv_gyruss = { "Konami Gyruss", gy_init, gy_exit, gy_reset, gy_frame, gy_save, gy_load,
                                    gy_supported, gy_render, NULL };

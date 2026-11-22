/*  Konami Contra. Ported from MAME src/mame/konami/contra.cpp (BSD-3-Clause, copyright-holders: Carlos A. Lozano, Phil
    Stroffolino, Jose Tejada, Manuel Abadia) and the devices it uses (k007121.cpp, k007452.cpp). An HD6309E at 3 MHz, two
    K007121s (their three 32x32 tilemaps: the two playfields and the fixed text strip, as the driver's), the indirect
    palette through the colour PROMs; the sound: a 6809E at 1.79 MHz and a YM2151, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/m6809/m6809.h"
#include "../../sound/rbx_ym.h"
#include "../video/k007121.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define SND_CLOCK 1789772u     // (3.579545 MHz / 2)
#define YM_CLOCK 3579545u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 280, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 280
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 4096             // (2 * 8 * 16 * 16)

static uint8_t *s_rom, *s_srom;
static uint32_t s_rom_size, s_srom_size;
static uint8_t *s_gfx[2];        // (the K007121s' 8x8 elements, 4 bpp: one byte per pixel)
static uint32_t s_gfx_count[2];
static uint16_t s_ctab[NPENS];   // (the pens' indirect colours, from the PROMs)
static uint16_t s_transmask[2][128];   // (a colour's pens drawn as transparent: those of indirect colour 0)
static hd6309_t s_cpu;
static m6809_t s_snd;
static k007121_t s_k121[2];
static int s_port_system, s_port_p[2], s_port_dsw[3];

static struct {
	uint8_t ram[0x6000];   // 0000-5fff (0c00-0cff the palette, 2000-2fff and 4000-47ff the tilemaps, 3000 / 5000 sprites)
	uint8_t mainbank;
	uint8_t math[6];       // (the K007452)
	uint16_t mul, quo, rem;
	uint8_t sram[0x800], soundlatch, snd_irq;
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, s_frac, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x100], tiles[3][0x800], ctrl[2][8]; k007121_t k121[2]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.ram + 0x0c00, sizeof(s_vs.pal));
	memcpy(s_vs.tiles[0], t.ram + 0x2000, 0x800);
	memcpy(s_vs.tiles[2], t.ram + 0x2800, 0x800);
	memcpy(s_vs.tiles[1], t.ram + 0x4000, 0x800);
	for (int i = 0; i < 2; i++)
		memcpy(&s_vs.k121[i], &s_k121[i], sizeof(k007121_t));
}

// a tile (get_tile_info / get_tx_tile_info): its code and colour from the chip's control registers
static void tile_info(int which, int tile_index, uint32_t *code, int *color)
{
	const uint8_t *tl = s_vs.tiles[which];
	const uint8_t *c = s_vs.k121[which == 1 ? 1 : 0].ctrl;
	const int attr = tl[tile_index];
	const int bit0 = c[5] & 3, bit1 = (c[5] >> 2) & 3, bit2 = (c[5] >> 4) & 3, bit3 = (c[5] >> 6) & 3;
	int bank = ((attr >> (bit0 + 3)) & 1) | ((attr >> (bit1 + 2)) & 2) | ((attr >> (bit2 + 1)) & 4) | ((attr >> bit3) & 8);
	if (which != 2) {
		const int mask = (c[4] & 0xf0) >> 4;
		bank = (bank & ~mask) | (c[4] & mask);
		bank = ((attr & 0x80) >> 7) | (bank << 1) | ((c[3] & 1) << 5);
	} else
		bank = ((attr & 0x80) >> 7) | (bank << 1);
	*code = (uint32_t)(tl[0x400 + tile_index] + bank * 256);
	*color = ((c[6] & 0x30) * 2 + 16) + (attr & 7);
}

// a 256x256 tilemap of 8x8 tiles at (scrollx, scrolly) within the clip (the screen's coordinates)
static void draw_layer(int which, int scrollx, int scrolly, int opaque, int cx0, int cx1)
{
	const int chip = which == 1 ? 1 : 0;
	const uint8_t *gfx = s_gfx[chip];
	const uint32_t count = s_gfx_count[chip];
	const int base = chip ? 2048 : 0;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		const int ty = (y + scrolly) & 255;
		uint16_t *d = bm + (y - VIS_Y0) * pitch - VIS_X0;
		for (int x = cx0; x <= cx1;) {
			const int tx = (x + scrollx) & 255;
			int n = 8 - (tx & 7);
			if (n > cx1 - x + 1)
				n = cx1 - x + 1;
			uint32_t code;
			int color;
			tile_info(which, (ty >> 3) * 32 + (tx >> 3), &code, &color);
			const uint8_t *src = gfx + (code % count) * 64 + (ty & 7) * 8 + (tx & 7);
			const int pb = base + color * 16;
			for (int i = 0; i < n; i++) {
				const int p = src[i];
				if (p || opaque)
					d[x + i] = (uint16_t)(pb + p);
			}
			x += n;
		}
	}
}

static void sprite_cb(int *code, int *color, int colbank) { (void)code; *color += colbank; }

static void ct_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++) {   // (the indirect palette: xBGR_555, little endian)
		const int e = s_ctab[i];
		const uint16_t v = (uint16_t)(s_vs.pal[2 * e] | s_vs.pal[2 * e + 1] << 8);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	const int bgpen = (s_vs.k121[1].ctrl[6] & 0x30) * 2 + 16;
	uint16_t *bm = rbx_video.bitmap;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			bm[y * rbx_video.pitch + x] = (uint16_t)(0x800 | bgpen << 4);
	// (the playfields right of the text strip's 40 columns; flipped: left of it)
	const int flip = s_vs.k121[0].flipscreen;
	const int c0x0 = flip ? 0 : 40, c0x1 = flip ? VIS_W - 1 - 40 : VIS_W - 1;
	const int c1x0 = flip ? VIS_W - 40 : 0, c1x1 = flip ? VIS_W - 1 : 39;
	draw_layer(1, s_vs.k121[1].ctrl[0] - 40, s_vs.k121[1].ctrl[2], 1, c0x0, c0x1);
	draw_layer(0, s_vs.k121[0].ctrl[0] - 40, s_vs.k121[0].ctrl[2], 0, c0x0, c0x1);
	for (int i = 0; i < 2; i++)
		k007121_sprites_draw(&s_vs.k121[i], s_gfx[i], s_gfx_count[i], i ? 2048 : 0, s_transmask[i], sprite_cb, bm,
		                     rbx_video.pitch, VIS_X0, VIS_Y0, c0x0, VIS_Y0, c0x1, VIS_Y0 + VIS_H - 1, NULL, 0);
	draw_layer(2, 0, 0, 0, c1x0, c1x1);
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
typedef struct { uint16_t kind, arg; uint32_t v; } q_ev_t;
static q_ev_t (*s_q_hp)[2][Q_MAX];
#define s_q (*s_q_hp)
static int s_qn[2], s_qcur;
static int32_t s_out_l[2048], s_out_r[2048];
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

// ---- the main CPU's bus (main_map: unmapped reads 0xff) ----
static void irq_lines(int set, int cleared)   // (the K007121 #0's IRQ)
{
	if (cleared & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 0);
	if (set & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 1);
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0100) {
		if (a >= 0x0008 && a < 0x0010) {   // (the K007452's results)
			switch (a & 7) {
			case 0: return (uint8_t)t.mul;
			case 1: return (uint8_t)(t.mul >> 8);
			case 2: return (uint8_t)t.rem;
			case 3: return (uint8_t)(t.rem >> 8);
			case 4: return (uint8_t)t.quo;
			case 5: return (uint8_t)(t.quo >> 8);
			default: return 0;
			}
		}
		switch (a) {
		case 0x0010: return (uint8_t)rbx_port_read(s_port_system);
		case 0x0011: case 0x0012: return (uint8_t)rbx_port_read(s_port_p[a - 0x0011]);
		case 0x0014: case 0x0015: case 0x0016: return (uint8_t)rbx_port_read(s_port_dsw[a - 0x0014]);
		case 0x001e: return 0;   // (watchdog)
		}
		if (a >= 0x0020 && a < 0x0060)
			return s_k121[0].scroll[(a - 0x0020) & 0x3f];
		if (a >= 0x0080 && a < 0x00c0)
			return s_k121[1].scroll[(a - 0x0080) & 0x3f];
		return 0xff;
	}
	if (a < 0x0c00)
		return 0xff;
	if (a < 0x6000)
		return (a >= 0x0d00 && a < 0x1000) ? 0xff : t.ram[a];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[0x18000 + (a - 0x8000)];
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0100) {
		if (a < 0x0008)
			irq_lines(0, k007121_ctrl_w(&s_k121[0], a, v));
		else if (a < 0x0010) {   // (the K007452)
			const int o = a & 7;
			if (o < 6)
				t.math[o] = v;
			if (o == 1)
				t.mul = (uint16_t)(t.math[0] * t.math[1]);
			else if (o == 5) {
				const uint16_t dividend = (uint16_t)(t.math[4] << 8 | t.math[5]), divisor = (uint16_t)(t.math[2] << 8 | t.math[3]);
				t.quo = divisor ? (uint16_t)(dividend / divisor) : 0xffff;
				t.rem = divisor ? (uint16_t)(dividend % divisor) : 0;
			}
		} else if (a == 0x001a)
			q_put(Q_IRQ, 0);   // (sh_irqtrigger_w: ASSERT_LINE)
		else if (a == 0x001c)
			q_put(Q_LATCH, v);
		else if (a >= 0x0020 && a < 0x0060)
			s_k121[0].scroll[(a - 0x0020) & 0x3f] = v;
		else if (a >= 0x0060 && a < 0x0068)
			k007121_ctrl_w(&s_k121[1], a - 0x0060, v);   // (the K007121 #1's interrupts not connected)
		else if (a >= 0x0080 && a < 0x00c0)
			s_k121[1].scroll[(a - 0x0080) & 0x3f] = v;
		return;
	}
	if (a == 0x7000) {   // (bankswitch_w)
		t.mainbank = v & 0x0f;
		for (int p = 0; p < 0x20; p++)
			s_cpu.rpage[0x60 + p] = s_rom + ((uint32_t)t.mainbank * 0x2000 + (uint32_t)p * 256) % s_rom_size;
		return;
	}
	if (a >= 0x0c00 && a < 0x6000 && !(a >= 0x0d00 && a < 0x1000))
		t.ram[a] = v;
}

// ---- the sound board (sound_map: unmapped reads 0xff) ----
static uint8_t snd_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a == 0x0000)
		return t.soundlatch;
	if (a == 0x2000 || a == 0x2001)
		return rbx_ym_read(0, a & 1);
	if (a >= 0x6000 && a < 0x6800)
		return t.sram[a - 0x6000];
	if (a >= 0x8000)
		return s_srom[(a - 0x8000) % s_srom_size];
	return 0xff;
}
static void snd_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a == 0x2000 || a == 0x2001)
		rbx_ym_write(0, a & 1, v);
	else if (a == 0x4000) {   // (sirq_clear_w)
		t.snd_irq = 0;
		m6809_set_irq(&s_snd, 0);
	} else if (a >= 0x6000 && a < 0x6800)
		t.sram[a - 0x6000] = v;
}

static void sound_run(int n)   // (a line's samples: the 6809 for their cycles, the YM2151 0.60 stereo)
{
	if (n <= 0)
		return;
	t.s_frac += (uint64_t)SND_CLOCK * (uint64_t)n;
	const int64_t cyc = (int64_t)(t.s_frac / SND_RATE);
	t.s_frac -= (uint64_t)cyc * SND_RATE;
	m6809_run_until(&s_snd, t.snd_end += cyc);
	if (n > 64)
		n = 64;
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++) {
		const int k = yn > 0 ? i * yn / n : -1;
		s_out_l[s_out_pos] = k >= 0 ? yl[k] : 0;
		s_out_r[s_out_pos] = k >= 0 ? yr[k] : 0;
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
		case Q_IRQ:
			t.snd_irq = 1;
			m6809_set_irq(&s_snd, 1);
			break;
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
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void ct_exit(void);
static void ct_reset(void);

static int decode_gfx(int i, const uint8_t *rom, uint32_t size)   // (gfx_8x8x4_packed_msb)
{
	s_gfx_count[i] = size / 32;
	s_gfx[i] = (uint8_t *)malloc((size_t)s_gfx_count[i] * 64);
	if (!s_gfx[i])
		return -1;
	for (uint32_t n = 0; n < s_gfx_count[i]; n++)
		for (int b = 0; b < 32; b++) {
			const uint8_t v = rom[n * 32 + b];
			s_gfx[i][n * 64 + b * 2] = v >> 4;
			s_gfx[i][n * 64 + b * 2 + 1] = v & 15;
		}
	return 0;
}

static int ct_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t g0, g1, psize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	const uint8_t *gfx0 = rbx_region("k007121_1", &g0), *gfx1 = rbx_region("k007121_2", &g1);
	const uint8_t *prom = rbx_region("proms", &psize);
	if (!s_rom || !s_srom || !gfx0 || !gfx1 || !prom || psize < 0x400) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	if (decode_gfx(0, gfx0, g0) || decode_gfx(1, gfx1, g1)) {
		rbx_fail("%s: out of memory", g->set);
		ct_exit();
		return -1;
	}
	for (int chip = 0; chip < 2; chip++)   // (palette: the pens' indirect colours)
		for (int pal = 0; pal < 8; pal++) {
			const int clut = (chip << 1) | (pal & 1);
			for (int i = 0; i < 0x100; i++) {
				const uint8_t pv = prom[(clut << 8) | i];
				s_ctab[(chip << 11) | (pal << 8) | i] = (uint16_t)((!(pal & 1) && pv == 0) ? 0 : (pal << 4) | (pv & 0x0f));
			}
		}
	for (int chip = 0; chip < 2; chip++)   // (transpen_mask(gfx, color, 0): the pens of indirect colour 0)
		for (int c = 0; c < 128; c++) {
			uint16_t m = 0;
			for (int p = 0; p < 16; p++)
				if (s_ctab[(chip ? 2048 : 0) + (c * 16 + p) % 2048] == 0)
					m |= (uint16_t)(1u << p);
			s_transmask[chip][c] = m;
		}
	k007121_init(&s_k121[0], 40, 16);   // (set_sprite_offsets(40, 16))
	k007121_init(&s_k121[1], 40, 16);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = 154;   // (0.60)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		ct_exit();
		return -1;
	}
	s_port_system = rbx_port_find("SYSTEM");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		ct_exit();
		return -1;
	}
	ct_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void ct_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	for (int i = 0; i < 2; i++) {
		free(s_gfx[i]);
		s_gfx[i] = NULL;
	}
	free(s_q_hp);
	s_q_hp = NULL;
}

static void ct_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 2; i++)
		k007121_reset(&s_k121[i]);
	memset(s_k121[0].scroll, 0, sizeof(s_k121[0].scroll));
	memset(s_k121[1].scroll, 0, sizeof(s_k121[1].scroll));
	rbx_ym_reset(0);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + 0x18000 + (uint32_t)(p - 0x80) * 256;
	for (int p = 0x0c; p < 0x60; p++)   // (the RAM; 0d00-0fff unmapped)
		if (p < 0x0d || p >= 0x10)
			s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x60 + p] = s_rom + (uint32_t)p * 256;
	hd6309_reset(&s_cpu);
	m6809_init(&s_snd);
	s_snd.read = snd_rd;
	s_snd.write = snd_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = (uint32_t)(p - 0x80) * 256 + 256 <= s_srom_size ? s_srom + (p - 0x80) * 256 : NULL;
	for (int p = 0x60; p < 0x68; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.sram + (p - 0x60) * 256;
	m6809_reset(&s_snd);
}

static void ct_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the screen's update)
			capture();
			rbx_video_frame_done();
		}
		if (!(vpos & 15))   // (the K007121s' scanline timers: every 16 lines)
			for (int i = 0; i < 2; i++) {
				const int lines = k007121_scanline(&s_k121[i], vpos, t.frame, t.ram + (i ? 0x5000 : 0x3000));
				if (!i)
					irq_lines(lines, 0);
			}
		t.cyc_frac += (uint64_t)CPU_CLOCK * HTOTAL;
		const int64_t cyc = (int64_t)(t.cyc_frac / PIX_CLOCK);
		t.cyc_frac -= (uint64_t)cyc * PIX_CLOCK;
		hd6309_run_until(&s_cpu, t.cpu_end += cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int ct_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "ct_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "ct_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "ct", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 2; i++)
		rc |= rbx_state_io(ctx, i ? "ct_k121b" : "ct_k121a", &s_k121[i], (uint32_t)offsetof(k007121_t, state_end));
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "ct_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "ct_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "ct_outr", s_out_r, sizeof(s_out_r));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		for (int p = 0; p < 0x20; p++)
			s_cpu.rpage[0x60 + p] = s_rom + ((uint32_t)t.mainbank * 0x2000 + (uint32_t)p * 256) % s_rom_size;
	}
	return rc ? -1 : 0;
}

static int ct_save(void *ctx) { return ct_state(ctx, 0); }
static int ct_load(void *ctx) { return ct_state(ctx, 1); }
static int ct_supported(const char *machine) { return !strcmp(machine, "contra"); }

const rbx_driver rbx_drv_contra = { "Konami Contra", ct_init, ct_exit, ct_reset, ct_frame, ct_save, ct_load,
                                    ct_supported, ct_render, NULL };

/*  Technos Super Dodge Ball / Nekketsu Koukou Dodgeball Bu. Ported from MAME src/mame/technos/spdodgeb.cpp
    (BSD-3-Clause, copyright-holders: Paul Hampson, Nicola Salmoria).
    M6502 at 2 MHz (IRQ held every 8 lines, NMI at line 256), an HD63701Y0 MCU at 4 MHz reading the controls (ports 2,
    5, 6) for the main CPU through a latch and five registers, MC6809 sound CPU at 1.5 MHz with a YM3812 (FIRQ) and two
    MSM5205. Video: a 512x256 tilemap of 8x8 (palette banks), 16x16 sprites; colours from PROMs.
    MAME draws the screen in pieces (update_partial(scanline + 16) at every 8-line IRQ): the registers and the video
    RAM are taken at those times here too.
    The HD63701Y0 runs on the HD63701V0 core (cpu/m6800/m6801), the Y0's ports 5 / 6 and its RAM done on its bus here.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/m6502/m6502.h"
#include "../../cpu/m6809/m6809.h"
#include "../../cpu/m6800/m6801.h"
#include "../../sound/msm5205.h"
#include "../../sound/rbx_ym.h"
#include "../../video/gfxlayout.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MHZ 57445          // set_raw(12 MHz / 2, 384, 0, 256, 272, 0, 240)
#define LINES 272
#define VIS_H 240
#define VIS_W 256
#define MAIN_CLOCK 2000000
#define SND_CLOCK 1500000
#define MCU_CLOCK 1000000        // (4 MHz / 4)
#define YM_CLOCK 3000000

typedef struct {
	uint16_t scroll;
	uint8_t flip, tpal, spal;
} line_regs;

#define MAXSEG 32
typedef struct {
	int y0;
	uint8_t vram[0x1000], spr[0x100];
} vseg;
typedef struct {
	int n;
	vseg seg[MAXSEG];
} vbuf;

static uint8_t *s_rom, *s_srom, *s_mrom, *s_adpcm;
static uint32_t s_rom_size, s_srom_size, s_adpcm_size;
static rbx_gfx s_gchar, s_gspr;
static uint32_t s_pal[1024];
static int s_port[5];
static m6502_t s_main;
static m6809_t s_snd;
static m6801 s_mcu;
static msm5205_t s_msm[2];
static int32_t *s_mix;
static int s_mix_n, s_mix_cap;
static int s_line;
static vbuf *s_vb[2];
static int s_vb_cur, s_vdirty;

static struct {
	uint8_t ram[0x1000], vram[0x1000], spr[0x100];
	uint8_t sndram[0x1000];
	uint8_t mcuram[0x140];
	uint8_t inputs[5], mcu_status, mculatch;
	uint8_t p5_ddr, p5_data, p6_ddr, p6_data;
	uint16_t lastscroll;
	uint8_t flip, bank, tpal, spal;
	uint8_t latch;
	int32_t ym_irq;
	uint32_t adpcm_pos[2], adpcm_end[2];
	uint8_t adpcm_idle[2];
	int32_t adpcm_data[2];
	int64_t main_t, snd_t;
	uint64_t main_frac, snd_frac, mcu_frac, ym_frac;
	uint32_t msm_acc;
	line_regs lines[VIS_H];
	uint8_t state_end;
} t;

static struct {
	line_regs lines[VIS_H];
	const vbuf *vb;
} *s_snap;

// ---- sound ----
static void ym_irq(int state)
{
	t.ym_irq = state;
	m6809_set_firq(&s_snd, state);
}

static void adpcm_int(int w)
{
	if (t.adpcm_pos[w] >= t.adpcm_end[w] || t.adpcm_pos[w] >= 0x10000) {
		t.adpcm_idle[w] = 1;
		msm5205_reset_w(&s_msm[w], 1);
	} else if (t.adpcm_data[w] != -1) {
		msm5205_data_w(&s_msm[w], (uint8_t)(t.adpcm_data[w] & 0x0f));
		t.adpcm_data[w] = -1;
	} else {
		const uint32_t a = 0x10000u * (uint32_t)w + t.adpcm_pos[w]++;
		t.adpcm_data[w] = a < s_adpcm_size ? s_adpcm[a] : 0;
		msm5205_data_w(&s_msm[w], (uint8_t)(t.adpcm_data[w] >> 4));
	}
}
static void adpcm_int0(void) { adpcm_int(0); }
static void adpcm_int1(void) { adpcm_int(1); }

static uint8_t snd_read(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x1000) return t.sndram[a];
	if (a == 0x1000) {
		m6809_set_irq(&s_snd, 0);
		return t.latch;
	}
	if (a >= 0x8000) return s_srom[rbx_wrap((a - 0x8000), s_srom_size)];
	return 0;
}
static void snd_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a < 0x1000) t.sndram[a] = v;
	else if (a == 0x2800 || a == 0x2801) rbx_ym_write(0, a & 1, v);
	else if (a >= 0x3800 && a < 0x3808) {
		const int chip = a & 1;
		switch ((a & 7) >> 1) {
		case 3: t.adpcm_idle[chip] = 1; msm5205_reset_w(&s_msm[chip], 1); break;
		case 2: t.adpcm_pos[chip] = (uint32_t)(v & 0x7f) * 0x200; break;
		case 1: t.adpcm_end[chip] = (uint32_t)(v & 0x7f) * 0x200; break;
		case 0: t.adpcm_idle[chip] = 0; msm5205_reset_w(&s_msm[chip], 0); break;
		}
	}
}

static void sound_line(void)
{
	const uint32_t clocks = (uint32_t)rbx_line_cycles(&t.ym_frac, YM_CLOCK, FRAME_MHZ, LINES);
	const int n = rbx_ym_advance(0, clocks, s_mix + s_mix_n, s_mix_cap - s_mix_n);
	const uint32_t yr = rbx_ym_rate(0);
	for (int i = 0; i < n; i++) {
		t.msm_acc += s_msm[0].clock;
		const uint32_t k = t.msm_acc / yr;
		t.msm_acc -= k * yr;
		msm5205_advance(&s_msm[0], k);
		msm5205_advance(&s_msm[1], k);
		s_mix[s_mix_n + i] += (msm5205_output(&s_msm[0]) + msm5205_output(&s_msm[1])) / 2;
	}
	s_mix_n += n;
}

// ---- MCU (HD63701Y0: mcu_map, ports 2 / 5 / 6) ----
static uint8_t mcu_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0xc000) return s_mrom[a - 0xc000];
	if ((a >= 0x40 && a < 0x80) || (a >= 0x100 && a < 0x140)) return t.mcuram[a];
	if (a == 0x8080) return t.mculatch;
	if (a == 0x15) {   // (hd6301y p5_data_r)
		if (t.p5_ddr == 0xff) return t.p5_data;
		const uint8_t d = (uint8_t)rbx_port_read(s_port[4]);
		return (uint8_t)((d & ~t.p5_ddr) | (t.p5_data & t.p5_ddr));
	}
	if (a == 0x17) {
		const uint8_t d = (uint8_t)rbx_port_read(s_port[3]);
		return (uint8_t)((d & ~t.p6_ddr) | (t.p6_data & t.p6_ddr));
	}
	return 0xff;
}
static void p5_out(void) { t.mcu_status = (uint8_t)(((t.p5_data & t.p5_ddr) | (t.p5_ddr ^ 0xff)) & 0xc0); }
static void mcu_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if ((a >= 0x40 && a < 0x80) || (a >= 0x100 && a < 0x140)) t.mcuram[a] = v;
	else if (a >= 0x8081 && a <= 0x8085) t.inputs[a - 0x8081] = v;
	else if (a == 0x15) { t.p5_data = v; p5_out(); }
	else if (a == 0x20) { if (t.p5_ddr != v) { t.p5_ddr = v; p5_out(); } }
	else if (a == 0x16) t.p6_ddr = v;
	else if (a == 0x17) t.p6_data = v;
}
static uint8_t mcu_port_r(void *u, int port)
{
	(void)u;
	return port == 2 ? (uint8_t)rbx_port_read(s_port[2]) : 0xff;
}

// ---- main CPU ----
static void map_bank(void)
{
	for (int p = 0; p < 0x40; p++)
		s_main.rpage[0x40 + p] = s_rom + (uint32_t)t.bank * 0x4000 + p * 256;
}

static uint8_t main_read(void *u, uint16_t a)
{
	(void)u;
	if (a >= 0x8000) return s_rom[a];
	if (a >= 0x4000) return s_rom[(uint32_t)t.bank * 0x4000 + (a - 0x4000)];
	if (a < 0x1000) return t.ram[a];
	if (a >= 0x2000 && a < 0x3000) return t.vram[a - 0x2000];
	if (a == 0x3000) {
		rbx_port_custom(s_port[0], 0x03, (s_line >= VIS_H ? 1u : 0) | ((t.mcu_status & 0x80) ? 2u : 0));
		return (uint8_t)rbx_port_read(s_port[0]);
	}
	if (a == 0x3001) return (uint8_t)rbx_port_read(s_port[1]);
	if (a >= 0x3801 && a <= 0x3805) return t.inputs[a - 0x3801];
	return 0;
}
static void main_write(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x4000) return;
	if (a < 0x1000) t.ram[a] = v;
	else if (a < 0x1100) { t.spr[a - 0x1000] = v; s_vdirty = 1; }
	else if (a >= 0x2000 && a < 0x3000) { t.vram[a - 0x2000] = v; s_vdirty = 1; }
	else switch (a) {
	case 0x3002: t.latch = v; m6809_set_irq(&s_snd, 1); break;
	case 0x3004: t.lastscroll = (uint16_t)((t.lastscroll & 0x100) | v); break;
	case 0x3005:   // mcu_nmi_w (pulse)
		m6801_set_irq_line(&s_mcu, M6801_NMI_LINE, 1);
		m6801_set_irq_line(&s_mcu, M6801_NMI_LINE, 0);
		break;
	case 0x3006:   // ctrl_w
		t.flip = v & 1;
		t.bank = (~v & 2) >> 1;
		map_bank();
		t.lastscroll = (uint16_t)((t.lastscroll & 0xff) | ((v & 4) << 6));
		t.tpal = (v & 0x30) >> 4;
		t.spal = (v & 0xc0) >> 6;
		break;
	case 0x3800: t.mculatch = v; break;
	}
}

// ---- video ----
static int s_clip0, s_clip1;

static void draw_spr16(uint32_t code, int color, int fx, int fy, int sx, int sy)
{
	code %= s_gspr.count;
	if (!s_gspr.used[code] || sy >= s_clip1 || sy + 16 <= s_clip0)
		return;
	const uint8_t *src = s_gspr.pix + code * 256;
	for (int j = 0; j < 16; j++) {
		const int yy = sy + j;
		if (yy < s_clip0 || yy >= s_clip1)
			continue;
		const uint8_t *s = src + (fy ? 15 - j : j) * 16;
		uint16_t *d = rbx_video.bitmap + yy * rbx_video.pitch;
		for (int i = 0; i < 16; i++) {
			const int xx = sx + i;
			if ((unsigned)xx >= VIS_W)
				continue;
			const uint8_t p = s[fx ? 15 - i : i];
			if (p)
				d[xx] = (uint16_t)(0x200 + color * 16 + p);
		}
	}
}

static void draw_piece(const vseg *sg, const line_regs *r, int y0, int y1)
{
	for (int y = y0; y < y1; y++) {
		uint16_t *d = rbx_video.bitmap + y * rbx_video.pitch;
		const line_regs *l = &s_snap->lines[y];
		const uint32_t sc = (uint32_t)l->scroll + 5;
		const uint32_t ty = (uint32_t)(l->flip ? 239 - y : y) & 255;
		for (int x = 0; x < VIS_W; x++) {
			const uint32_t tx = (uint32_t)(l->flip ? 255 - x + sc : x + sc) & 511;
			const uint32_t col = tx >> 3, row = ty >> 3;
			const uint32_t idx = (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x20) << 5);
			const uint8_t attr = sg->vram[idx + 0x800];
			const uint32_t code = rbx_wrap((uint32_t)(sg->vram[idx] + ((attr & 0x1f) << 8)), s_gchar.count);
			d[x] = (uint16_t)((((attr & 0xe0) >> 5) + 8 * l->tpal) * 16 + s_gchar.pix[code * 64 + (ty & 7) * 8 + (tx & 7)]);
		}
	}
	s_clip0 = y0;
	s_clip1 = y1;
	const int flip = r->flip;
	for (int i = 0; i < 0x100; i += 4) {
		const uint8_t *s = &sg->spr[i];
		const int attr = s[1];
		uint32_t which = (uint32_t)(s[2] + ((attr & 7) << 8));
		int sx = s[3], sy = 240 - s[0];
		const int size = (attr & 0x80) >> 7, color = ((attr & 0x38) >> 3) + 8 * r->spal;
		int fx = (~attr & 0x40) != 0, fy = 0, dy = -16;
		if (flip) {
			sx = 240 - sx;
			sy = 240 - sy;
			fx = !fx;
			fy = !fy;
			dy = -dy;
		}
		if (sx < -8) sx += 256; else if (sx > 248) sx -= 256;
		if (!size) {
			if (sy < -8) sy += 256; else if (sy > 248) sy -= 256;
			draw_spr16(which, color, fx, fy, sx, sy);
		} else {
			if (flip) { if (sy > 240) sy -= 256; } else { if (sy < 0) sy += 256; }
			which &= ~1u;
			draw_spr16(which, color, fx, fy, sx, sy + dy);
			draw_spr16(which + 1, color, fx, fy, sx, sy);
		}
	}
}

static void sd_render(void)
{
	memcpy(rbx_video.palette, s_pal, sizeof(s_pal));
	const vbuf *vb = s_snap->vb;
	for (int k = 0; k < vb->n; k++) {
		const int y0 = vb->seg[k].y0, y1 = k + 1 < vb->n ? vb->seg[k + 1].y0 : VIS_H;
		draw_piece(&vb->seg[k], &s_snap->lines[y0], y0, y1);
	}
}

// (MAME update_partial: the lines up to there drawn with the registers and RAM now)
static int s_next;   // the first line not drawn yet

static void update_partial(int upto)
{
	if (upto >= VIS_H)
		upto = VIS_H - 1;
	if (upto < s_next)
		return;
	line_regs r;
	r.scroll = t.lastscroll;
	r.flip = t.flip;
	r.tpal = t.tpal;
	r.spal = t.spal;
	vbuf *b = s_vb[s_vb_cur];
	if (s_next == 0 || s_vdirty || memcmp(&r, &t.lines[s_next - 1], sizeof(r))) {
		vseg *sg;
		if (b->n < MAXSEG) {
			sg = &b->seg[b->n++];
			sg->y0 = s_next;
		} else
			sg = &b->seg[MAXSEG - 1];
		memcpy(sg->vram, t.vram, sizeof(t.vram));
		memcpy(sg->spr, t.spr, sizeof(t.spr));
		s_vdirty = 0;
	}
	for (int y = s_next; y <= upto; y++)
		t.lines[y] = r;
	s_next = upto + 1;
}

static void vblank_capture(void)
{
	rbx_video_wait_idle();
	memcpy(s_snap->lines, t.lines, sizeof(t.lines));
	s_snap->vb = s_vb[s_vb_cur];
	s_vb_cur ^= 1;
	rbx_video_frame_done();
}

// ---- init / frame ----
static void sd_reset(void)
{
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.adpcm_idle[0] = t.adpcm_idle[1] = 1;
	t.adpcm_data[0] = t.adpcm_data[1] = -1;
	m6502_init(&s_main, M6502_NMOS);
	s_main.read = main_read;
	s_main.write = main_write;
	s_main.irq_hold = 1;
	for (int p = 0; p < 0x10; p++)
		s_main.rpage[p] = s_main.wpage[p] = t.ram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_main.rpage[p] = s_rom + p * 256;
	map_bank();
	m6502_reset(&s_main);
	m6809_init(&s_snd);
	s_snd.read = snd_read;
	s_snd.write = snd_write;
	for (int p = 0; p < 0x10; p++)
		s_snd.rpage[p] = s_snd.wpage[p] = t.sndram + p * 256;
	for (int p = 0x80; p < 0x100; p++)
		s_snd.rpage[p] = s_srom + ((uint32_t)(p - 0x80) * 256) % s_srom_size;
	m6809_reset(&s_snd);
	memset(&s_mcu, 0, sizeof(s_mcu));
	s_mcu.read = mcu_read;
	s_mcu.write = mcu_write;
	s_mcu.port_r = mcu_port_r;
	s_mcu.type = M6801_TYPE_HD63701V0;
	s_mcu.rom = s_mrom;
	s_mcu.rom_base = 0xc000;
	s_mcu.rom_size = 0x4000;
	m6801_init(&s_mcu);
	m6801_reset(&s_mcu);
	rbx_ym_reset(0);
	for (int i = 0; i < 2; i++) {
		msm5205_reset(&s_msm[i]);
		msm5205_reset_w(&s_msm[i], 1);
	}
	s_mix_n = 0;
	s_line = 0;
	s_next = 0;
	s_vdirty = 0;
}

static void sd_exit(void);

static int sd_init(const rbx_game *g)
{
	uint32_t csize, ssize, psize, msize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_srom = rbx_region("audiocpu", &s_srom_size);
	s_mrom = rbx_region("mcu", &msize);
	uint8_t *chars = rbx_region("text", &csize), *spr = rbx_region("sprites", &ssize), *prom = rbx_region("proms", &psize);
	s_adpcm = rbx_region("adpcm", &s_adpcm_size);
	if (!s_rom || s_rom_size < 0x10000 || !s_srom || !s_mrom || msize < 0x4000 || !chars || !spr || !prom ||
	    psize < 0x800 || !s_adpcm) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	for (int i = 0; i < 1024; i++) {
		const uint8_t c = prom[i], c2 = prom[1024 + i];
		const int r = 0x0e * (c & 1) + 0x1f * ((c >> 1) & 1) + 0x43 * ((c >> 2) & 1) + 0x8f * ((c >> 3) & 1);
		const int gg = 0x0e * ((c >> 4) & 1) + 0x1f * ((c >> 5) & 1) + 0x43 * ((c >> 6) & 1) + 0x8f * ((c >> 7) & 1);
		const int b = 0x0e * (c2 & 1) + 0x1f * ((c2 >> 1) & 1) + 0x43 * ((c2 >> 2) & 1) + 0x8f * ((c2 >> 3) & 1);
		s_pal[i] = (uint32_t)(r << 16 | gg << 8 | b);
	}
	{	// charlayout, spritelayout (RGN_FRAC(1,2))
		static const uint32_t pc[4] = { 0, 2, 4, 6 };
		static const uint32_t xc[8] = { 1, 0, 64 + 1, 64 + 0, 128 + 1, 128 + 0, 192 + 1, 192 + 0 };
		static const uint32_t yc[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
		static const uint32_t xt[16] = { 3, 2, 1, 0, 16 * 8 + 3, 16 * 8 + 2, 16 * 8 + 1, 16 * 8 + 0,
		                                 32 * 8 + 3, 32 * 8 + 2, 32 * 8 + 1, 32 * 8 + 0, 48 * 8 + 3, 48 * 8 + 2, 48 * 8 + 1, 48 * 8 + 0 };
		static uint32_t yt[16];
		for (int i = 0; i < 16; i++)
			yt[i] = (uint32_t)i * 8;
		const uint32_t ps[4] = { ssize * 4, ssize * 4 + 4, 0, 4 };
		const rbx_gfx_layout lc = { 8, 8, 4, pc, xc, yc, 32 * 8 };
		const rbx_gfx_layout ls = { 16, 16, 4, ps, xt, yt, 64 * 8 };
		if (rbx_gfx_decode(&s_gchar, &lc, chars, csize) || rbx_gfx_decode(&s_gspr, &ls, spr, ssize / 2)) {
			rbx_fail("%s: out of memory (graphics)", g->set);
			sd_exit();
			return -1;
		}
	}
	s_snap = calloc(1, sizeof(*s_snap));
	s_vb[0] = calloc(1, sizeof(vbuf));
	s_vb[1] = calloc(1, sizeof(vbuf));
	s_vb_cur = 0;
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = YM_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (!s_snap || !s_vb[0] || !s_vb[1] || rbx_ym_init(0, &yc) || rbx_ym_set_async(0, 1)) {
		rbx_fail("%s: out of memory", g->set);
		sd_exit();
		return -1;
	}
	s_mix_cap = (int)(rbx_ym_rate(0) / 50) + 256;
	s_mix = (int32_t *)calloc((size_t)s_mix_cap, sizeof(int32_t));
	if (!s_mix || rbx_audio_setup(rbx_ym_rate(0)) || rbx_video_setup(VIS_W, VIS_H, 1024)) {
		rbx_fail("%s: out of memory", g->set);
		sd_exit();
		return -1;
	}
	msm5205_init(&s_msm[0], 384000, MSM5205_S48_4B, adpcm_int0);
	msm5205_init(&s_msm[1], 384000, MSM5205_S48_4B, adpcm_int1);
	static const char *const names[5] = { "IN0", "DSW", "P1", "P2", "IN1" };
	for (int i = 0; i < 5; i++)
		s_port[i] = rbx_port_find(names[i]);
	sd_reset();
	return 0;
}

static void sd_exit(void)
{
	rbx_video_wait_idle();
	rbx_ym_exit(0);
	free(s_mix);
	s_mix = NULL;
	rbx_gfx_free(&s_gchar);
	rbx_gfx_free(&s_gspr);
	free(s_snap);
	s_snap = NULL;
	free(s_vb[0]);
	free(s_vb[1]);
	s_vb[0] = s_vb[1] = NULL;
}

static void sd_frame(void)
{
	rbx_audio.frame_mhz = FRAME_MHZ;
	s_vb[s_vb_cur]->n = 0;
	s_next = 0;
	for (int line = 0; line < LINES; line++) {
		s_line = line;
		if (line == VIS_H) {
			update_partial(VIS_H - 1);
			vblank_capture();
		}
		if (line == 256) {
			m6502_set_nmi(&s_main, 1);
			m6502_set_nmi(&s_main, 0);
		} else if ((line % 8) == 0) {
			m6502_set_irq(&s_main, 1);
			if (line + 16 < VIS_H + 16 && line < VIS_H)
				update_partial(line + 16);
		}
		m6502_run_until(&s_main, t.main_t += rbx_line_cycles(&t.main_frac, MAIN_CLOCK, FRAME_MHZ, LINES));
		m6801_execute(&s_mcu, rbx_line_cycles(&t.mcu_frac, MCU_CLOCK, FRAME_MHZ, LINES));
		m6809_run_until(&s_snd, t.snd_t += rbx_line_cycles(&t.snd_frac, SND_CLOCK, FRAME_MHZ, LINES));
		sound_line();
	}
	rbx_ym_async_take(0, s_mix, s_mix_n);
	for (int i = 0; i < s_mix_n; i++)
		rbx_audio_put(s_mix[i]);
	memset(s_mix, 0, sizeof(int32_t) * (size_t)s_mix_n);
	s_mix_n = 0;
}

static int sd_state(void *ctx, int load)
{
	int rc = 0;
	rc |= rbx_state_io(ctx, "sd", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "sd_main", &s_main, (uint32_t)offsetof(m6502_t, rpage));
	rc |= rbx_state_io(ctx, "sd_snd", &s_snd, (uint32_t)offsetof(m6809_t, rpage));
	rc |= rbx_state_io(ctx, "sd_mcu", m6801_state_data(&s_mcu), m6801_state_size());
	rc |= rbx_ym_state(0, ctx, load);
	rc |= rbx_state_io(ctx, "sd_msm0", &s_msm[0], (uint32_t)offsetof(msm5205_t, vck));
	rc |= rbx_state_io(ctx, "sd_msm1", &s_msm[1], (uint32_t)offsetof(msm5205_t, vck));
	if (!rc && load) {
		map_bank();
		s_vdirty = 1;
	}
	return rc ? -1 : 0;
}

static int sd_save(void *ctx) { return sd_state(ctx, 0); }
static int sd_load(void *ctx) { return sd_state(ctx, 1); }
static int sd_supported(const char *machine) { return !strcmp(machine, "spdodgeb"); }

const rbx_driver rbx_drv_spdodgeb = { "Technos Super Dodge Ball", sd_init, sd_exit, sd_reset, sd_frame, sd_save,
                                      sd_load, sd_supported, sd_render, NULL };

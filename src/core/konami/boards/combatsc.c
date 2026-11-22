/*  Konami Combat School. Ported from MAME src/mame/konami/combatsc.cpp, combatsc_v.cpp (BSD-3-Clause,
    copyright-holders: Phil Stroffolino, Manuel Abadia) and the devices it uses (k007121.cpp, k007452.cpp,
    upd7759.cpp). An HD6309E at 3 MHz, two K007121s (their 32x32 tilemaps with row scroll and the text layer as the
    driver's; the CPU sees one chip at a time), the indirect palette through the colour PROMs; the sound: a Z80 at
    1.5 MHz, a YM2203 and a uPD7759, a frame behind on the worker. Not the bootleg (combatscb).
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/rbx_ym.h"
#include "../../sound/upd7759.h"
#include "../video/k007121.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define Z80_CLOCK 1500000u     // (24 MHz / 16)
#define YM_CLOCK 3000000u      // (24 MHz / 8)
#define UPD_CLOCK 640000u
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 2048             // (8 * 16 * 16)

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_gfx[2];
static uint32_t s_gfx_count[2];
static uint16_t s_ctab[NPENS];
static uint16_t s_transmask[128];
static hd6309_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k007121_t s_k121[2];
static upd7759_t s_upd;
static int s_port_in0, s_port_in1, s_port_dsw[3], s_port_track[4];
static uint8_t (*s_pri_hp)[VIS_W * VIS_H];
#define s_pri (*s_pri_hp)

static struct {
	uint8_t pal[0x100];        // 0600-06ff (mirror 0700)
	uint8_t ram[0x1800];       // 0800-1fff
	uint8_t video[2][0x2000];  // 2000-3fff, the chip's view
	uint8_t mainbank, view, priority, vreg;
	uint8_t math[6];           // (the K007452)
	uint16_t mul, quo, rem;
	uint8_t pos[4], sign[4];   // (trackball_r)
	uint8_t zram[0x800], soundlatch, z80_irq;
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x100], video[2][0x1000], priority, vreg; k007121_t k121[2]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.video[0], t.video[0], 0x1000);
	memcpy(s_vs.video[1], t.video[1], 0x1000);
	s_vs.priority = t.priority;
	s_vs.vreg = t.vreg;
	for (int i = 0; i < 2; i++)
		memcpy(&s_vs.k121[i], &s_k121[i], sizeof(k007121_t));
}

// get_tile_info0 / 1: the code, the colour and the category (attribute bit 6)
static void tile_info(int which, int idx, uint32_t *code, int *color, int *cat)
{
	const uint8_t *v = s_vs.video[which];
	const int attr = v[idx];
	int bank = 4 * ((which ? s_vs.vreg >> 4 : s_vs.vreg & 0x0f) - 1);
	if (bank < 0)
		bank = 0;
	if (!(attr & 0xb0))
		bank = 0;   // (text bank)
	if (attr & 0x80)
		bank += 1;
	if (attr & 0x10)
		bank += 2;
	if (attr & 0x20)
		bank += 4;
	*color = ((s_vs.k121[which].ctrl[6] & 0x10) * 2 + 16 + (which ? 4 * 16 : 0)) + (attr & 0x0f);
	*code = (uint32_t)(v[idx + 0x400] + 256 * bank);
	*cat = (attr & 0x40) >> 6;
}

// a playfield (cat: the category drawn), row scroll from the chip's scroll RAM when its ctrl 1 bit 1 is set
static void draw_bg(int which, int cat, int opaque, uint8_t priority)
{
	const k007121_t *k = &s_vs.k121[which];
	const int rows = (k->ctrl[1] & 0x02) != 0, scrolly = k->ctrl[2];
	const int sx0 = k->ctrl[0] | ((k->ctrl[1] & 0x01) << 8);
	const uint8_t *gfx = s_gfx[which];
	const uint32_t count = s_gfx_count[which];
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		const int ty = (y + scrolly) & 255;
		const int scrollx = rows ? (k->scroll[ty >> 3] | ((k->scroll[(ty >> 3) | 0x20] & 1) << 8)) : sx0;
		uint16_t *d = bm + (y - VIS_Y0) * pitch;
		uint8_t *pr = s_pri + (y - VIS_Y0) * VIS_W;
		for (int x = 0; x < VIS_W;) {
			const int tx = (x + scrollx) & 255;
			int n = 8 - (tx & 7);
			if (n > VIS_W - x)
				n = VIS_W - x;
			uint32_t code;
			int color, tc;
			tile_info(which, (ty >> 3) * 32 + (tx >> 3), &code, &color, &tc);
			if (tc == cat) {
				const uint8_t *src = gfx + (code % count) * 64 + (ty & 7) * 8 + (tx & 7);
				for (int i = 0; i < n; i++) {
					const int p = src[i];
					if (p || opaque) {
						d[x + i] = (uint16_t)(color * 16 + p);
						pr[x + i] |= priority;
					}
				}
			}
			x += n;
		}
	}
}

// the text layer (get_text_info: chip 0's 0800 / 0c00) on the 8-line rows its scroll RAM's bit 0 enables
static void draw_text(void)
{
	const k007121_t *k = &s_vs.k121[0];
	const int opaque = (k->ctrl[1] & 0x08) != 0;
	const uint8_t *v = s_vs.video[0];
	uint16_t *bm = rbx_video.bitmap;
	for (int i = 0; i < 32; i++) {
		const int base_scroll = k->flipscreen ? 0x3f - i : 0x20 + i;
		if (!(k->scroll[base_scroll] & 1))
			continue;
		for (int y = i * 8; y < i * 8 + 8; y++) {
			if (y < VIS_Y0 || y >= VIS_Y0 + VIS_H)
				continue;
			uint16_t *d = bm + (y - VIS_Y0) * rbx_video.pitch;
			for (int x = 0; x < VIS_W; x++) {
				const int idx = (y >> 3) * 32 + (x >> 3);
				const int p = s_gfx[0][(uint32_t)(v[0xc00 + idx] % s_gfx_count[0]) * 64 + (y & 7) * 8 + (x & 7)];
				if (p || opaque)
					d[x] = (uint16_t)((16 + (v[0x800 + idx] & 0x0f)) * 16 + p);
			}
		}
	}
}

static int s_spr_which;
static void sprite_cb(int *code, int *color, int colbank) { (void)code; *color += (s_spr_which << 6) + (colbank & 0x20); }

static void draw_sprites(int which, uint32_t pmask)
{
	s_spr_which = which;
	k007121_sprites_draw(&s_vs.k121[which], s_gfx[which], s_gfx_count[which], 0, s_transmask, sprite_cb, rbx_video.bitmap,
	                     rbx_video.pitch, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0, VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1, s_pri, pmask);
}

static void cs_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++) {   // (the indirect palette: xBGR_555, little endian)
		const int e = s_ctab[i];
		const uint16_t v = (uint16_t)(s_vs.pal[2 * e] | s_vs.pal[2 * e + 1] << 8);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	memset(s_pri, 0, sizeof(s_pri));
	if (!s_vs.priority) {
		draw_bg(1, 0, 1, 4);
		draw_bg(1, 1, 1, 8);
		draw_bg(0, 0, 0, 1);
		draw_bg(0, 1, 0, 2);
		draw_sprites(1, 0x0f00);
		draw_sprites(0, 0x4444);
	} else {
		draw_bg(0, 0, 1, 1);
		draw_bg(0, 1, 1, 2);
		draw_sprites(1, 0x0f00);
		draw_sprites(0, 0x4444);
		draw_bg(1, 1, 0, 4);
		draw_bg(1, 0, 0, 8);
	}
	draw_text();
	if (s_vs.k121[0].ctrl[3] & 0x40)   // (the leftmost and rightmost 8 columns black)
		for (int y = 0; y < VIS_H; y++)
			for (int x = 0; x < 8; x++) {
				rbx_video.bitmap[y * rbx_video.pitch + x] = 0;
				rbx_video.bitmap[y * rbx_video.pitch + VIS_W - 1 - x] = 0;
			}
}

// ---- the sound board a frame behind on the worker (rbx_aux): the latch and the IRQ queued with the samples ----
#define Q_MAX 2048
enum { Q_ADV, Q_LATCH, Q_IRQ };
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
static void irq_lines(int set, int cleared)   // (the K007121 #0's IRQ)
{
	if (cleared & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 0);
	if (set & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 1);
}

static uint8_t port_or(int p, uint8_t def) { return p >= 0 ? (uint8_t)rbx_port_read(p) : def; }

static uint8_t trackball_r(int offset)
{
	if (offset == 0) {
		int dir[4];
		for (int i = 0; i < 4; i++) {
			const uint8_t curr = port_or(s_port_track[i], 0xff);
			dir[i] = (uint8_t)(curr - t.pos[i]);
			t.sign[i] = (uint8_t)(dir[i] & 0x80);
			t.pos[i] = curr;
		}
		if (dir[0] || dir[1]) {
			if (!dir[0]) t.sign[0] = t.sign[1] ^ 0x80;
			if (!dir[1]) t.sign[1] = t.sign[0];
		}
		if (dir[2] || dir[3]) {
			if (!dir[2]) t.sign[2] = t.sign[3] ^ 0x80;
			if (!dir[3]) t.sign[3] = t.sign[2];
		}
	}
	return (uint8_t)(t.sign[offset] | (t.pos[offset] & 0x7f));
}

static void map_view(void)   // (the RAM views: 2000-3fff the chip's video RAM; 4000-7fff the ROM's bank)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x20 + p] = s_cpu.wpage[0x20 + p] = t.video[t.view] + p * 256;
	for (int p = 0; p < 0x40; p++)
		s_cpu.rpage[0x40 + p] = s_rom + ((uint32_t)t.mainbank * 0x4000 + (uint32_t)p * 256) % s_rom_size;
}

static void bankselect_w(uint8_t v)
{
	t.priority = (v >> 5) & 1;
	t.view = (v >> 6) & 1;
	t.mainbank = (v & 0x10) ? (uint8_t)((v & 0x0e) >> 1) : (uint8_t)(8 + (v & 1));
	map_view();
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0200) {
		const int o = a & 0x7f;   // (mirror 0x0180)
		if (o == 0x1f)
			return 0xff;   // (unk_r)
		if (o >= 0x20 && o < 0x60)
			return s_k121[t.view].scroll[o - 0x20];
		return 0;
	}
	if (a < 0x0600) {
		if (a >= 0x0200 && a < 0x0208) {   // (the K007452's results)
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
		case 0x0400: return port_or(s_port_in0, 0xff);
		case 0x0401: return port_or(s_port_dsw[2], 0xff);
		case 0x0402: return port_or(s_port_dsw[0], 0xff);
		case 0x0403: return port_or(s_port_dsw[1], 0xff);
		case 0x0404: return s_port_in1 >= 0 ? (uint8_t)rbx_port_read(s_port_in1) : trackball_r(0);   // (init_combatsc: IN1)
		case 0x0405: case 0x0406: case 0x0407: return trackball_r(a - 0x0404);
		}
		return 0;
	}
	if (a < 0x0800)
		return t.pal[a & 0xff];
	if (a < 0x2000)
		return t.ram[a - 0x0800];
	if (a < 0x4000)
		return t.video[t.view][a - 0x2000];
	if (a < 0x8000)
		return s_rom[((uint32_t)t.mainbank * 0x4000 + (a - 0x4000)) % s_rom_size];
	return s_rom[0x28000 + (a - 0x8000)];
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0200) {
		const int o = a & 0x7f;
		if (o < 8) {
			const int cleared = k007121_ctrl_w(&s_k121[t.view], o, v);
			if (!t.view)
				irq_lines(0, cleared);
		} else if (o >= 0x20 && o < 0x60)
			s_k121[t.view].scroll[o - 0x20] = v;
		return;
	}
	if (a < 0x0600) {
		if (a >= 0x0200 && a < 0x0208) {   // (the K007452)
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
			return;
		}
		switch (a) {
		case 0x040c: t.vreg = v; return;           // (vreg_w)
		case 0x0410: bankselect_w(v); return;
		case 0x0414: q_put(Q_LATCH, v); return;
		case 0x0418: q_put(Q_IRQ, 0); return;      // (sh_irqtrigger_w: HOLD_LINE)
		}
		return;
	}
	if (a < 0x0800)
		t.pal[a & 0xff] = v;
	else if (a < 0x2000)
		t.ram[a - 0x0800] = v;
	else if (a < 0x4000)
		t.video[t.view][a - 0x2000] = v;
}

// ---- the sound board (sound_map) ----
static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a == 0xb000)
		return upd7759_busy_r(&s_upd) ? 1 : 0;
	if (a == 0xd000)
		return t.soundlatch;
	if (a == 0xe000 || a == 0xe001)
		return rbx_ym_read(0, a & 1);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a == 0x9000)   // (play_w)
		upd7759_start_w(&s_upd, !((v >> 1) & 1));
	else if (a == 0xa000)
		upd7759_port_w(&s_upd, v);
	else if (a == 0xc000)   // (voice_reset_w)
		upd7759_reset_w(&s_upd, v & 1);
	else if (a == 0xe000 || a == 0xe001)
		rbx_ym_write(0, a & 1, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

static void sound_run(int n)   // (a line's samples: the Z80, the YM2203 0.20, the uPD7759 0.70; mono)
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
	int32_t ym[96], up[64];
	memset(ym, 0, sizeof(ym));
	t.ym_frac += (uint64_t)YM_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance(0, yc, ym, 96);
	upd7759_render(&s_upd, up, n);
	for (int i = 0; i < n && s_out_pos < 2048; i++) {
		const int k = yn > 0 ? i * yn / n : -1;
		s_out[s_out_pos++] = (k >= 0 ? ym[k] : 0) + up[i] * 70 / 100;
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
		rbx_audio_put_stereo(s_out[i], s_out[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void cs_exit(void);
static void cs_reset(void);

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

static int cs_init(const rbx_game *g)
{
	if (!s_pri_hp && !(s_pri_hp = calloc(1, sizeof(*s_pri_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t g0, g1, psize, usize;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *gfx0 = rbx_region("gfx1", &g0), *gfx1 = rbx_region("gfx2", &g1);
	const uint8_t *prom = rbx_region("proms", &psize), *upd = rbx_region("upd", &usize);
	if (!s_rom || !s_zrom || !gfx0 || !gfx1 || !prom || psize < 0x400 || !upd) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	if (decode_gfx(0, gfx0, g0) || decode_gfx(1, gfx1, g1)) {
		rbx_fail("%s: out of memory", g->set);
		cs_exit();
		return -1;
	}
	static const int clut[8] = { 1, 1, 1, 1, 2, 3, 2, 3 };   // (palette: other sprites, background, player sprites, foreground)
	for (int pal = 0; pal < 8; pal++)
		for (int i = 0; i < 0x100; i++) {
			const uint8_t pv = prom[(clut[pal] << 8) | i];
			s_ctab[(pal << 8) | i] = (uint16_t)((!(pal & 1) && pv == 0) ? 0 : (pal << 4) | (pv & 0x0f));
		}
	for (int c = 0; c < 128; c++) {   // (transpen_mask(gfx, color, 0))
		uint16_t m = 0;
		for (int p = 0; p < 16; p++)
			if (s_ctab[(c * 16 + p) % NPENS] == 0)
				m |= (uint16_t)(1u << p);
		s_transmask[c] = m;
	}
	k007121_init(&s_k121[0], 0, 0);
	k007121_init(&s_k121[1], 0, 0);
	upd7759_init(&s_upd, upd, usize, UPD_CLOCK, SND_RATE);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2203;
	yc.clock = YM_CLOCK;
	yc.gain[0] = yc.gain[1] = yc.gain[2] = yc.gain[3] = 51;   // (ALL_OUTPUTS 0.20)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2203", g->set);
		cs_exit();
		return -1;
	}
	s_port_in0 = rbx_port_find("IN0");
	s_port_in1 = rbx_port_find("IN1");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" }, *const tn[4] = { "TRACK0_Y", "TRACK0_X", "TRACK1_Y", "TRACK1_X" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	for (int i = 0; i < 4; i++)
		s_port_track[i] = rbx_port_find(tn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		cs_exit();
		return -1;
	}
	cs_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void cs_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	for (int i = 0; i < 2; i++) {
		free(s_gfx[i]);
		s_gfx[i] = NULL;
	}
	free(s_pri_hp);
	s_pri_hp = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void cs_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	t.vreg = 0xff;   // (machine_reset: m_vreg = -1)
	for (int i = 0; i < 2; i++) {
		k007121_reset(&s_k121[i]);
		memset(s_k121[i].scroll, 0, sizeof(s_k121[i].scroll));
	}
	upd7759_reset(&s_upd);
	rbx_ym_reset(0);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + 0x28000 + (uint32_t)(p - 0x80) * 256;
	for (int p = 0x08; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x08) * 256;
	bankselect_w(0);
	hd6309_reset(&s_cpu);
	z80_init(&s_z);
	s_z.read_byte = z_rd;
	s_z.write_byte = z_wr;
	s_z.port_in = z_in;
	s_z.port_out = z_out;
	s_z.int_data = 0xff;
	memset(s_zmap, 0, sizeof(s_zmap));
	for (int p = 0; p < 0x80; p++)
		s_zmap[p] = (uint32_t)p * 256 + 256 <= s_zrom_size ? s_zrom + p * 256 : NULL;
	for (int p = 0; p < 8; p++)
		s_zmap[0x80 + p] = t.zram + p * 256;
	s_z.read_map = s_zmap;
}

static void cs_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the screen's update)
			capture();
			rbx_video_frame_done();
		}
		if (!(vpos & 15))   // (the K007121s' scanline timers: every 16 lines; spriteram the video RAM's 1000)
			for (int i = 0; i < 2; i++) {
				const int lines = k007121_scanline(&s_k121[i], vpos, t.frame, t.video[i] + 0x1000);
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

static int cs_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "cs_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "cs", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 2; i++)
		rc |= rbx_state_io(ctx, i ? "cs_k121b" : "cs_k121a", &s_k121[i], (uint32_t)offsetof(k007121_t, state_end));
	rc |= upd7759_state(&s_upd, ctx, load, "cs_upd");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "cs_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "cs_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "cs_z80", &z, sizeof(z));
	if (!rc && load) {
		s_out_n = qn >= 0 && qn <= 2048 ? qn : 0;
		s_qn[0] = s_qn[1] = 0;
		s_job_frame = 0;
		z.read_byte = z_rd;
		z.write_byte = z_wr;
		z.port_in = z_in;
		z.port_out = z_out;
		z.userdata = NULL;
		z.read_map = s_zmap;
		z.op_rom = NULL;
		s_z = z;
		map_view();
	}
	return rc ? -1 : 0;
}

static int cs_save(void *ctx) { return cs_state(ctx, 0); }
static int cs_load(void *ctx) { return cs_state(ctx, 1); }
static int cs_supported(const char *machine) { return !strcmp(machine, "combatsc"); }

const rbx_driver rbx_drv_combatsc = { "Konami Combat School", cs_init, cs_exit, cs_reset, cs_frame, cs_save, cs_load,
                                      cs_supported, cs_render, NULL };

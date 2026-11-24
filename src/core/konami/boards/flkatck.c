/*  Konami Flak Attack / MX5000. Ported from MAME src/mame/konami/flkatck.cpp (BSD-3-Clause, copyright-holders: Manuel
    Abadia) and the devices it uses (k007121.cpp, k007452.cpp, k007232.cpp). An HD6309E at 3 MHz, one K007121 (its two
    32x32 tilemaps as the driver's), the direct palette; the sound: a Z80 at 3.58 MHz, a YM2151, a K007232 and a
    K007452, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../cheat_cpu.h"
#include "../../rbx.h"
#include "../../cpu/hd6309/hd6309.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/rbx_ym.h"
#include "../video/k007121.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 8)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u    // (the YM2151, the K007232)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 280, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 280
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 512

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_gfx;
static uint32_t s_gfx_count;
static uint16_t s_transmask[32];
static hd6309_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k007121_t s_k121;
static k007232_t s_k232;
static int s_port_coin, s_port_p[2], s_port_dsw[3];

static struct {
	uint8_t pal[0x400];      // 0800-0bff
	uint8_t spr[0x1000];     // 1000-1fff
	uint8_t vram[0x1000];    // 2000-2fff
	uint8_t ram[0x1000];     // 3000-3fff
	uint8_t mainbank;
	uint8_t math[6];         // (the K007452 on the sound board)
	uint16_t mul, quo, rem;
	uint8_t zram[0x800], soundlatch, z80_irq;
	uint64_t k232_frac;
	int32_t k232_a, k232_b;
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x400], vram[0x1000]; k007121_t k121; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.pal, sizeof(s_vs.pal));
	memcpy(s_vs.vram, t.vram, sizeof(s_vs.vram));
	memcpy(&s_vs.k121, &s_k121, sizeof(k007121_t));
}

// tilemap A (get_tile_info_a, flip y by attribute bit 5) or B (the score: get_tile_info_b) between the screen's x0, x1
static void draw_layer(int which, int scrollx, int scrolly, int opaque, int x0, int x1)
{
	const uint8_t *v = s_vs.vram + (which ? 0x800 : 0), *k = s_vs.k121.ctrl;
	const int bit0 = k[5] & 3, bit1 = (k[5] >> 2) & 3, bit2 = (k[5] >> 4) & 3, bit3 = (k[5] >> 6) & 3;
	const int mask = (k[4] & 0xf0) >> 4;
	if (x0 < VIS_X0)
		x0 = VIS_X0;
	if (x1 > VIS_X0 + VIS_W - 1)
		x1 = VIS_X0 + VIS_W - 1;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		const int ty = (y + scrolly) & 255;
		uint16_t *d = rbx_video.bitmap + (y - VIS_Y0) * rbx_video.pitch - VIS_X0;
		for (int x = x0; x <= x1;) {
			const int tx = (x + scrollx) & 255;
			int n = 8 - (tx & 7);
			if (n > x1 - x + 1)
				n = x1 - x + 1;
			const int idx = (ty >> 3) * 32 + (tx >> 3), attr = v[idx];
			uint32_t code = v[idx + 0x400];
			int fy = 0;
			if (!which) {
				int bank = ((attr >> (bit0 + 3)) & 1) | ((attr >> (bit1 + 2)) & 2) | ((attr >> (bit2 + 1)) & 4) | ((attr >> bit3) & 8);
				bank = (bank & ~mask) | (k[4] & mask);
				bank = ((attr & 0x80) >> 7) | (bank << 1) | ((k[3] & 1) << 5);
				if (attr == 0x0d && !k[0] && !k[2])
					bank = 0;   // (the text in every bank)
				code += 256u * (uint32_t)bank;
				fy = (attr & 0x20) != 0;
			}
			const uint8_t *src = s_gfx + (code % s_gfx_count) * 64 + (fy ? 7 - (ty & 7) : (ty & 7)) * 8 + (tx & 7);
			const int pb = ((attr & 0x0f) + 16) * 16;
			for (int i = 0; i < n; i++) {
				const int p = src[i];
				if (p || opaque)
					d[x + i] = (uint16_t)(pb + p);
			}
			x += n;
		}
	}
}

static void fk_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++) {   // (xBGR_555, little endian)
		const uint16_t v = (uint16_t)(s_vs.pal[2 * i] | s_vs.pal[2 * i + 1] << 8);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	const k007121_t *k = &s_vs.k121;
	for (int y = 0; y < VIS_H; y++)
		for (int x = 0; x < VIS_W; x++)
			rbx_video.bitmap[y * rbx_video.pitch + x] = 0x100;
	const int vx0 = VIS_X0, vx1 = VIS_X0 + VIS_W - 1;
	int c0x0 = vx0, c0x1 = vx1, c1x0 = vx0, c1x1 = vx1;
	if (k->flipscreen) {
		c0x1 -= 40;
		c1x0 = c1x1 - 39;
	} else {
		c0x0 += 40;
		c1x1 = 39;
	}
	draw_layer(0, k->ctrl[0] - 40, k->ctrl[2], 1, c0x0, c0x1);
	k007121_sprites_draw(k, s_gfx, s_gfx_count, 0, s_transmask, NULL, rbx_video.bitmap, rbx_video.pitch, VIS_X0, VIS_Y0,
	                     c0x0, VIS_Y0, c0x1, VIS_Y0 + VIS_H - 1, NULL, 0);
	draw_layer(1, 0, 0, 0, c1x0, c1x1);
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

// ---- the main CPU's bus (main_map) ----
static void map_bank(void)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x40 + p] = s_rom + ((uint32_t)t.mainbank * 0x2000 + (uint32_t)p * 256) % s_rom_size;
}

static void cpu_lines(int set, int cleared)   // (the K007121's IRQ)
{
	if (cleared & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 0);
	if (set & K007121_IRQ)
		hd6309_set_irq(&s_cpu, 1);
}

static uint8_t ls138_r(int o)
{
	switch ((o & 0x1c) >> 2) {
	case 0:
		if (o & 2)
			return (uint8_t)rbx_port_read((o & 1) ? s_port_coin : s_port_dsw[2]);
		return (uint8_t)rbx_port_read(s_port_p[o & 1]);
	case 1:
		if (o & 2)
			return (uint8_t)rbx_port_read((o & 1) ? s_port_dsw[0] : s_port_dsw[1]);
		return 0;
	}
	return 0;
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0800) {
		if (a >= 0x0020 && a < 0x0060)
			return s_k121.scroll[a - 0x0020];
		if (a >= 0x0400 && a < 0x0420)
			return ls138_r(a - 0x0400);
		return 0;
	}
	if (a < 0x0c00)
		return t.pal[a - 0x0800];
	if (a < 0x1000)
		return 0;
	if (a < 0x2000)
		return t.spr[a - 0x1000];
	if (a < 0x3000)
		return t.vram[a - 0x2000];
	if (a < 0x4000)
		return t.ram[a - 0x3000];
	if (a < 0x6000)
		return s_rom[((uint32_t)t.mainbank * 0x2000 + (a - 0x4000)) % s_rom_size];
	return s_rom[a];
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0800) {
		if (a < 0x0008)
			cpu_lines(0, k007121_ctrl_w(&s_k121, a, v));
		else if (a >= 0x0020 && a < 0x0060)
			s_k121.scroll[a - 0x0020] = v;
		else if (a >= 0x0400 && a < 0x0420)
			switch (((a - 0x0400) & 0x1c) >> 2) {   // (ls138_w)
			case 4:   // (bankswitch_w)
				if ((v & 3) != 3) {
					t.mainbank = v & 3;
					map_bank();
				}
				break;
			case 5: q_put(Q_LATCH, v); break;
			case 6: q_put(Q_IRQ, 0); break;   // (HOLD_LINE)
			}
		return;
	}
	if (a < 0x0c00)
		t.pal[a - 0x0800] = v;
	else if (a >= 0x1000 && a < 0x2000)
		t.spr[a - 0x1000] = v;
	else if (a >= 0x2000 && a < 0x3000)
		t.vram[a - 0x2000] = v;
	else if (a >= 0x3000 && a < 0x4000)
		t.ram[a - 0x3000] = v;
}

// ---- the sound board (sound_map) ----
static void k007232_port(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232, 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232, 1, 0, (data & 0x0f) * 0x11);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a >= 0x9000 && a < 0x9008) {   // (the K007452's results)
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
	if (a == 0xa000)
		return t.soundlatch;
	if (a >= 0xb000 && a < 0xb00e)
		return k007232_read(&s_k232, a - 0xb000);
	if (a == 0xc000 || a == 0xc001)
		return rbx_ym_read(0, a & 1);
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a >= 0x9000 && a < 0x9008) {   // (the K007452)
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
	} else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232, a - 0xb000, v);
	else if (a == 0xc000 || a == 0xc001)
		rbx_ym_write(0, a & 1, v);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// a line's samples: the Z80, the YM2151 0.60 stereo, the K007232 0.30 to both (held between its samples)
static void sound_run(int n)
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
	int32_t yl[96], yr[96];
	memset(yl, 0, sizeof(yl));
	memset(yr, 0, sizeof(yr));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance_lr(0, yc, yl, yr, 96);
	for (int i = 0; i < n && s_out_pos < 2048; i++, s_out_pos++) {
		t.k232_frac += CHIP_CLOCK / 128;
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			k007232_sample(&s_k232, &t.k232_a, &t.k232_b);
		}
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t pcm = (t.k232_a + t.k232_b) * 30 / 100;
		s_out_l[s_out_pos] = (k >= 0 ? yl[k] : 0) + pcm;
		s_out_r[s_out_pos] = (k >= 0 ? yr[k] : 0) + pcm;
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
		rbx_audio_put_stereo(s_out_l[i], s_out_r[i]);
	s_out_n = 0;
	s_qcur ^= 1;
	__atomic_store_n(&s_job_frame, 1, __ATOMIC_RELEASE);
	rbx_aux_kick();
}

// ---- init / frame ----
static void fk_exit(void);
static void fk_reset(void);

static int fk_init(const rbx_game *g)
{
	rbx_cheat_prefer("hd6309");   // (the cheats' main CPU: another CPU is set up first)
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t gsize, pcm_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *gfx = rbx_region("gfx", &gsize), *pcm = rbx_region("k007232", &pcm_size);
	if (!s_rom || !s_zrom || !gfx || !pcm) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	s_gfx_count = gsize / 32;   // (gfx_8x8x4_packed_msb)
	s_gfx = (uint8_t *)malloc((size_t)s_gfx_count * 64);
	if (!s_gfx) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	for (uint32_t n = 0; n < s_gfx_count; n++)
		for (int b = 0; b < 32; b++) {
			s_gfx[n * 64 + b * 2] = gfx[n * 32 + b] >> 4;
			s_gfx[n * 64 + b * 2 + 1] = gfx[n * 32 + b] & 15;
		}
	for (int c = 0; c < 32; c++)   // (no indirect colours: pen 0 transparent)
		s_transmask[c] = 1;
	k007121_init(&s_k121, 40, 16);   // (set_sprite_offsets(40, 16))
	k007232_init(&s_k232, pcm, pcm_size, k007232_port);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM2151;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = yc.gain[1] = 154;   // (0.60)
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM2151", g->set);
		fk_exit();
		return -1;
	}
	s_port_coin = rbx_port_find("COIN");
	s_port_p[0] = rbx_port_find("P1");
	s_port_p[1] = rbx_port_find("P2");
	static const char *const dn[3] = { "DSW1", "DSW2", "DSW3" };
	for (int i = 0; i < 3; i++)
		s_port_dsw[i] = rbx_port_find(dn[i]);
	if (rbx_audio_setup_stereo(SND_RATE) || rbx_video_setup(VIS_W, VIS_H, NPENS)) {
		rbx_fail("%s: out of memory", g->set);
		fk_exit();
		return -1;
	}
	fk_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void fk_exit(void)
{
	rbx_aux_set_job(NULL);
	rbx_ym_exit(0);
	free(s_gfx);
	s_gfx = NULL;
	free(s_q_hp);
	s_q_hp = NULL;
}

static void fk_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	k007121_reset(&s_k121);
	memset(s_k121.scroll, 0, sizeof(s_k121.scroll));
	k007232_reset(&s_k232);
	k007232_set_bank(&s_k232, 0, 1);   // (machine_reset)
	rbx_ym_reset(0);
	hd6309_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	for (int p = 0x60; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + (uint32_t)p * 256;
	for (int p = 0x10; p < 0x20; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.spr + (p - 0x10) * 256;
	for (int p = 0x30; p < 0x40; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + (p - 0x30) * 256;
	map_bank();
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

static void fk_frame(void)
{
	for (int line = 0; line < LINES; line++) {
		const int vpos = (VIS_Y0 + VIS_H + line) % LINES;   // (the frame from the vblank start)
		if (vpos == VIS_Y0 + VIS_H) {   // (vblank: the screen's update)
			capture();
			rbx_video_frame_done();
		}
		if (!(vpos & 15))   // (the K007121's scanline timer: every 16 lines)
			cpu_lines(k007121_scanline(&s_k121, vpos, t.frame, t.spr), 0);
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

static int fk_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "fk_cpu", &s_cpu, (uint32_t)offsetof(hd6309_t, rpage));
	rc |= rbx_state_io(ctx, "fk", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	rc |= rbx_state_io(ctx, "fk_k121", &s_k121, (uint32_t)offsetof(k007121_t, state_end));
	rc |= k007232_state(&s_k232, ctx, load, "fk_k232");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "fk_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "fk_outl", s_out_l, sizeof(s_out_l));
	rc |= rbx_state_io(ctx, "fk_outr", s_out_r, sizeof(s_out_r));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "fk_z80", &z, sizeof(z));
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
		map_bank();
	}
	return rc ? -1 : 0;
}

static int fk_save(void *ctx) { return fk_state(ctx, 0); }
static int fk_load(void *ctx) { return fk_state(ctx, 1); }
static int fk_supported(const char *machine) { return !strcmp(machine, "flkatck"); }

const rbx_driver rbx_drv_flkatck = { "Konami Flak Attack", fk_init, fk_exit, fk_reset, fk_frame, fk_save, fk_load,
                                     fk_supported, fk_render, NULL };

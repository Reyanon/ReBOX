/*  Konami Haunted Castle. Ported from MAME src/mame/konami/hcastle.cpp (BSD-3-Clause, copyright-holders: Bryan
    McPhail) and the devices it uses (k007121.cpp, k007232.cpp, k051649.cpp). The Konami CPU at 3 MHz, two K007121s
    (their 64x32 tilemaps as the driver's), the indirect palette through the colour PROMs; the sound: a Z80 at 3.58 MHz,
    a K007232, a YM3812 (its IRQ the Z80's NMI) and a K051649, a frame behind on the worker.
    This file is part of ReBOX. GPL-2.0-or-later.
*/
#include "../../rbx.h"
#include "../../cpu/konami/konami.h"
#include "../../cpu/z80/z80.h"
#include "../../sound/k007232.h"
#include "../../sound/k051649.h"
#include "../../sound/rbx_ym.h"
#include "../video/k007121.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define CPU_CLOCK 3000000u     // (24 MHz / 2, the CPU's internal divider of 4)
#define Z80_CLOCK 3579545u
#define CHIP_CLOCK 3579545u    // (the K007232, the YM3812, the K051649)
#define SND_RATE 48000
#define PIX_CLOCK 6000000u     // (set_raw(24 MHz / 4, 384, 0, 256, 264, 16, 240))
#define HTOTAL 384
#define LINES 264
#define VIS_X0 0
#define VIS_W 256
#define VIS_Y0 16
#define VIS_H 224
#define NPENS 4096

static uint8_t *s_rom, *s_zrom;
static uint32_t s_rom_size, s_zrom_size;
static uint8_t *s_gfx[2];
static uint32_t s_gfx_count[2];
static uint16_t s_ctab[NPENS];
static uint16_t s_transmask[2][128];
static konami_t s_cpu;
static z80 s_z;
static const uint8_t *s_zmap[256];
static k007121_t s_k121[2];
static k007232_t s_k232;
static k051649_t s_scc;
static int s_port_system, s_port_p[2], s_port_dsw[3];

static struct {
	uint8_t ram[0x6000];       // 0000-5fff (0600-06ff the palette, 1000-1fff RAM, 2000 / 4000 the tilemaps, 3000 / 5000 sprites)
	uint8_t bankedram[0x1000]; // (0800-0fff: two banks)
	uint8_t mainbank, rambank, gfx_bank;
	uint8_t zram[0x800], soundlatch, z80_irq, nmi_line;
	uint64_t k232_frac;
	int32_t k232_a, k232_b, scc_out;
	int64_t cycles_done;
	int64_t cpu_end, snd_end;   // (the CPUs' line targets: absolute, an instruction's overshoot not carried on)
	uint64_t cyc_frac, z_frac, z_end, ym_frac, snd_frac;
	uint32_t frame;
	uint8_t state_end;
} t;

// ---- the drawing's snapshot ----
static struct { uint8_t pal[0x100], pf[2][0x1000], gfx_bank; k007121_t k121[2]; } s_vs;

static void capture(void)
{
	memcpy(s_vs.pal, t.ram + 0x0600, sizeof(s_vs.pal));
	memcpy(s_vs.pf[0], t.ram + 0x2000, 0x1000);
	memcpy(s_vs.pf[1], t.ram + 0x4000, 0x1000);
	s_vs.gfx_bank = t.gfx_bank;
	for (int i = 0; i < 2; i++)
		memcpy(&s_vs.k121[i], &s_k121[i], sizeof(k007121_t));
}

// a 512x256 tilemap of 8x8 tiles (tilemap_scan, get_tile_info) at (scrollx, scrolly)
static void draw_layer(int which, int opaque)
{
	const k007121_t *k = &s_vs.k121[which];
	const uint8_t *c = k->ctrl, *pf = s_vs.pf[which];
	int bankbase = which ? 0x4000 * ((s_vs.gfx_bank & 2) >> 1) : 0;
	if (c[3] & 1)
		bankbase += 0x2000;
	const int bit0 = c[5] & 3, bit1 = (c[5] >> 2) & 3, bit2 = (c[5] >> 4) & 3, bit3 = (c[5] >> 6) & 3;
	const int colbase = (c[6] & 0x30) * 2 + 16, scrollx = (c[1] << 8 & 0x100) | c[0], scrolly = c[2];
	const uint8_t *gfx = s_gfx[which];
	const uint32_t count = s_gfx_count[which];
	const int base = which ? 2048 : 0;
	uint16_t *bm = rbx_video.bitmap;
	const int pitch = rbx_video.pitch;
	for (int y = VIS_Y0; y < VIS_Y0 + VIS_H; y++) {
		const int ty = (y + scrolly) & 255;
		uint16_t *d = bm + (y - VIS_Y0) * pitch - VIS_X0;
		for (int x = VIS_X0; x < VIS_X0 + VIS_W;) {
			const int tx = (x + scrollx) & 511;
			int n = 8 - (tx & 7);
			if (n > VIS_X0 + VIS_W - x)
				n = VIS_X0 + VIS_W - x;
			const int col = tx >> 3, row = ty >> 3;
			const int idx = (col & 0x1f) + ((row & 0x1f) << 5) + ((col & 0x20) << 6);
			const int attr = pf[idx];
			int bank = ((attr >> (bit0 + 3)) & 1) | ((attr >> (bit1 + 2)) & 2) | ((attr >> (bit2 + 1)) & 4) | ((attr >> bit3) & 8);
			bank = ((attr & 0x80) >> 7) | (bank << 1);
			const uint32_t code = (uint32_t)(pf[idx + 0x400] + bank * 0x100 + bankbase) % count;
			const uint8_t *src = gfx + code * 64 + (ty & 7) * 8 + (tx & 7);
			const int pb = base + (colbase + (attr & 7)) * 16;
			for (int i = 0; i < n; i++) {
				const int p = src[i];
				if (p || opaque)
					d[x + i] = (uint16_t)(pb + p);
			}
			x += n;
		}
	}
}

static void sprite_cb0(int *code, int *color, int colbank)
{
	*color += colbank;
	*code += 0x4000 * (s_vs.gfx_bank & 1);
}
static void sprite_cb1(int *code, int *color, int colbank) { (void)code; *color += colbank; }

static void draw_sprites(int which)
{
	k007121_sprites_draw(&s_vs.k121[which], s_gfx[which], s_gfx_count[which], which ? 2048 : 0, s_transmask[which],
	                     which ? sprite_cb1 : sprite_cb0, rbx_video.bitmap, rbx_video.pitch, VIS_X0, VIS_Y0, VIS_X0, VIS_Y0,
	                     VIS_X0 + VIS_W - 1, VIS_Y0 + VIS_H - 1, NULL, 0);
}

static void hc_render(void)   // (screen_update)
{
	for (int i = 0; i < NPENS; i++) {   // (the indirect palette: xBGR_555, the high byte first)
		const int e = s_ctab[i];
		const uint16_t v = (uint16_t)(s_vs.pal[2 * e] << 8 | s_vs.pal[2 * e + 1]);
		const int r = v & 31, g = (v >> 5) & 31, b = (v >> 10) & 31;
		rbx_video.palette[i] = (uint32_t)((r << 3 | r >> 2) << 16 | (g << 3 | g >> 2) << 8 | (b << 3 | b >> 2));
	}
	draw_layer(1, 1);
	if (!(s_vs.gfx_bank & 0x04)) {
		draw_sprites(0);
		draw_sprites(1);
		draw_layer(0, 0);
	} else {
		draw_layer(0, 0);
		draw_sprites(0);
		draw_sprites(1);
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
		konami_set_irq(&s_cpu, 0);
	if (set & K007121_IRQ)
		konami_set_irq(&s_cpu, 1);
}

static void map_banks(void)
{
	for (int p = 0; p < 0x20; p++)
		s_cpu.rpage[0x60 + p] = s_rom + (0x10000u + (uint32_t)t.mainbank * 0x2000 + (uint32_t)p * 256) % s_rom_size;
	for (int p = 0; p < 8; p++)
		s_cpu.rpage[0x08 + p] = s_cpu.wpage[0x08 + p] = t.bankedram + t.rambank * 0x800 + p * 256;
}

static uint8_t main_rd(void *ctx, uint16_t a)
{
	(void)ctx;
	if (a < 0x0600) {
		if (a >= 0x0020 && a < 0x0060)
			return s_k121[0].scroll[a - 0x0020];
		if (a >= 0x0220 && a < 0x0260)
			return s_k121[1].scroll[a - 0x0220];
		switch (a) {
		case 0x0410: return (uint8_t)rbx_port_read(s_port_system);
		case 0x0411: case 0x0412: return (uint8_t)rbx_port_read(s_port_p[a - 0x0411]);
		case 0x0413: return (uint8_t)rbx_port_read(s_port_dsw[2]);
		case 0x0414: return (uint8_t)rbx_port_read(s_port_dsw[0]);
		case 0x0415: return (uint8_t)rbx_port_read(s_port_dsw[1]);
		case 0x0418: return t.gfx_bank;
		}
		return 0;
	}
	if (a < 0x0700)
		return t.ram[a];
	if (a < 0x0800)
		return 0;
	if (a < 0x1000)
		return t.bankedram[t.rambank * 0x800 + (a - 0x0800)];
	if (a < 0x6000)
		return t.ram[a];
	if (a < 0x8000)
		return s_rom[(0x10000u + (uint32_t)t.mainbank * 0x2000 + (a - 0x6000)) % s_rom_size];
	return s_rom[a];
}

static void main_wr(void *ctx, uint16_t a, uint8_t v)
{
	(void)ctx;
	if (a < 0x0600) {
		if (a < 0x0008)
			irq_lines(0, k007121_ctrl_w(&s_k121[0], a, v));
		else if (a >= 0x0020 && a < 0x0060)
			s_k121[0].scroll[a - 0x0020] = v;
		else if (a >= 0x0200 && a < 0x0208)
			k007121_ctrl_w(&s_k121[1], a - 0x0200, v);   // (the K007121 #1's interrupts not connected)
		else if (a >= 0x0220 && a < 0x0260)
			s_k121[1].scroll[a - 0x0220] = v;
		else if (a == 0x0400) {   // (bankswitch_w: the ROM's bank, bit 5 the RAM's)
			t.mainbank = v & 0x0f;
			t.rambank = (v >> 5) & 1;
			map_banks();
		} else if (a == 0x0404)
			q_put(Q_LATCH, v);
		else if (a == 0x0408)
			q_put(Q_IRQ, 0);   // (soundirq_w: HOLD_LINE)
		else if (a == 0x0418)
			t.gfx_bank = v;
		return;
	}
	if (a < 0x0700)
		t.ram[a] = v;
	else if (a >= 0x0800 && a < 0x1000)
		t.bankedram[t.rambank * 0x800 + (a - 0x0800)] = v;
	else if (a >= 0x1000 && a < 0x6000)
		t.ram[a] = v;
}

// ---- the sound board (sound_map) ----
static void k007232_port(uint8_t data)   // (volume_callback)
{
	k007232_set_volume(&s_k232, 0, (data >> 4) * 0x11, 0);
	k007232_set_volume(&s_k232, 1, 0, (data & 0x0f) * 0x11);
}
static void ym_irq(int state)   // (the YM3812's IRQ to the Z80's NMI)
{
	if (state && !t.nmi_line)
		z80_gen_nmi(&s_z);
	t.nmi_line = (uint8_t)(state != 0);
}

static uint8_t z_rd(void *u, uint16_t a)
{
	(void)u;
	if (a < 0x8000)
		return a < s_zrom_size ? s_zrom[a] : 0;
	if (a < 0x8800)
		return t.zram[a - 0x8000];
	if (a >= 0x9800 && a < 0x9900)
		return k051649_scc_r(&s_scc, a & 0xff, 0);
	if (a == 0xa000 || a == 0xa001)
		return rbx_ym_read(0, a & 1);
	if (a >= 0xb000 && a < 0xb00e)
		return k007232_read(&s_k232, a - 0xb000);
	if (a == 0xd000)
		return t.soundlatch;
	return 0;
}
static void z_wr(void *u, uint16_t a, uint8_t v)
{
	(void)u;
	if (a >= 0x8000 && a < 0x8800)
		t.zram[a - 0x8000] = v;
	else if (a >= 0x9800 && a < 0x9900)
		k051649_scc_w(&s_scc, a & 0xff, v);
	else if (a == 0xa000 || a == 0xa001)
		rbx_ym_write(0, a & 1, v);
	else if (a >= 0xb000 && a < 0xb00e)
		k007232_write(&s_k232, a - 0xb000, v);
	else if (a == 0xc000)   // (sound_bank_w)
		k007232_set_bank(&s_k232, v & 3, (v >> 2) & 3);
}
static uint8_t z_in(z80 *z, uint8_t p) { (void)z; (void)p; return 0xff; }
static void z_out(z80 *z, uint8_t p, uint8_t v) { (void)z; (void)p; (void)v; }

// a line's n samples: the Z80, the YM3812 0.70, the K007232 0.44 / 0.50 (held between its samples), the K051649 0.45
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
	int32_t ym[96];
	memset(ym, 0, sizeof(ym));
	t.ym_frac += (uint64_t)CHIP_CLOCK * (uint64_t)n;
	const uint32_t yc = (uint32_t)(t.ym_frac / SND_RATE);
	t.ym_frac -= (uint64_t)yc * SND_RATE;
	const int yn = rbx_ym_advance(0, yc, ym, 96);
	for (int i = 0; i < n; i++) {
		t.k232_frac += CHIP_CLOCK / 128;
		while (t.k232_frac >= SND_RATE) {
			t.k232_frac -= SND_RATE;
			k007232_sample(&s_k232, &t.k232_a, &t.k232_b);
		}
		k051649_render(&s_scc, &t.scc_out, 1);
		const int k = yn > 0 ? i * yn / n : -1;
		const int32_t m = (k >= 0 ? ym[k] * 70 / 100 : 0) + t.k232_a * 44 / 100 + t.k232_b / 2 + t.scc_out * 45 / 100;
		if (s_out_pos < 2048)
			s_out[s_out_pos++] = m;
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
static void hc_exit(void);
static void hc_reset(void);

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

static int hc_init(const rbx_game *g)
{
	if (!s_q_hp && !(s_q_hp = calloc(1, sizeof(*s_q_hp)))) {
		rbx_fail("%s: out of memory", g->set);
		return -1;
	}
	uint32_t g0, g1, psize, pcm_size;
	s_rom = rbx_region("maincpu", &s_rom_size);
	s_zrom = rbx_region("audiocpu", &s_zrom_size);
	const uint8_t *gfx0 = rbx_region("k007121_1", &g0), *gfx1 = rbx_region("k007121_2", &g1);
	const uint8_t *prom = rbx_region("proms", &psize), *pcm = rbx_region("k007232", &pcm_size);
	if (!s_rom || !s_zrom || !gfx0 || !gfx1 || !prom || psize < 0x400 || !pcm) {
		rbx_fail("%s: ROM regions missing", g->set);
		return -1;
	}
	memset(&t, 0, sizeof(t));
	if (decode_gfx(0, gfx0, g0) || decode_gfx(1, gfx1, g1)) {
		rbx_fail("%s: out of memory", g->set);
		hc_exit();
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
	for (int chip = 0; chip < 2; chip++)   // (transpen_mask(gfx, color, 0))
		for (int c = 0; c < 128; c++) {
			uint16_t m = 0;
			for (int p = 0; p < 16; p++)
				if (s_ctab[(chip ? 2048 : 0) + (c * 16 + p) % 2048] == 0)
					m |= (uint16_t)(1u << p);
			s_transmask[chip][c] = m;
		}
	k007121_init(&s_k121[0], 0, 0);
	k007121_init(&s_k121[1], 0, 0);
	k007232_init(&s_k232, pcm, pcm_size, k007232_port);
	k051649_init(&s_scc, CHIP_CLOCK);
	k051649_set_rate(&s_scc, SND_RATE);
	rbx_ym_config yc;
	memset(&yc, 0, sizeof(yc));
	yc.type = RBX_YM3812;
	yc.clock = CHIP_CLOCK;
	yc.gain[0] = 256;
	yc.irq = ym_irq;
	if (rbx_ym_init(0, &yc)) {
		rbx_fail("%s: YM3812", g->set);
		hc_exit();
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
		hc_exit();
		return -1;
	}
	hc_reset();
	rbx_aux_set_job(job);
	return 0;
}

static void hc_exit(void)
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

static void hc_reset(void)
{
	rbx_aux_sync();
	s_qn[0] = s_qn[1] = 0;
	s_qcur = 0;
	s_out_n = s_out_pos = 0;
	s_job_frame = 0;
	memset(&t, 0, offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 2; i++) {
		k007121_reset(&s_k121[i]);
		memset(s_k121[i].scroll, 0, sizeof(s_k121[i].scroll));
	}
	k007232_reset(&s_k232);
	k051649_reset(&s_scc);
	rbx_ym_reset(0);
	konami_init(&s_cpu);
	s_cpu.read = main_rd;
	s_cpu.write = main_wr;
	s_cpu.ctx = NULL;
	for (int p = 0x80; p < 0x100; p++)
		s_cpu.rpage[p] = s_rom + (uint32_t)p * 256;
	for (int p = 0x10; p < 0x60; p++)
		s_cpu.rpage[p] = s_cpu.wpage[p] = t.ram + p * 256;
	map_banks();
	konami_reset(&s_cpu);
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

static void hc_frame(void)
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
		konami_run_until(&s_cpu, t.cpu_end += cyc);
		t.snd_frac += (uint64_t)SND_RATE * HTOTAL;
		const int ns = (int)(t.snd_frac / PIX_CLOCK);
		t.snd_frac -= (uint64_t)ns * PIX_CLOCK;
		if (ns > 0)
			q_put(Q_ADV, (uint32_t)ns);
	}
	sound_frame();
	t.frame++;
}

static int hc_state(void *ctx, int load)
{
	int rc = 0;
	rbx_aux_sync();
	rc |= rbx_state_io(ctx, "hc_cpu", &s_cpu, KONAMI_STATE_SIZE);
	rc |= rbx_state_io(ctx, "hc", &t, (uint32_t)offsetof(__typeof__(t), state_end));
	for (int i = 0; i < 2; i++)
		rc |= rbx_state_io(ctx, i ? "hc_k121b" : "hc_k121a", &s_k121[i], (uint32_t)offsetof(k007121_t, state_end));
	rc |= k007232_state(&s_k232, ctx, load, "hc_k232");
	rc |= k051649_state(&s_scc, ctx, load, "hc_scc");
	rc |= rbx_ym_state(0, ctx, load);
	int32_t qn = s_out_n;
	rc |= rbx_state_io(ctx, "hc_q", &qn, sizeof(qn));
	rc |= rbx_state_io(ctx, "hc_out", s_out, sizeof(s_out));
	z80 z = s_z;
	rc |= rbx_state_io(ctx, "hc_z80", &z, sizeof(z));
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
		map_banks();
	}
	return rc ? -1 : 0;
}

static int hc_save(void *ctx) { return hc_state(ctx, 0); }
static int hc_load(void *ctx) { return hc_state(ctx, 1); }
static int hc_supported(const char *machine) { return !strcmp(machine, "hcastle"); }

const rbx_driver rbx_drv_hcastle = { "Konami Haunted Castle", hc_init, hc_exit, hc_reset, hc_frame, hc_save, hc_load,
                                     hc_supported, hc_render, NULL };
